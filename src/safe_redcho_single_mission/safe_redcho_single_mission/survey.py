"""Read-only ground survey and immutable shared experiment configuration.

ROS imports are deferred: numerical validation, commit and resolve are testable
without ROS. No flight service clients or command publishers are created here.
"""
import argparse
from contextlib import contextmanager
import fcntl
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import sys
from pathlib import Path
import time
import uuid

import numpy as np
import yaml
from .health import History, tilt, yaw, rotation_error
from .settings import NAMES, make_plan, validate
from .protocol import fingerprint

WINDOW_S = 2.0
COVERAGE_S = 1.8


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def read_base(path):
    c = yaml.safe_load(Path(path).read_text())
    if not isinstance(c, dict) or c.get('names') != NAMES or c.get('frame_id') != 'world' or c.get('namespace') != '/safe_redcho_single_mission':
        raise ValueError('Survey requires cf2/cf3/cf4 and the world mission profile')
    names = [o.get('name') for o in c.get('obstacles', [])]
    if any(not isinstance(n, str) or not n or n in NAMES for n in names) or len(set(names)) != len(names):
        raise ValueError('Invalid or duplicate obstacle mocap names')
    return c


class SurveyWindow:
    def __init__(self, cfg):
        self.c = cfg
        self.obstacles = {o['name']: o for o in cfg.get('obstacles', [])}
        self.histories = {name: History('/poses['+name+']') for name in NAMES+list(self.obstacles)}
        self.rejected = Counter()
        self.receipt_frames = 0

    def ingest(self, frame_id, stamp, poses, ros_now, mono):
        # poses contains (name, xyz, quaternion_xyzw), all from one mocap frame.
        self.receipt_frames += 1
        if frame_id != self.c['frame_id']:
            for h in self.histories.values(): h.rows.clear()
            self.rejected['wrong_frame'] += 1
            return
        counts = Counter(name for name, _, _ in poses)
        for name, h in self.histories.items():
            if counts[name] != 1:
                h.rows.clear()
                self.rejected[name+':missing_or_duplicate'] += 1
                continue
            _, p, q = next(row for row in poses if row[0] == name)
            p, q = np.asarray(p, float), np.asarray(q, float)
            norm = np.linalg.norm(q)
            if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all() or not .95 <= norm <= 1.05:
                h.rows.clear()
                self.rejected[name+':invalid_pose'] += 1
                continue
            if not h.add(p, q/norm, stamp, ros_now, mono, self.c['safety']['mocap_maximum_age_s']):
                h.rows.clear()
                self.rejected[name+':invalid_timestamp'] += 1

    def result(self, ros_now, mono):
        positions, quaternions, evidence = {}, {}, {}
        for name, h in self.histories.items():
            h.latest(ros_now, mono, self.c['safety']['mocap_maximum_age_s'])
            recent = [r for r in h.rows if mono-r[1] <= WINDOW_S]
            if len(recent) < 5:
                raise RuntimeError(name+': waiting for a stable survey window')
            source = np.array([r[0] for r in recent])
            receipts = np.array([r[1] for r in recent])
            coverage = float(source[-1]-source[0])
            rate = (len(recent)-1)/coverage if coverage > 0 else 0
            gap = max(float(np.max(np.diff(source))), float(np.max(np.diff(receipts))))
            if coverage < COVERAGE_S or rate < self.c['safety']['minimum_mocap_rate_hz'] or gap > self.c['safety']['mocap_maximum_age_s']:
                raise RuntimeError(name+': insufficient survey coverage/rate or a pose gap')
            xyz = np.array([r[2] for r in recent]); center = np.median(xyz, axis=0)
            span = np.ptp(xyz, axis=0)
            last_q = recent[-1][3]
            if max(rotation_error(last_q, r[3]) for r in recent) > .10:
                raise RuntimeError(name+': orientation unstable during survey')
            if name in self.obstacles:
                o = self.obstacles[name]
                # Keep the surveyed reference well within the live movement guard.
                xy_error = float(np.max(np.linalg.norm(xyz[:,:2]-center[:2],axis=1)))
                z_error = float(np.max(np.abs(xyz[:,2]-center[2])))
                if xy_error > o['center_tolerance_m']/2 or z_error > o['height_tolerance_m']/2:
                    raise RuntimeError(name+': obstacle moving or tracking unstable')
                if any(tilt(r[3]) > o['maximum_tilt_rad'] for r in recent):
                    raise RuntimeError(name+': obstacle is not upright')
            else:
                if np.max(span) > .03 or abs(center[2]-self.c['arena']['floor_z_m']) > .08:
                    raise RuntimeError(name+': keep aircraft stationary on the floor')
                if any(tilt(r[3]) > self.c['safety']['measured_maximum_tilt_rad'] for r in recent):
                    raise RuntimeError(name+': aircraft not level')
                if np.linalg.norm(h.velocity(self.c['safety']['mocap_velocity_window_s'], self.c['safety']['mocap_maximum_age_s'])) > .05:
                    raise RuntimeError(name+': aircraft is moving')
            positions[name], quaternions[name] = center.tolist(), last_q.tolist()
            evidence[name] = dict(samples=len(recent), coverage_s=coverage, accepted_rate_hz=rate,
                maximum_gap_s=gap, axis_span_m=span.tolist(), median_xyz_m=center.tolist(),
                source_age_s=ros_now-source[-1], receipt_age_s=mono-receipts[-1])
        return dict(positions=positions, quaternions=quaternions, evidence=evidence,
                    rejected=dict(self.rejected))


def quiet_graph(node, cfg):
    names = {name for name, _ in node.get_node_names_and_namespaces()}
    if names & ({'single_mission_coordinator','single_mission_watchdog'} | {'single_mission_agent_'+n for n in NAMES}):
        raise RuntimeError('Stop coordinator, agents and watchdog before surveying; keep aircraft grounded')
    topics = [cfg['namespace']+suffix for suffix in ('/frame','/agent_ready','/heartbeat')]
    topics += ['/'+n+'/'+suffix for n in NAMES for suffix in
               ('cmd_position','cmd_full_state','cmd_vel_legacy','cmd_hover','cmd_velocity_world')]
    if any(node.count_publishers(topic) for topic in topics):
        raise RuntimeError('Active mission/command publisher found; stop control processes before survey')


def commit_survey(base_path, output_path, sample):
    base_path, output_path = Path(base_path).resolve(), Path(output_path).resolve()
    if base_path == output_path:
        raise ValueError('Survey output must be separate from the source YAML')
    source_bytes = base_path.read_bytes()
    c = read_base(base_path)
    if set(sample['positions']) != set(NAMES) | {o['name'] for o in c.get('obstacles', [])}:
        raise ValueError('Survey body set does not match source configuration')
    for o in c.get('obstacles', []):
        xyz = sample['positions'][o['name']]
        o['center_xy'] = [float(xyz[0]), float(xyz[1])]
        o['pivot_height_m'] = float(xyz[2])
    # Schema, uncertainty, footprint and all reference starts/goals remain checked.
    validate(c)
    ground = [sample['positions'][n] for n in NAMES]
    yaws = [yaw(sample['quaternions'][n]) for n in NAMES]
    plan = make_plan(c, ground, yaws)
    from .obstacles import check_measured_clearance
    check_measured_clearance(ground, c)
    survey_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_'+uuid.uuid4().hex[:8]
    directory = Path(c['workspace']).expanduser().resolve()/'results/single_mission_surveys'/survey_id
    starts_path = directory/'starts.yaml'
    c['starts_file'] = str(starts_path)
    capture_hash = fingerprint(c)
    snapshot = dict(captured_utc=datetime.now(timezone.utc).isoformat(), frame_id=c['frame_id'],
        positions=dict(zip(NAMES,ground)), yaw_rad=dict(zip(NAMES,map(float,yaws))),
        planned_assignment=plan['assignment'], capture_config_hash=capture_hash, survey_id=survey_id)
    starts_text = yaml.safe_dump(snapshot,sort_keys=False)
    c['_survey'] = dict(version=1, id=survey_id, source=str(base_path), source_sha256=sha256(source_bytes),
        config_hash=capture_hash, starts_sha256=sha256(starts_text.encode()), report=str(directory/'survey.json'))
    # Commit immutable supporting files first, then atomically publish the shared profile.
    directory.mkdir(parents=True)
    (directory/'starts.yaml').write_text(starts_text)
    (directory/'survey.json').write_text(json.dumps(dict(survey_id=survey_id,source=str(base_path),
        scope='Ground mocap survey; no flight commands; geometry radius remains user-specified',
        measurements=sample,obstacles=c['obstacles'],plan=plan,config_hash=capture_hash),indent=2))
    profile_text = yaml.safe_dump(c,sort_keys=False)
    (directory/'profile.yaml').write_text(profile_text)
    output_path.parent.mkdir(parents=True,exist_ok=True)
    temp = output_path.with_name(output_path.name+'.'+survey_id+'.tmp')
    try:
        temp.write_text(profile_text)
        temp.replace(output_path)
    finally:
        if temp.exists():temp.unlink()
    return c, directory


def resolve_profile(base_path, output_path):
    base_path, output_path = Path(base_path).resolve(), Path(output_path).resolve()
    if not output_path.is_file():
        raise RuntimeError('No ground survey found. Run the experiment capture command first.')
    c = yaml.safe_load(output_path.read_text()); meta = c.get('_survey', {})
    if meta.get('version') != 1 or meta.get('source') != str(base_path) or meta.get('source_sha256') != sha256(base_path.read_bytes()):
        raise RuntimeError('Source YAML changed or survey belongs to another profile. Stop control processes and capture again.')
    validate(c)
    if fingerprint(c) != meta.get('config_hash'):
        raise RuntimeError('Surveyed profile was edited. Edit the source YAML and capture again.')
    starts_path = Path(c['starts_file'])
    if not starts_path.is_file() or sha256(starts_path.read_bytes()) != meta.get('starts_sha256'):
        raise RuntimeError('Surveyed starts were removed/edited. Capture again.')
    immutable = Path(meta['report']).with_name('profile.yaml')
    if not immutable.is_file() or immutable.read_bytes() != output_path.read_bytes():
        raise RuntimeError('Immutable survey profile missing or changed. Capture again.')
    return immutable.resolve()


@contextmanager
def survey_lock(output_path):
    directory = Path(output_path).resolve().parent
    directory.mkdir(parents=True,exist_ok=True)
    with (directory/'.ground_survey.lock').open('a') as handle:
        try:
            fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Another ground survey is running; wait for it to finish')
        try:
            yield
        finally:
            fcntl.flock(handle,fcntl.LOCK_UN)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--resolve', action='store_true')
    parser.add_argument('--timeout', type=float, default=45.)
    args = parser.parse_args()
    if args.resolve:
        print(resolve_profile(args.config,args.output)); return
    if not np.isfinite(args.timeout) or not 3 <= args.timeout <= 120:
        raise ValueError('Survey timeout must be between 3 and 120 seconds')
    with survey_lock(args.output):
        collect_survey(args)


def collect_survey(args):
    c = read_base(args.config)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from motion_capture_tracking_interfaces.msg import NamedPoseArray
    rclpy.init(); node = Node('hocbf_ground_survey'); window = SurveyWindow(c)
    def receive(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        poses = [(p.name,[p.pose.position.x,p.pose.position.y,p.pose.position.z],
                  [p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w]) for p in msg.poses]
        window.ingest(msg.header.frame_id, stamp, poses, node.get_clock().now().nanoseconds*1e-9,time.monotonic())
    sub = node.create_subscription(NamedPoseArray,'/poses',receive,qos_profile_sensor_data)
    end = time.monotonic()+args.timeout; last_log = 0.; began = time.monotonic()
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node,timeout_sec=.02)
            # Continuously reject graph changes; no profile is updated while agents run.
            quiet_graph(node,c)
            try:
                if time.monotonic()-began < 2.0 or node.count_publishers('/poses') != 1:
                    raise RuntimeError('Waiting for exactly one /poses publisher and graph discovery')
                sample = window.result(node.get_clock().now().nanoseconds*1e-9,time.monotonic())
                for o in c.get('obstacles',[]):
                    xyz=sample['positions'][o['name']]
                    node.get_logger().info(f"Detected {o['name']}: XY={xyz[:2]}, pivot Z={xyz[2]:.5f} m")
                updated,directory = commit_survey(args.config,args.output,sample)
                node.get_logger().info('SURVEY PASS: shared profile '+str(Path(args.output).resolve()))
                node.get_logger().info('Captured aircraft starts and obstacle poses; report '+str(directory/'survey.json'))
                return
            except RuntimeError as exc:
                if time.monotonic()-last_log > 1:
                    node.get_logger().info(str(exc)); last_log=time.monotonic()
        raise RuntimeError('Ground survey timed out; shared profile was not updated')
    finally:
        node.destroy_node();rclpy.try_shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Ground survey cancelled; no flight commands were sent',file=sys.stderr)
        raise SystemExit(130)
    except (RuntimeError,ValueError,OSError) as exc:
        print('SURVEY ERROR: '+str(exc),file=sys.stderr)
        raise SystemExit(2)

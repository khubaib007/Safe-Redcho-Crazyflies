"""ROS state reception, six-agent cycle barrier, services and radio handover."""
import itertools
import json
from collections import deque
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.duration import Duration
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import FullState, Status
from crazyflie_interfaces.srv import Arm, Takeoff, Land, NotifySetpointsStop
from safe_redcho_bringup.fleet_receiver import FleetReceiver
from safe_redcho_bringup.fleet_configuration import load_fleet
from safe_redcho_bringup.pose_metrics import coordinates, distance, tilt, orientation_error, quaternion
from .settings import slots, pair_distance
from .measurement_health import PoseHistory, rotation_error, touchdown_evidence
from .protocol import encode, decode, fingerprint
from . import pose_receiver_fix as _pose_receiver_fix


def fit_velocity(stream, window):
    now = time.monotonic()
    rows = [(t, p) for t, p in stream.samples if t >= now-window]
    if len(rows) < 5 or rows[-1][0]-rows[0][0] < window*0.5:
        raise RuntimeError('Insufficient fresh mocap velocity history')
    ts = np.array([t-rows[0][0] for t, _ in rows])
    ts -= ts.mean()
    positions = np.array([p for _, p in rows])
    return ts@positions/max(float(ts@ts), 1e-12)


class FlightTransport(FleetReceiver):
    def __init__(self, cfg, output):
        if not cfg.get('workspace') or not cfg.get('fleet_config_file'):
            raise ValueError('Use the installed hardware profile, not the offline test fixture')
        fleet = load_fleet(cfg['fleet_config_file'], require_starts=True, require_arena=True)
        super().__init__(fleet, 'safe_redcho_hardware_mission')
        self.profile, self.output = cfg, Path(output)
        _pose_receiver_fix.require_joined_receivers(self)
        self.output.mkdir(parents=True, exist_ok=True)
        self.avoidance_configuration = {}
        self.stage_name = 'GROUND'
        self.measurement_fault = None
        self.mocap_history = {n: PoseHistory() for n in self.names}
        self.estimator_history = {n: PoseHistory() for n in self.names}
        self.health_records = deque(maxlen=2400)
        self.last_health_record = -float('inf')
        self.last_fault = None
        (self.output/'effective_profile.json').write_text(
            json.dumps(cfg, indent=2, allow_nan=False)+'\n')
        self.get_logger().info(
            f'Effective profile={cfg.get("_path", "unknown")}; '
            f'speed guard={cfg["safety"]["measured_maximum_speed_mps"]:.3f} m/s; '
            f'orientation guard={cfg["safety"]["maximum_estimator_attitude_error_rad"]:.3f} rad; '
            f'velocity window={cfg["safety"]["mocap_velocity_window_s"]:.3f}s')
        self.latest_header_ns = None
        self.mocap_frame_error = None
        self.batteries, self.readiness, self.command_results = {}, {}, {}
        self.watchdog_seen = None
        self.flight_active, self.streaming, self.expected = False, False, None
        self.session, self.sequence = '', 0
        self.frame_publisher = self.create_publisher(String, '/safe_redcho/frame', 8)
        self.heartbeat_publisher = self.create_publisher(String, '/safe_redcho/heartbeat', 8)
        self.fullstate_publishers = {n: self.create_publisher(FullState, f'/{n}/cmd_full_state', 2) for n in self.names}
        self.flight_service_clients = {n: {label: self.create_client(cls, f'/{n}/{label}') for label, cls in
            [('arm', Arm), ('takeoff', Takeoff), ('land', Land), ('notify_setpoints_stop', NotifySetpointsStop)]}
            for n in self.names}
        self.create_subscription(String, '/safe_redcho/agent_ready', self.ready, 16)
        self.create_subscription(String, '/safe_redcho/commands', self.commands, 16)
        self.create_subscription(String, '/safe_redcho/watchdog_ready', self.watchdog_ready, 4)
        for n in self.names:
            self.create_subscription(Status, f'/{n}/status', lambda msg, n=n: self.battery(msg, n), 4)
        self.create_timer(0.04, self.pulse)

    def create_subscription(self, msg_type, topic, callback, qos_profile, *args, **kwargs):
        callback = _pose_receiver_fix.joined_callback(self, topic, callback)
        return super().create_subscription(msg_type, topic, callback, qos_profile, *args, **kwargs)

    def receive(self, msg):
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        now = self.get_clock().now().nanoseconds
        if msg.header.frame_id != 'world':
            self.mocap_frame_error = 'Mocap world frame changed'
            return
        if stamp <= 0 or now-stamp > 150_000_000 or stamp-now > 50_000_000:
            return  # Does not refresh per-aircraft receipt ages.
        if self.latest_header_ns is not None and stamp <= self.latest_header_ns:
            return
        self.latest_header_ns = stamp
        received = time.monotonic()
        counts = {n: sum(p.name == n for p in msg.poses) for n in self.names}
        for item in msg.poses:
            if item.name in self.mocap_history and counts[item.name] == 1:
                try:
                    self.mocap_history[item.name].add(stamp*1e-9, received,
                        coordinates(item.pose), quaternion(item.pose))
                except RuntimeError as exc:
                    self.measurement_fault = f'{item.name}: {exc}'
        super().receive(msg)

    def receive_estimator_history(self, msg, name):
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        source_now = self.get_clock().now().nanoseconds*1e-9
        maximum_age = self.profile['safety']['maximum_estimator_age_s']
        if msg.header.frame_id != self.profile['frame_id']:
            self.measurement_fault = f'{name}: onboard pose frame mismatch'
            _pose_receiver_fix.reject(self, name, 'frame_mismatch')
            return False
        if stamp <= 0 or source_now-stamp*1e-9 > maximum_age or stamp*1e-9-source_now > .05:
            _pose_receiver_fix.reject(self, name, 'missing_stale_or_future_header')
            return False
        p, q = np.asarray(coordinates(msg.pose), float), np.asarray(quaternion(msg.pose), float)
        if not np.isfinite(p).all() or not np.isfinite(q).all() or not .95 <= np.linalg.norm(q) <= 1.05:
            _pose_receiver_fix.reject(self, name, 'invalid_pose_or_quaternion')
            return False
        rows = self.estimator_history[name].rows
        if rows and stamp*1e-9 <= rows[-1][0]:
            _pose_receiver_fix.reject(self, name, 'duplicate_or_out_of_order_header')
            return False
        try:
            self.estimator_history[name].add(stamp*1e-9, time.monotonic(), p, q)
            if not self.estimator_history[name].rows or abs(self.estimator_history[name].rows[-1][0]-stamp*1e-9) > 1e-6:
                _pose_receiver_fix.reject(self, name, 'history_did_not_accept_sample')
                return False
            return True
        except RuntimeError as exc:
            self.measurement_fault = f'{name}: {exc}'
            _pose_receiver_fix.reject(self, name, 'history_validation_failure')
            return False

    def diagnostics(self):
        return {'pose_receiver_version': _pose_receiver_fix.VERSION,
                'joined_pose_names': list(getattr(self, '_joined_pose_names', [])),
                'pose_rejections': getattr(self, 'pose_rejections', {}),
                'last_fault': self.last_fault, 'samples': list(self.health_records),
                'timestamp_note': 'Mocap velocity uses accepted ROS header time. '
                'Onboard header time may represent server publication, not firmware acquisition. '
                'Raw ROS epoch and host monotonic clocks are stored separately.'}

    def guard_fault(self, message, **details):
        details = _pose_receiver_fix.clean(details)
        self.last_fault = dict(stage=self.stage_name, monotonic_s=time.monotonic(),
                               message=message, **details)
        _pose_receiver_fix.persist_fault(self)
        raise RuntimeError(message+' | '+json.dumps(details, allow_nan=False))

    def battery(self, msg, name):
        # Status.battery_voltage is in volts. Reject unsupported units/layout.
        value = float(msg.battery_voltage)
        if math.isfinite(value) and 2.5 < value < 4.5:
            self.batteries[name] = (time.monotonic(), value)

    def ready(self, msg):
        try:
            value = decode(msg.data)
            if value['name'] in self.names:
                self.readiness[(value['name'], value['pid'])] = (time.monotonic(), value['config_hash'])
        except (ValueError, KeyError): pass

    def watchdog_ready(self, msg):
        try:
            packet = decode(msg.data)
            if packet['config_hash'] == fingerprint(self.profile) and not packet.get('triggered', False):
                self.watchdog_seen = time.monotonic()
        except (ValueError, KeyError): pass

    def commands(self, msg):
        try:
            value = decode(msg.data)
            if value.get('session') != self.session or value.get('seq') != self.sequence or value.get('name') not in self.names:
                return
            if value['name'] in self.command_results and value != self.command_results[value['name']]:
                self.command_results[value['name']] = {'error': 'Conflicting duplicate controller response'}
            else: self.command_results[value['name']] = value
        except (ValueError, KeyError): pass

    def pulse(self):
        self.heartbeat_publisher.publish(String(data=encode({'session': self.session,
            'active': self.flight_active, 'streaming': self.streaming, 'seq': self.sequence,
            'monotonic_stamp': time.monotonic(),
            'config_hash': fingerprint(self.profile)})))

    def snapshot(self):
        source_now, received = self.get_clock().now().nanoseconds*1e-9, time.monotonic()
        s = self.profile['safety']
        velocities = []
        self.velocity_details = {}
        for n in self.names:
            try:
                history = self.mocap_history[n]
                history.latest(source_now, received, s['mocap_maximum_age_s'])
                velocity, details = history.velocity(s['mocap_velocity_window_s'], s['mocap_maximum_age_s'])
                velocities.append(velocity)
                details['source_stamp_s'] = history.rows[-1][0]
                details['receipt_monotonic_s'] = history.rows[-1][1]
                self.velocity_details[n] = details
            except RuntimeError as exc:
                self.guard_fault(f'{n}: timestamped velocity unavailable: {exc}', name=n)
        return np.array([coordinates(self.mocap[n].pose) for n in self.names]), np.array(velocities)

    def health(self, ground=False, airborne=False):
        cfg, now = self.profile, time.monotonic()
        s = cfg['safety']
        if self.measurement_fault:
            self.guard_fault(self.measurement_fault)
        alignment = {}
        source_now = self.get_clock().now().nanoseconds*1e-9
        if self.count_publishers('/poses') != 1: raise RuntimeError('Require one /poses publisher')
        if self.mocap_frame_error: raise RuntimeError(self.mocap_frame_error)
        # A briefly absent body is allowed only while its last VALID sample is
        # within the same bounded age budget. Duplicates remain an immediate fault.
        if any(self.names_seen.count(n) > 1 for n in self.names): raise RuntimeError('Duplicate rigid-body names')
        for i, name in enumerate(self.names):
            mc, est = self.mocap[name], self.onboard[name]
            stats, es = mc.stats(), est.stats()
            if mc.pose is None or mc.frame_id != cfg['frame_id'] or stats['age_s'] > s['mocap_maximum_age_s']:
                _pose_receiver_fix.pose_fault(self, name, f'{name}: mocap age exceeds {s["mocap_maximum_age_s"]:.3f}s')
            if stats['coverage_s'] < 1 or stats['rate_hz'] < s['minimum_mocap_rate_hz']:
                _pose_receiver_fix.pose_fault(self, name, f'{name}: insufficient mocap rate/history')
            if stats['maximum_gap_s'] > s['mocap_maximum_age_s']:
                _pose_receiver_fix.pose_fault(self, name, f'{name}: mocap reception gap exceeds budget')
            if (est.pose is None or est.frame_id != cfg['frame_id'] or es['age_s'] > s['maximum_estimator_age_s']
                    or es['rate_hz'] < s['minimum_estimator_rate_hz']):
                _pose_receiver_fix.pose_fault(self, name, f'{name}: missing/stale onboard estimate')
            try:
                erow = self.estimator_history[name].latest(source_now, now, s['maximum_estimator_age_s'])
                mhistory = self.mocap_history[name]
                mhistory.latest(source_now, now, s['mocap_maximum_age_s'])
                mp, mq, skew, method = mhistory.at(erow[0], .04, s['mocap_maximum_age_s'])
                angle = rotation_error(mq, erow[3])
                raw_angle = rotation_error(mhistory.rows[-1][3], erow[3])
                position_error = float(np.linalg.norm(mp-erow[2]))
            except RuntimeError as exc:
                self.guard_fault(f'{name}: pose alignment invalid: {exc}', name=name)
            alignment[name] = dict(orientation_error_rad=angle,
                raw_latest_orientation_error_rad=raw_angle,
                position_error_m=position_error, matching=method, skew_s=skew,
                onboard_header_s=erow[0], mocap_latest_header_s=mhistory.rows[-1][0],
                mocap_quaternion_xyzw=mq.tolist(), onboard_quaternion_xyzw=erow[3].tolist())
            if position_error > s['maximum_estimator_position_error_m']:
                self.guard_fault(f'{name}: estimator/mocap position disagreement',
                    name=name, limit_m=s['maximum_estimator_position_error_m'], **alignment[name])
            if angle > s['maximum_estimator_attitude_error_rad']:
                self.guard_fault(f'{name}: estimator/mocap orientation disagreement',
                    name=name, limit_rad=s['maximum_estimator_attitude_error_rad'], **alignment[name])
            if tilt(mc.pose) > s['measured_maximum_tilt_rad']: raise RuntimeError(f'{name}: excessive tilt')
            voltage = self.batteries.get(name)
            if voltage is None or now-voltage[0] > cfg['battery']['status_maximum_age_s']:
                raise RuntimeError(f'{name}: missing/stale battery Status telemetry')
            voltage_limit = cfg['battery']['minimum_ground_voltage_v'] if ground else cfg['battery']['land_voltage_v']
            if voltage[1] < voltage_limit: raise RuntimeError(f'{name}: battery {voltage[1]:.2f}V below {voltage_limit:.2f}V')
            if ground:
                p = coordinates(mc.pose)
                if math.dist(p[:2], slots(cfg)[i, :2]) > cfg['mission']['initial_xy_tolerance_m']:
                    raise RuntimeError(f'{name}: place on its compact-formation floor slot and recapture')
                if math.dist(p, self.config['_starts'][name]) > .08:
                    raise RuntimeError(f'{name}: moved from captured start')
                if not cfg['arena']['floor_z_m']-.03 <= p[2] <= cfg['arena']['floor_z_m']+.10:
                    raise RuntimeError(f'{name}: ground start required')
                if max(stats['span_m']) > .03: raise RuntimeError(f'{name}: moving on floor')
                x, y, z, w = quaternion(mc.pose)
                yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
                if abs(yaw) > cfg['mission']['initial_heading_tolerance_rad']:
                    raise RuntimeError(f'{name}: body +X must face mocap world +X')
        p, velocity = self.snapshot()
        if not np.isfinite(p).all() or not np.isfinite(velocity).all():
            self.guard_fault('Nonfinite physical position/velocity')
        speeds = np.linalg.norm(velocity, axis=1)
        minimum = min(np.linalg.norm(p[i, :2]-p[j, :2]) for i in range(6) for j in range(i))
        self.latest_measurement = dict(stage=self.stage_name, monotonic_s=now,
            source_now_s=source_now, names=list(self.names), position=p.tolist(),
            velocity=velocity.tolist(), speed_mps=speeds.tolist(), minimum_pair_m=float(minimum),
            speed_limit_mps=float(s['measured_maximum_speed_mps']),
            orientation_limit_rad=float(s['maximum_estimator_attitude_error_rad']),
            orientation=alignment, velocity_fit=self.velocity_details)
        if now-self.last_health_record >= .1:
            self.health_records.append(self.latest_measurement)
            self.last_health_record = now
        if minimum < s['measured_abort_xy_separation_m']:
            raise RuntimeError(f'Measured pair distance {minimum:.3f}m below abort separation')
        if ground and minimum < pair_distance(cfg):
            raise RuntimeError('Ground formation lacks the full virtual tracking-tube separation')
        if airborne:
            if self.watchdog_seen is None or now-self.watchdog_seen > 1.5:
                raise RuntimeError('Independent watchdog heartbeat is stale')
            if np.max(speeds) > s['measured_maximum_speed_mps']:
                i = int(np.argmax(speeds))
                self.guard_fault('Measured speed exceeds commissioning limit', name=self.names[i],
                    speed_mps=float(speeds[i]), limit_mps=float(s['measured_maximum_speed_mps']),
                    velocity_xyz_mps=velocity[i].tolist(),
                    fit=self.velocity_details[self.names[i]])
            if self.expected is not None and np.max(np.linalg.norm(p-self.expected, axis=1)) > s['tracking_tube_m']:
                raise RuntimeError('Measured drone left its reference tracking tube')
        a = cfg['arena']
        margin = a['body_radius_m']+a['wall_surface_clearance_m']
        if np.any(p[:, 0] < a['x_limits'][0]+margin) or np.any(p[:, 0] > a['x_limits'][1]-margin) or \
           np.any(p[:, 1] < a['y_limits'][0]+margin) or np.any(p[:, 1] > a['y_limits'][1]-margin):
            raise RuntimeError('Measured wall clearance violated')
        if np.any(p[:, 2] < a['floor_z_m']-.03) or np.any(p[:, 2] > a['ceiling_z_m']-a['ceiling_clearance_m']):
            raise RuntimeError('Measured floor/ceiling boundary violated')
        for obstacle in cfg['obstacles']:
            clearance = np.linalg.norm(p[:, :2]-obstacle['center_xy'], axis=1)-obstacle['radius_m']-a['body_radius_m']
            if np.min(clearance) < obstacle['surface_clearance_m']:
                raise RuntimeError('Measured obstacle-surface clearance violated')
        return p, velocity, float(minimum)

    def cycle(self, stage, q, v, previous, guard=True):
        self.sequence += 1
        self.command_results = {}
        frame = {'session': self.session, 'seq': self.sequence, 'config_hash': fingerprint(self.profile),
                 'stage': stage, 'q': q.tolist(), 'v': v.tolist(), 'previous': previous.tolist(),
                 'voltage': [self.batteries[n][1] for n in self.names],
                 'flight_active': self.flight_active, 'streaming': self.streaming}
        self.frame_publisher.publish(String(data=encode(frame)))
        deadline = time.monotonic()+self.profile['control']['cycle_deadline_s']
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.001)
            if guard: self.health(airborne=True)
            for n, result in self.command_results.items():
                if result.get('error'): raise RuntimeError(n+': '+result['error'])
            if set(self.command_results) == set(self.names):
                values = [self.command_results[n] for n in self.names]
                u = np.asarray([r['u'] for r in values], float)
                if u.shape != (6, 3) or not np.isfinite(u).all(): raise RuntimeError('Invalid acceleration batch')
                return values, u
        raise RuntimeError(f'Six-agent cycle {self.sequence} deadline missed; no partial command batch applied')

    def call_many(self, label, requests, timeout=2.0):
        pending, failures = {}, {}
        for name, req in requests.items():
            client = self.flight_service_clients[name][label]
            try:
                if not client.service_is_ready(): raise RuntimeError('service unavailable')
                pending[name] = client.call_async(req)
            except Exception as exc: failures[name] = str(exc)
        deadline = time.monotonic()+timeout
        while pending and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.01)
            for name, future in list(pending.items()):
                if future.done():
                    try:
                        result = future.result()
                        if result is None or (hasattr(result, 'success') and not result.success):
                            raise RuntimeError('service failed')
                    except Exception as exc: failures[name] = str(exc)
                    del pending[name]
        failures.update({n: 'service timeout; physical command outcome uncertain' for n in pending})
        if failures: raise RuntimeError(label+': '+str(failures))

    def publish_setpoints(self, q, v, u):
        started = time.monotonic()
        for i, name in enumerate(self.names):
            msg = FullState()
            msg.header.frame_id = self.profile['frame_id']
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, q[i])
            msg.pose.orientation.w = 1.0  # world yaw zero
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v[i])
            msg.acc.x, msg.acc.y, msg.acc.z = map(float, u[i])
            self.fullstate_publishers[name].publish(msg)
        return time.monotonic()-started

    def release_stream(self):
        self.streaming = False  # All stream publication has already stopped.
        requests = {}
        for n in self.names:
            req = NotifySetpointsStop.Request()
            req.group_mask, req.remain_valid_millisecs = 0, 100
            requests[n] = req
        self.call_many('notify_setpoints_stop', requests)
        self.spin_for(.20)

    def land(self):
        report = {}
        if self.streaming:
            try: self.release_stream()
            except Exception as exc:
                report['handover_error'] = str(exc)
                # A lost notification can leave the low-level priority active.
                # A pause longer than its normal priority validity precedes retry.
                self.spin_for(1.0)
        requests = {}
        for n in self.names:
            req = Land.Request()
            req.group_mask = 0
            req.height = float(self.config['_starts'][n][2]+self.profile['mission']['landing_offset_m'])
            req.duration = Duration(seconds=self.profile['mission']['landing_duration_s']).to_msg()
            requests[n] = req
        try: self.call_many('land', requests)
        except Exception as exc: report['land_error'] = str(exc)
        self.spin_for(self.profile['mission']['landing_duration_s']+2.0)
        # All six must have current evidence at the end, rather than keeping
        # an old successful sample from an aircraft that subsequently moved.
        self.stage_name = 'LANDING_VERIFY'
        deadline, verified, evidence = time.monotonic()+5., [], {}
        while time.monotonic() < deadline:
            self.spin_for(.05)
            now, verified, evidence = time.monotonic(), [], {}
            for n in self.names:
                item = touchdown_evidence(self.mocap[n], self.config['_starts'][n][2], now,
                                          self.profile['safety']['mocap_maximum_age_s'])
                try:
                    self.mocap_history[n].latest(self.get_clock().now().nanoseconds*1e-9, now,
                                                self.profile['safety']['mocap_maximum_age_s'])
                except RuntimeError as exc:
                    item.update(verified=False, reason=str(exc))
                evidence[n] = item
                if item['verified']: verified.append(n)
            if len(verified) == len(self.names): break
        report['touchdown_evidence'] = evidence
        report['touchdown_verified'] = verified
        report['touchdown_not_verified'] = [n for n in self.names if n not in verified]
        requests = {}
        for n in verified:
            req = Arm.Request(); req.arm = False; requests[n] = req
        try:
            if requests: self.call_many('arm', requests)
            report['disarm_requests_acknowledged'] = verified
        except Exception as exc: report['disarm_error'] = str(exc)
        return report

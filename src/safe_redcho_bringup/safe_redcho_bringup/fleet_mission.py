"""Six-Crazyflie commissioning with serial buffered swaps and monitored recovery.

Uses stock high-level services, not decentralized REDCHO/HOCBF or direct motors.
"""
import argparse
import csv
from datetime import datetime, timezone
import itertools
import json
import math
from pathlib import Path
import signal
import time
import rclpy
from rclpy.duration import Duration
from rclpy.signals import SignalHandlerOptions
from crazyflie_interfaces.srv import Arm, Takeoff, Land, GoTo
from rcl_interfaces.srv import GetParameters
from .backend_audit import audit_workspace
from .onboard_avoidance import firmware_parameters
from .fleet_configuration import load_fleet
from .fleet_receiver import FleetReceiver
from .pose_metrics import coordinates, distance, orientation_error, quaternion, tilt
from .swap_planner import build_plan, check_segment, point_segment_distance


# SAFE_REDCHO_HARDWARE_SETTLE_V1
from .hardware_settling import configure_runtime, command_yaw, wait_for_settle


class FleetMission(FleetReceiver):
    def __init__(self, config, output):
        super().__init__(config, 'safe_redcho_fleet_mission')
        self.m = config['mission']
        self.output = Path(output).expanduser()
        self.output.mkdir(parents=True, exist_ok=True)
        self.flight_service_clients = {name: {label: self.create_client(cls, f'/{name}/{label}')
                               for label, cls in [('arm', Arm), ('takeoff', Takeoff),
                                                  ('land', Land), ('go_to', GoTo)]}
                        for name in self.names}
        self.expected = {}
        self.active = {}
        self.airborne_possible = set()
        self.arm_attempted = False
        self.flight_start = None
        self.phase = 'PREFLIGHT'
        self.last_log = 0.
        self.last_row = 0.
        self.plan = None
        self.minimum_measured_xy_separation = math.inf
        self.recovery = {}
        self.avoidance_configuration = {}
        self.csv_handle = (self.output/'poses.csv').open('w', newline='')
        self.writer = csv.writer(self.csv_handle)
        self.writer.writerow(['elapsed_s', 'phase', 'robot', 'x', 'y', 'z', 'mocap_age_s', 'estimator_age_s'])

    def check_health(self, stationary=False):
        m, frame = self.m, self.config['_lab']['mocap']['frame_id']
        if self.count_publishers('/poses') != 1:
            raise RuntimeError('Require exactly one /poses publisher')
        if any(self.names_seen.count(n) != 1 for n in self.names):
            raise RuntimeError(f'Missing or duplicate rigid bodies; last frame={self.names_seen}')
        for name in self.names:
            mc, est = self.mocap[name], self.onboard[name]
            stats, es = mc.stats(), est.stats()
            if mc.frame_id != frame or est.frame_id != frame:
                raise RuntimeError(f'{name}: missing pose or inconsistent world frame')
            if stats['coverage_s'] < 1.0 or stats['rate_hz'] < m['minimum_mocap_rate_hz']:
                raise RuntimeError(f'{name}: insufficient mocap rate/history')
            if stats['maximum_gap_s'] > m['maximum_receive_gap_s']:
                raise RuntimeError(f'{name}: stale/missing mocap samples')
            if es['age_s'] > m['maximum_estimator_age_s'] or es['rate_hz'] < m['minimum_estimator_rate_hz']:
                raise RuntimeError(f'{name}: stale/low-rate onboard estimate')
            if distance(coordinates(mc.pose), coordinates(est.pose)) > m['maximum_estimator_position_error_m']:
                raise RuntimeError(f'{name}: onboard/mocap position mismatch')
            if orientation_error(mc.pose, est.pose) > m['maximum_estimator_attitude_error_rad']:
                raise RuntimeError(f'{name}: onboard/mocap orientation mismatch')
            if stationary:
                if max(stats['span_m']) > .03 or tilt(mc.pose) > m['maximum_initial_tilt_rad']:
                    raise RuntimeError(f'{name}: place aircraft stationary and level on floor')
                if distance(coordinates(mc.pose), self.config['_starts'][name]) > m['maximum_initial_position_error_m']:
                    raise RuntimeError(f'{name}: differs from captured start; recapture correct physical IDs')

    def guard(self):
        self.check_health()
        now = time.monotonic()
        if self.flight_start is not None and now-self.flight_start > self.m['maximum_flight_duration_s']:
            raise RuntimeError('Maximum flight duration exceeded')
        positions = {n: coordinates(self.mocap[n].pose) for n in self.names}
        min_pair = min(math.dist(positions[a][:2], positions[b][:2])
                       for a, b in itertools.combinations(self.names, 2))
        self.minimum_measured_xy_separation = min(self.minimum_measured_xy_separation, min_pair)
        if min_pair < self.m['abort_separation_m']:
            raise RuntimeError(f'Measured XY separation {min_pair:.3f} m below abort threshold')
        arena = self.config['arena']
        body = arena['body_radius_m']
        for name, p in positions.items():
            if name in self.active:
                start, end = self.active[name]
                error = point_segment_distance(p, start, end)
            else:
                error = distance(p, self.expected[name])
            if error > self.m['tracking_tolerance_m']:
                raise RuntimeError(f'{name}: left planned tracking corridor, error={error:.3f} m')
            for index, key in ((0, 'x_limits'), (1, 'y_limits')):
                margin = arena['wall_clearance_m']+body
                if not arena[key][0]+margin <= p[index] <= arena[key][1]-margin:
                    raise RuntimeError(f'{name}: wall boundary exceeded')
            if not arena['floor_z_m']-.03 <= p[2] <= arena['ceiling_z_m']-arena['ceiling_clearance_m']:
                raise RuntimeError(f'{name}: floor/ceiling boundary exceeded')
            for obstacle in self.config['obstacles']:
                clearance = math.dist(p[:2], obstacle['center_xy'])-obstacle['radius_m']-body
                if clearance < self.m['obstacle_surface_clearance_m']:
                    raise RuntimeError(f'{name}: obstacle-surface clearance exceeded')
        if now-self.last_row >= .04:
            elapsed = 0. if self.flight_start is None else now-self.flight_start
            for name in self.names:
                self.writer.writerow([elapsed, self.phase, name, *positions[name],
                                      self.mocap[name].stats()['age_s'], self.onboard[name].stats()['age_s']])
            self.csv_handle.flush()
            self.last_row = now
        if now-self.last_log >= 1:
            self.get_logger().info(f'{self.phase} | minimum XY separation={min_pair:.3f} m')
            self.last_log = now

    def wait_ready(self):
        deadline, good_since = time.monotonic()+35., None
        last_notice = 0.
        while time.monotonic() < deadline:
            self.spin_for(.02)
            try:
                self.check_health(stationary=True)
                if not all(c.service_is_ready() for group in self.flight_service_clients.values() for c in group.values()):
                    raise RuntimeError('Waiting for all six arm/takeoff/land/go_to services')
                good_since = time.monotonic() if good_since is None else good_since
                if time.monotonic()-good_since > 3:
                    return
            except RuntimeError as exc:
                good_since = None
                if time.monotonic()-last_notice > 1:
                    self.get_logger().info(str(exc))
                    last_notice = time.monotonic()
        raise RuntimeError('Six-aircraft preflight timed out; no flight commands sent')

    def verify_avoidance(self):
        from .firmware_preflight import verify_avoidance
        return verify_avoidance(self)

    def call_many(self, action, requests, monitor=True, timeout=2.0):
        # Send all requests first; one failure must not prevent requests to other aircraft.
        pending, errors = {}, {}
        for name, request in requests.items():
            try:
                client = self.flight_service_clients[name][action]
                if not client.service_is_ready():
                    raise RuntimeError('service not ready')
                pending[name] = client.call_async(request)
            except Exception as exc:
                errors[name] = str(exc)
        deadline = time.monotonic()+timeout
        while pending and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.01)
            if monitor:
                self.guard()
            for name, future in list(pending.items()):
                if future.done():
                    try:
                        result = future.result()
                        if result is None or (hasattr(result, 'success') and not result.success):
                            raise RuntimeError('service returned failure')
                    except Exception as exc:
                        errors[name] = str(exc)
                    del pending[name]
        errors.update({n: 'service timeout; physical command outcome uncertain' for n in pending})
        if errors:
            raise RuntimeError(f'{action} request errors: {errors}')

    def arm_requests(self, arm):
        requests = {}
        for name in self.names:
            req = Arm.Request()
            req.arm = arm
            requests[name] = req
        return requests

    def settle(self, targets, duration=None):
        return wait_for_settle(self, targets, duration)

    def recover(self):
        self.phase = 'LANDING'
        if not self.airborne_possible:
            if self.arm_attempted:
                try:
                    self.call_many('arm', self.arm_requests(False), monitor=False)
                except Exception as exc:
                    self.recovery['disarm_error'] = str(exc)
            return
        requests = {}
        for name in self.names:
            req = Land.Request()
            req.group_mask = 0
            req.height = float(self.plan['ground_starts'][name][2]+self.m['landing_offset_m'])
            req.duration = Duration(seconds=float(self.m['landing_duration_s'])).to_msg()
            requests[name] = req
        try:
            self.call_many('land', requests, monitor=False)
        except Exception as exc:
            self.recovery['landing_request_error'] = str(exc)
        self.spin_for(self.m['landing_duration_s']+2.)
        verified = []
        for name in self.names:
            stream = self.mocap[name]
            if stream.pose and stream.stats()['age_s'] <= self.m['maximum_receive_gap_s']:
                p = coordinates(stream.pose)
                start_z = self.plan['ground_starts'][name][2]
                if abs(p[2]-start_z) < .06 and max(stream.stats(window=.5)['span_m']) < .02:
                    verified.append(name)
        self.recovery['touchdown_verified'] = verified
        self.recovery['touchdown_not_verified'] = [n for n in self.names if n not in verified]
        if verified:
            try:
                self.call_many('arm', {n: q for n, q in self.arm_requests(False).items() if n in verified},
                               monitor=False)
                self.recovery['disarm_requests_acknowledged'] = verified
            except Exception as exc:
                self.recovery['disarm_error'] = str(exc)
        if self.recovery['touchdown_not_verified']:
            self.get_logger().error('Touchdown not verified for all aircraft; use independent operator recovery')

    def run(self, mode, check_only=False):
        status, error = 'FAIL', None
        try:
            self.wait_ready()
            self.verify_avoidance()
            configure_runtime(self)
            actual = {n: list(coordinates(self.mocap[n].pose)) for n in self.names}
            self.plan = build_plan(self.config, actual, mode)
            (self.output/'plan.json').write_text(json.dumps(self.plan, indent=2)+'\n')
            self.expected = actual
            print(json.dumps({k: v for k, v in self.plan.items() if k != 'legs'}, indent=2))
            if check_only:
                status = 'CHECK_ONLY_PASS'
                return
            self.arm_attempted = True
            self.call_many('arm', self.arm_requests(True), monitor=False)
            self.check_health(stationary=True)
            requests = {}
            self.phase = 'TAKEOFF'
            for name in self.names:
                req = Takeoff.Request()
                req.group_mask = 0
                req.height = float(self.plan['hover_height_world_m'])
                req.duration = Duration(seconds=float(self.m['takeoff_duration_s'])).to_msg()
                requests[name] = req
                self.active[name] = (actual[name], self.plan['initial_hover_positions'][name])
            self.flight_start = time.monotonic()
            self.airborne_possible = set(self.names)  # Set before sending uncertain commands.
            self.call_many('takeoff', requests)
            self.spin_for(self.m['takeoff_duration_s'], self.guard)
            self.expected = self.plan['initial_hover_positions'].copy()
            self.settle(self.expected)
            self.active.clear()
            self.phase = 'HOVER'
            self.spin_for(self.m['hover_duration_s'], self.guard)
            yaw = {}
            for name in self.names:
                x, y, z, w = quaternion(self.mocap[name].pose)
                yaw[name] = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
            for index, leg in enumerate(self.plan['legs']):
                name, goal = leg['robot'], leg['goal']
                self.settle(self.expected)
                actual = {n: coordinates(self.mocap[n].pose) for n in self.names}
                check_segment(actual[name], goal, {n: p for n, p in actual.items() if n != name}, self.config)
                self.active = {name: (actual[name], goal)}
                self.phase = f'SWAP_{index+1}_{name}'
                req = GoTo.Request()
                req.group_mask, req.relative = 0, False
                req.goal.x, req.goal.y, req.goal.z = (float(v) for v in goal)
                req.yaw = command_yaw(self, name, yaw[name])
                req.duration = Duration(seconds=float(leg['duration_s'])).to_msg()
                self.call_many('go_to', {name: req})
                self.spin_for(leg['duration_s'], self.guard)
                self.expected[name] = list(goal)
                self.settle(self.expected)
                self.active.clear()
            self.phase = 'FINAL_HOVER'
            self.spin_for(self.m['hover_duration_s'], self.guard)
            status = 'PASS'
        except BaseException as exc:
            error = f'{type(exc).__name__}: {exc}'
            self.get_logger().error(error)
        finally:
            # Preserve the ROS context/server during bounded landing cleanup.
            old = {s: signal.signal(s, signal.SIG_IGN) for s in (signal.SIGINT, signal.SIGTERM)}
            try:
                self.recover()
            except BaseException as exc:
                self.recovery['cleanup_error'] = str(exc)
            finally:
                for s, handler in old.items():
                    signal.signal(s, handler)
            if self.airborne_possible and (self.recovery.get('touchdown_not_verified') or
                    self.recovery.get('landing_request_error') or self.recovery.get('disarm_error') or
                    self.recovery.get('cleanup_error')):
                status = 'FAIL'
            report = {'status': status, 'mode': mode, 'error': error,
                      'min_measured_xy_separation_m': (self.minimum_measured_xy_separation
                          if math.isfinite(self.minimum_measured_xy_separation) else None),
                      'recovery': self.recovery, 'scope': 'PID + onboard BVCA commissioning; no REDCHO/HOCBF',
                      'avoidance_configuration': self.avoidance_configuration,
                      'aircraft': self.config['robots']}
            (self.output/'mission_status.json').write_text(json.dumps(report, indent=2)+'\n')
            self.csv_handle.close()
            print(json.dumps(report, indent=2))
            self.success = status in ('PASS', 'CHECK_ONLY_PASS')


def interrupted(signum, frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description='Six-aircraft monitored hover or serial cyclic position swap')
    parser.add_argument('--config')
    parser.add_argument('--mode', choices=('hover', 'swap'), default='hover')
    parser.add_argument('--check-only', action='store_true')
    parser.add_argument('--output')
    args = parser.parse_args()
    cfg = load_fleet(args.config, require_starts=True, require_arena=True)
    out = args.output or (Path(cfg['_path']).parent.parent/'results'/
                         ('six_'+args.mode+'_'+datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')))
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    old = {s: signal.signal(s, interrupted) for s in (signal.SIGINT, signal.SIGTERM)}
    node = FleetMission(cfg, out)
    try:
        node.run(args.mode, args.check_only)
        if not node.success:
            raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for s, handler in old.items():
            signal.signal(s, handler)

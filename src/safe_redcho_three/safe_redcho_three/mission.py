"""Ground preflight, streamed hover or one-metre REDCHO mission, and recovery."""
import argparse
import json
from pathlib import Path
import signal
import time
import uuid
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.signals import SignalHandlerOptions
from crazyflie_interfaces.srv import Arm, Takeoff
from .transport import FlightTransport
from .settings import load, slots, virtual_box
from .protocol import fingerprint
from .cbf_qp import joint_pair_margin, validate_batch
from .telemetry_trace import command_subscribers_valid
from .pose_receiver_fix import validate_forward_budget


class Mission(FlightTransport):
    def __init__(self, cfg, output):
        super().__init__(cfg, output)
        from .commissioning import setup_commissioning
        setup_commissioning(self)
        self.arm_attempted, self.airborne_possible = False, False
        self.rows = []
        self.minimum_measured_pair, self.maximum_tracking, self.maximum_cycle = np.inf, 0., 0.
        self.result = {'active_names': list(self.names), 'command_interface': 'position', 'status': 'FAIL', 'scope': 'Offboard dual REDCHO + local HOCBF-QP reference governor; onboard PID + BVCA'}

    def wait_preflight(self):
        deadline, good_since, last_log = time.monotonic()+45., None, 0.
        while time.monotonic() < deadline:
            self.spin_for(.02)
            now = time.monotonic()
            try:
                self.health(ground=True)
                from .commissioning import check_command_routes
                check_command_routes(self)
                fresh = {n: [(pid, h) for (name, pid), (stamp, h) in self.readiness.items()
                             if name == n and now-stamp < 2.] for n in self.names}
                if any(len(values) != 1 or values[0][1] != fingerprint(self.profile) for values in fresh.values()):
                    raise RuntimeError('Require three unique fresh agent processes with matching configuration')
                if len({values[0][0] for values in fresh.values()}) != len(self.names):
                    raise RuntimeError('Agent process IDs must be distinct')
                if self.watchdog_seen is None or now-self.watchdog_seen > 2.:
                    raise RuntimeError('Independent watchdog is not ready')
                if not all(c.service_is_ready() for group in self.flight_service_clients.values() for c in group.values()):
                    raise RuntimeError('Arm/takeoff/land/notify_setpoints_stop services must all be available')
                if not command_subscribers_valid(self, self.names):
                    raise RuntimeError('Require exactly one crazyflie_server subscriber and at most one known read-only telemetry recorder per cmd_position topic')
                if any(self.count_publishers(f'/{n}/cmd_position') != 1 for n in self.names):
                    raise RuntimeError('Another flight-command publisher is active')
                if self.count_publishers('/safe_redcho/frame') != 1:
                    raise RuntimeError('Require one mission coordinator')
                good_since = now if good_since is None else good_since
                if now-good_since >= 3:
                    self.result['agent_process_ids'] = {n: values[0][0] for n, values in fresh.items()}
                    return
            except RuntimeError as exc:
                good_since = None
                if now-last_log >= 1:
                    self.get_logger().info(str(exc)); last_log = now
        raise RuntimeError('Ground preflight timed out; no arm/takeoff commands sent')

    def reset_session(self):
        self.session, self.sequence = str(uuid.uuid4()), 0

    def shadow_check(self):
        self.reset_session()
        q, v, previous = slots(self.profile), np.zeros((len(self.names), 3)), np.zeros((len(self.names), 3))
        timings = []
        for _ in range(60):
            start = time.monotonic()
            self.health(ground=True)
            _, u = self.cycle('ALIGN', q, v, previous, guard=False)
            dt = self.profile['control']['dt_s']
            q, v = validate_batch(q, v, previous, u, self.profile)
            previous = u
            timings.append(time.monotonic()-start)
            self.spin_for(max(0., dt-(time.monotonic()-start)))
        self.result['shadow_cycles'] = 60
        self.result['shadow_p99_cycle_s'] = float(np.quantile(timings, .99))
        self.result['shadow_median_cycle_s'] = float(np.median(timings))
        dt = self.profile['control']['dt_s']
        if (max(timings) >= self.profile['control']['cycle_deadline_s']
            or np.median(timings) > dt or np.quantile(timings, .99) > 1.5*dt):
            raise RuntimeError('Live three-agent transport/QP timing exceeds configured deadline')

    def takeoff(self):
        from .commissioning import prepare_handover
        prepare_handover(self)  # Invalid reference geometry must fail BEFORE arming.
        self.stage_name = 'TAKEOFF'
        self.arm_attempted = True
        requests = {}
        for n in self.names:
            req = Arm.Request(); req.arm = True; requests[n] = req
        self.call_many('arm', requests)
        self.health(ground=True)
        requests = {}
        for i, n in enumerate(self.names):
            req = Takeoff.Request()
            req.group_mask, req.height = 0, float(slots(self.profile)[i, 2])
            req.duration = Duration(seconds=self.profile['mission']['takeoff_duration_s']).to_msg()
            requests[n] = req
        self.flight_active, self.airborne_possible = True, True
        self.pulse()
        self.call_many('takeoff', requests)
        self.spin_for(self.profile['mission']['takeoff_duration_s'],
                      lambda: self.health(airborne=True))
        self.stage_name = 'TAKEOFF_SETTLE'
        self.expected = self.takeoff_hover_reference.copy()
        deadline, good_since = time.monotonic()+10., None
        while time.monotonic() < deadline:
            self.spin_for(.02, lambda: self.health(airborne=True))
            p, velocity, _ = self.health(airborne=True)
            good = (np.max(np.linalg.norm(p-self.takeoff_hover_reference, axis=1)) < .10
                    and np.max(np.linalg.norm(velocity, axis=1)) < .10)
            if good:
                good_since = time.monotonic() if good_since is None else good_since
                if time.monotonic()-good_since >= .5: return
            else: good_since = None
        raise RuntimeError('Takeoff did not settle around its planned hover reference')

    def stream_mission(self, hover_only=False):
        cfg, dt = self.profile, self.profile['control']['dt_s']
        q, v, previous = slots(cfg), np.zeros((len(self.names), 3)), np.zeros((len(self.names), 3))
        end = slots(cfg)+[cfg['mission']['distance_m'], 0., 0.]
        self.reset_session()
        self.expected, self.streaming = q.copy(), True
        self.pulse()
        stage, logical, last_notice = 'ALIGN', 0., -np.inf
        final_since, started, last_publish = None, time.monotonic(), None
        while time.monotonic()-started < cfg['mission']['maximum_wall_duration_s']:
            self.stage_name = stage
            cycle_start = time.monotonic()
            p, measured_v, minimum = self.health(airborne=True)
            self.minimum_measured_pair = min(self.minimum_measured_pair, minimum)
            self.maximum_tracking = max(self.maximum_tracking, float(np.max(np.linalg.norm(p-q, axis=1))))
            results, u = self.cycle(stage, q, v, previous)
            if joint_pair_margin(q, v, u, cfg) < -10*cfg['control']['qp_tolerance']:
                raise RuntimeError('Combined reciprocal pair constraints failed validation')
            q_next, v_next = validate_batch(q, v, previous, u, cfg)
            now = time.monotonic()
            if last_publish is not None and now-last_publish > cfg['control']['maximum_publish_gap_s']:
                raise RuntimeError('Full-state publication gap exceeded its budget')
            publication_span = self.publish_setpoints(q_next, v_next, u)
            if publication_span > dt:
                raise RuntimeError('Local publication of three setpoints exceeded one control period')
            last_publish = now
            q, v, previous, self.expected = q_next, v_next, u, q_next.copy()
            logical += dt
            target = np.array([r['target'] for r in results])
            self.rows.append((logical, p.copy(), q.copy(), measured_v.copy(), v.copy(), target,
                              np.array([r['velocity_estimate'] for r in results]),
                              np.array([r['position_estimate'] for r in results]), u.copy()))
            settled_initial = np.max(np.linalg.norm(p-slots(cfg), axis=1)) < cfg['mission']['endpoint_tolerance_m']
            slow = np.max(np.linalg.norm(measured_v, axis=1)) < cfg['mission']['endpoint_speed_mps']
            if stage == 'ALIGN' and logical >= cfg['mission']['observer_warmup_s'] and settled_initial and slow:
                if hover_only:
                    stage, final_since = 'STOP', now
                else: stage = 'FORWARD'
            if stage == 'ALIGN' and logical > cfg['mission']['observer_warmup_s']+10.:
                raise RuntimeError('Formation alignment did not settle before forward flight')
            if stage == 'FORWARD' and all(r['local_phase'] >= 1 for r in results):
                stage = 'STOP'
            if stage == 'STOP':
                final_target = slots(cfg) if hover_only else end
                good = (np.max(np.linalg.norm(p-final_target, axis=1)) < cfg['mission']['endpoint_tolerance_m']
                        and slow and np.ptp(p[:, 2]) < cfg['mission']['height_spread_tolerance_m'])
                if good:
                    final_since = now if final_since is None else final_since
                    if now-final_since >= cfg['mission']['endpoint_dwell_s']+cfg['mission']['final_hold_s']:
                        self.result['final_measured_endpoint_errors_m'] = np.linalg.norm(p-final_target, axis=1).tolist()
                        self.result['measured_centroid_forward_displacement_m'] = float(np.mean(p[:, 0])-np.mean(slots(cfg)[:, 0]))
                        self.result['final_height_spread_m'] = float(np.ptp(p[:, 2]))
                        return
                else: final_since = None
            cycle_wall = time.monotonic()-cycle_start
            self.maximum_cycle = max(self.maximum_cycle, cycle_wall)
            if now-last_notice >= 1:
                self.get_logger().info(f'{stage} | x advance={np.mean(p[:, 0])-np.mean(slots(cfg)[:, 0]):.3f}m | '
                                       f'min pair={minimum:.3f}m | max tracking={np.max(np.linalg.norm(p-q, axis=1)):.3f}m')
                last_notice = now
            # No burst catch-up: a slow cycle stretches the wall-time mission.
            self.spin_for(max(0., dt-cycle_wall))
        raise RuntimeError('Maximum streamed mission wall duration exceeded')

    def run(self, fly=False, hover_only=False):
        error, recovery = None, {}
        try:
            if fly and not hover_only: validate_forward_budget(self.profile)
            self.wait_preflight()
            from safe_redcho_bringup.firmware_preflight import verify_avoidance
            verify_avoidance(self)
            self.shadow_check()
            if not fly:
                self.result['status'] = 'CHECK_ONLY_PASS'
            else:
                self.takeoff()
                from .commissioning import handover
                handover(self)
                self.stream_mission(hover_only)
                self.result['status'] = 'PASS'
        except BaseException as exc:
            error = f'{type(exc).__name__}: {exc}'
            self.get_logger().error(error)
            self.result['status'] = 'FAIL'
        finally:
            old = {s: signal.signal(s, signal.SIG_IGN) for s in (signal.SIGINT, signal.SIGTERM)}
            try:
                if self.airborne_possible:
                    recovery = self.land()
                    if recovery.get('touchdown_not_verified') or any(k.endswith('error') for k in recovery):
                        self.result['status'] = 'FAIL'
                elif self.arm_attempted:
                    requests = {}
                    for n in self.names:
                        req = Arm.Request(); req.arm = False; requests[n] = req
                    self.call_many('arm', requests)
            except BaseException as exc:
                recovery['cleanup_error'] = f'{type(exc).__name__}: {exc}'
                self.result['status'] = 'FAIL'
            finally:
                for s, previous in old.items(): signal.signal(s, previous)
            # Keep watchdog armed if touchdown for any aircraft remains uncertain.
            self.flight_active = self.airborne_possible and len(recovery.get('touchdown_verified', [])) != len(self.names)
            self.pulse()
            if not self.flight_active:
                # Keep the publisher alive while the watchdog receives the
                # final inactive state, before destroying the ROS context.
                self.spin_for(.15)
            self.result.update(error=error, recovery=recovery, hover_only=hover_only,
                               parameter_preflight=self.avoidance_configuration,
                               minimum_measured_pair_m=float(self.minimum_measured_pair) if np.isfinite(self.minimum_measured_pair) else None,
                               maximum_tracking_error_m=self.maximum_tracking, maximum_cycle_wall_s=self.maximum_cycle)
            (self.output/'measurement_diagnostics.json').write_text(
                json.dumps(self.diagnostics(), indent=2, allow_nan=False)+'\n')
            (self.output/'mission_status.json').write_text(json.dumps(self.result, indent=2, allow_nan=False)+'\n')
            if self.rows:
                keys = ['time', 'measured_position', 'reference_position', 'measured_velocity', 'reference_velocity',
                        'formation_target', 'velocity_redcho', 'pose_redcho', 'safe_acceleration']
                np.savez_compressed(self.output/'flight_trace.npz', **{
                    key: np.array([row[i] for row in self.rows]) for i, key in enumerate(keys)})
            print(json.dumps(self.result, indent=2, allow_nan=False))


def interrupted(signum, frame): raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--fly', action='store_true', help='Send actual arm/takeoff/setpoint/landing commands')
    parser.add_argument('--hover-only', action='store_true', help='Commission the streamed handover without translation')
    args, ros_args = parser.parse_known_args()
    cfg = load(args.config)
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = Mission(cfg, args.output)
    previous = {s: signal.signal(s, interrupted) for s in (signal.SIGINT, signal.SIGTERM)}
    try: node.run(args.fly, args.hover_only)
    finally:
        for s, handler in previous.items(): signal.signal(s, handler)
        node.destroy_node()
        rclpy.try_shutdown()
    if node.result['status'] == 'FAIL': raise RuntimeError('Mission failed; inspect mission_status.json; keep backend running for recovery')

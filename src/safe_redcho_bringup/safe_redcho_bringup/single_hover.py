"""Bounded high-level hover test. No REDCHO or HOCBF is active in this step."""
import argparse
import math
import signal
import time
import rclpy
from rclpy.duration import Duration
from rclpy.signals import SignalHandlerOptions
from crazyflie_interfaces.srv import Arm, Takeoff, Land
from .configuration import load_lab
from .pose_metrics import coordinates, distance, orientation_error, tilt
from .receiver import Receiver


class Hover(Receiver):
    def __init__(self, lab):
        super().__init__(lab, 'safe_redcho_single_hover')
        self.f = lab['flight']
        self.arm_client = self.create_client(Arm, f'/{self.robot}/arm')
        self.takeoff_client = self.create_client(Takeoff, f'/{self.robot}/takeoff')
        self.land_client = self.create_client(Land, f'/{self.robot}/land')
        self.airborne_possible = False
        self.start_position = None

    def call(self, client, request, timeout=3.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'Service unavailable: {client.srv_name}')
        future = client.call_async(request)
        deadline = time.monotonic()+timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            raise RuntimeError(f'No service response: {client.srv_name}; command outcome uncertain')
        result = future.result()
        if result is None or (hasattr(result, 'success') and not result.success):
            raise RuntimeError(f'Service reported a failure: {client.srv_name}')
        # A response means processed by the server; it does not prove aircraft motion.

    def health(self, stationary=False):
        f = self.f
        stats = self.mocap.stats()
        if self.count_publishers('/poses') != 1:
            return 'Expected exactly one /poses publisher'
        if self.mocap.frame_id != self.lab['mocap']['frame_id']:
            return 'Mocap frame does not match configured world frame'
        if stats['coverage_s'] < 1.0 or stats['rate_hz'] < f['minimum_mocap_rate_hz']:
            return f'Insufficient mocap rate/history: {stats["rate_hz"]:.1f} Hz'
        if stats['maximum_gap_s'] > f['maximum_receive_gap_s']:
            return 'Mocap reception gap exceeded limit'
        if self.onboard.stats()['age_s'] > f['maximum_estimator_age_s']:
            return 'Onboard pose log missing or stale'
        if self.onboard.frame_id != self.lab['mocap']['frame_id']:
            return 'Onboard pose frame differs from world frame'
        if distance(coordinates(self.mocap.pose), coordinates(self.onboard.pose)) > f['maximum_estimator_position_error_m']:
            return 'Onboard position and mocap disagree'
        if orientation_error(self.mocap.pose, self.onboard.pose) > f['maximum_estimator_attitude_error_rad']:
            return 'Onboard orientation and Motive body orientation disagree'
        if stationary:
            if max(stats['span_m']) > 0.04:
                return 'Aircraft must be stationary for preflight'
            if tilt(self.mocap.pose) > f['maximum_initial_tilt_rad']:
                return 'Aircraft body axes/tilt invalid for ground takeoff'
            if distance(coordinates(self.mocap.pose), self.lab['robot']['initial_position']) > f['maximum_initial_position_error_m']:
                return 'Measured start differs from robot.initial_position'
        return None

    def preflight(self):
        deadline = time.monotonic()+25.0
        good_since = None
        last_log = 0.0
        while time.monotonic() < deadline:
            self.spin_for(0.02)
            reason = self.health(stationary=True)
            now = time.monotonic()
            if reason:
                good_since = None
            else:
                good_since = now if good_since is None else good_since
                if now-good_since >= 3.0:
                    self.start_position = coordinates(self.mocap.pose)
                    self.get_logger().info(f'PREFLIGHT PASS; world start={self.start_position}')
                    return
            if now-last_log > 1:
                self.get_logger().info(f'Preflight: {reason or "collecting stable history"}')
                last_log = now
        raise RuntimeError('Preflight did not pass within 25 seconds; no takeoff sent')

    def monitor(self, seconds, hover=False):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            self.spin_for(0.02)
            reason = self.health()
            if reason:
                raise RuntimeError(reason)
            p = coordinates(self.mocap.pose)
            if math.hypot(p[0]-self.start_position[0], p[1]-self.start_position[1]) > self.f['maximum_horizontal_excursion_m']:
                raise RuntimeError('Horizontal excursion exceeded first-hover limit')
            maximum = self.start_position[2]+self.f['height_above_start_m']+self.f['maximum_height_overshoot_m']
            if p[2] > maximum or p[2] < self.start_position[2]-0.05:
                raise RuntimeError('Height outside first-hover envelope')
            target = self.start_position[2]+self.f['height_above_start_m']
            if hover and abs(p[2]-target) > self.f['maximum_hover_height_error_m']:
                raise RuntimeError('Hover height error exceeded limit')

    def land(self):
        request = Land.Request()
        request.group_mask = 0
        request.height = float(self.start_position[2]+self.f['landing_offset_m'])
        request.duration = Duration(seconds=float(self.f['landing_duration_s'])).to_msg()
        self.call(self.land_client, request)
        self.get_logger().warning('Landing command sent; waiting for measured touchdown')
        self.spin_for(self.f['landing_duration_s']+1.0)
        # Do not disarm an aircraft that is still airborne or whose pose is unavailable.
        if (self.mocap.stats()['age_s'] > self.f['maximum_receive_gap_s'] or
                abs(coordinates(self.mocap.pose)[2]-self.start_position[2]) > 0.06):
            raise RuntimeError('Touchdown not verified. Use the independent operator recovery procedure')
        request = Arm.Request()
        request.arm = False
        self.call(self.arm_client, request)
        self.airborne_possible = False
        self.get_logger().info('Touchdown verified; disarm command sent')

    def run(self, check_only):
        self.preflight()
        if check_only:
            self.get_logger().info('Check-only complete; no arm/takeoff/land commands sent')
            return
        arm_attempted = False
        try:
            request = Arm.Request()
            request.arm = True
            arm_attempted = True
            self.call(self.arm_client, request)
            self.spin_for(0.3)
            reason = self.health(stationary=True)
            if reason:
                raise RuntimeError(reason)
            request = Takeoff.Request()
            request.group_mask = 0
            request.height = float(self.start_position[2]+self.f['height_above_start_m'])
            request.duration = Duration(seconds=float(self.f['takeoff_duration_s'])).to_msg()
            self.airborne_possible = True
            self.call(self.takeoff_client, request)
            self.monitor(self.f['takeoff_duration_s']+0.75)
            target = self.start_position[2]+self.f['height_above_start_m']
            if abs(coordinates(self.mocap.pose)[2]-target) > self.f['maximum_hover_height_error_m']:
                raise RuntimeError('Takeoff target not reached')
            self.get_logger().info(f'HOVER: target world z={target:.3f} m')
            self.monitor(self.f['hover_duration_s'], hover=True)
        finally:
            if self.airborne_possible:
                # A mocap/radio failure can also prevent this recovery. Never claim
                # that a landing request alone proves a safe landing.
                self.land()
            elif arm_attempted:
                request = Arm.Request()
                request.arm = False
                self.call(self.arm_client, request)


def interrupted(signum, frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description='One-drone high-level takeoff/hover/land')
    parser.add_argument('--config')
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args()
    lab, _ = load_lab(args.config, require_radio=True)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    previous = {s: signal.signal(s, interrupted) for s in (signal.SIGINT, signal.SIGTERM)}
    node = Hover(lab)
    try:
        node.run(args.check_only)
    except KeyboardInterrupt:
        node.get_logger().error('Interrupted; inspect whether the recovery landing was verified')
        raise SystemExit(130)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)

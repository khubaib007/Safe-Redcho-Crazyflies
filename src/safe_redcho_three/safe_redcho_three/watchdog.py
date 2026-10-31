"""Separate-process heartbeat watchdog; never publishes streaming setpoints."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from crazyflie_interfaces.srv import Arm, Land, NotifySetpointsStop
from .fleet_adapter import load_fleet
from safe_redcho_bringup.pose_metrics import valid_pose
from .settings import load
from .protocol import encode, decode, fingerprint


class Watchdog(Node):
    def __init__(self, cfg):
        if not cfg.get('workspace') or not cfg.get('fleet_config_file'):
            raise ValueError('Use the installed hardware profile, not the offline test fixture')
        super().__init__('safe_redcho_watchdog')
        self.cfg, self.names = cfg, cfg['names']
        self.fleet = load_fleet(cfg, require_starts=True, require_arena=True)
        self.last_heartbeat, self.active, self.streaming = None, False, False
        self.triggered, self.stage, self.next_action = False, None, None
        self.poses, self.requests, self.last_ready = {}, {}, -1.
        self.height_history = {n: deque(maxlen=200) for n in self.names}
        self.report = {}
        self.ready_publisher = self.create_publisher(String, '/safe_redcho/watchdog_ready', 4)
        self.service_handles = {n: {key: self.create_client(cls, f'/{n}/{key}') for key, cls in
            [('notify_setpoints_stop', NotifySetpointsStop), ('land', Land), ('arm', Arm)]} for n in self.names}
        self.create_subscription(String, '/safe_redcho/heartbeat', self.heartbeat, 8)
        from rclpy.qos import qos_profile_sensor_data
        self.create_subscription(NamedPoseArray, '/poses', self.on_poses, qos_profile_sensor_data)
        self.create_timer(.025, self.tick)

    def heartbeat(self, msg):
        try:
            value = decode(msg.data)
            if value['config_hash'] != fingerprint(self.cfg): return
            if not self.triggered:
                stamp = float(value['monotonic_stamp'])
                age = time.monotonic()-stamp
                if not math.isfinite(stamp) or not 0 <= age <= self.cfg['safety']['watchdog_timeout_s']:
                    return
                self.active, self.streaming = bool(value['active']), bool(value['streaming'])
                self.last_heartbeat = stamp
        except (KeyError, ValueError): pass

    def on_poses(self, msg):
        if msg.header.frame_id != self.cfg['frame_id']: return
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if abs(self.get_clock().now().nanoseconds-stamp) > 150_000_000: return
        counts = {n: sum(p.name == n for p in msg.poses) for n in self.names}
        for p in msg.poses:
            if p.name in counts and counts[p.name] == 1 and valid_pose(p.pose):
                old = self.poses.get(p.name)
                if old is None or stamp > old[2]:
                    self.poses[p.name] = (time.monotonic(), p.pose.position.z, stamp)
                    self.height_history[p.name].append((time.monotonic(), p.pose.position.z))

    def send(self, action, requests):
        for name, request in requests.items():
            try:
                client = self.service_handles[name][action]
                if not client.service_is_ready(): raise RuntimeError('service unavailable')
                self.requests[(action, name)] = client.call_async(request)
            except Exception as exc:
                self.report.setdefault('request_errors', []).append(f'{name}/{action}: {exc}')

    def tick(self):
        now = time.monotonic()
        if now-self.last_ready >= .5:
            self.ready_publisher.publish(String(data=encode({'config_hash': fingerprint(self.cfg),
                                                            'triggered': self.triggered})))
            self.last_ready = now
        if not self.triggered:
            if not self.active or self.last_heartbeat is None or now-self.last_heartbeat <= self.cfg['safety']['watchdog_timeout_s']:
                return
            self.triggered, self.stage, self.next_action = True, 'NOTIFY', now+.25
            self.report = {'status': 'TRIGGERED', 'reason': 'Mission heartbeat expired'}
            self.get_logger().error('Mission heartbeat expired: watchdog is requesting landing')
            requests = {}
            for n in self.names:
                req = NotifySetpointsStop.Request()
                req.group_mask, req.remain_valid_millisecs = 0, 100
                requests[n] = req
            self.send('notify_setpoints_stop', requests)
            return
        if self.next_action is None or now < self.next_action: return
        if self.stage == 'NOTIFY':
            requests = {}
            for n in self.names:
                req = Land.Request()
                req.group_mask = 0
                req.height = float(self.fleet['_starts'][n][2]+self.cfg['mission']['landing_offset_m'])
                req.duration = Duration(seconds=self.cfg['mission']['landing_duration_s']).to_msg()
                requests[n] = req
            self.send('land', requests)
            self.stage, self.next_action = 'LAND', now+self.cfg['mission']['landing_duration_s']+2.
        elif self.stage == 'LAND':
            verified = []
            for n, (stamp, z, _) in self.poses.items():
                heights = [h for t, h in self.height_history[n] if now-t <= .5]
                if (now-stamp <= self.cfg['safety']['mocap_maximum_age_s']
                    and abs(z-self.fleet['_starts'][n][2]) < .06
                    and len(heights) >= 5 and max(heights)-min(heights) < .02):
                    verified.append(n)
            requests = {}
            for n in verified:
                req = Arm.Request(); req.arm = False; requests[n] = req
            self.send('arm', requests)
            self.report.update(touchdown_height_verified=verified,
                               touchdown_not_verified=[n for n in self.names if n not in verified],
                               scope='Landing/disarm requests only; acknowledgments do not prove RF delivery')
            self.stage, self.next_action = 'DONE', now+2.
        elif self.stage == 'DONE':
            for (action, name), future in self.requests.items():
                if not future.done(): self.report.setdefault('pending_requests', []).append(f'{name}/{action}')
                else:
                    try:
                        if future.result() is None: raise RuntimeError('empty response')
                    except Exception as exc:
                        self.report.setdefault('request_errors', []).append(f'{name}/{action}: {exc}')
            path = Path(self.cfg['workspace'])/'results/redcho_watchdog'
            path.mkdir(parents=True, exist_ok=True)
            (path/'watchdog_report.json').write_text(json.dumps(self.report, indent=2)+'\n')
            self.next_action = None
            self.get_logger().error('Watchdog recovery finished; inspect '+str(path/'watchdog_report.json'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Watchdog(load(args.config))
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

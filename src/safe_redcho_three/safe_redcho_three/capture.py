"""Capture only three grounded active bodies; this node sends no commands."""
import argparse
from datetime import datetime, timezone
from pathlib import Path
import math
import time
import numpy as np
import yaml
import rclpy
from safe_redcho_bringup.fleet_receiver import FleetReceiver
from safe_redcho_bringup.pose_metrics import quaternion, coordinates
from .fleet_adapter import load_fleet
from .settings import load, slots, pair_distance


class Capture(FleetReceiver):
    def __init__(self, cfg):
        self.profile, self.last_stamp = cfg, None
        super().__init__(load_fleet(cfg, require_arena=True), 'safe_redcho_three_capture')

    def receive(self, msg):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        now = self.get_clock().now().nanoseconds
        if msg.header.frame_id != 'world' or not 0 < stamp or not -.05e9 <= now-stamp <= .08e9:
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        super().receive(msg)

    def reason(self):
        if self.count_publishers('/poses') != 1: return 'Require exactly one /poses publisher'
        expected = slots(self.profile)
        for i, name in enumerate(self.names):
            stream = self.mocap[name]
            stats = stream.stats()
            if self.names_seen.count(name) != 1: return name+': missing or duplicate body'
            if stream.pose is None or stream.frame_id != 'world': return name+': missing world pose'
            if stats['coverage_s'] < 1.5 or stats['rate_hz'] < self.profile['safety']['minimum_mocap_rate_hz']:
                return name+': waiting for mocap history'
            if max(stats['maximum_gap_s'], stats['age_s']) > self.profile['safety']['mocap_maximum_age_s']:
                return name+': stale/gapped mocap'
            if max(stats['span_m']) > .03: return name+': keep stationary'
            p = np.array(coordinates(stream.pose))
            if np.linalg.norm(p[:2]-expected[i, :2]) > self.profile['mission']['initial_xy_tolerance_m']:
                return name+': place on the three-aircraft floor slot'
            if abs(p[2]-self.profile['arena']['floor_z_m']) > .08: return name+': require ground pose'
            x,y,z,w = quaternion(stream.pose)
            if not np.isfinite([x,y,z,w]).all() or abs(np.linalg.norm([x,y,z,w])-1) > .05:
                return name+': invalid orientation'
            if abs(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))) > self.profile['mission']['initial_heading_tolerance_rad']:
                return name+': face body +X along world +X'
        p = np.array([coordinates(self.mocap[n].pose) for n in self.names])
        if min(np.linalg.norm(p[i,:2]-p[j,:2]) for i in range(3) for j in range(i)) < pair_distance(self.profile):
            return 'Measured floor separation lacks the complete tracking allowance'
        return None


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    args=parser.parse_args()
    cfg=load(args.config)
    rclpy.init()
    node=Capture(cfg)
    begun=time.monotonic(); good=None; last_log=0.
    try:
        while time.monotonic()-begun < 45.:
            node.spin_for(.02)
            now=time.monotonic(); reason=node.reason()
            good=now if reason is None and good is None else good if reason is None else None
            if now-last_log > 1:
                node.get_logger().info(reason or 'Collecting stationary cf2/cf3/cf4 starts'); last_log=now
            if good is not None and now-good >= 3.: break
        else: raise RuntimeError('Three-aircraft capture timed out; no file written')
        positions={n: np.mean([p for t,p in node.mocap[n].samples if now-t <= 1.], axis=0).tolist() for n in node.names}
        snapshot={'captured_utc':datetime.now(timezone.utc).isoformat(), 'frame_id':'world',
                  'positions':positions, 'scope':'Ground capture of cf2/cf3/cf4 only; no flight commands'}
        out=Path(cfg['three_starts_file']); out.write_text(yaml.safe_dump(snapshot, sort_keys=False))
        print(yaml.safe_dump(snapshot, sort_keys=False)); print('Written:',out)
    finally: node.destroy_node(); rclpy.try_shutdown()

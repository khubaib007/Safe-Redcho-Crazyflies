import argparse
import json
import time
import rclpy
from .configuration import load_lab
from .pose_metrics import coordinates, quaternion
from .receiver import Receiver


def main():
    parser = argparse.ArgumentParser(description='Receive vendor pose; no flight commands')
    parser.add_argument('--config')
    parser.add_argument('--duration', type=float, default=10.0)
    args = parser.parse_args()
    if not 2 <= args.duration <= 300:
        parser.error('--duration must be in [2, 300] seconds')
    lab, _ = load_lab(args.config)
    rclpy.init(args=[])
    node = Receiver(lab, 'safe_redcho_pose_check')
    begin = time.monotonic()
    last_log = begin
    max_gap = 0.0
    try:
        while time.monotonic()-begin < args.duration:
            node.spin_for(0.02)
            if time.monotonic()-last_log >= 1:
                stats = node.mocap.stats()
                if node.mocap.count:
                    max_gap = max(max_gap, stats['maximum_gap_s'])
                node.get_logger().info(
                    f'names={node.names} | {node.robot} position='
                    f'{coordinates(node.mocap.pose) if node.mocap.pose else None} | '
                    f'{stats["rate_hz"]:.1f} Hz | age={stats["age_s"]:.3f}s')
                last_log = time.monotonic()
        stats = node.mocap.stats()
        f = lab['flight']
        passed = (node.count_publishers('/poses') == 1 and
                  node.mocap.frame_id == lab['mocap']['frame_id'] and
                  stats['coverage_s'] >= 1.0 and
                  stats['rate_hz'] >= f['minimum_mocap_rate_hz'] and
                  max(max_gap, stats['maximum_gap_s']) <= f['maximum_receive_gap_s'] and
                  node.mocap.invalid_samples == 0)
        report = {'status': 'PASS' if passed else 'FAIL', 'robot': node.robot,
                  'names_seen_last_frame': node.names, 'frame': node.mocap.frame_id,
                  'pose_publishers': node.count_publishers('/poses'),
                  'valid_samples': node.mocap.count, 'invalid_samples': node.mocap.invalid_samples,
                  'tail_reception_statistics': stats,
                  'max_observed_receive_gap_s': max_gap,
                  'position_m': coordinates(node.mocap.pose) if node.mocap.pose else None,
                  'quaternion_xyzw': quaternion(node.mocap.pose) if node.mocap.pose else None,
                  'scope': 'Transport and basic pose checks; not axis/body-frame or flight certification'}
        print(json.dumps(report, indent=2))
        if not passed:
            raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

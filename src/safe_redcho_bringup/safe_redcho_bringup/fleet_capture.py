import argparse
from datetime import datetime, timezone
from pathlib import Path
import time
import yaml
import rclpy
from .fleet_configuration import load_fleet
from .fleet_receiver import FleetReceiver


def capture_reason(node):
    m = node.config['mission']
    if node.count_publishers('/poses') != 1:
        return 'Expected exactly one /poses publisher'
    for name, stream in node.mocap.items():
        stats = stream.stats()
        if stream.frame_id != node.config['_lab']['mocap']['frame_id']:
            return f'{name}: missing pose or wrong frame; names seen={node.names_seen}'
        if stats['coverage_s'] < 1.5 or stats['rate_hz'] < m['minimum_mocap_rate_hz']:
            return f'{name}: waiting for sufficient mocap rate/history'
        if stats['maximum_gap_s'] > m['maximum_receive_gap_s']:
            return f'{name}: reception gap exceeds limit'
        if max(stats['span_m']) > .03:
            return f'{name}: keep all six aircraft stationary on the ground'
    return None


def main():
    parser = argparse.ArgumentParser(description='Capture six stationary Motive start poses, no flight commands')
    parser.add_argument('--config')
    parser.add_argument('--output')
    args = parser.parse_args()
    cfg = load_fleet(args.config)
    rclpy.init(args=[])
    node = FleetReceiver(cfg, 'safe_redcho_fleet_capture')
    begin = time.monotonic()
    good_since, last_log = None, 0.
    try:
        while time.monotonic()-begin < 35:
            node.spin_for(.02)
            reason = capture_reason(node)
            now = time.monotonic()
            if reason:
                good_since = None
            else:
                good_since = now if good_since is None else good_since
                if now-good_since > 3.0:
                    break
            if now-last_log > 1:
                node.get_logger().info(reason or 'Collecting stable six-aircraft start poses')
                last_log = now
        else:
            raise RuntimeError('Six-body pose capture timed out; no start file was written')
        positions = {}
        for name, stream in node.mocap.items():
            recent = [p for t, p in stream.samples if t >= now-1.0]
            positions[name] = [sum(p[k] for p in recent)/len(recent) for k in range(3)]
        snapshot = {'captured_utc': datetime.now(timezone.utc).isoformat(),
                    'frame_id': cfg['_lab']['mocap']['frame_id'], 'positions': positions,
                    'scope': 'Stationary received poses; identify each physical aircraft by hand before flight'}
        out = Path(args.output or cfg['_starts_path']).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(yaml.safe_dump(snapshot, sort_keys=False))
        print(yaml.safe_dump(snapshot, sort_keys=False))
        print('Written:', out)
    finally:
        node.destroy_node()
        rclpy.shutdown()

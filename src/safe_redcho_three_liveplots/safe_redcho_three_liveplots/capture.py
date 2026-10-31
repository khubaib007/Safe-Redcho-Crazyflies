"""Read-only acquisition. Publishes visualization topics only; no flight clients."""
import argparse
import json
import queue
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import Status
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from .configuration import GROUPS, NAMES, default_config, load, limits
from .engine import Engine, number
from .io import CommandTail, EventWriter


def stamp_ns(msg):
    h = getattr(msg, 'header', None)
    return int(h.stamp.sec)*1_000_000_000+int(h.stamp.nanosec) if h else None


def parse(text):
    if len(text) > 300000: raise ValueError('Oversized JSON')
    def reject(x): raise ValueError('Nonfinite JSON '+x)
    p = json.loads(text, parse_constant=reject)
    if not isinstance(p, dict): raise ValueError('JSON must be an object')
    return p


class Capture(Node):
    def __init__(self, cfg):
        super().__init__('redcho_live_capture')
        self.cfg = cfg
        self.engine = Engine(cfg, self.get_clock().now().nanoseconds)
        self.last_ros = self.get_clock().now().nanoseconds
        self.sequence = {g: 0 for g in GROUPS}
        qos = QoSProfile(depth=32, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.bus_publishers = {g: self.create_publisher(String, '/safe_redcho/live/'+g, qos) for g in GROUPS}
        self.create_subscription(NamedPoseArray, '/poses', self.mocap, qos)
        for n in NAMES:
            self.create_subscription(PoseStamped, '/'+n+'/pose', lambda m,n=n: self.pose(m,n), qos)
            self.create_subscription(Status, '/'+n+'/status', lambda m,n=n: self.battery(m,n), qos)
            self.create_subscription(String, '/safe_redcho/'+n+'/algorithm_telemetry', lambda m,n=n: self.algorithm(m,n), qos)
        for kind in ('frame', 'heartbeat'):
            self.create_subscription(String, '/safe_redcho/'+kind, lambda m,k=kind: self.on_context(m,k), qos)
        if cfg['onboard_blocks']:
            from crazyflie_interfaces.msg import LogDataGeneric
            for topic, block in cfg['onboard_blocks'].items():
                for n in NAMES:
                    self.create_subscription(LogDataGeneric, '/'+n+'/'+topic,
                        lambda m,n=n,b=block: self.onboard_log(m,n,b), qos)
        self.tail = CommandTail(cfg['results_root'], cfg['command_file'])
        self.writer = EventWriter(cfg['output_root'], cfg) if cfg['record_events'] else None
        self.create_timer(1./cfg['refresh_hz'], self.tick)
        self.get_logger().info('Read-only capture; no cmd_position subscription and no radio log setup.')
        if self.writer: self.get_logger().info('Plot data output: '+str(self.writer.directory))

    def clock_values(self): return self.get_clock().now().nanoseconds, time.monotonic_ns()

    def mocap(self, msg):
        receipt, mono = self.clock_values(); stamp = stamp_ns(msg)
        if msg.header.frame_id != self.cfg['profile']['frame_id']:
            self.engine.rejected += 1; return
        counts = {n: sum(x.name == n for x in msg.poses) for n in NAMES}
        positions = {}
        for x in msg.poses:
            if x.name not in NAMES: continue
            if counts[x.name] != 1: self.engine.rejected += 1; continue
            p, q = x.pose.position, x.pose.orientation
            if self.engine.pose(x.name, [p.x,p.y,p.z], [q.x,q.y,q.z,q.w], stamp, receipt, mono):
                positions[x.name] = [p.x,p.y,p.z]
        self.engine.pair_batch(positions, stamp, receipt, mono)

    def pose(self, msg, name):
        receipt, mono = self.clock_values()
        if msg.header.frame_id != self.cfg['profile']['frame_id']:
            self.engine.rejected += 1; return
        p, q = msg.pose.position, msg.pose.orientation
        self.engine.pose(name, [p.x,p.y,p.z], [q.x,q.y,q.z,q.w], stamp_ns(msg), receipt, mono, 'onboard')

    def battery(self, msg, name):
        receipt, mono = self.clock_values()
        self.engine.status(name, msg.battery_voltage, stamp_ns(msg), receipt, mono)

    def algorithm(self, msg, name):
        receipt, mono = self.clock_values()
        try: self.engine.algorithm(parse(msg.data), name, receipt, mono)
        except (ValueError, KeyError, TypeError, IndexError, OverflowError): self.engine.rejected += 1

    def on_context(self, msg, kind):
        receipt, mono = self.clock_values()
        try: self.engine.context_packet(parse(msg.data), kind, receipt, mono)
        except (ValueError, KeyError, TypeError): self.engine.rejected += 1

    def onboard_log(self, msg, name, block):
        # Existing mappings only. Optional raw firmware fields are recorded without
        # claiming they acknowledge an individual command or equal PID output.
        receipt, mono = self.clock_values()
        fields = block.get('fields', [])
        if len(fields) != len(msg.values): self.engine.rejected += 1; return
        values = {name+'.firmware.'+f: number(v) for f,v in zip(fields,msg.values)}
        maximum_age=max(1.,3./float(block.get('frequency',1)))
        for field in values:
            self.engine.touch(field,stamp_ns(msg),receipt,mono)
            self.engine.streams[field]['maximum_age_s']=maximum_age
        self.engine.emit('control' if block.get('recorder') == 'control' else 'flight',
                         stamp_ns(msg), values, receipt, mono,
                         {'firmware_timestamp_raw': getattr(msg, 'timestamp', None),
                          'scope': 'sampled onboard log; not a per-command acknowledgement'})

    def tick(self):
        receipt, mono = self.clock_values()
        if receipt < self.last_ros-50_000_000:
            self.engine = Engine(self.cfg, receipt); self.sequence = {g: 0 for g in GROUPS}
            self.get_logger().warning('ROS clock moved backwards: new plot epoch')
        self.last_ros = receipt
        for _ in range(1200):
            try: path, row = self.tail.queue.get_nowait()
            except queue.Empty: break
            if self.engine.command_path and path != self.engine.command_path:
                for n in NAMES:
                    self.engine.streams.pop(n+'.command', None)
                    self.engine.last_stamp.pop(n+'.command', None)
            self.engine.command_path = path
            self.engine.command(row, receipt, mono)
        self.engine.ages(receipt, mono)
        state = self.engine.banner(receipt, mono)
        state.update(csv_tail_drops=self.tail.dropped, csv_tail_errors=self.tail.errors,
                     writer_drops=self.writer.dropped if self.writer else 0)
        for group in GROUPS:
            events = list(self.engine.out[group]); self.engine.out[group].clear()
            if self.writer:
                for event in events: self.writer.submit(group, dict(event, epoch=self.engine.epoch))
            self.sequence[group] += 1
            payload = dict(schema_version=1, group=group, epoch=self.engine.epoch,
                           origin_ns=self.engine.origin_ns, batch=self.sequence[group],
                           now_ns=receipt, events=events, status=state, limits=limits(self.cfg))
            self.bus_publishers[group].publish(String(data=json.dumps(payload, allow_nan=False, separators=(',', ':'))))

    def destroy_node(self):
        self.tail.close()
        # Drain any accepted normalized events left since the final transport tick.
        if self.writer:
            for group in GROUPS:
                for event in self.engine.out[group]: self.writer.submit(group, dict(event, epoch=self.engine.epoch))
            self.get_logger().info('Plot data writer: '+str(self.writer.close()))
            receipt, mono = self.clock_values()
            summary = self.engine.banner(receipt, mono)
            summary.update(csv_tail_drops=self.tail.dropped,csv_tail_errors=self.tail.errors,
                           csv_rows_remaining=self.tail.queue.qsize())
            (self.writer.directory/'capture_summary.json').write_text(json.dumps(summary,indent=2))
        return super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=None)
    parser.add_argument('--profile', default=None)
    parser.add_argument('--command-file', default=None)
    args, ros_args = parser.parse_known_args()
    cfg = load(args.config or default_config(), args.profile)
    if args.command_file is not None: cfg['command_file'] = args.command_file
    rclpy.init(args=ros_args); node = Capture(cfg)
    try: rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException): pass
    finally: node.destroy_node(); rclpy.try_shutdown()

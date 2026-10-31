"""One OS process owns one vehicle's two REDCHO banks and one hard HOCBF-QP."""
import argparse
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .settings import load
from .protocol import encode, decode, fingerprint
from .controller import LocalController
from .telemetry_trace import DiagnosticEmitter


class Agent(Node):
    def __init__(self, cfg, index):
        super().__init__('safe_redcho_agent', namespace=cfg['names'][index])
        self.cfg, self.index, self.agent_name = cfg, index, cfg['names'][index]
        self.trace_enabled = os.environ.get('SAFE_REDCHO_TELEMETRY', '0').lower() in ('1', 'true', 'yes')
        self.emitter = DiagnosticEmitter(self, self.agent_name) if self.trace_enabled else None
        self.core = LocalController(cfg, index, trace=self.trace_enabled)
        self.session, self.active_frame, self.sent = None, None, False
        self.peers, self.failed = {}, False
        self.peer_publisher = self.create_publisher(String, f'/safe_redcho/{self.agent_name}/neighbor', 8)
        self.command_publisher = self.create_publisher(String, '/safe_redcho/commands', 16)
        self.ready_publisher = self.create_publisher(String, '/safe_redcho/agent_ready', 8)
        self.create_subscription(String, '/safe_redcho/frame', self.on_frame, 8)
        for neighbor in self.core.neighbors:
            self.create_subscription(String, f'/safe_redcho/{neighbor}/neighbor',
                                     lambda msg, n=neighbor: self.on_peer(msg, n), 8)
        self.create_timer(0.5, self.announce)

    def announce(self):
        self.ready_publisher.publish(String(data=encode({'name': self.agent_name, 'pid': os.getpid(),
                                                        'config_hash': fingerprint(self.cfg)})))

    def fault(self, exc):
        self.failed = True
        frame = self.active_frame or {}
        self.command_publisher.publish(String(data=encode({'name': self.agent_name, 'pid': os.getpid(),
            'session': frame.get('session'), 'seq': frame.get('seq'), 'error': f'{type(exc).__name__}: {exc}'})))
        if self.emitter:
            self.emit_trace(getattr(self.core, 'trace', {}), error=f'{type(exc).__name__}: {exc}')

    def on_frame(self, msg):
        try:
            frame = decode(msg.data)
            if frame['config_hash'] != fingerprint(self.cfg): raise ValueError('Configuration mismatch')
            if self.session != frame['session']:
                if frame['seq'] != 1: return
                self.core = LocalController(self.cfg, self.index, trace=self.trace_enabled)
                self.session, self.failed = frame['session'], False
                self.peers = {k: v for k, v in self.peers.items() if k[0] == self.session}
            if self.failed: return
            if frame['seq'] <= self.core.sequence: return
            self.active_frame, self.sent = frame, False
            outgoing = self.core.prepare(frame)
            outgoing.update(name=self.agent_name, session=self.session, seq=frame['seq'])
            self.peer_publisher.publish(String(data=encode(outgoing)))
            self.try_finish()
        except Exception as exc:
            self.fault(exc)

    def on_peer(self, msg, neighbor):
        try:
            packet = decode(msg.data)
            if packet['name'] != neighbor: raise ValueError('Unexpected neighbor identity')
            key = (packet['session'], packet['seq'], neighbor)
            previous = self.peers.get(key)
            if previous is not None and previous != packet: raise ValueError('Conflicting duplicate ring packet')
            self.peers[key] = packet
            if len(self.peers) > 24:
                del self.peers[next(iter(self.peers))]
            self.try_finish()
        except Exception as exc:
            self.fault(exc)

    def try_finish(self):
        if self.failed or self.sent or self.active_frame is None or not self.core.prepared: return
        seq = self.active_frame['seq']
        keys = [(self.session, seq, n) for n in self.core.neighbors]
        if not all(k in self.peers for k in keys): return
        result = self.core.finish({n: self.peers[(self.session, seq, n)] for n in self.core.neighbors})
        trace = result.pop('diagnostics', None)
        result.update(name=self.agent_name, pid=os.getpid(), session=self.session, seq=seq)
        self.command_publisher.publish(String(data=encode(result)))
        self.sent = True
        if trace is not None: self.emit_trace(trace)
        for key in keys: self.peers.pop(key, None)

    def emit_trace(self, trace, error=None):
        frame = self.active_frame or {}
        packet = {'schema_version': 1, 'name': self.agent_name, 'pid': os.getpid(),
            'session': self.session, 'seq': frame.get('seq'), 'stage': frame.get('stage'),
            'flight_active': frame.get('flight_active'), 'streaming': frame.get('streaming'),
            'config_hash': fingerprint(self.cfg), 'producer_ros_ns': self.get_clock().now().nanoseconds,
            'producer_monotonic_ns': time.monotonic_ns(), 'error': error, 'data': trace}
        self.emitter.submit(packet)

    def destroy_node(self):
        if self.emitter: self.emitter.close()
        return super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--agent', type=int, choices=(2, 3, 4), required=True)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    cfg = load(args.config)
    name = f'cf{args.agent}'
    if name not in cfg['names']: raise ValueError('Inactive aircraft ID')
    node = Agent(cfg, cfg['names'].index(name))
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

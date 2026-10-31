import argparse
from copy import deepcopy
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from .common import AsyncCSV, FIELDS, make_rows, selected, utc_now


class Recorder(Node):
    def __init__(self, key, cfg, directory):
        super().__init__('redcho_telemetry_control' if key=='control' else f'redcho_three_telemetry_{key}')
        self.key, self.cfg, self.options = key, cfg, deepcopy(cfg['nodes'][key])
        self.names = cfg['names']
        self.recording_start = time.monotonic()
        self.switches = dict(self.options.get('variables', {}))
        self.declare_parameter('record_enabled', self.options.get('record_enabled', True))
        self.declare_parameter('terminal_enabled', self.options.get('terminal_enabled', True))
        self.declare_parameter('terminal_period_s', float(self.options.get('terminal_period_s', 1.)))
        self.declare_parameter('include_variables', self.options.get('include_variables', ['*']))
        self.declare_parameter('exclude_variables', self.options.get('exclude_variables', ['__none__']))
        for path, enabled in self.switches.items(): self.declare_parameter(f'variables.{path}', bool(enabled))
        self.add_on_set_parameters_callback(self.parameter_change)
        self.writer = AsyncCSV(Path(directory)/f'{key}_{os.getpid()}',
                               cfg['writer']['queue_batches'], cfg['writer']['flush_s'], cfg['writer'].get('layout','wide'))
        (self.writer.directory/'configuration_snapshot.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
        (self.writer.directory/'README.txt').write_text(
            'Default wide CSV: one row per received sample, selected scalar variables as columns. No sample-and-hold fabrication.\n'
            'Receipt, source, firmware and algorithm times are different clocks; session+seq identifies algorithm cycles.\n'
            'Blank source stamps / sequence numbers mean the source did not supply that information.\n'
            'Blank values plus invalid_fields identify missing/nonfinite fields. Column units are in column_dictionary.json.\n'
            'Enabling new variables may roll a stream into _partNNNN.csv with an expanded header.\n'
            'Availability CSV reports missing/stale streams. Optional long layout uses valid=0 for missing data.\n'
            'Position messages have no algorithm cycle ID; their sequence columns stay blank.\n'
            'Algorithm absence between cycles/takeoff/handover is expected: inspect cycle_frame/heartbeat.\n'
            'Best-effort DDS and bounded queues can lose data; inspect recorder_status.json and writer_summary.json.\n')
        self.latest, self.expected, self.seen, self.bad_packets = {}, {}, {}, 0
        self.received, self.late, self.skipped, self.last_sequence = 0, 0, 0, {}
        self.last_terminal, self.last_health = 0., 0.
        self.qos = QoSProfile(depth=int(cfg['subscription_depth']), reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST)
        self.create_timer(.2, self.tick)
        self.get_logger().info(f'CSV output: {self.writer.directory}')

    def parameter_change(self, parameters):
        for p in parameters:
            if p.name.startswith('variables.') and not isinstance(p.value, bool):
                return SetParametersResult(successful=False, reason='Variable switches must be boolean')
            if p.name in ('record_enabled', 'terminal_enabled') and not isinstance(p.value, bool):
                return SetParametersResult(successful=False, reason='Enable flags must be boolean')
            if p.name == 'terminal_period_s' and (isinstance(p.value,bool) or not isinstance(p.value, (float,int)) or not math.isfinite(p.value) or p.value < .2):
                return SetParametersResult(successful=False, reason='Terminal period must be >=0.2 seconds')
            if p.name in ('include_variables','exclude_variables') and (not isinstance(p.value,list) or not p.value or
                    any(not isinstance(s,str) for s in p.value)):
                return SetParametersResult(successful=False, reason='Use a nonempty string list; use __none__ for no matches')
        return SetParametersResult(successful=True)

    def expect(self, key, maximum_age_s): self.expected[key] = maximum_age_s
    def touch(self, key): self.seen[key] = time.monotonic()

    def record(self, stream, agent, data, topic, source, frame='world', stamp='',
               firmware_stamp='', session='', seq='', stage=''):
        self.received += 1
        self.touch(f'{agent}:{stream}')
        metadata = dict.fromkeys(FIELDS, '')
        metadata.update(record_utc=utc_now(), receipt_monotonic_ns=time.monotonic_ns(),
                        receipt_ros_ns=self.get_clock().now().nanoseconds, source_stamp_ns=stamp,
                        firmware_timestamp_raw=firmware_stamp, session=session, seq=seq, agent=agent,
                        stage=stage, source_topic=topic, source=source, frame=frame)
        metadata = {k: v for k,v in metadata.items() if k not in ('variable','value','unit','valid','reason')}
        self.switches={path:bool(self.get_parameter('variables.'+path).value) for path in self.switches}
        rows = make_rows(data, metadata, self.switches, self.get_parameter('include_variables').value,
                         self.get_parameter('exclude_variables').value)
        if self.get_parameter('record_enabled').value: self.writer.submit(stream, rows)
        for row in rows:
            self.latest[(agent,row['variable'])] = row

    def diagnostic_subscriptions(self):
        for name in self.names:
            topic = f'/safe_redcho/{name}/algorithm_telemetry'
            self.expect(f'{name}:algorithm', 1.)
            self.create_subscription(String, topic, lambda msg,n=name,t=topic: self.diagnostic(msg,n,t), self.qos)

    def diagnostic(self, msg, name, topic):
        try:
            if len(msg.data) > 300_000: raise ValueError('Oversized diagnostic packet')
            def reject(token): raise ValueError(token)
            p = json.loads(msg.data, parse_constant=reject)
            if p['schema_version'] != 1 or p['name'] != name or not isinstance(p['data'],dict):
                raise ValueError('Unsupported diagnostic schema/identity')
            if p.get('config_hash')!=self.cfg['expected_config_hash']:raise ValueError('Diagnostic configuration mismatch')
            seq = p['seq']
            if isinstance(seq,bool) or not isinstance(seq,int) or seq < 1: raise ValueError('Invalid sequence')
            key = (name,p['session'])
            previous = self.last_sequence.get(key,0)
            if seq <= previous:
                self.late += 1; return
            if previous: self.skipped += max(0,seq-previous-1)
            self.last_sequence[key] = seq
            if len(self.last_sequence) > 30: self.last_sequence.pop(next(iter(self.last_sequence)))
            self.touch(f'{name}:algorithm')
            data = {group:p['data'][group] for group in self.options['algorithm_groups'] if group in p['data']}
            data['diagnostic_meta'] = {k:p.get(k) for k in ('error','emitter_dropped_before','emitter_errors_before',
                                                         'flight_active','streaming','config_hash','producer_monotonic_ns')}
            self.record('algorithm',name,data,topic,'offboard_exact_instrumentation',stamp=p['producer_ros_ns'],
                        session=p['session'],seq=seq,stage=p['stage'])
            self.on_diagnostic(p)
        except (KeyError,TypeError,ValueError,IndexError,OverflowError) as exc:
            self.bad_packets += 1
            if self.bad_packets <= 3: self.get_logger().warning(f'Rejected diagnostics: {exc}')

    def on_diagnostic(self, packet): pass

    def tick(self):
        now = time.monotonic()
        if now-self.last_health >= 2.:
            self.last_health = now
            availability = {}
            for key, maximum_age in self.expected.items():
                age = now-self.seen[key] if key in self.seen else None
                availability[key] = {'available': age is not None and age <= maximum_age,
                                     'receipt_age_s': age, 'maximum_age_s': maximum_age}
            self.record('availability','fleet',{'availability':availability},'', 'recorder_health',frame='')
            status = {'received':self.received,'bad_packets':self.bad_packets,'out_of_order_or_duplicate':self.late,
                      'observed_sequence_gaps':self.skipped,'dropped_csv_batches':self.writer.dropped_batches,
                      'writer_errors':self.writer.errors,'availability':availability}
            (self.writer.directory/'recorder_status.json').write_text(json.dumps(status,indent=2)+'\n')
            missing = [k for k,v in availability.items() if not v['available']]
            if missing: self.get_logger().warning('Missing/stale: '+', '.join(missing))
        if self.get_parameter('terminal_enabled').value and now-self.last_terminal >= self.get_parameter('terminal_period_s').value:
            self.last_terminal = now
            patterns = self.options.get('terminal_variables',['*'])
            limit = int(self.options.get('terminal_values_per_agent',12))
            for agent in self.names+['fleet']:
                self.switches={path:bool(self.get_parameter('variables.'+path).value) for path in self.switches}
                rows = [row for (name,_),row in self.latest.items() if name == agent and
                        selected(row['variable'],self.switches,patterns,self.get_parameter('exclude_variables').value) and
                        selected(row['variable'],self.switches,self.get_parameter('include_variables').value,[])]
                from fnmatch import fnmatchcase
                rows.sort(key=lambda row:next((i for i,p in enumerate(patterns) if fnmatchcase(row['variable'],p)),len(patterns)))
                if not rows: continue
                values = []
                for row in rows[:limit]:
                    age = (time.monotonic_ns()-int(row['receipt_monotonic_ns']))/1e9
                    val = row['value']
                    if isinstance(val,float): val = f'{val:.5g}'
                    values.append(f"{row['variable']}={val} (age {age:.2f}s)")
                extra = f' | +{len(rows)-limit} hidden values' if len(rows)>limit else ''
                self.get_logger().info(agent+' | '+' | '.join(values)+extra)

    def destroy_node(self):
        summary = self.writer.close()
        self.get_logger().info(f'CSV closed: {summary}')
        return super().destroy_node()


def run(key, cls):
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=str(Path.home()/'safe_redcho_hardware/config/telemetry_three.yaml'))
    parser.add_argument('--output', default=None)
    args, ros_args = parser.parse_known_args()
    from .configuration import load_config
    cfg = load_config(args.config)
    output = args.output or str(Path(cfg['output_root']).expanduser()/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    rclpy.init(args=ros_args)
    node = cls(key,cfg,output)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node(); rclpy.try_shutdown()

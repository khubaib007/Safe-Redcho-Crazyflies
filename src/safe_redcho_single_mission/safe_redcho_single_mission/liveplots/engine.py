"""ROS/Qt-free normalization, timestamp matching and exact-cycle metrics.

Only finite observed values are emitted; unavailable scalars are None. No
sample-and-hold command inference, controller output synthesis or zero filling.
"""
from collections import OrderedDict, defaultdict, deque
import bisect
import math
import uuid
import numpy as np
from .configuration import GROUPS, NAMES

AXES = ('x', 'y', 'z')


def number(value):
    if isinstance(value, (bool, int, float)) and math.isfinite(float(value)): return float(value)
    return None


def path(data, dotted):
    try:
        for part in dotted.split('.'):
            data = data[int(part)] if isinstance(data, (list, tuple)) else data[part]
        return number(data)
    except (KeyError, IndexError, ValueError, TypeError): return None


def orientation(q):
    q = np.asarray(q, float)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.isfinite(q).all() or not .95 <= norm <= 1.05:
        raise ValueError('Invalid quaternion')
    x, y, z, w = q/norm
    return [math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(float(np.clip(2*(w*y-z*x), -1, 1))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))]


class Engine:
    def __init__(self, cfg, ros_ns):
        self.cfg = cfg
        self.epoch = uuid.uuid4().hex
        self.origin_ns = int(ros_ns)
        self.out = {g: deque(maxlen=3000) for g in GROUPS}
        self.streams, self.sequences, self.pending = {}, OrderedDict(), OrderedDict()
        self.pose_history = {n: deque(maxlen=600) for n in NAMES}
        self.last_stamp = {}
        self.rejected = self.skipped = self.incomplete = self.dropped = 0
        self.stage = 'WAITING'
        self.frame_time = None
        self.context = None
        self.command_path = ''
        self.algorithm_errors = {}

    def emit(self, group, stamp, values, receipt, mono, meta=None):
        event = dict(source_ros_ns=stamp, receipt_ros_ns=int(receipt), receipt_monotonic_ns=int(mono),
                     t_ns=int(stamp or receipt), values={k: number(v) for k, v in values.items()}, meta=meta or {})
        if len(self.out[group]) == self.out[group].maxlen: self.dropped += 1
        self.out[group].append(event)

    def touch(self, key, stamp, receipt, mono):
        self.streams[key] = dict(source_ns=stamp, receipt_ns=int(receipt), mono_ns=int(mono))

    def valid_stamp(self, key, stamp, receipt, maximum_age):
        if (not isinstance(stamp, int) or stamp <= 0 or not -.05 <= (receipt-stamp)/1e9 <= maximum_age
                or stamp <= self.last_stamp.get(key, 0)):
            self.rejected += 1; return False
        self.last_stamp[key] = stamp
        return True

    def pose(self, name, p, q, stamp, receipt, mono, kind='mocap'):
        if name not in NAMES: return False
        key = name+'.'+kind
        age = self.cfg['profile']['safety']['mocap_maximum_age_s' if kind == 'mocap' else 'maximum_estimator_age_s']
        try:
            p = np.asarray(p, float); angles = orientation(q)
            if p.shape != (3,) or not np.isfinite(p).all(): raise ValueError('Invalid position')
        except (ValueError, TypeError): self.rejected += 1; return False
        if not self.valid_stamp(key, stamp, receipt, age): return False
        self.touch(key, stamp, receipt, mono)
        vals = {f'{name}.{kind}.{a}': v for a, v in zip(AXES, p)}
        vals.update({f'{name}.{kind}.{a}': v for a, v in zip(('roll', 'pitch', 'yaw'), angles)})
        if kind == 'mocap':
            history = self.pose_history[name]; history.append((stamp, p.tolist()))
            window = self.cfg['profile']['safety']['mocap_velocity_window_s']
            rows = [(t, x) for t, x in history if t >= stamp-window*1e9]
            velocity = None
            if len(rows) >= 5 and rows[-1][0]-rows[0][0] >= .5*window*1e9:
                gaps = np.diff([r[0] for r in rows])/1e9
                if np.max(gaps) <= self.cfg['profile']['safety']['mocap_maximum_age_s']:
                    t = np.array([(r[0]-rows[0][0])/1e9 for r in rows]); t -= t.mean()
                    velocity = t@np.array([r[1] for r in rows])/max(float(t@t), 1e-12)
            vals[f'{name}.mocap.speed'] = None if velocity is None else float(np.linalg.norm(velocity))
            vals.update({f'{name}.mocap.v{a}': None if velocity is None else velocity[i] for i, a in enumerate(AXES)})
        self.emit('flight', stamp, vals, receipt, mono)
        return True

    def pair_batch(self, positions, stamp, receipt, mono):
        if set(positions) != set(NAMES): return
        vals = {f'pair.{NAMES[j]}_{NAMES[i]}': float(np.linalg.norm(
                np.asarray(positions[NAMES[i]])[:2]-np.asarray(positions[NAMES[j]])[:2]))
                for i in range(3) for j in range(i)}
        self.emit('flight', stamp, vals, receipt, mono)

    def status(self, name, voltage, stamp, receipt, mono):
        if name not in NAMES or number(voltage) is None: self.rejected += 1; return
        if stamp and not self.valid_stamp(name+'.battery', stamp, receipt, self.cfg['profile']['battery']['status_maximum_age_s']): return
        self.touch(name+'.battery', stamp, receipt, mono)
        self.emit('battery', stamp, {name+'.voltage': voltage}, receipt, mono)

    def command(self, row, receipt, mono):
        """Mission CSV publication, never a new subscription on a command topic."""
        try:
            name, stamp = row['name'], int(row['ros_ns'])
            p = [float(row['published_'+a+'_m']) for a in AXES]
            original_mono = float(row['wall_monotonic_s'])
            if (name not in NAMES or not np.isfinite(p).all() or not math.isfinite(original_mono)
                    or row.get('interface') != 'cmd_position'): raise ValueError('Invalid command row')
        except (KeyError, ValueError, TypeError): self.rejected += 1; return
        key = name+'.command'
        previous = self.streams.get(key)
        if not self.valid_stamp(key, stamp, receipt, self.cfg['command_max_age_s']): return
        self.touch(key, stamp, receipt, mono)
        self.streams[key]['original_mono'] = original_mono
        self.streams[key]['stage'] = row.get('stage', '')
        vals = {f'{name}.command.{a}': v for a, v in zip(AXES, p)}
        self.emit('flight', stamp, vals, receipt, mono,
                  {'delivery': 'host CSV publication; onboard receipt unverified',
                   'command_file': self.command_path, 'stage': row.get('stage',''),
                   'original_publication_monotonic_s': original_mono})
        if previous and 'original_mono' in previous:
            gap = (original_mono-previous['original_mono'])*1000
            self.emit('timing', stamp, {name+'.command_gap_ms': gap if gap > 0 else None}, receipt, mono)
        rows = list(self.pose_history[name]); times = [r[0] for r in rows]
        k = bisect.bisect_left(times, stamp)
        matched, skew = None, None
        limit = self.cfg['alignment_max_skew_s']*1e9
        if 0 < k < len(rows) and times[k]-times[k-1] <= 2*limit:
            f = (stamp-times[k-1])/(times[k]-times[k-1])
            matched = (1-f)*np.array(rows[k-1][1])+f*np.array(rows[k][1]); skew = 0.
        elif rows:
            closest = min(rows, key=lambda r: abs(r[0]-stamp))
            if abs(closest[0]-stamp) <= limit:
                matched = np.array(closest[1]); skew = abs(closest[0]-stamp)/1e9
        errors = None if matched is None else matched-np.array(p)
        vals = {name+'.tracking.norm': None if errors is None else np.linalg.norm(errors)}
        vals.update({f'{name}.tracking.{a}': None if errors is None else errors[i] for i, a in enumerate(AXES)})
        self.emit('flight', stamp, vals, receipt, mono, {'comparison': 'mocap vs published command; not exact mission guard', 'skew_s': skew})

    def algorithm(self, packet, name, receipt, mono):
        try:
            p = packet
            if (p['schema_version'] != 1 or p['name'] != name or name not in NAMES
                or p['config_hash'] != self.cfg['config_hash'] or not isinstance(p['data'], dict)
                or not isinstance(p['session'], str) or not p['session']
                or isinstance(p['seq'], bool) or not isinstance(p['seq'], int) or p['seq'] < 1
                or not isinstance(p['flight_active'], bool) or not isinstance(p['streaming'], bool)):
                raise ValueError('Diagnostic identity/schema/profile mismatch')
            stamp = int(p['producer_ros_ns'])
            if not -.05 <= (receipt-stamp)/1e9 <= 1.: raise ValueError('Diagnostic source stale/future')
            seqkey = (name, p['session'])
            previous = self.sequences.get(seqkey, 0)
            if p['seq'] <= previous: raise ValueError('Duplicate/out-of-order cycle')
            self.sequences[seqkey] = p['seq']
            if len(self.sequences) > 30: self.sequences.popitem(last=False)
            self.skipped += max(0, p['seq']-previous-1) if previous else 0
        except (KeyError, TypeError, ValueError, OverflowError): self.rejected += 1; return
        self.touch(name+'.algorithm', stamp, receipt, mono)
        self.algorithm_errors[name] = str(p['error'])[:160] if p.get('error') else None
        self.context = dict(stage=p.get('stage','?'), active=p['flight_active'], streaming=p['streaming'], mono=mono)
        data = p['data']
        meta = {k: p.get(k) for k in ('session', 'seq', 'stage', 'flight_active', 'streaming', 'error')}
        self.emit('battery', stamp, {name+'.scale': path(data, 'governor.speed_scale'),
                    name+'.governor_voltage': path(data, 'governor.input_voltage_v')}, receipt, mono, meta)
        ref = {}
        for axis in AXES:
            for tag, dotted in [('raw', 'governor.input_unscaled_velocity_mps'),
                                ('governed', 'governor.output_velocity_mps'),
                                ('svf_v', 'svf.output_velocity_mps'), ('svf_a', 'svf.output_acceleration_mps2'),
                                ('svf_j', 'svf.output_jerk_mps3')]:
                ref[f'{name}.{tag}.{axis}'] = path(data, dotted+'.'+axis)
        self.emit('reference', stamp, ref, receipt, mono, meta)
        for bank, variables in [('position_redcho', ('position_m', 'velocity_mps', 'acceleration_mps2'))]:
            vals = {f'{name}.{side}.{var}.{axis}': path(data, f'{bank}.{side}.{var}.{axis}')
                    for side in ('input', 'output') for var in variables for axis in AXES}
            self.emit(bank, stamp, vals, receipt, mono, meta)
        vals = {}
        for tag, dotted in [('target', 'nominal.desired_position_m'), ('virtual', 'nominal.virtual_position_m'),
                            ('desired_v', 'nominal.desired_velocity_mps'), ('virtual_v', 'nominal.virtual_velocity_mps'),
                            ('feedforward', 'nominal.desired_acceleration_mps2'), ('nominal', 'qp.nominal_acceleration_mps2'),
                            ('filtered', 'qp.filtered_acceleration_mps2')]:
            for i, axis in enumerate(AXES): vals[f'{name}.{tag}.{axis}'] = path(data, dotted+'.'+str(i))
        vals[name+'.correction'] = path(data, 'qp.intervention_norm_mps2')
        vals[name+'.avoidance_active']=path(data,'avoidance.active')
        vals[name+'.avoidance_blend']=path(data,'avoidance.blend')
        vals[name+'.motion_progress']=path(data,'governor.motion_progress_scale')
        self.emit('control', stamp, vals, receipt, mono, meta)
        qp = data.get('qp', {}); rows = qp.get('rows', {})
        def minimum(field, squared=None):
            values = [number(r.get(field)) for label, r in rows.items()
                      if squared is None or label.startswith(('pair_', 'obstacle_')) == squared]
            values = [v for v in values if v is not None]
            return min(values) if values else None
        vals = {name+'.residual': minimum('residual_b_minus_Au'),
                name+'.h_squared': minimum('h', True), name+'.h_wall': minimum('h', False),
                name+'.psi_squared': minimum('psi1', True), name+'.psi_wall': minimum('psi1', False)}
        for tag, field in [('violation', 'max_violation_mps2'), ('solver_invoked', 'inequality_solver_invoked'),
                           ('binding', 'hocbf_rows_binding'), ('success', 'solver_success')]:
            vals[name+'.'+tag] = number(qp.get(field))
        wall = number(qp.get('solver_wall_s')); vals[name+'.solve_ms'] = None if wall is None else wall*1000
        self.emit('hocbf', stamp, vals, receipt, mono, meta)
        self.emit('timing', stamp, {name+'.emitter_drops': number(p.get('emitter_dropped_before'))}, receipt, mono, meta)
        if not p.get('error'):
            key = tuple(p[k] for k in ('session', 'seq', 'config_hash', 'stage', 'flight_active', 'streaming'))
            self.pending.setdefault(key, {})[name] = p
            if len(self.pending[key]) == 3:
                self.convergence(self.pending.pop(key), receipt, mono)
            while len(self.pending) > 32: self.pending.popitem(last=False); self.incomplete += 1

    def convergence(self, packets, receipt, mono):
        for bank, variables in [('position_redcho', ('position_m', 'velocity_mps', 'acceleration_mps2'))]:
            vals = {}
            for var in variables:
                try:
                    inputs = np.array([[packets[n]['data'][bank]['input'][var][a] for a in AXES] for n in NAMES], float)
                    outputs = np.array([[packets[n]['data'][bank]['output'][var][a] for a in AXES] for n in NAMES], float)
                    if not np.isfinite(inputs).all() or not np.isfinite(outputs).all(): raise ValueError('Nonfinite bank')
                    average = inputs.mean(axis=0)
                    vals['fleet.disagreement.'+var] = max(np.linalg.norm(outputs[i]-outputs[j]) for i in range(3) for j in range(i))
                    vals['fleet.average_error.'+var] = np.max(np.linalg.norm(outputs-average, axis=1))
                    vals.update({f'fleet.average.{var}.{a}': average[i] for i, a in enumerate(AXES)})
                except (KeyError, TypeError, ValueError):
                    vals['fleet.disagreement.'+var] = None; vals['fleet.average_error.'+var] = None
            p = packets[NAMES[0]]; stamp = max(x['producer_ros_ns'] for x in packets.values())
            self.emit(bank, stamp, vals, receipt, mono, {k: p[k] for k in ('session', 'seq', 'stage', 'flight_active', 'streaming')})

    def context_packet(self, data, kind, receipt, mono):
        if data.get('config_hash') != self.cfg['config_hash']: self.rejected += 1; return
        self.touch('fleet.'+kind, None, receipt, mono)
        if kind == 'frame':
            if self.frame_time is not None:
                self.emit('timing', None, {'fleet.frame_interval_ms': (mono-self.frame_time)/1e6}, receipt, mono)
            self.frame_time = mono
            self.context = dict(stage=data.get('stage', '?'), active=data.get('flight_active', False),
                                streaming=data.get('streaming', False), mono=mono)

    def ages(self, receipt, mono):
        vals = {}
        for n in NAMES:
            for kind in ('mocap', 'onboard', 'command'):
                record = self.streams.get(n+'.'+kind)
                vals[f'{n}.{kind}_age_ms'] = (receipt-record['source_ns'])/1e6 if record and record['source_ns'] else None
        hb = self.streams.get('fleet.heartbeat')
        vals['fleet.heartbeat_age_ms'] = (mono-hb['mono_ns'])/1e6 if hb else None
        self.emit('timing', None, vals, receipt, mono)

    def banner(self, receipt, mono):
        if self.context and mono-self.context['mono'] < 1_000_000_000:
            self.stage = self.context['stage']+(' [ACTIVE]' if self.context['active'] else ' [SHADOW / no flight commands]')
        else:
            cmd = [r for k, r in self.streams.items() if k.endswith('.command') and mono-r['mono_ns'] < 2_000_000_000]
            self.stage = cmd[-1].get('stage', 'HOST COMMANDS') if cmd else 'NO FRESH ALGORITHM FRAME'
        states = {}
        for n in NAMES:
            for kind, maximum in [('mocap', self.cfg['profile']['safety']['mocap_maximum_age_s']),
                                  ('onboard', self.cfg['profile']['safety']['maximum_estimator_age_s']),
                                  ('battery', self.cfg['profile']['battery']['status_maximum_age_s']),
                                  ('algorithm', 1.), ('command', self.cfg['command_max_age_s'])]:
                key = n+'.'+kind; r = self.streams.get(key)
                age = None if r is None else max((mono-r['mono_ns'])/1e9, (receipt-r['source_ns'])/1e9 if r['source_ns'] else 0.)
                states[key] = 'MISSING' if age is None else 'STALE' if age > maximum else 'LIVE'
        for key,r in self.streams.items():
            if '.firmware.' not in key: continue
            age=max((mono-r['mono_ns'])/1e9,(receipt-r['source_ns'])/1e9 if r['source_ns'] else 0.)
            states[key]='STALE' if age>r['maximum_age_s'] else 'LIVE'
        return dict(stage=self.stage, streams=states, command_file=self.command_path,
                    rejected=self.rejected, skipped_cycles=self.skipped, incomplete_batches=self.incomplete,
                    capture_overflow=self.dropped,algorithm_errors=dict(self.algorithm_errors))

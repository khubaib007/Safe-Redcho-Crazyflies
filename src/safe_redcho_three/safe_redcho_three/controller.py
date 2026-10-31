import numpy as np
from .observers import Bank, Reference
from .cbf_qp import solve
from .settings import slots
from .telemetry_trace import reference_trace, bank_trace, QPTraceError


class LocalController:
    def __init__(self, cfg, index, trace=False):
        self.cfg, self.index = cfg, index
        self.trace_enabled, self.trace = trace, {}
        self.name = cfg['names'][index]
        self.neighbors = (cfg['names'][(index-1)%len(cfg['names'])], cfg['names'][(index+1)%len(cfg['names'])])
        dt, r = cfg['control']['dt_s'], cfg['redcho']
        self.velocity = Bank(r, r['velocity_L'], dt)
        self.position = Bank(r, r['position_L'], dt)
        self.reference = Reference(cfg, index)
        self.sequence = 0
        self.prepared = False
        self.base = slots(cfg)[index]

    def prepare(self, frame):
        if self.prepared or frame['seq'] != self.sequence+1:
            raise ValueError('Local controller sequence/order violation')
        if frame['stage'] not in ('ALIGN', 'FORWARD', 'STOP'):
            raise ValueError('Invalid mission stage')
        if self.trace_enabled:
            before_phase, before_state = self.reference.phase, self.reference.state.copy()
            self.trace_v_before, self.trace_p_before = self.velocity.X.copy(), self.position.X.copy()
        z, pose, self.telemetry = self.reference.step(frame['stage'], frame['voltage'][self.index])
        if self.trace_enabled:
            self.trace_z, self.trace_pose = z.copy(), pose.copy()
            self.trace = reference_trace(self.cfg, self.index, frame, before_phase, before_state, z, pose,
                dict(self.telemetry, svf_after=self.reference.state.copy(), phase_after=self.reference.phase))
        self.vy, self.py = self.velocity.prepare(z), self.position.prepare(pose)
        self.frame, self.prepared = frame, True
        return {'velocity': self.vy[:, 0].tolist(), 'position': self.py[:, 0].tolist()}

    def finish(self, peers):
        if not self.prepared or set(peers) != set(self.neighbors):
            raise ValueError('Expected exactly two same-cycle ring peers')
        self.velocity.finish({n: peers[n]['velocity'] for n in peers})
        self.position.finish({n: peers[n]['position'] for n in peers})
        q, v, previous = (np.asarray(self.frame[key], float) for key in ('q', 'v', 'previous'))
        target = self.base+self.py[:, 0]
        velocity = self.vy[:, 0].copy()
        feedforward = self.vy[:, 1].copy()
        velocity[2], feedforward[2] = self.py[2, 1], self.py[2, 2]
        c = self.cfg['control']
        nominal = feedforward+c['position_gain']*(target-q[self.index])+c['velocity_gain']*(velocity-v[self.index])
        if self.trace_enabled:
            self.trace['velocity_redcho'] = bank_trace(self.trace_z, self.vy, self.trace_v_before,
                self.velocity.X, {n: peers[n]['velocity'] for n in peers}, 'velocity')
            self.trace['position_redcho'] = bank_trace(self.trace_pose, self.py, self.trace_p_before,
                self.position.X, {n: peers[n]['position'] for n in peers}, 'position')
            self.trace['nominal'] = {
                'desired_position_m': target.tolist(), 'desired_velocity_mps': velocity.tolist(),
                'desired_acceleration_mps2': feedforward.tolist(),
                'virtual_position_m': q[self.index].tolist(), 'virtual_velocity_mps': v[self.index].tolist(),
                'position_error_m': (target-q[self.index]).tolist(),
                'velocity_error_mps': (velocity-v[self.index]).tolist(),
                'position_feedback_mps2': (c['position_gain']*(target-q[self.index])).tolist(),
                'velocity_feedback_mps2': (c['velocity_gain']*(velocity-v[self.index])).tolist(),
                'position_gain': c['position_gain'], 'velocity_gain': c['velocity_gain'],
                'u_nom_mps2': nominal.tolist(), 'base_slot_m': self.base.tolist()}
        try:
            safe, info = solve(self.index, nominal, q, v, previous, self.cfg, diagnostics=self.trace_enabled)
        except QPTraceError as exc:
            self.trace['qp'] = exc.diagnostics
            raise
        if self.trace_enabled:
            self.trace['qp'] = info.pop('diagnostics')
        self.sequence, self.prepared = self.frame['seq'], False
        result = dict(self.telemetry, **info, u=safe.tolist(), target=target.tolist(),
                    velocity_estimate=self.vy.tolist(), position_estimate=self.py.tolist())
        if self.trace_enabled: result['diagnostics'] = self.trace
        return result

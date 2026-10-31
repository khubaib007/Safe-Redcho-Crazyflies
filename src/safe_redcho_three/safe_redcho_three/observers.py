"""The two order-2 REDCHO banks from the finalized pose-consensus simulation.

The same gamma/G transform and smoothed signed-power injection are used. This
hardware profile samples at 50 Hz; it is not a bitwise 100 Hz replay.
"""
import numpy as np


def matrices(gamma):
    gamma = np.asarray(gamma, float)
    A = np.diag(-gamma)+np.diag(np.ones(2), 1)
    C = np.array([1., 0., 0.])
    return A, np.stack([C, C@A, C@A@A])


def local_derivative(X, Y, peers, cfg, L):
    injection = np.zeros((3, 3))
    for peer in sorted(peers):
        e = Y[:, 0]-np.asarray(peers[peer], float)
        for mu in range(3):
            alpha, beta = (2-mu)/3, (mu+1)/3
            power = e*(e*e+cfg['epsilon']**2)**((alpha-1)/2)
            injection[:, mu] += cfg['gains'][mu]*cfg['theta']**(mu+1)*L**beta*power
    derivative = -X*np.asarray(cfg['gamma'])[None, :]+injection
    derivative[:, :2] += X[:, 1:]
    return derivative


class Bank:
    def __init__(self, cfg, L, dt):
        self.cfg, self.L, self.dt = cfg, L, dt
        _, self.G = matrices(cfg['gamma'])
        self.X = np.zeros((3, 3))

    def prepare(self, reference):
        self.Y = np.asarray(reference, float)-(self.G@self.X.T).T
        if self.Y.shape != (3, 3) or not np.isfinite(self.Y).all():
            raise ValueError('Invalid REDCHO reference/estimate')
        return self.Y.copy()

    def finish(self, peers):
        if len(peers) != 2 or any(np.asarray(v).shape != (3,) or not np.isfinite(v).all() for v in peers.values()):
            raise ValueError('REDCHO requires two finite same-cycle zeroth-order ring estimates')
        self.X += self.dt*local_derivative(self.X, self.Y, peers, self.cfg, self.L)
        if not np.isfinite(self.X).all(): raise RuntimeError('Nonfinite REDCHO state')


class Reference:
    def __init__(self, cfg, index):
        self.cfg, self.index = cfg, index
        self.phase = 0.0
        self.state = np.zeros((3, 4))  # integral(v), v, a, jerk

    def step(self, stage, voltage):
        cfg, dt = self.cfg, self.cfg['control']['dt_s']
        b = cfg['battery']
        if not np.isfinite(voltage) or not 2.5 < voltage < 4.5:
            raise ValueError('Battery voltage must be a measured value in volts')
        if voltage < b['land_voltage_v']: raise RuntimeError('Battery below landing voltage')
        scale = float(np.clip((voltage-b['land_voltage_v'])/
                              (b['full_speed_voltage_v']-b['land_voltage_v']), b['minimum_speed_scale'], 1.))
        before = self.phase
        if stage == 'FORWARD':
            self.phase = min(1., before+dt*scale/cfg['mission']['nominal_forward_duration_s'])
        def smooth(s): return s**3*(10-15*s+6*s*s)
        # Incremental area is exactly the local desired displacement increment.
        # No fabricated SoC, modeled voltage, thrust-based current or fault reset.
        raw = np.array([cfg['mission']['distance_m']*(smooth(self.phase)-smooth(before))/dt, 0., 0.])
        omega = cfg['redcho']['svf_omega']
        def derivative(state):
            out = np.empty_like(state)
            out[:, :3] = state[:, 1:]
            out[:, 3] = (-omega**3*(state[:, 1]-raw)-3*omega**2*state[:, 2]-3*omega*state[:, 3])
            return out
        self.state += dt*derivative(self.state+0.5*dt*derivative(self.state))
        velocity = self.state[:, 1:].copy()
        pose = self.state[:, :3].copy()
        pose[2] = [cfg['mission']['height_proposal_offsets_m'][self.index], 0., 0.]
        coefficients = np.poly(-np.asarray(cfg['redcho']['gamma']))[1:][::-1]
        snap = (-omega**3*(self.state[:, 1]-raw)-3*omega**2*self.state[:, 2]-3*omega*self.state[:, 3])
        velocity_budget = float(np.linalg.norm(snap+velocity@coefficients))
        third = self.state[:, 3].copy(); third[2] = 0.
        position_budget = float(np.linalg.norm(third+pose@coefficients))
        if (velocity_budget > cfg['redcho']['velocity_L']/2
            or position_budget > cfg['redcho']['position_L']/2
            or np.linalg.norm(pose[:, 0]) > cfg['mission']['distance_m']+.05):
            raise RuntimeError('Local reference exceeded the configured REDCHO regularity/horizon budget')
        return velocity, pose, {'local_phase': self.phase, 'battery_scale': scale,
                                'raw_velocity': raw.tolist(), 'local_displacement': self.state[:, 0].tolist(),
                                'velocity_regularity_norm': velocity_budget,
                                'position_regularity_norm': position_budget}

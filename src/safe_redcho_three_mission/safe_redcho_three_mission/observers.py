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



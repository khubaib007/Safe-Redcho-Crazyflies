"""Reciprocal decentralized HOCBF-QP for the virtual reference plant.

Pair rows use the finalized simulation's common previous-command anchor and
equal responsibility. Wall/ceiling/floor and cylinder rows are added. No safety
slack or command clipping is accepted after solving.
"""
import time
import numpy as np
from scipy.optimize import minimize
from .settings import pair_distance, virtual_box
from .telemetry_trace import qp_trace, QPTraceError


def constraints(index, q, v, previous, cfg):
    s = cfg['safety']
    k1, k2 = s['hocbf_k1'], s['hocbf_k2']
    d, uncertainty = pair_distance(cfg), s['virtual_acceleration_uncertainty_mps2']
    rows, rhs, labels = [], [], []
    def add(a, b, label):
        norm = max(float(np.linalg.norm(a)), 1e-12)
        rows.append(np.asarray(a)/norm)
        rhs.append(float(b)/norm-s['row_reserve_mps2'])
        labels.append(label)
    for j in range(len(cfg['names'])):
        if j == index: continue
        r = np.r_[q[index, :2]-q[j, :2], 0.]
        rv = np.r_[v[index, :2]-v[j, :2], 0.]
        h, hd = r@r-d*d, 2*r@rv
        joint = 2*rv@rv+(k1+k2)*hd+k1*k2*h-4*np.linalg.norm(r)*uncertainty
        qi, qj = -2*r@previous[index], 2*r@previous[j]
        add(-2*r, qi+0.5*(joint-qi-qj), f'pair_{cfg["names"][j]}')
    lo, hi = virtual_box(cfg)
    for k in range(3):
        unit = np.eye(3)[k]
        add(unit, -(k1+k2)*v[index, k]+k1*k2*(hi[k]-q[index, k])-uncertainty, f'upper_{k}')
        add(-unit, (k1+k2)*v[index, k]+k1*k2*(q[index, k]-lo[k])-uncertainty, f'lower_{k}')
    a = cfg['arena']
    extra = (a['body_radius_m']+s['tracking_tube_m']+
             s['mocap_position_uncertainty_m']+s['wall_reserve_m'])
    for j, obstacle in enumerate(cfg['obstacles']):
        r = np.r_[q[index, :2]-obstacle['center_xy'], 0.]
        rv = np.r_[v[index, :2], 0.]
        radius = obstacle['radius_m']+obstacle['surface_clearance_m']+extra
        h, hd = r@r-radius*radius, 2*r@rv
        b = 2*rv@rv+(k1+k2)*hd+k1*k2*h-2*np.linalg.norm(r)*uncertainty
        add(-2*r, b, f'obstacle_{j}')
    return np.asarray(rows), np.asarray(rhs), labels


def solve(index, nominal, q, v, previous, cfg, diagnostics=False):
    c = cfg['control']
    q, v, previous = (np.asarray(x, float) for x in (q, v, previous))
    if any(x.shape != (len(cfg['names']), 3) or not np.isfinite(x).all() for x in (q, v, previous)):
        raise ValueError('QP requires finite common-cycle active N x 3 state/anchor batches')
    nominal = np.asarray(nominal, float)
    if nominal.shape != (3,) or not np.isfinite(nominal).all(): raise ValueError('Invalid nominal input')
    A, b, labels = constraints(index, q, v, previous, cfg)
    limit, dt = c['maximum_reference_acceleration_mps2'], c['dt_s']
    slew = c['maximum_acceleration_slew_mps3']*dt
    vmax = c['maximum_reference_speed_mps']
    lower = np.maximum.reduce([np.full(3, -limit), previous[index]-slew, (-vmax-v[index])/dt])
    upper = np.minimum.reduce([np.full(3, limit), previous[index]+slew, (vmax-v[index])/dt])
    if np.any(lower > upper): raise RuntimeError('Empty acceleration/slew/velocity box')
    weight = c['qp_previous_weight']
    free = (nominal+weight*previous[index])/(1+weight)
    clipped = np.clip(free, lower, upper)
    result = None
    started = time.perf_counter()
    if np.max(A@clipped-b) <= c['qp_tolerance']:
        value, iterations = clipped, 0
    else:
        result = minimize(lambda x: .5*(1+weight)*x@x-(nominal+weight*previous[index])@x,
                          clipped, jac=lambda x: (1+weight)*x-nominal-weight*previous[index],
                          bounds=list(zip(lower, upper)),
                          constraints=[{'type': 'ineq', 'fun': lambda x: b-A@x, 'jac': lambda x: -A}],
                          method='SLSQP', options={'maxiter': int(c['qp_max_iterations']), 'ftol': 1e-11})
        value, iterations = result.x, int(result.nit)
    violation = max(0., float(np.max(A@value-b)), float(np.max(lower-value)), float(np.max(value-upper)))
    elapsed = time.perf_counter()-started
    trace = qp_trace(index, nominal, q, v, previous, value, A, b, labels, lower, upper,
                     clipped, iterations, elapsed, violation, result, cfg) if diagnostics else None
    if not np.isfinite(value).all() or violation > c['qp_tolerance']:
        worst = labels[int(np.argmax(A@value-b))]
        message = f'{cfg["names"][index]} hard QP infeasible: violation={violation:.6g}, row={worst}'
        if diagnostics: raise QPTraceError(message, trace)
        raise RuntimeError(message)
    info = {'qp_violation': violation, 'qp_iterations': iterations,
            'qp_wall_s': elapsed, 'qp_correction': float(np.linalg.norm(value-nominal))}
    if diagnostics: info['diagnostics'] = trace
    return value, info


def joint_pair_margin(q, v, accelerations, cfg):
    s, d = cfg['safety'], pair_distance(cfg)
    margins = []
    for i in range(len(cfg['names'])):
        for j in range(i):
            r, rv = q[i, :2]-q[j, :2], v[i, :2]-v[j, :2]
            margin = (2*r@(accelerations[i, :2]-accelerations[j, :2])+2*rv@rv+
                      (s['hocbf_k1']+s['hocbf_k2'])*2*r@rv+
                      s['hocbf_k1']*s['hocbf_k2']*(r@r-d*d)-
                      4*np.linalg.norm(r)*s['virtual_acceleration_uncertainty_mps2'])
            margins.append(margin)
    return min(margins)


def minimum_held_distance(r, rv, ra, dt):
    coefficients = np.trim_zeros(np.array([ra@ra, 3*rv@ra, 2*(rv@rv+r@ra), 2*r@rv]), 'f')
    candidates = [0., dt]
    if len(coefficients) > 1:
        candidates += [float(root.real) for root in np.roots(coefficients)
                       if abs(root.imag) < 1e-9 and 0 < root.real < dt]
    return min(np.linalg.norm(r+rv*t+.5*ra*t*t) for t in candidates)


def validate_batch(q, v, previous, u, cfg):
    """Verify the complete proposed batch without solving any centralized QP."""
    c, dt = cfg['control'], cfg['control']['dt_s']
    for i in range(len(cfg['names'])):
        A, b, _ = constraints(i, q, v, previous, cfg)
        violation = max(float(np.max(A@u[i]-b)),
                        float(np.max(np.abs(u[i])))-c['maximum_reference_acceleration_mps2'],
                        float(np.max(np.abs(u[i]-previous[i])))-c['maximum_acceleration_slew_mps3']*dt,
                        float(np.max(np.abs(v[i]+dt*u[i])))-c['maximum_reference_speed_mps'])
        if violation > c['qp_tolerance']:
            raise RuntimeError(f'{cfg["names"][i]}: proposed command batch violates a local hard constraint')
    next_q, next_v = q+dt*v+.5*dt*dt*u, v+dt*u
    lo, hi = virtual_box(cfg)
    if np.any(next_q <= lo) or np.any(next_q >= hi):
        raise RuntimeError('Next reference step would leave inflated arena')
    for i in range(len(cfg['names'])):
        for k in range(3):
            if abs(u[i, k]) > 1e-12:
                extremum = -v[i, k]/u[i, k]
                if 0 < extremum < dt:
                    value = q[i, k]+v[i, k]*extremum+.5*u[i, k]*extremum**2
                    if not lo[k] < value < hi[k]:
                        raise RuntimeError('Reference would cross an arena boundary between samples')
    for i in range(len(cfg['names'])):
        for j in range(i):
            # Exact minimum distance during this constant-acceleration hold:
            # derivative of ||r+rv*t+0.5*ra*t^2||^2 is a cubic polynomial.
            r, rv, ra = q[i, :2]-q[j, :2], v[i, :2]-v[j, :2], u[i, :2]-u[j, :2]
            if minimum_held_distance(r, rv, ra, dt) < pair_distance(cfg)-1e-8:
                raise RuntimeError('Reference pair clearance would be violated between command samples')
        a, s = cfg['arena'], cfg['safety']
        extra = a['body_radius_m']+s['tracking_tube_m']+s['mocap_position_uncertainty_m']+s['wall_reserve_m']
        for obstacle in cfg['obstacles']:
            radius = obstacle['radius_m']+obstacle['surface_clearance_m']+extra
            if minimum_held_distance(q[i, :2]-obstacle['center_xy'], v[i, :2], u[i, :2], dt) < radius-1e-8:
                raise RuntimeError('Reference would cross obstacle buffer between command samples')
    return next_q, next_v

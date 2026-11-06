"""Reciprocal decentralized HOCBF-QP for the virtual reference plant.

Pair rows use the finalized simulation's common previous-command anchor and
equal responsibility. Wall/ceiling/floor and cylinder rows are added. No safety
slack or command clipping is accepted after solving.
"""
import time
import numpy as np
from .settings import pair_distance, virtual_box
from .telemetry_trace import qp_trace, QPTraceError
from .obstacles import obstacle_radius
from .qp_priority import solve_priority, velocity_slew_bounds, violation as command_violation
from .barrier_domain import enabled as predictive_enabled, check_domain, predictive_rows, check_next_domain
from .responsibility import enabled as adaptive_enabled, Allocator


def static_rows(index,q,v,cfg):
    """Raw static HOCBF rows, before the shared normalization/reserve step."""
    s=cfg['safety'];k1,k2=s['hocbf_k1'],s['hocbf_k2']
    uncertainty=s['virtual_acceleration_uncertainty_mps2'];lo,hi=virtual_box(cfg);rows=[]
    for k in range(3):
        unit=np.eye(3)[k]
        rows.append((unit,-(k1+k2)*v[index,k]+k1*k2*(hi[k]-q[index,k])-uncertainty,f'upper_{k}'))
        rows.append((-unit,(k1+k2)*v[index,k]+k1*k2*(q[index,k]-lo[k])-uncertainty,f'lower_{k}'))
    for j,o in enumerate(cfg['obstacles']):
        r=np.r_[q[index,:2]-o['center_xy'],0.];rv=np.r_[v[index,:2],0.]
        radius=obstacle_radius(o,cfg)
        h,hd=r@r-radius*radius,2*r@rv
        b=2*rv@rv+(k1+k2)*hd+k1*k2*h-2*np.linalg.norm(r)*uncertainty
        rows.append((-2*r,b,f'obstacle_{j}'))
    return rows


def static_allocator(q,v,previous,cfg):
    """Build each agent's reserved static feasible polytope from shared data."""
    reserved=[]
    for index in range(len(cfg['names'])):
        raw=static_rows(index,q,v,cfg)
        if predictive_enabled(cfg):
            raw.extend(row for row in predictive_rows(index,q,v,previous,cfg) if not row[2].startswith('next_pair_'))
        A=[];b=[]
        for row,rhs,_ in raw:
            norm=max(float(np.linalg.norm(row)),1e-12)
            A.append(np.asarray(row)/norm);b.append(float(rhs)/norm-cfg['safety']['row_reserve_mps2'])
        reserved.append((np.asarray(A),np.asarray(b)))
    return Allocator(reserved,v,previous,cfg)


def constraints(index, q, v, previous, cfg, audit=False):
    if predictive_enabled(cfg):check_domain(q,v,cfg)
    s = cfg['safety']
    k1, k2 = s['hocbf_k1'], s['hocbf_k2']
    d, uncertainty = pair_distance(cfg), s['virtual_acceleration_uncertainty_mps2']
    rows, rhs, labels = [], [], []
    allocator=static_allocator(q,v,previous,cfg) if adaptive_enabled(cfg) else None
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
        label=f'pair_{cfg["names"][j]}'
        bound=allocator.allocate(index,j,-2*r,joint,label) if allocator is not None else qi+0.5*(joint-qi-qj)
        add(-2*r,bound,label)
    for a,b,label in static_rows(index,q,v,cfg):add(a,b,label)
    if predictive_enabled(cfg):
        for a,b,label in predictive_rows(index,q,v,previous,cfg,
                allocator.allocate if allocator is not None else None):add(a,b,label)
    result=(np.asarray(rows),np.asarray(rhs),labels)
    if audit:
        return (*result,dict(policy='static_authority_support' if allocator is not None else 'equal_previous_anchor',
                            allocations=allocator.allocations if allocator is not None else []))
    return result


def solve(index, nominal, q, v, previous, cfg, diagnostics=False):
    filter_started=time.perf_counter()
    c = cfg['control']
    q, v, previous = (np.asarray(x, float) for x in (q, v, previous))
    if any(x.shape != (len(cfg['names']), 3) or not np.isfinite(x).all() for x in (q, v, previous)):
        raise ValueError('QP requires finite common-cycle active N x 3 state/anchor batches')
    nominal = np.asarray(nominal, float)
    if nominal.shape != (3,) or not np.isfinite(nominal).all(): raise ValueError('Invalid nominal input')
    A, b, labels, responsibility = constraints(index, q, v, previous, cfg,audit=True)
    constraint_elapsed=time.perf_counter()-filter_started
    weight = c['qp_previous_weight']
    free = (nominal+weight*previous[index])/(1+weight)
    started = time.perf_counter()
    priority_cfg=dict(c,anticipate_velocity_slew=True) if predictive_enabled(cfg) else c
    value, result, lower, upper, priority = solve_priority(nominal, previous[index], v[index], A, b, priority_cfg)
    clipped = np.clip(free, lower, upper)
    iterations = int(result.nit) if result is not None else 0
    violation = command_violation(value, A, b, lower, upper)
    elapsed = time.perf_counter()-started
    trace = qp_trace(index, nominal, q, v, previous, value, A, b, labels, lower, upper,
                     clipped, iterations, elapsed, violation, result, cfg) if diagnostics else None
    if trace is not None:
        trace['slew_priority']=priority
        trace['inequality_solver_invoked']=priority['inequality_solver_invoked']
        trace['pair_responsibility']=responsibility
        trace['constraint_wall_s']=constraint_elapsed
    if not np.isfinite(value).all() or np.any(lower > upper) or violation > c['qp_tolerance']:
        worst = labels[int(np.argmax(A@value-b))]
        message = f'{cfg["names"][index]} hard QP infeasible: violation={violation:.6g}, row={worst}'
        if np.any(lower > upper):message+='; empty acceleration/slew/velocity bounds'
        if diagnostics: raise QPTraceError(message, trace)
        raise RuntimeError(message)
    info = {'qp_violation': violation, 'qp_iterations': iterations,
            'qp_wall_s': elapsed, 'qp_correction': float(np.linalg.norm(value-nominal)),
            'qp_constraint_wall_s':constraint_elapsed,'qp_total_wall_s':time.perf_counter()-filter_started,
            'qp_slew_relaxed':priority['slew_relaxation_applied'],
            'qp_slew_excess_mps2':priority.get('slew_excess_mps2',0.),
            'pair_responsibility_policy':responsibility['policy']}
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
                        float(np.max(np.abs(v[i]+dt*u[i])))-c['maximum_reference_speed_mps'])
        slew_limit=(c.get('maximum_safety_acceleration_slew_mps3',c['maximum_acceleration_slew_mps3'])
                    if c.get('allow_slew_relaxation_for_safety',False) else c['maximum_acceleration_slew_mps3'])
        violation=max(violation,float(np.max(np.abs(u[i]-previous[i])))-slew_limit*dt)
        if predictive_enabled(cfg):
            viable_lower,viable_upper=velocity_slew_bounds(v[i],c)
            violation=max(violation,float(np.max(viable_lower-u[i])),float(np.max(u[i]-viable_upper)))
        if violation > c['qp_tolerance']:
            raise RuntimeError(f'{cfg["names"][i]}: proposed command batch violates a local hard constraint')
    next_q, next_v = q+dt*v+.5*dt*dt*u, v+dt*u
    if predictive_enabled(cfg):check_next_domain(q,v,u,cfg)
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
            radius = obstacle_radius(obstacle, cfg)
            if minimum_held_distance(q[i, :2]-obstacle['center_xy'], v[i, :2], u[i, :2], dt) < radius-1e-8:
                raise RuntimeError('Reference would cross obstacle buffer between command samples')
    return next_q, next_v

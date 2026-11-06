"""Small convex QPs with an explicit, optional reference-slew priority.

Only the ordinary acceleration-slew target can be relaxed, up to an explicit
hard safety-slew cap. A @ u <= b, acceleration limits and next-step velocity
limits are never softened. A feasible current step does not
establish recursive feasibility or provide an obstacle-routing policy.
"""
from functools import lru_cache
from itertools import combinations
import numpy as np
from scipy.optimize import minimize, OptimizeResult


def violation(x, A, b, lower, upper):
    if not np.isfinite(x).all():
        return float('inf')
    return max(0., float(np.max(A@x-b)), float(np.max(lower-x)),
               float(np.max(x-upper)))


@lru_cache(maxsize=16)
def active_indices(count, size):
    return np.asarray(list(combinations(range(count), size)), dtype=int)


def active_set_projection(target, A, b, lower, upper, tolerance):
    """Fallback projection in R^3; enumerate independent active sets of size <=3.

    Every returned point is checked against every original row and bound.
    Nearly singular active sets are skipped rather than accepted unchecked.
    Failure to find a point is a conservative rejection, not an infeasibility
    certificate for an ill-conditioned numerical problem.
    """
    if np.any(lower > upper):
        return None
    C = np.vstack((A, np.eye(3), -np.eye(3)))
    d = np.r_[b, upper, -lower]
    candidates = [target[None, :]]
    for size in (1, 2, 3):
        ids = active_indices(len(d), size)
        M = C[ids]
        gram = M@M.transpose(0, 2, 1)
        # Rows from the mission are normalized. Avoid unstable intersections.
        good = np.linalg.det(gram) > 1e-12
        if not np.any(good):
            continue
        M, gram, ids = M[good], gram[good], ids[good]
        rhs = M@target-d[ids]
        multipliers = np.linalg.solve(gram, rhs[..., None])[..., 0]
        points = target-np.einsum('nki,nk->ni', M, multipliers)
        candidates.append(points)
    points = np.concatenate(candidates)
    good = np.isfinite(points).all(axis=1)
    good &= np.all(C@points.T <= d[:, None]+tolerance, axis=0)
    if not np.any(good):
        return None
    points = points[good]
    return points[np.argmin(np.sum((points-target)**2, axis=1))].copy()


def attempt(nominal, previous, A, b, lower, upper, cfg, initial=None):
    """Keep the original objective and fast path; verify all solver outputs."""
    tol, weight = cfg['qp_tolerance'], cfg['qp_previous_weight']
    target = (nominal+weight*previous)/(1+weight)
    clipped = np.clip(target, lower, upper)
    if np.any(lower > upper):
        return clipped, None, 'empty_bounds'
    if violation(clipped, A, b, lower, upper) <= tol:
        return clipped, None, 'bounded_objective_minimizer'
    x0 = clipped if initial is None else np.clip(initial, lower, upper)
    result = minimize(lambda x: .5*(1+weight)*x@x-(nominal+weight*previous)@x,
                      x0, jac=lambda x: (1+weight)*x-nominal-weight*previous,
                      bounds=list(zip(lower, upper)),
                      constraints=[{'type':'ineq', 'fun':lambda x: b-A@x,
                                    'jac':lambda x: -A}], method='SLSQP',
                      options={'maxiter':int(cfg['qp_max_iterations']), 'ftol':1e-11})
    if violation(result.x, A, b, lower, upper) <= tol:
        return result.x, result, 'SLSQP'
    point = active_set_projection(target, A, b, lower, upper, tol)
    if point is not None:
        verified = OptimizeResult(x=point, success=True, status=0, nit=int(result.nit),
            message='Verified 3D active-set fallback; SLSQP status '+str(result.status))
        return point, verified, 'verified_active_set'
    return result.x if np.isfinite(result.x).all() else clipped, result, 'no_verified_solution'


def velocity_slew_bounds(velocity, cfg):
    """Reserve velocity headroom for ramping acceleration to zero.

    A commanded component u is held for dt before it can change again. With
    hard acceleration-slew cap J, a conservative remaining velocity increase
    is u_positive²/(2 J): future discrete ramps have no larger area than that
    continuous triangle. Thus dt*u + u²/(2 J) must fit current speed headroom.
    This protects the speed/slew subsystem only, not joint HOCBF feasibility.
    """
    velocity=np.asarray(velocity,float)
    dt,vmax=cfg['dt_s'],cfg['maximum_reference_speed_mps']
    J=(cfg.get('maximum_safety_acceleration_slew_mps3',cfg['maximum_acceleration_slew_mps3'])
       if cfg.get('allow_slew_relaxation_for_safety',False) else cfg['maximum_acceleration_slew_mps3'])
    if velocity.shape!=(3,) or not np.isfinite(velocity).all():
        raise ValueError('Velocity viability requires a finite 3-vector')
    if any(isinstance(x,bool) or not isinstance(x,(int,float)) or not np.isfinite(x) or x<=0
           for x in (dt,vmax,J)):
        raise ValueError('Velocity viability requires positive finite dt, speed and hard slew')
    def cap(headroom):
        headroom=np.maximum(headroom,0.)
        # Stable version of sqrt((J*dt)^2 + 2*J*headroom) - J*dt.
        numerator=2*J*headroom
        return numerator/(np.sqrt((J*dt)**2+numerator)+J*dt)
    return -cap(vmax+velocity),cap(vmax-velocity)


def solve_priority(nominal, previous, velocity, A, b, cfg):
    dt, limit = cfg['dt_s'], cfg['maximum_reference_acceleration_mps2']
    vmax, tol = cfg['maximum_reference_speed_mps'], cfg['qp_tolerance']
    requested = cfg['maximum_acceleration_slew_mps3']*dt
    base_lower = np.maximum(np.full(3, -limit), (-vmax-velocity)/dt)
    base_upper = np.minimum(np.full(3, limit), (vmax-velocity)/dt)
    if cfg.get('anticipate_velocity_slew',False):
        viable_lower,viable_upper=velocity_slew_bounds(velocity,cfg)
        base_lower=np.maximum(base_lower,viable_lower)
        base_upper=np.minimum(base_upper,viable_upper)
    strict_lower = np.maximum(base_lower, previous-requested)
    strict_upper = np.minimum(base_upper, previous+requested)
    value, result, backend = attempt(nominal, previous, A, b, strict_lower, strict_upper, cfg)
    strict_ok = (np.all(strict_lower <= strict_upper) and
                 violation(value, A, b, strict_lower, strict_upper) <= tol)
    info = dict(slew_policy='safety_priority' if cfg.get('allow_slew_relaxation_for_safety', False) else 'hard',
                requested_slew_bound_mps2=float(requested), slew_relaxation_applied=False,
                strict_slew_lower_bound_mps2=strict_lower.tolist(),
                strict_slew_upper_bound_mps2=strict_upper.tolist(),
                strict_solve_backend=backend, solver_backend=backend,
                inequality_solver_invoked=result is not None,
                hard_constraint_solution_verified=bool(strict_ok))
    if cfg.get('anticipate_velocity_slew',False):
        info.update(velocity_slew_anticipation=True,
                    velocity_viability_lower_mps2=viable_lower.tolist(),
                    velocity_viability_upper_mps2=viable_upper.tolist())
    if strict_ok or not cfg.get('allow_slew_relaxation_for_safety', False):
        return value, result, strict_lower, strict_upper, info

    # Replace the ordinary target with the explicitly configured hard cap.
    # If this also fails, no further relaxation is allowed to hide the failure.
    safety_slew = cfg.get('maximum_safety_acceleration_slew_mps3',
                          cfg['maximum_acceleration_slew_mps3'])*dt
    base_lower = np.maximum(base_lower, previous-safety_slew)
    base_upper = np.minimum(base_upper, previous+safety_slew)
    info.update(inequality_solver_invoked=True,
                safety_slew_limit_mps3=float(safety_slew/dt))
    value, result, backend = attempt(nominal, previous, A, b, base_lower, base_upper, cfg)
    if np.any(base_lower > base_upper) or violation(value, A, b, base_lower, base_upper) > tol:
        info.update(solver_backend=backend, hard_constraint_solution_verified=False,
                    priority_result='No verified solution within hard safety rows, acceleration, speed and safety-slew cap')
        return value, result, base_lower, base_upper, info

    # Minimize the extra infinity-norm acceleration change, keeping every
    # safety row and the physical bounds hard. This LP uses SLSQP in R^4;
    # retain the already verified feasible point if that refinement fails.
    delta = max(0., float(np.max(np.abs(value-previous)))-requested)
    G = np.vstack((np.column_stack((A, np.zeros(len(b)))),
                   np.column_stack((np.eye(3), -np.ones(3))),
                   np.column_stack((-np.eye(3), -np.ones(3)))))
    rhs = np.r_[b, previous+requested, -previous+requested]
    refinement = minimize(lambda x: x[3], np.r_[value, delta],
        jac=lambda x: np.array([0.,0.,0.,1.]), method='SLSQP',
        bounds=list(zip(base_lower, base_upper))+[(0.,delta+10*tol)],
        constraints=[{'type':'ineq','fun':lambda x: rhs-G@x,'jac':lambda x: -G}],
        options={'maxiter':int(cfg['qp_max_iterations']), 'ftol':1e-11})
    candidate = refinement.x[:3]
    candidate_delta = max(0., float(np.max(np.abs(candidate-previous)))-requested)
    refined = (np.isfinite(refinement.x).all() and
               violation(candidate, A, b, base_lower, base_upper) <= tol and
               candidate_delta <= delta+tol)
    if refined:
        value, delta = candidate, candidate_delta
    width = requested+delta+10*tol
    lower = np.maximum(base_lower, previous-width)
    upper = np.minimum(base_upper, previous+width)
    optimized, optimized_result, backend = attempt(nominal, previous, A, b, lower, upper, cfg, initial=value)
    if violation(optimized, A, b, lower, upper) <= tol:
        value, result = optimized, optimized_result
    else:
        # The refinement point was verified above and remains admissible.
        result = OptimizeResult(x=value, success=True, status=0, nit=0,
                                message='Retained verified safety-priority point')
        backend = 'verified_priority_point'
    excess = max(0., float(np.max(np.abs(value-previous)))-requested)
    info.update(slew_relaxation_applied=bool(excess > tol), solver_backend=backend,
        actual_maximum_slew_mps3=float(np.max(np.abs(value-previous))/dt),
        slew_excess_mps2=float(excess), effective_slew_bound_mps2=float(width),
        minimum_slew_refinement_success=bool(refinement.success and refined),
        hard_constraint_solution_verified=True,
        priority_result='Safety rows, acceleration and next-step speed remain hard')
    return value, result, lower, upper, info

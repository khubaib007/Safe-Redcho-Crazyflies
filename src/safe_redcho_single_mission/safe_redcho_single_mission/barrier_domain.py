"""Optional one-hold HOCBF domain checks for the virtual double integrator.

Checking h >= 0 and psi1 >= 0 is a necessary HOCBF initialization condition,
not a viability or recursive-feasibility certificate under bounded actuation.
The squared-distance next-psi1 function is convex quadratic in acceleration;
its tangent is a global affine lower bound. Uncertainty reserves account for
one zero-order hold of a norm-bounded acceleration error, not physical aircraft
tracking (which remains covered separately by configured tracking tubes).
"""
import numpy as np

from .settings import pair_distance, virtual_box
from .obstacles import obstacle_radius


def enabled(cfg):
    return cfg.get('avoidance', {}).get('predictive_barriers', False) is True


def _state(q, v, cfg):
    q, v = np.asarray(q, float), np.asarray(v, float)
    shape = (len(cfg['names']), 3)
    if q.shape != shape or v.shape != shape or not np.isfinite(q).all() or not np.isfinite(v).all():
        raise ValueError('Predictive barriers require finite common-cycle N x 3 q/v')
    return q, v


def squared_psi1(r, velocity, acceleration, radius, dt, k1):
    """Exact held-command endpoint psi1, in m²/s, for relative XY state."""
    r, velocity, acceleration = (np.asarray(x, float) for x in (r, velocity, acceleration))
    qn = r + dt*velocity + .5*dt*dt*acceleration
    vn = velocity + dt*acceleration
    value = float(2*qn@vn + k1*(qn@qn-radius*radius))
    gradient = dt*dt*vn + (2*dt+k1*dt*dt)*qn
    return value, gradient


def squared_reserve(r, velocity, dt, k1, acceleration_bound, uncertainty):
    """Conservative one-hold psi1 error bound, in m²/s.

    If ||a_error|| <= uncertainty, endpoint position/velocity errors satisfy
    ep=.5 dt² uncertainty [m], ev=dt uncertainty [m/s]. Q and V bound the
    nominal endpoint norms for any permitted acceleration. Triangle bounds on
    2 q·v + k1(q²-R²) then give this direction-independent conservative loss.
    """
    Q = np.linalg.norm(r) + dt*np.linalg.norm(velocity) + .5*dt*dt*acceleration_bound
    V = np.linalg.norm(velocity) + dt*acceleration_bound
    ep, ev = .5*dt*dt*uncertainty, dt*uncertainty
    return float(2*Q*ev + 2*V*ep + 2*ep*ev + k1*(2*Q*ep+ep*ep))


def domain_rows(q, v, cfg):
    """Current h and psi1, with labels and native physical units."""
    q, v = _state(q, v, cfg)
    lo, hi = virtual_box(cfg); k1 = cfg['safety']['hocbf_k1']
    rows = []
    for i in range(len(q)):
        for j in range(i):
            r, rv = q[i,:2]-q[j,:2], v[i,:2]-v[j,:2]
            h = float(r@r-pair_distance(cfg)**2)
            rows.append((f'pair_{cfg["names"][j]}_{cfg["names"][i]}',h,float(2*r@rv+k1*h),'m²','m²/s'))
        for k in range(3):
            for label,h,hd in (('lower',q[i,k]-lo[k],v[i,k]),('upper',hi[k]-q[i,k],-v[i,k])):
                rows.append((f'{cfg["names"][i]}_{label}_{k}',float(h),float(hd+k1*h),'m','m/s'))
        for n,o in enumerate(cfg['obstacles']):
            r, rv = q[i,:2]-o['center_xy'], v[i,:2]
            h = float(r@r-obstacle_radius(o,cfg)**2)
            rows.append((f'{cfg["names"][i]}_obstacle_{n}',h,float(2*r@rv+k1*h),'m²','m²/s'))
    return rows


def check_domain(q, v, cfg):
    """Reject an inadmissible initial/current reference state; no recovery reset."""
    for label,h,psi,hu,pu in domain_rows(q,v,cfg):
        # Tolerance is numerical only: 1e-9 in the stated native row units.
        if h < -1e-9 or psi < -1e-9:
            raise RuntimeError(f'HOCBF domain invalid: {label}, h={h:.9g} {hu}, psi1={psi:.9g} {pu}')


def predictive_rows(index, q, v, previous, cfg, pair_allocator=None):
    """Return unnormalized A,b,label rows enforcing next psi1 conservatively."""
    q,v = _state(q,v,cfg)
    previous = np.asarray(previous,float)
    if previous.shape != q.shape or not np.isfinite(previous).all():
        raise ValueError('Predictive barriers require finite common-cycle acceleration anchors')
    dt = cfg['control']['dt_s']; k1 = cfg['safety']['hocbf_k1']
    amax = np.sqrt(2)*cfg['control']['maximum_reference_acceleration_mps2']
    uncertainty = cfg['safety']['virtual_acceleration_uncertainty_mps2']
    rows=[]
    for j in range(len(q)):
        if j==index:continue
        r=q[index,:2]-q[j,:2]; rv=v[index,:2]-v[j,:2]
        anchor=previous[index,:2]-previous[j,:2]
        F,g=squared_psi1(r,rv,anchor,pair_distance(cfg),dt,k1)
        reserve=squared_reserve(r,rv,dt,k1,2*amax,2*uncertainty)
        joint=F-g@anchor-reserve
        g=np.r_[g,0.]
        # Both agents compute the same affine joint inequality with reversed g.
        # Their rows sum to -g·a_i + g·a_j <= joint, around one common anchor.
        own,peer=-g@previous[index],g@previous[j]
        label=f'next_pair_{cfg["names"][j]}'
        rhs=(pair_allocator(index,j,-g,joint,label) if pair_allocator is not None
             else float(own+.5*(joint-own-peer)))
        rows.append((-g,rhs,label))
    lo,hi=virtual_box(cfg)
    factor=dt+.5*k1*dt*dt
    reserve=dt*uncertainty+.5*k1*dt*dt*uncertainty # m/s
    for k in range(3):
        unit=np.eye(3)[k]
        upper=-v[index,k]+k1*(hi[k]-q[index,k]-dt*v[index,k])-reserve
        lower=v[index,k]+k1*(q[index,k]+dt*v[index,k]-lo[k])-reserve
        rows.extend(((factor*unit,float(upper),f'next_upper_{k}'),(-factor*unit,float(lower),f'next_lower_{k}')))
    for j,o in enumerate(cfg['obstacles']):
        r=q[index,:2]-o['center_xy']; rv=v[index,:2]
        F,g=squared_psi1(r,rv,previous[index,:2],obstacle_radius(o,cfg),dt,k1)
        reserve=squared_reserve(r,rv,dt,k1,amax,uncertainty)
        rhs=float(F-g@previous[index,:2]-reserve)
        rows.append((-np.r_[g,0.],rhs,f'next_obstacle_{j}'))
    return rows


def check_next_domain(q, v, u, cfg):
    """Reverify exact endpoint psi1 and one-hold acceleration-error reserves."""
    q,v=_state(q,v,cfg);u=np.asarray(u,float)
    if u.shape!=q.shape or not np.isfinite(u).all():
        raise ValueError('Predictive barriers require finite common-cycle accelerations')
    dt=cfg['control']['dt_s'];k1=cfg['safety']['hocbf_k1']
    amax=np.sqrt(2)*cfg['control']['maximum_reference_acceleration_mps2']
    uncertainty=cfg['safety']['virtual_acceleration_uncertainty_mps2']
    qn,vn=q+dt*v+.5*dt*dt*u,v+dt*u
    check_domain(qn,vn,cfg)
    for i in range(len(q)):
        for j in range(i):
            r,rv=q[i,:2]-q[j,:2],v[i,:2]-v[j,:2]
            F,_=squared_psi1(r,rv,u[i,:2]-u[j,:2],pair_distance(cfg),dt,k1)
            reserve=squared_reserve(r,rv,dt,k1,2*amax,2*uncertainty)
            if F<reserve-1e-9:raise RuntimeError('Next pair psi1 uncertainty reserve violated')
        lo,hi=virtual_box(cfg);reserve=dt*uncertainty+.5*k1*dt*dt*uncertainty
        if np.any(-vn[i]+k1*(hi-qn[i])<reserve-1e-9) or np.any(vn[i]+k1*(qn[i]-lo)<reserve-1e-9):
            raise RuntimeError('Next arena psi1 uncertainty reserve violated')
        for o in cfg['obstacles']:
            r,rv=q[i,:2]-o['center_xy'],v[i,:2]
            F,_=squared_psi1(r,rv,u[i,:2],obstacle_radius(o,cfg),dt,k1)
            reserve=squared_reserve(r,rv,dt,k1,amax,uncertainty)
            if F<reserve-1e-9:raise RuntimeError('Next obstacle psi1 uncertainty reserve violated')

"""Optional reciprocal pair allocation from bounded static-control authority.

Each node builds the same per-agent 2D polytopes from one common snapshot.
No joint QP, external LP solver, safety slack or constraint removal is used.
Supports cover static obstacles/arena and hard command bounds, but do not
include other pair rows; simultaneous multi-pair conflicts remain possible.
"""
import numpy as np

from .qp_priority import velocity_slew_bounds
from .barrier_domain import enabled as predictive_enabled


def enabled(cfg):
    return cfg.get('avoidance',{}).get('adaptive_pair_responsibility',False) is True


def vertices(A,b,lower,upper):
    """Enumerate all vertices of a finite bounded XY half-space intersection."""
    A,b,lower,upper=(np.asarray(x,float) for x in (A,b,lower,upper))
    if A.size==0:A=np.empty((0,2))
    if A.ndim!=2 or A.shape[1]!=2 or b.shape!=(len(A),) or lower.shape!=(2,) or upper.shape!=(2,):
        raise ValueError('Static authority requires XY rows and two component bounds')
    if not all(np.isfinite(x).all() for x in (A,b,lower,upper)):
        raise ValueError('Static authority requires finite rows and bounds')
    if np.any(lower>upper):raise RuntimeError('Empty static feasible control polytope: command bounds')
    C=np.vstack((A,np.eye(2),-np.eye(2)));d=np.r_[b,upper,-lower]
    first,second=np.triu_indices(len(C),1)
    a0,a1=C[first],C[second];b0,b1=d[first],d[second]
    det=a0[:,0]*a1[:,1]-a0[:,1]*a1[:,0]
    independent=np.abs(det)>1e-12
    a0,a1,b0,b1,det=(x[independent] for x in (a0,a1,b0,b1,det))
    points=np.column_stack(((b0*a1[:,1]-a0[:,1]*b1)/det,(a0[:,0]*b1-b0*a1[:,0])/det))
    feasible=np.isfinite(points).all(axis=1)&np.all(C@points.T<=d[:,None]+1e-9,axis=0)
    if not np.any(feasible):raise RuntimeError('Empty static feasible control polytope: static rows')
    return points[feasible]


def hard_bounds(velocity,previous,cfg):
    c=cfg['control'];dt=c['dt_s'];limit=c['maximum_reference_acceleration_mps2'];vmax=c['maximum_reference_speed_mps']
    velocity,previous=np.asarray(velocity,float),np.asarray(previous,float)
    if velocity.shape!=(3,) or previous.shape!=(3,) or not np.isfinite(velocity).all() or not np.isfinite(previous).all():
        raise ValueError('Static authority requires finite velocity and previous acceleration')
    J=(c.get('maximum_safety_acceleration_slew_mps3',c['maximum_acceleration_slew_mps3'])
       if c.get('allow_slew_relaxation_for_safety',False) else c['maximum_acceleration_slew_mps3'])
    lower=np.maximum(np.full(3,-limit),(-vmax-velocity)/dt)
    upper=np.minimum(np.full(3,limit),(vmax-velocity)/dt)
    lower=np.maximum(lower,previous-J*dt);upper=np.minimum(upper,previous+J*dt)
    if predictive_enabled(cfg):
        viable_lower,viable_upper=velocity_slew_bounds(velocity,c)
        lower=np.maximum(lower,viable_lower);upper=np.minimum(upper,viable_upper)
    return lower,upper


class Allocator:
    """Support minima for static rows, already normalized and reserved by caller."""
    def __init__(self,static_rows,velocity,previous,cfg):
        self.points=[];self.reserve=cfg['safety']['row_reserve_mps2'];self.allocations=[];self.names=cfg['names']
        for index,(A,b) in enumerate(static_rows):
            A,b=np.asarray(A,float),np.asarray(b,float)
            if A.ndim!=2 or A.shape[1]!=3 or b.shape!=(len(A),) or not np.isfinite(A).all() or not np.isfinite(b).all():
                raise ValueError('Static authority requires finite 3D static rows')
            lower,upper=hard_bounds(velocity[index],previous[index],cfg)
            xy_rows=[];xy_rhs=[]
            for row,rhs in zip(A,b):
                if abs(row[2])>1e-12:
                    if np.linalg.norm(row[:2])>1e-12:
                        raise ValueError('Static authority requires separable XY and height rows')
                    if row[2]>0:upper[2]=min(upper[2],rhs/row[2])
                    else:lower[2]=max(lower[2],rhs/row[2])
                else:xy_rows.append(row[:2]);xy_rhs.append(rhs)
            if np.any(lower>upper):
                raise RuntimeError(f'{cfg["names"][index]}: empty static feasible control polytope')
            try:self.points.append(vertices(xy_rows,xy_rhs,lower[:2],upper[:2]))
            except RuntimeError as exc:raise RuntimeError(cfg['names'][index]+': '+str(exc)) from exc

    def minimum(self,index,direction):
        direction=np.asarray(direction,float)
        if direction.shape!=(3,) or not np.isfinite(direction).all() or abs(direction[2])>1e-12:
            raise ValueError('Pair authority requires a finite XY-only direction')
        return float(np.min(self.points[index]@direction[:2]))

    def allocate(self,index,peer,direction,joint,label='pair'):
        """Return raw own RHS; opposite agent receives its complementary share.

        For A_i u_i + A_j u_j <= B and m_k=min A_k u_k over each static
        polytope, b_i=m_i+(B-m_i-m_j)/2. Thus b_i+b_j=B exactly. Both row
        reserves are applied afterward, preserving the existing stronger joint
        inequality rather than paying for more authority with weaker safety.
        """
        direction=np.asarray(direction,float)
        if not np.isfinite(joint):raise ValueError('Pair authority requires finite joint bound')
        own=self.minimum(index,direction);other=self.minimum(peer,-direction)
        slack=float(joint-own-other)
        combined_reserve=2*max(float(np.linalg.norm(direction)),1e-12)*self.reserve
        if slack<combined_reserve-1e-10:
            raise RuntimeError(f'{label}: incompatible static pair authority; joint slack={slack:.9g}, required reserves={combined_reserve:.9g}')
        rhs=float(own+.5*slack)
        self.allocations.append(dict(agent=self.names[index],peer=self.names[peer],label=label,
            direction=direction.tolist(),joint_bound_raw=float(joint),own_static_minimum_raw=own,
            peer_static_minimum_raw=other,combined_slack_raw=slack,own_bound_raw=rhs,
            own_effective_bound_raw=rhs-.5*combined_reserve,
            raw_unit='m²/s' if label.startswith('next_') else 'm²/s²'))
        return rhs

"""Deterministic local passing guidance upstream of the hard safety QP.

This is a bounded nominal-velocity modification, not a general route planner
or a safety certificate. Side choices are latched per encounter. Every command
still passes the same local/joint HOCBF, actuation and held-path checks.
"""
import numpy as np
from .settings import pair_distance, virtual_box
from .obstacles import obstacle_radius


DEFAULTS = dict(enabled=False, predictive_barriers=False, adaptive_pair_responsibility=False, route_guidance=False, lookahead_s=3.,
    activation_margin_m=.035, release_margin_m=.06, orbit_margin_m=.02,
    passing_speed_mps=.13, radial_gain_per_s=.7, blend_time_s=.25,
    active_progress_scale=.65, lag_slowdown_m=.12, lag_stop_m=.30,
    minimum_progress_scale=.1, stall_window_s=12., stall_motion_m=.04,
    stall_trigger_s=.6, stall_speed_mps=.035, stall_target_error_m=.10,
    route_replan_s=.5, route_margin_m=.025, route_wall_margin_m=.012,
    route_circle_samples=24, route_max_nodes=256, route_waypoint_tolerance_m=.08,
    route_goal_change_m=.10)


def options(cfg):
    return {**DEFAULTS, **cfg.get('avoidance', {})}


def norm_limit(value, limit):
    value=np.asarray(value,float);return value*min(1.,limit/max(float(np.linalg.norm(value)),1e-12))


def closest(r, velocity, horizon):
    t=float(np.clip(-r@velocity/max(float(velocity@velocity),1e-12),0.,horizon))
    return float(np.linalg.norm(r+t*velocity)),t


def motion_progress(results, cfg):
    """Common next-cycle progress factor; no estimator/filter state reset."""
    a=options(cfg)
    if not a['enabled'] or not results:return 1.
    factor=1.
    for row in results:
        report=row.get('avoidance',{})
        if report.get('active'):
            factor=min(factor,a['active_progress_scale']);continue
        lag=float(report.get('target_error_m',0.))
        fraction=np.clip((lag-a['lag_slowdown_m'])/(a['lag_stop_m']-a['lag_slowdown_m']),0.,1.)
        factor=min(factor,1.-fraction*(1.-a['minimum_progress_scale']))
    return float(factor)


class PassingGuide:
    def __init__(self,cfg,index,initial_positions=None,final_goals=None):
        self.cfg,self.index,self.a=cfg,index,options(cfg)
        self.sides={};self.encounters=set();self.blend=0.;self.last_velocity=np.zeros(2);self.blocked_time=0.
        self.side_geometry=None if initial_positions is None else np.asarray(initial_positions,float).copy()
        self.final_goals=None if final_goals is None else np.asarray(final_goals,float).copy()
        from .route_guidance import VisibilityRoute
        self.route=VisibilityRoute(cfg,index,self.a)

    def clearance(self,points,excluded=()):
        lo,hi=virtual_box(self.cfg);score=float(np.min(np.r_[np.asarray(points)-lo[:2],hi[:2]-np.asarray(points)]))
        for o in self.cfg['obstacles']:
            if o['name'] in excluded:continue
            score=min(score,float(np.min(np.linalg.norm(np.asarray(points)-o['center_xy'],axis=1)))-obstacle_radius(o,self.cfg))
        return score

    def pair_side(self,i,j,q):
        key=('pair',min(i,j),max(i,j))
        if key not in self.sides:
            q=q if self.side_geometry is None else self.side_geometry
            low,high=key[1:];r=q[low,:2]-q[high,:2];t=np.array([-r[1],r[0]])/max(np.linalg.norm(r),1e-12)
            width=.5*pair_distance(self.cfg)+self.a['orbit_margin_m']
            scores=[]
            for sign in (1.,-1.):
                points=np.array([q[low,:2]+sign*width*t,q[high,:2]-sign*width*t])
                score=self.clearance(points)
                for k in range(len(q)):
                    if k not in (low,high):score=min(score,float(np.min(np.linalg.norm(points-q[k,:2],axis=1)))-pair_distance(self.cfg))
                if self.final_goals is not None:
                    desired=(self.final_goals[low,:2]-q[low,:2])-(self.final_goals[high,:2]-q[high,:2])
                    score+=.4*float(sign*t@desired)/max(float(np.linalg.norm(desired)),1e-12)
                scores.append(score)
            self.sides[key]=1. if scores[0]>=scores[1]-1e-9 else -1.
        return key,self.sides[key]

    def apply(self,nominal,target,desired_velocity,q,v,stage):
        a=self.a;i=self.index;nominal=np.asarray(nominal,float).copy()
        report=dict(active=False,blend=0.,target_error_m=float(np.linalg.norm(target-q[i])),
                    constraints_softened=False,hazards=[],method='deterministic_local_circulation')
        if not a['enabled'] or stage not in ('FORMATION','SETTLE','FORWARD','STOP'):
            return nominal,report
        if a['route_guidance']:
            route_velocity,route=self.route.velocity(target,q)
            report['route']=route
            if route.get('status')=='clear':
                # Recover the actual current REDCHO goal, including FORWARD
                # and STOP translation, as soon as its whole segment is clear.
                # Keep the fallback encounter memory across route handoffs.
                # Dropping it can restart/reverse a partially completed pass
                # when a temporarily occupied goal becomes blocked again.
                report.update(method='visibility_waypoints',route_status='clear')
                return nominal,report
            if route_velocity is not None:
                original=nominal.copy();nominal[:2]=self.cfg['control']['velocity_gain']*(route_velocity-v[i,:2])
                # Routing owns this cycle's nominal input, while the latched
                # fallback side/blend/stall context remains available intact.
                report.update(active=True,blend=1.,method='visibility_waypoints',route_status=route['status'],
                    hazards=route.get('hazards',[]),guided_velocity_mps=route_velocity.tolist(),
                    nominal_correction_mps2=(nominal-original).tolist(),unguided_acceleration_mps2=original.tolist())
                return nominal,report
            report.update(route_status='blocked',fallback='deterministic_local_circulation')
        error=np.asarray(target[:2])-q[i,:2];distance=float(np.linalg.norm(error))
        stalled=np.linalg.norm(v[i,:2])<a['stall_speed_mps'] and distance>a['stall_target_error_m']
        self.blocked_time=self.blocked_time+self.cfg['control']['dt_s'] if stalled else 0.
        armed=self.blocked_time>=a['stall_trigger_s'] or bool(self.encounters)
        speed=min(a['passing_speed_mps'],max(float(np.linalg.norm(desired_velocity[:2])),self.cfg['control']['position_gain']*distance))
        aim=speed*error/max(distance,1e-12) if distance>1e-6 else np.asarray(desired_velocity[:2],float)
        hazards=[];current=set()
        for j in range(len(q)):
            if i==j:continue
            r=q[i,:2]-q[j,:2];relative=aim-v[j,:2];d,t=closest(r,relative,a['lookahead_s'])
            key=('pair',min(i,j),max(i,j));margin=a['release_margin_m'] if key in self.encounters else a['activation_margin_m']
            radius=pair_distance(self.cfg)
            if t>1e-5 and d<radius+margin and np.linalg.norm(aim)>1e-5:
                key,side=self.pair_side(i,j,q);current.add(key)
                hazards.append((d-radius,r,radius,side,v[j,:2],key))
        for o in self.cfg['obstacles']:
            r=q[i,:2]-o['center_xy'];d,t=closest(r,aim,a['lookahead_s']);key=('obstacle',o['name'])
            radius=obstacle_radius(o,self.cfg);margin=a['release_margin_m'] if key in self.encounters else a['activation_margin_m']
            if t>1e-5 and d<radius+margin and np.linalg.norm(aim)>1e-5:
                if key not in self.sides:
                    tangent=np.array([-r[1],r[0]])/max(np.linalg.norm(r),1e-12)
                    # Favor a passing side pointing toward the target; break symmetry
                    # using available arena/obstacle clearance, then a fixed sign.
                    scores=[self.clearance([q[i,:2]+s*.45*tangent],(o['name'],))+.1*float(s*tangent@aim) for s in (1.,-1.)]
                    self.sides[key]=1. if scores[0]>=scores[1]-1e-9 else -1.
                current.add(key);hazards.append((d-radius,r,radius,self.sides[key],np.zeros(2),key))
        if not armed:hazards=[];current=set()
        self.encounters=current
        if hazards:
            _,r,radius,side,peer_velocity,key=min(hazards,key=lambda h:(h[0],str(h[5])))
            normal=r/max(np.linalg.norm(r),1e-12);tangent=side*np.array([-normal[1],normal[0]])
            radial=np.clip(a['radial_gain_per_s']*(radius+a['orbit_margin_m']-np.linalg.norm(r)),0.,.5*a['passing_speed_mps'])
            # A common pair chirality gives opposite tangential motions and breaks
            # a perfectly symmetric head-on stop without changing the QP rows.
            self.last_velocity=norm_limit(tangent*a['passing_speed_mps']+normal*radial,a['passing_speed_mps'])
            report.update(active=True,hazards=[str(h[5]) for h in hazards],selected=str(key),passing_side=side,
                          guided_velocity_mps=self.last_velocity.tolist())
        target_blend=1. if hazards else 0.
        self.blend+=(self.cfg['control']['dt_s']/(a['blend_time_s']+self.cfg['control']['dt_s']))*(target_blend-self.blend)
        if self.blend<1e-4:self.blend=0.
        guided=self.cfg['control']['velocity_gain']*(self.last_velocity-v[i,:2])
        original=nominal.copy();nominal[:2]=(1.-self.blend)*nominal[:2]+self.blend*guided
        report.update(blend=float(self.blend),blocked_time_s=float(self.blocked_time),nominal_correction_mps2=(nominal-original).tolist(),
                      unguided_acceleration_mps2=original.tolist())
        return nominal,report

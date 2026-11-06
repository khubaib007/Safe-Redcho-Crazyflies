"""Single position REDCHO baseline, shared by ROS and numerical preview.

The sole order-2 bank reconstructs absolute position and its first two
derivatives. No independently estimated velocity signal enters tracking.
"""
import math
from collections import deque
import numpy as np
from .observers import Bank
from .cbf_qp import solve, validate_batch, joint_pair_margin
from .settings import NAMES, pair_distance
from .telemetry_trace import QPTraceError
from .avoidance import PassingGuide, options as avoidance_options

STAGES=('WARMUP','HEIGHT','YAW','FORMATION','SETTLE','FORWARD','STOP')
def vector(x): return dict(zip(('x','y','z'),np.asarray(x,float).tolist()))
def smooth(s): return s**3*(10-15*s+6*s*s)
def wrap(x): return (np.asarray(x)+np.pi)%(2*np.pi)-np.pi

class SVF:
    def __init__(self,dt,omega): self.dt,self.omega=dt,omega; self.state=np.zeros((3,4))
    def step(self,raw):
        w=self.omega
        def f(x):
            d=np.empty_like(x);d[:,:3]=x[:,1:];d[:,3]=-w**3*(x[:,1]-raw)-3*w*w*x[:,2]-3*w*x[:,3];return d
        self.state+=self.dt*f(self.state+.5*self.dt*f(self.state))
        return self.state.copy()

class Reference:
    def __init__(self,c,index,plan):
        self.c,self.i,self.plan=c,index,plan; self.phase=0.;self.forward_phase=0.;self.shape_phase=0.
        dt,w=c['control']['dt_s'],c['redcho']['svf_omega']
        self.acq,self.forward,self.shape=(SVF(dt,w) for _ in range(3))
        self.rho0=np.asarray(plan['hover'])[index]-np.asarray(plan['center']);self.rho0[2]=0.
        self.rho1=np.asarray(plan['offsets'])[index]
    def step(self,stage,voltages,progress_scale=1.):
        c=self.c;dt=c['control']['dt_s'];b=c['battery'];volts=np.asarray(voltages,float)
        if volts.shape!=(3,) or not np.isfinite(volts).all() or np.any(volts<=b['land_voltage_v']) or np.any(volts>4.5): raise RuntimeError('Missing/invalid/landing-threshold battery voltage')
        scales=np.clip((volts-b['land_voltage_v'])/(b['full_speed_voltage_v']-b['land_voltage_v']),b['minimum_speed_scale'],1.)
        if isinstance(progress_scale,bool) or not isinstance(progress_scale,(int,float)) or not np.isfinite(progress_scale) or not 0<=progress_scale<=1:raise ValueError('Invalid common motion progress scale')
        scale=float(scales[self.i])*progress_scale; raw=np.zeros(3);unscaled=np.zeros(3)
        if stage=='FORMATION':
            old=self.phase;self.phase=min(1.,old+dt*scale/c['acquisition']['nominal_duration_s'])
            delta=np.asarray(self.plan['delta']);raw=delta*(smooth(self.phase)-smooth(old))/dt
            unscaled=delta*(smooth(min(1.,old+dt/c['acquisition']['nominal_duration_s']))-smooth(old))/dt
        ast=self.acq.step(raw)
        fraw=np.zeros(3)
        if stage=='FORWARD':
            old=self.forward_phase;self.forward_phase=min(1.,old+dt*scale/c['mission']['nominal_forward_duration_s'])
            fraw[0]=c['mission']['distance_m']*(smooth(self.forward_phase)-smooth(old))/dt
            unscaled[0]=c['mission']['distance_m']*(smooth(min(1.,old+dt/c['mission']['nominal_forward_duration_s']))-smooth(old))/dt
        fst=self.forward.step(fraw)
        # Shared bounded shape clock uses the weakest voltage scale. Shape
        # motion stays per-aircraft; it is NOT averaged into a common velocity.
        sraw=np.zeros(3)
        if stage=='FORMATION':
            old=self.shape_phase;self.shape_phase=min(1.,old+dt*float(min(scales))*progress_scale/c['acquisition']['nominal_duration_s'])
            sraw[0]=(smooth(self.shape_phase)-smooth(old))/dt
        shape=self.shape.step(sraw)[0,:3]
        state=ast+fst
        velocity=state[:,1:].copy();position=state[:,:3].copy();position[:,0]+=np.asarray(self.plan.get('proposals',self.plan['hover']))[self.i]
        r=self.c['redcho'];coeff=np.poly(-np.asarray(r['gamma']))[1:][::-1]
        pb=float(np.linalg.norm(state[:,3]+position@coeff))
        if pb>r['position_L']/2: raise RuntimeError('Reference exceeded position REDCHO regularity budget')
        dr=self.rho1-self.rho0;rho=self.rho0+dr*shape[0];rd=dr*shape[1];rdd=dr*shape[2]
        trace={'governor':{'input_voltage_v':float(volts[self.i]),'speed_scale':float(scales[self.i]),'motion_progress_scale':float(progress_scale),'fleet_shape_scale':float(min(scales)),
            'input_unscaled_velocity_mps':vector(unscaled),'output_velocity_mps':vector(raw+fraw),'phase_after':self.phase,'forward_phase':self.forward_phase},
            'svf':{'input_velocity_mps':vector(raw+fraw),'output_displacement_m':vector(state[:,0]),'output_velocity_mps':vector(state[:,1]),
            'output_acceleration_mps2':vector(state[:,2]),'output_jerk_mps3':vector(state[:,3])},
            'formation_shape':{'progress':float(shape[0]),'offset_m':vector(rho),'velocity_mps':vector(rd),'acceleration_mps2':vector(rdd)}}
        return velocity,position,rho,rd,rdd,trace

class LocalController:
    def __init__(self,c,index,plan,trace=True):
        self.c,self.i,self.plan,self.trace_enabled=c,index,plan,trace
        r,dt=c['redcho'],c['control']['dt_s'];self.position=Bank(r,r['position_L'],dt)
        self.reference=Reference(c,index,plan);self.sequence=0;self.prepared=False;self.neighbors=tuple(n for n in NAMES if n!=NAMES[index])
        self.guide=PassingGuide(c,index,plan['hover'],np.asarray(plan['target_center'])+np.asarray(plan['offsets']))
    def prepare(self,frame):
        if self.prepared or frame['seq']!=self.sequence+1 or frame['stage'] not in STAGES: raise ValueError('Controller sequence/stage violation')
        self.frame=frame;z,p,self.rho,self.rd,self.rdd,self.trace=self.reference.step(frame['stage'],frame['voltage'],frame.get('motion_progress_scale',1.))
        self.p=p;self.py=self.position.prepare(p);self.prepared=True
        return {'position':self.py[:,0].tolist()}
    def finish(self,peers):
        if not self.prepared or set(peers)!=set(self.neighbors): raise ValueError('Require both same-cycle peers')
        if any(set(peers[n]) - {'position','name','session','seq'} or 'position' not in peers[n] for n in peers):
            raise ValueError('Single-bank peers must carry only the position estimate and identity')
        self.position.finish({n:peers[n]['position'] for n in peers})
        q,v,previous=(np.asarray(self.frame[k],float) for k in ('q','v','previous'));stage=self.frame['stage']
        pd=np.asarray(self.plan['hover'])[self.i].copy();vd=np.zeros(3);ad=np.zeros(3)
        if stage!='WARMUP': pd[2]=self.py[2,0];vd[2]=self.py[2,1];ad[2]=self.py[2,2]
        if stage in ('FORMATION','SETTLE','FORWARD','STOP'):
            pd[:2]=self.py[:2,0]+self.rho[:2];vd[:2]=self.py[:2,1]+self.rd[:2];ad[:2]=self.py[:2,2]+self.rdd[:2]
        ctrl=self.c['control'];nom=ad+ctrl['position_gain']*(pd-q[self.i])+ctrl['velocity_gain']*(vd-v[self.i])
        nom,avoidance=self.guide.apply(nom,pd,vd,q,v,stage);self.trace['avoidance']=avoidance
        try:u,info=solve(self.i,nom,q,v,previous,self.c,diagnostics=self.trace_enabled)
        except QPTraceError as e:
            self.trace['qp']=e.diagnostics
            raise
        if self.trace_enabled:
            def bank(inp,out,labels): return {'input':{k:vector(inp[:,j]) for j,k in enumerate(labels)},'output':{k:vector(out[:,j]) for j,k in enumerate(labels)}}
            self.trace['architecture']={'name':'single_position_redcho','bank_count':1,
                'order':2,'motion_derivative_source':'position_redcho.output'}
            self.trace['position_redcho']=bank(self.p,self.py,('position_m','velocity_mps','acceleration_mps2'))
            self.trace['position_redcho']['position_semantics']='absolute world center and height proposal'
            self.trace['nominal']={'desired_position_m':pd.tolist(),'desired_velocity_mps':vd.tolist(),'desired_acceleration_mps2':ad.tolist(),
                'virtual_position_m':q[self.i].tolist(),'virtual_velocity_mps':v[self.i].tolist(),'u_nom_mps2':nom.tolist(),
                'position_error_m':(pd-q[self.i]).tolist(),'velocity_error_mps':(vd-v[self.i]).tolist(),
                'position_feedback_mps2':(ctrl['position_gain']*(pd-q[self.i])).tolist(),
                'velocity_feedback_mps2':(ctrl['velocity_gain']*(vd-v[self.i])).tolist(),
                'position_gain':ctrl['position_gain'],'velocity_gain':ctrl['velocity_gain']}
            self.trace['qp']=info.pop('diagnostics')
        self.sequence=self.frame['seq'];self.prepared=False
        return dict(u=u.tolist(),target=pd.tolist(),avoidance=avoidance,motion_derivatives=self.py[:,1:].tolist(),position_estimate=self.py.tolist(),
                    local_phase=self.reference.phase,forward_phase=self.reference.forward_phase,shape_phase=self.reference.shape_phase,
                    beta=self.reference.shape.state[0,0],position_input=self.p.tolist(),
                    battery_scale=self.trace['governor']['speed_scale'],**info,diagnostics=self.trace if self.trace_enabled else {})

def accept_batch(q,v,previous,u,c):
    if any(np.asarray(x).shape!=(3,3) or not np.isfinite(x).all() for x in (q,v,previous,u)):
        raise RuntimeError('Reject nonfinite or incomplete reference batch')
    if joint_pair_margin(q,v,u,c)<-10*c['control']['qp_tolerance']: raise RuntimeError('Joint reciprocal HOCBF validation failed')
    return validate_batch(q,v,previous,u,c)

class StageMachine:
    """Identical measured convergence gates in hardware and the synthetic preview."""
    def __init__(self,c,plan,acquire_only=False):
        self.c,self.plan,self.acquire_only=c,plan,acquire_only;self.stage='WARMUP';self.elapsed=0.;self.total=0.;self.good=0.;self.yaw=np.asarray(plan['initial_yaw']);self.events=[(0.,'WARMUP')]
        self.motion_history=deque();self.avoidance=avoidance_options(c)
    def goal(self,forward=False):
        p=np.asarray(self.plan['target_center'])+np.asarray(self.plan['offsets'])
        if forward:p=p+[self.c['mission']['distance_m'],0,0]
        return p
    def yaw_reference(self):
        if self.stage in ('WARMUP','HEIGHT'):return self.yaw.copy()
        if self.stage=='YAW':
            duration=max(self.c['acquisition']['yaw_duration_s'],1.875*float(np.max(np.abs(wrap(-self.yaw))))/self.c['acquisition']['yaw_rate_limit_radps'])
            return self.yaw+wrap(-self.yaw)*smooth(min(1.,self.elapsed/duration))
        return np.zeros(3)
    def advance(self,results,p,measured_v,yaw):
        c=self.c;dt=c['control']['dt_s'];a=c['acquisition'];m=c['mission'];self.elapsed+=dt;self.total+=dt
        if self.total>m['maximum_wall_duration_s']:raise RuntimeError('Mission logical time budget exceeded')
        p,v,yaw=np.asarray(p),np.asarray(measured_v),np.asarray(yaw)
        slow=np.max(np.linalg.norm(v,axis=1))<m['endpoint_speed_mps']
        outs=np.array([r['position_estimate'] for r in results])[:,:,0];ins=np.array([r['position_input'] for r in results])[:,:,0]
        agree=max(np.linalg.norm(outs[i]-outs[j]) for i in range(3) for j in range(i))<a['observer_agreement_m']
        reconstruct=np.max(np.linalg.norm(outs-ins.mean(axis=0),axis=1))<a['observer_reconstruction_m']
        hgood=np.max(np.abs(p[:,2]-self.plan['target_center'][2]))<m['endpoint_tolerance_m'] and np.ptp(p[:,2])<m['height_spread_tolerance_m']
        endpoint=np.max(np.linalg.norm(p-self.goal(self.stage=='STOP' and not self.acquire_only),axis=1))<m['endpoint_tolerance_m']
        yawgood=np.max(np.abs(wrap(yaw)))<a['yaw_tolerance_rad']
        if self.avoidance['enabled'] and self.stage in ('FORMATION','SETTLE','FORWARD','STOP'):
            history=self.motion_history;history.append((self.total,p.copy()))
            while len(history)>1 and self.total-history[1][0]>=self.avoidance['stall_window_s']:history.popleft()
            if not endpoint and self.total-history[0][0]>=self.avoidance['stall_window_s']:
                travel=max(float(np.max(np.linalg.norm(row[1]-history[0][1],axis=1))) for row in history)
                if travel<self.avoidance['stall_motion_m']:raise RuntimeError('Avoidance stalled: no measured motion toward a valid formation/transport goal; inspect blocked routes')
        complete=False;next_stage=None;good=False;dwell=a['settle_dwell_s']
        if self.stage=='WARMUP':
            good=agree and reconstruct and self.elapsed>=m['observer_warmup_s']
            if self.elapsed>m['observer_warmup_s']+12: raise RuntimeError('REDCHO warmup did not converge')
            next_stage='HEIGHT'
        elif self.stage=='HEIGHT':
            good=hgood and slow and agree and reconstruct;next_stage='YAW'
            if self.elapsed>a['height_timeout_s']:raise RuntimeError('Measured height consensus did not settle')
        elif self.stage=='YAW':
            good=yawgood and slow and hgood and np.max(np.abs(wrap(self.yaw_reference())))<.001;next_stage='FORMATION'
            if self.elapsed>a['yaw_timeout_s']:raise RuntimeError('Measured heading did not align')
        elif self.stage=='FORMATION':
            good=all(r['local_phase']>=1 and r['shape_phase']>=1 for r in results);next_stage='SETTLE';dwell=dt
            if self.elapsed>a['stage_timeout_s']:raise RuntimeError('Acquisition phases timed out')
        elif self.stage=='SETTLE':
            good=endpoint and hgood and yawgood and slow and agree and reconstruct;next_stage='STOP' if self.acquire_only else 'FORWARD';dwell=m['endpoint_dwell_s']
            if self.elapsed>a['stage_timeout_s']:raise RuntimeError('Formation acquisition did not reach measured endpoints; possible blocked goals')
        elif self.stage=='FORWARD':
            good=all(r['forward_phase']>=1 for r in results);next_stage='STOP';dwell=dt
            timeout=m['nominal_forward_duration_s']/c['battery']['minimum_speed_scale']+15
            if self.avoidance['enabled'] and self.avoidance['route_guidance']:
                # Detours deliberately slow the common phase clock. Allow the
                # configured bounded route stage budget, while stall checks
                # and the overall mission time budget remain in force.
                timeout=max(timeout,a['stage_timeout_s'])
            if self.elapsed>timeout:raise RuntimeError('Forward phase timed out')
        elif self.stage=='STOP':
            good=endpoint and hgood and yawgood and slow;dwell=m['endpoint_dwell_s']+m['final_hold_s'];complete=True
            if self.elapsed>a['stage_timeout_s']:raise RuntimeError('Final measured hold timed out')
        self.good=self.good+dt if good else 0.
        if self.good>=dwell:
            if complete:return True
            self.stage=next_stage;self.elapsed=0.;self.good=0.;self.motion_history.clear();self.events.append((self.total,self.stage))
        return False

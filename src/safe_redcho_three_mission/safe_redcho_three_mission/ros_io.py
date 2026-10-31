"""Single-reception ROS state adapter and selective aircraft service access."""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import Status
from crazyflie_interfaces.srv import Arm,Takeoff,Land,NotifySetpointsStop
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from .health import History,pose_arrays,yaw,tilt,rotation_error
from .settings import NAMES

def stamp(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

class StateNode(Node):
    def __init__(self,c,name,onboard=True):
        super().__init__(name);self.c=c;self.mc={n:History('/poses['+n+']') for n in NAMES};self.est={n:History('/'+n+'/pose') for n in NAMES};self.batteries={};self.measurement_fault=None;self.rejected={};self.last_fault=None
        self.obstacle_histories={o['name']:History('/poses['+o['name']+']') for o in c.get('obstacles',[])}
        self.obstacle_evidence={}
        self._state_receiver=None;self._onboard_input={};self._direct_input={}
        if onboard and c.get('backend',{}).get('state_receiver_thread',False):
            from .state_receiver import StateReceiver
            specs=[('mocap',NamedPoseArray,'/poses',256)]
            for n in NAMES:
                specs.extend([('pose/'+n,PoseStamped,'/'+n+'/pose',64),
                              ('status/'+n,Status,'/'+n+'/status',16)])
            self._state_receiver=StateReceiver(self,name+'_state_rx',specs)
        else:
            self.create_subscription(NamedPoseArray,'/poses',self.mocap,qos_profile_sensor_data)
        if onboard and self._state_receiver is None:
            for n in NAMES:
                self.create_subscription(PoseStamped,'/'+n+'/pose',lambda m,n=n:self.onboard(m,n),qos_profile_sensor_data)
                self.create_subscription(Status,'/'+n+'/status',lambda m,n=n:self.battery(m,n),qos_profile_sensor_data)
    def clocks(self):return self.get_clock().now().nanoseconds*1e-9,time.monotonic()
    def reject(self,key):self.rejected[key]=self.rejected.get(key,0)+1
    def _drain_state(self):
        rx=self._state_receiver
        if rx is None:return
        for key,msg,ros_now,mono in rx.drain():
            received=(ros_now,mono)
            if key=='mocap':self.mocap(msg,received)
            elif key.startswith('pose/'):self.onboard(msg,key[5:],received)
            else:self.battery(msg,key[7:],received)
    def destroy_node(self):
        if self._state_receiver is not None:
            self._state_receiver.close();self._state_receiver=None
        return super().destroy_node()
    def mocap(self,msg,received=None):
        if msg.header.frame_id!=self.c['frame_id']:self.measurement_fault='Mocap world frame changed';return
        now,mono=self.clocks() if received is None else received;names=[e.name for e in msg.poses]
        self._direct_input['mocap']=(now,mono)
        for n in list(self.mc)+list(self.obstacle_histories):
            if names.count(n)>1:self.measurement_fault='Duplicate mocap body '+n
        for e in msg.poses:
            histories=self.mc if e.name in self.mc else self.obstacle_histories
            if e.name not in histories or names.count(e.name)!=1:continue
            try:
                p,q=pose_arrays(e.pose)
                if not histories[e.name].add(p,q,stamp(msg),now,mono,self.c['safety']['mocap_maximum_age_s']):self.reject('rejected_mocap_'+e.name)
            except ValueError:self.reject('invalid_mocap_'+e.name)
    def onboard(self,msg,n,received=None):
        now,mono=self.clocks() if received is None else received
        previous=self._onboard_input.get(n,{})
        self._onboard_input[n]=dict(received=previous.get('received',0)+1,
            last_ros=now,last_mono=mono,last_header_stamp=stamp(msg),
            maximum_callback_gap_s=max(previous.get('maximum_callback_gap_s',0.),
                mono-previous['last_mono'] if 'last_mono' in previous else 0.))
        if msg.header.frame_id!=self.c['frame_id']:self.reject('onboard_frame_'+n);return
        try:
            p,q=pose_arrays(msg.pose)
            if not self.est[n].add(p,q,stamp(msg),now,mono,self.c['safety']['maximum_estimator_age_s'],True):
                self.reject('rejected_onboard_'+n)
        except ValueError:self.reject('invalid_onboard_'+n)
    def battery(self,msg,n,received=None):
        v=float(msg.battery_voltage)
        if np.isfinite(v) and 2.5<v<4.5:self.batteries[n]=(time.monotonic() if received is None else received[1],v)
    def measurement_diagnostics(self):
        now,mono=self.clocks()
        def describe(h):
            row=h.rows[-1] if h.rows else None;rate,coverage=h.stats()
            return dict(source_age_s=now-row[0] if row else None,
                receipt_age_s=mono-row[1] if row else None,rate_hz=rate,coverage_s=coverage,
                rejected=h.rejected,last_rejection=getattr(h,'last_rejection',None),
                timestamp_basis=h.basis)
        inputs={n:{**s,'callback_age_s':mono-s['last_mono']} for n,s in self._onboard_input.items()}
        return dict(receiver=self._state_receiver.report(mono) if self._state_receiver else dict(mode='coordinator_executor'),
                    mocap={n:describe(h) for n,h in self.mc.items()},
                    onboard={n:describe(h) for n,h in self.est.items()},onboard_input=inputs)
    def snapshot(self):
        self._drain_state()
        now,mono=self.clocks();s=self.c['safety'];rows=[self.mc[n].latest(now,mono,s['mocap_maximum_age_s']) for n in NAMES]
        return np.array([r[2] for r in rows]),np.array([self.mc[n].velocity(s['mocap_velocity_window_s'],s['mocap_maximum_age_s']) for n in NAMES]),np.array([yaw(r[3]) for r in rows])
    def health(self,ground=False,stream=False,expected=None):
        try:
            self._drain_state()
            if self._state_receiver is not None and self._state_receiver.error:
                raise RuntimeError('State receiver executor failed: '+self._state_receiver.error)
            if self.measurement_fault:raise RuntimeError(self.measurement_fault)
            if self.count_publishers('/poses')!=1:raise RuntimeError('Require exactly one /poses publisher')
            c=self.c;s=c['safety'];a=c['arena'];now,mono=self.clocks();p,v,ys=self.snapshot()
            for i,n in enumerate(NAMES):
                mr=self.mc[n].latest(now,mono,s['mocap_maximum_age_s']);er=self.est[n].latest(now,mono,s['maximum_estimator_age_s'])
                for hist,minrate in ((self.mc[n],s['minimum_mocap_rate_hz']),(self.est[n],s['minimum_estimator_rate_hz'])):
                    rate,coverage=hist.stats()
                    if rate<minrate or coverage<.4:raise RuntimeError(n+': insufficient pose rate/history')
                mp,mq=self.mc[n].at(er[0]) if self.est[n].basis=='ros_header' else (mr[2],mr[3])
                pe=float(np.linalg.norm(mp-er[2]));ae=rotation_error(mq,er[3])
                if pe>s['maximum_estimator_position_error_m'] or ae>s['maximum_estimator_attitude_error_rad']:raise RuntimeError(f'{n}: onboard/mocap mismatch: position={pe:.4f} m attitude={ae:.4f} rad')
                if tilt(mr[3])>s['measured_maximum_tilt_rad']:raise RuntimeError(n+': measured tilt exceeds limit')
                if n not in self.batteries or mono-self.batteries[n][0]>c['battery']['status_maximum_age_s']:raise RuntimeError(n+': missing/stale battery')
                if self.batteries[n][1]<(c['battery']['minimum_ground_voltage_v'] if ground else c['battery']['land_voltage_v']):raise RuntimeError(n+': battery below voltage threshold')
            from .obstacles import check_live_obstacles
            self.obstacle_evidence=check_live_obstacles(self.obstacle_histories,p,c,now,mono)
            minimum=min(np.linalg.norm(p[i,:2]-p[j,:2]) for i in range(3) for j in range(i))
            if minimum<s['measured_abort_xy_separation_m']:raise RuntimeError(f'Measured pair separation {minimum:.3f} m below abort limit')
            margin=a['body_radius_m']+a['wall_surface_clearance_m']
            if np.any(p[:,:2]<np.array([a['x_limits'][0],a['y_limits'][0]])+margin) or np.any(p[:,:2]>np.array([a['x_limits'][1],a['y_limits'][1]])-margin):raise RuntimeError('Measured wall clearance violated')
            if np.any(p[:,2]<a['floor_z_m']-.03) or np.any(p[:,2]>a['ceiling_z_m']-a['ceiling_clearance_m']):raise RuntimeError('Measured floor/ceiling violated')
            if stream and np.any(p[:,2]<a['floor_z_m']+s['airborne_minimum_height_above_floor_m']):raise RuntimeError('Measured airborne height below limit')
            if not ground and np.max(np.linalg.norm(v,axis=1))>s['measured_maximum_speed_mps']:raise RuntimeError('Measured speed exceeds limit: '+str(np.linalg.norm(v,axis=1).tolist()))
            if expected is not None:
                errors=np.linalg.norm(p-expected,axis=1)
                if max(errors)>s['tracking_tube_m']:
                    i=int(np.argmax(errors));raise RuntimeError(f'{NAMES[i]}: tracking tube error={errors[i]:.4f} m, measured={p[i].tolist()}, reference={expected[i].tolist()}')
            return p,v,ys
        except RuntimeError as e:
            self.last_fault=dict(error=str(e),ros_ns=self.get_clock().now().nanoseconds,monotonic_s=time.monotonic(),rejected=dict(self.rejected),telemetry=self.measurement_diagnostics());raise
    def spin_for(self,seconds,guard=None):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=min(.01,max(0.,end-time.monotonic())))
            self._drain_state()
            if guard:guard()

class Services:
    def __init__(self,node):
        self.node=node;self.clients={n:{k:node.create_client(t,'/'+n+'/'+k) for k,t in [('arm',Arm),('takeoff',Takeoff),('land',Land),('notify_setpoints_stop',NotifySetpointsStop)]} for n in NAMES}
    def ready(self):return all(cl.service_is_ready() for group in self.clients.values() for cl in group.values())
    def begin_landing(self, requests, release=True):
        from .landing import LandingRequests
        if set(requests)-set(NAMES):
            raise RuntimeError('Invalid selective landing route')
        return LandingRequests(self.clients, requests, NotifySetpointsStop, release)

    def call(self,key,requests,timeout=2.):
        if key not in ('arm','takeoff','land','notify_setpoints_stop') or set(requests)-set(NAMES):raise RuntimeError('Invalid selective service route')
        pending={};errors={}
        for n,request in requests.items():
            try:
                if not self.clients[n][key].service_is_ready():raise RuntimeError('Service unavailable')
                pending[n]=self.clients[n][key].call_async(request)
            except Exception as e:errors[n]=str(e)
        end=time.monotonic()+timeout
        while pending and time.monotonic()<end:
            rclpy.spin_once(self.node,timeout_sec=.01)
            for n,f in list(pending.items()):
                if f.done():
                    try:
                        r=f.result()
                        if r is None or (hasattr(r,'success') and not r.success):raise RuntimeError('Service returned failure')
                    except Exception as e:errors[n]=str(e)
                    del pending[n]
        for n,f in pending.items():
            f.cancel();errors[n]='Timed out; physical outcome uncertain'
        if errors:raise RuntimeError(key+': '+str(errors))

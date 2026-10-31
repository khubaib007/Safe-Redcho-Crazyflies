# Release preparation: include final journal-drain errors in mission status.
"""Default check-only coordinator. --fly is the only route to takeoff."""
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.parameter import Parameter
import argparse,csv,json,signal,time,uuid
from pathlib import Path
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.duration import Duration
from std_msgs.msg import String
from crazyflie_interfaces.msg import Position
from crazyflie_interfaces.srv import Arm,Takeoff,Land,NotifySetpointsStop
from .ros_io import StateNode,Services
from .settings import load,NAMES,path_of,make_plan,check_volume
from .protocol import fingerprint,encode,decode
from .core import StageMachine,accept_batch,wrap
from .backend import starts,verify_cache
from .preview import run as numerical_preview
from .journal import CommandJournal

STAGE_COLOURS = {
    'GROUND':          (8, 8, 8),      # Dim white
    'TAKEOFF':         (32, 32, 32),   # White
    'TAKEOFF_SETTLE':  (16, 16, 0),    # Dim yellow
    'WARMUP':          (0, 0, 40),     # Blue
    'HEIGHT':          (0, 32, 32),    # Cyan
    'YAW':             (32, 0, 32),    # Magenta
    'FORMATION':       (40, 32, 0),    # Yellow
    'SETTLE':          (12, 24, 40),   # Light blue
    'FORWARD':         (0, 40, 0),     # Green
    'STOP':            (40, 0, 0),     # Red
    'LANDING':         (40, 12, 0),    # Orange
}


class Mission(StateNode):
    def __init__(self,c,output):
        super().__init__(c,'three_mission_coordinator');self.output=Path(output);self.output.mkdir(parents=True,exist_ok=True);self.snapshot_file=starts(c)
        self.flight_active=False;self.streaming=False;self.arm_attempted=False;self.airborne_possible=False;self.session=str(uuid.uuid4());self.sequence=0
        self.readiness={};self.watchdog_seen=None;self.responses={};self.rows=[];self.stage='GROUND';self.command_file=None;self.expected=None
        ns=c['namespace'];self.frames=self.create_publisher(String,ns+'/frame',8);self.beats=self.create_publisher(String,ns+'/heartbeat',8);self.cmd={n:self.create_publisher(Position,'/'+n+'/cmd_position',2) for n in NAMES};self.flight_services=Services(self)
        self.create_subscription(String,ns+'/commands',self.response,16);self.create_subscription(String,ns+'/agent_ready',self.ready,16);self.create_subscription(String,ns+'/watchdog_ready',self.watchdog,4);self.create_timer(.04,self.pulse)
        self.result={'status':'FAIL','active_names':NAMES,'command_interface':'position','scope':'New isolated dual-bank acquisition mission; published position is not verified onboard receipt'}
        (self.output/'effective_profile.json').write_text(json.dumps(c,indent=2))
        self._led_stage = None
        self._led_targets = None
        self._led_pending = None
        self._led_retry_at = 0.0
        self._led_get_client = self.create_client(
            GetParameters, '/crazyflie_server/get_parameters'
        )
        self._led_client = self.create_client(
            SetParameters, '/crazyflie_server/set_parameters'
        )
        self.create_timer(0.1, self.update_stage_led)
    def pulse(self):self.beats.publish(String(data=encode(dict(session=self.session,active=self.flight_active,streaming=self.streaming,seq=self.sequence,stage=self.stage,monotonic_stamp=time.monotonic(),config_hash=fingerprint(self.c)))))
    def update_stage_led(self):
        now = time.monotonic()

        # Poll asynchronous requests without blocking flight control.
        if self._led_pending is not None:
            future, deadline, kind, sent_stage = self._led_pending
            client = (
                self._led_get_client
                if kind == 'detect' else self._led_client
            )

            if not future.done() and now < deadline:
                return

            self._led_pending = None
            try:
                if not future.done():
                    future.cancel()
                    client.remove_pending_request(future)
                    raise RuntimeError('LED parameter request timed out')

                response = future.result()
                if response is None:
                    raise RuntimeError('Empty LED parameter response')

                if kind == 'detect':
                    if len(response.values) != len(NAMES):
                        raise RuntimeError('Incomplete LED detection response')
                    if any(value.type != 2 for value in response.values):
                        raise RuntimeError('LED detection parameters unavailable')

                    self._led_targets = [
                        name for name, value in zip(NAMES, response.values)
                        if value.integer_value != 0
                    ]
                    self.get_logger().info(
                        'Bottom LED decks detected: '
                        + (', '.join(self._led_targets) or 'none')
                    )
                else:
                    if len(response.results) != len(self._led_targets):
                        raise RuntimeError('Incomplete LED write response')
                    failures = [
                        result.reason for result in response.results
                        if not result.successful
                    ]
                    if failures:
                        raise RuntimeError('; '.join(failures))

                    self._led_stage = sent_stage
                    self.get_logger().info(
                        f'LED stage: {sent_stage} -> '
                        + ', '.join(self._led_targets)
                    )
            except Exception as exc:
                self._led_retry_at = now + 2.0
                self.get_logger().warning(f'LED update: {exc}')
            return

        if now < self._led_retry_at:
            return

        try:
            if self._led_targets is None:
                if not self._led_get_client.service_is_ready():
                    return

                request = GetParameters.Request()
                request.names = [
                    f'{name}.params.deck.bcColorLedBot' for name in NAMES
                ]
                future = self._led_get_client.call_async(request)
                self._led_pending = (future, now + 2.0, 'detect', None)
                return

            if not self._led_targets:
                return

            stage = self.stage if self.flight_active else 'GROUND'
            rgb = STAGE_COLOURS.get(stage)
            if stage == self._led_stage or rgb is None:
                return
            if not self._led_client.service_is_ready():
                return

            red, green, blue = rgb
            colour = (red << 16) | (green << 8) | blue

            request = SetParameters.Request()
            request.parameters = [
                Parameter(
                    f'{name}.params.colorLedBot.wrgb8888',
                    Parameter.Type.INTEGER,
                    colour
                ).to_parameter_msg()
                for name in self._led_targets
            ]
            future = self._led_client.call_async(request)
            self._led_pending = (future, now + 2.0, 'write', stage)
        except Exception as exc:
            self._led_retry_at = now + 2.0
            self.get_logger().warning(f'LED request: {exc}')    

    
    def ready(self,msg):
        try:
            p=decode(msg.data)
            if p['name'] in NAMES:self.readiness[(p['name'],p['pid'])]=(time.monotonic(),p['config_hash'])
        except (ValueError,KeyError):pass
    def watchdog(self,msg):
        try:
            p=decode(msg.data)
            if p['config_hash']==fingerprint(self.c) and not p['triggered']:self.watchdog_seen=time.monotonic()
        except (ValueError,KeyError):pass
    def response(self,msg):
        try:
            p=decode(msg.data)
            if p.get('session')!=self.session or p.get('seq')!=self.sequence or p.get('name') not in NAMES:return
            n=p['name']
            if n in self.responses and self.responses[n]!=p:self.responses[n]={'error':'Conflicting duplicate result'}
            else:self.responses[n]=p
        except (ValueError,KeyError):pass
    def routes(self):
        if self.count_publishers(self.c['namespace']+'/frame')!=1:raise RuntimeError('Require one new mission coordinator')
        for n in NAMES:
            if self.count_publishers('/'+n+'/cmd_position')!=1:raise RuntimeError(n+': competing position publisher')
            for suffix in ('cmd_full_state','cmd_vel_legacy','cmd_hover','cmd_velocity_world'):
                if self.count_publishers('/'+n+'/'+suffix):raise RuntimeError(n+': competing command route '+suffix)
            identities=[(x.node_namespace,x.node_name,x.topic_type) for x in self.get_subscriptions_info_by_topic('/'+n+'/cmd_position')]
            backend=('/', 'crazyflie_server','crazyflie_interfaces/msg/Position');recorder=('/','redcho_telemetry_control','crazyflie_interfaces/msg/Position')
            if identities.count(backend)!=1 or identities.count(recorder)>1 or any(x not in (backend,recorder) for x in identities):raise RuntimeError('Require backend and at most one known control recorder on cmd_position')
        for n in ('cf1','cf5','cf6'):
            if self.count_publishers('/'+n+'/pose'):raise RuntimeError('Stop the six-aircraft backend before new mission')
    def preflight(self):
        end=time.monotonic()+50;good=None;last=0
        while time.monotonic()<end:
            self.spin_for(.02);now=time.monotonic()
            try:
                p,v,yaw=self.health(ground=True);self.routes()
                original=np.array([self.snapshot_file['positions'][n] for n in NAMES]);captured_yaw=np.array([self.snapshot_file['yaw_rad'][n] for n in NAMES])
                if max(np.linalg.norm(p-original,axis=1))>self.c['acquisition']['captured_start_tolerance_m'] or max(np.abs(wrap(yaw-captured_yaw)))>.35:raise RuntimeError('Aircraft moved since capture: recapture in current floor positions')
                if max(np.linalg.norm(v,axis=1))>.05:raise RuntimeError('Keep aircraft stationary on floor')
                fresh={n:[(pid,h) for (name,pid),(t,h) in self.readiness.items() if name==n and now-t<2] for n in NAMES}
                if any(len(x)!=1 or x[0][1]!=fingerprint(self.c) for x in fresh.values()) or len({x[0][0] for x in fresh.values() if x})!=3:raise RuntimeError('Require three unique matching new agent processes')
                if self.watchdog_seen is None or now-self.watchdog_seen>1.5:raise RuntimeError('Independent new watchdog not ready')
                if not self.flight_services.ready():raise RuntimeError('Selective arm/takeoff/land/notify services not ready')
                plan=make_plan(self.c,p,yaw);good=now if good is None else good
                if now-good>=3:return plan
            except (ValueError,RuntimeError) as e:
                good=None
                if now-last>1:self.get_logger().info(str(e));last=now
        raise RuntimeError('Ground preflight timed out; no arm/takeoff sent')
    def cycle(self,machine,q,v,previous,plan,ground=False):
        self.sequence+=1;self.responses={};self.stage=machine.stage
        frame=dict(session=self.session,seq=self.sequence,config_hash=fingerprint(self.c),stage=machine.stage,q=q.tolist(),v=v.tolist(),previous=previous.tolist(),
                   voltage=[self.batteries[n][1] for n in NAMES],flight_active=self.flight_active,streaming=self.streaming,plan=plan)
        self.frames.publish(String(data=encode(frame)));end=time.monotonic()+self.c['control']['cycle_deadline_s']
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.001);self.health(ground=ground,stream=not ground,expected=None if ground else q)
            if not ground and (self.watchdog_seen is None or time.monotonic()-self.watchdog_seen>1.5):raise RuntimeError('Watchdog readiness became stale')
            if any(r.get('error') for r in self.responses.values()):raise RuntimeError('Agent failure: '+str(self.responses))
            if set(self.responses)==set(NAMES):
                r=[self.responses[n] for n in NAMES];u=np.array([x['u'] for x in r],float)
                if u.shape!=(3,3) or not np.isfinite(u).all():raise RuntimeError('Invalid acceleration batch')
                return r,u
        raise RuntimeError(f'Cycle {self.sequence}: deadline missed; no partial command batch applied')
    def publish(self,q,v,u,yaws):
        if any(np.asarray(a).shape!=(3,3) or not np.isfinite(a).all() for a in (q,v,u)) or np.asarray(yaws).shape!=(3,) or not np.isfinite(yaws).all():raise RuntimeError('Invalid publication batch')
        if self.command_file is None:
            self.command_file=CommandJournal(self.output/'position_commands.csv')
        if self.command_file.errors:raise RuntimeError('Command journal failed: '+str(self.command_file.errors))
        began=time.monotonic();records=[]
        for i,n in enumerate(NAMES):
            msg=Position();now=self.get_clock().now();msg.header.stamp=now.to_msg();msg.header.frame_id=self.c['frame_id'];msg.x,msg.y,msg.z=map(float,q[i]);msg.yaw=float(wrap(yaws[i]));self.cmd[n].publish(msg)
            records.append([time.monotonic(),now.nanoseconds,self.stage,n,*q[i],msg.yaw,*v[i],*u[i],'cmd_position',False])
        span=time.monotonic()-began
        self.command_file.submit(records)
        if span>self.c['control']['dt_s']:raise RuntimeError('Publishing the three position messages exceeded one control period')
    def publication_slot(self,last_publish,q):
        if last_publish is not None:
            wait=self.c['control']['dt_s']-(time.monotonic()-last_publish)
            if wait>0:self.spin_for(wait,lambda:self.health(stream=True,expected=q))
            if time.monotonic()-last_publish>self.c['control']['maximum_publish_gap_s']:raise RuntimeError('Position publication gap exceeded budget')
        return time.monotonic()
    def takeoff(self,plan):
        self.stage='TAKEOFF';self.arm_attempted=True
        reqs={}
        for n in NAMES:r=Arm.Request();r.arm=True;reqs[n]=r
        self.flight_services.call('arm',reqs);self.health(ground=True)
        reqs={}
        for i,n in enumerate(NAMES):r=Takeoff.Request();r.group_mask=0;r.height=float(plan['hover'][i][2]);r.duration=Duration(seconds=self.c['mission']['takeoff_duration_s']).to_msg();reqs[n]=r
        self.airborne_possible=True;self.flight_active=True;self.pulse();self.flight_services.call('takeoff',reqs)
        self.spin_for(self.c['mission']['takeoff_duration_s'],lambda:self.health())
        self.stage='TAKEOFF_SETTLE';end=time.monotonic()+self.c['acquisition']['takeoff_settle_timeout_s'];good=None
        while time.monotonic()<end:
            self.spin_for(.02);p,v,y=self.health(stream=True,expected=np.asarray(plan['hover']))
            settled=max(np.linalg.norm(p-np.asarray(plan['hover']),axis=1))<.10 and max(np.linalg.norm(v,axis=1))<.10
            good=time.monotonic() if settled and good is None else good if settled else None
            if good is not None and time.monotonic()-good>=1.:
                plan['proposals']=np.asarray(plan['hover']).tolist()
                for i in range(3):plan['proposals'][i][2]=float(p[i,2])
                plan['target_center'][2]=float(p[:,2].mean());plan['initial_yaw']=y.tolist();check_volume(np.asarray(plan['target_center'])+np.asarray(plan['offsets']),self.c)
                return
        raise RuntimeError('Individual takeoff hovers did not settle')
    def land(self):
        report={};self.stage='LANDING';self.streaming=False
        reqs={}
        for n in NAMES:
            r=Land.Request();r.group_mask=0
            r.height=float(self.snapshot_file['positions'][n][2]+self.c['mission']['landing_offset_m'])
            r.duration=Duration(seconds=self.c['mission']['landing_duration_s']).to_msg()
            reqs[n]=r
        self.get_logger().info('LANDING: release -> land per aircraft; no release dwell')
        handover=self.flight_services.begin_landing(reqs,release=self.command_file is not None)
        while not handover.done:
            rclpy.spin_once(self,timeout_sec=.001)
            handover.tick()
        report.update(handover.report())
        self.spin_for(self.c['mission']['landing_duration_s']+2);end=time.monotonic()+5;evidence={}
        while time.monotonic()<end:
            self.spin_for(.05);now,mono=self.clocks();evidence={n:self.mc[n].touchdown(self.snapshot_file['positions'][n][2],now,mono,self.c['safety']['mocap_maximum_age_s']) for n in NAMES}
            if all(x['verified'] for x in evidence.values()):break
        verified=[n for n in NAMES if evidence.get(n,{}).get('verified')];report.update(touchdown_verified=verified,touchdown_not_verified=[n for n in NAMES if n not in verified],touchdown_evidence=evidence)
        reqs={}
        for n in verified:r=Arm.Request();r.arm=False;reqs[n]=r
        try:self.flight_services.call('arm',reqs)
        except Exception as e:report['disarm_error']=str(e)
        return report
    def run(self,fly=False,acquire_only=False):
        recovery={};began=time.monotonic()
        try:
            plan=self.preflight();self.result['plan']=plan;self.result['parameter_cache']=verify_cache(self,self.c)
            self.get_logger().info('Ground numerical route preview: no flight commands')
            preview,_=numerical_preview(self.c,plan,voltages=tuple(self.batteries[n][1] for n in NAMES),acquire_only=acquire_only,trace=True,evidence_path=self.output/'hocbf_preview_rows.csv')
            (self.output/'ground_preview.json').write_text(json.dumps(preview,indent=2));self.result['ground_preview']=preview
            # Resume ROS callbacks and reacquire fresh, stable ground measurements.
            live = self.preflight()
            if np.max(np.linalg.norm(
                    np.asarray(live['ground']) - np.asarray(plan['ground']),
                    axis=1)) > 0.02:
                raise RuntimeError(
                    'Starts changed during preview; recapture and repeat check'
                )
            # Sample the ACTUAL three-process communication path before arming.
            self.session=str(uuid.uuid4());self.sequence=0;machine=StageMachine(self.c,plan);q=np.asarray(plan['hover']);v=np.zeros((3,3));previous=v.copy();timings=[]
            for _ in range(60):
                start=time.monotonic();r,u=self.cycle(machine,q,v,previous,plan,ground=True);q,v=accept_batch(q,v,previous,u,self.c);previous=u;timings.append(time.monotonic()-start);self.spin_for(max(0.,self.c['control']['dt_s']-timings[-1]))
            self.result['shadow_cycle_max_s']=max(timings)
            # Preview and cache checks can take seconds. Revalidate live floor
            # positions, voltage and process readiness immediately before arming.
            live=self.preflight()
            if np.max(np.linalg.norm(np.array(live['ground'])-np.array(plan['ground']),axis=1))>.02:raise RuntimeError('Starts changed during preview; recapture and repeat check')
            self.result['obstacle_evidence']=self.obstacle_evidence
            if not fly:self.result['status']='CHECK_ONLY_PASS';return
            from .obstacles import require_verified_geometry
            require_verified_geometry(self.c)
            self.command_file=CommandJournal(self.output/'position_commands.csv')
            self.takeoff(plan);self.session=str(uuid.uuid4());self.sequence=0;machine=StageMachine(self.c,plan,acquire_only);q=np.asarray(plan['hover']);v=np.zeros((3,3));previous=v.copy();self.streaming=True;self.pulse();last_pub=None;last_stage=None
            while time.monotonic()-began<self.c['mission']['maximum_wall_duration_s']+60:
                start=time.monotonic();self.stage=machine.stage
                if last_stage!=self.stage:self.get_logger().info('STAGE '+self.stage);last_stage=self.stage
                p,mv,ys=self.health(stream=True,expected=q);r,u=self.cycle(machine,q,v,previous,plan);qn,vn=accept_batch(q,v,previous,u,self.c)
                now=self.publication_slot(last_pub,q)
                self.publish(qn,vn,u,machine.yaw_reference());last_pub=now;q,v,previous=qn,vn,u
                self.rows.append((machine.total,p.copy(),q.copy(),mv.copy(),v.copy(),np.array([x['target'] for x in r]),np.array([x['velocity_estimate'] for x in r]),np.array([x['position_estimate'] for x in r]),u.copy(),ys.copy()))
                if machine.advance(r,p,mv,ys):
                    self.result.update(status='PASS',stage_events=machine.events,final_measured_endpoint_errors_m=np.linalg.norm(p-machine.goal(not acquire_only),axis=1).tolist());break
                self.spin_for(max(0.,self.c['control']['dt_s']-(time.monotonic()-start)))
            else:raise RuntimeError('Maximum wall duration exceeded')
        except BaseException as e:self.result.update(status='FAIL',error=f'{type(e).__name__}: {e}');self.get_logger().error(self.result['error'])
        finally:
            old={s:signal.signal(s,signal.SIG_IGN) for s in (signal.SIGINT,signal.SIGTERM)}
            try:
                if self.airborne_possible:recovery=self.land()
                elif self.arm_attempted:
                    # Disarm only fresh, stationary near-floor aircraft.
                    now,mono=self.clocks();verified=[n for n in NAMES if self.mc[n].touchdown(self.snapshot_file['positions'][n][2],now,mono,self.c['safety']['mocap_maximum_age_s'])['verified']];reqs={}
                    for n in verified:r=Arm.Request();r.arm=False;reqs[n]=r
                    self.flight_services.call('arm',reqs);recovery['ground_disarm_verified']=verified
            except BaseException as e:recovery['cleanup_error']=str(e)
            finally:
                for s,h in old.items():signal.signal(s,h)
            if self.airborne_possible and (len(recovery.get('touchdown_verified',[]))!=3 or any(k.endswith('error') for k in recovery)):self.result['status']='FAIL'
            self.flight_active=self.airborne_possible and len(recovery.get('touchdown_verified',[]))!=3;self.streaming=False;self.pulse();self.spin_for(.15)
            self.result.update(recovery=recovery,last_measurement_fault=self.last_fault,rejected_measurements=self.rejected,obstacle_evidence=self.obstacle_evidence)
            if self.command_file:
                journal=self.command_file.close();self.result['command_journal']=journal
                if journal.get('errors') or journal.get('dropped_rows',0) or journal.get('pending_batches',0) or journal.get('worker_still_running',False):
                    self.result.update(status='FAIL',recording_error='Command journal did not close cleanly')
            if self.rows:
                keys=('time','measured_position','reference_position','measured_velocity','reference_velocity','formation_target','velocity_redcho','pose_redcho','safe_acceleration','measured_yaw');np.savez_compressed(self.output/'flight_trace.npz',**{k:np.array([r[i] for r in self.rows]) for i,k in enumerate(keys)})
            (self.output/'mission_status.json').write_text(json.dumps(self.result,indent=2,allow_nan=False));print(json.dumps(self.result,indent=2,allow_nan=False))

    
def interrupted(signum,frame):raise KeyboardInterrupt

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',required=True);p.add_argument('--output',required=True);p.add_argument('--fly',action='store_true');p.add_argument('--acquire-only',action='store_true');a,ros=p.parse_known_args();rclpy.init(args=ros,signal_handler_options=SignalHandlerOptions.NO);n=Mission(load(a.config),a.output)
    old={s:signal.signal(s,interrupted) for s in (signal.SIGINT,signal.SIGTERM)}
    try:n.run(a.fly,a.acquire_only)
    finally:
        for s,h in old.items():signal.signal(s,h)
        n.destroy_node();rclpy.try_shutdown()
    if n.result['status']=='FAIL':raise RuntimeError('Mission failed; inspect mission_status.json; keep backend/agents alive for recovery')

"""Actual ROS adapter classes exercised with explicit test doubles, not a ROS integration claim."""
import importlib,json,sys,tempfile,types,unittest,contextlib,io
from collections import deque
from pathlib import Path
from unittest.mock import patch
import numpy as np
import yaml
from test_core import cfg,plan
from safe_redcho_single_mission.settings import NAMES
from safe_redcho_single_mission.protocol import encode,fingerprint

class Message:
 def __init__(self,**kw):
  self.header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=2000,nanosec=0),frame_id='world');self.__dict__.update(kw)
class Service:
 Request=Message
class Clock:
 def __init__(self,t=10.):self.t=t
 def monotonic(self):return self.t
class Bus:
 def __init__(self):self.nodes=[];self.messages=deque();self.pubs=[];self.callbacks={}
 def drain(self):
  while self.messages:
   topic,msg=self.messages.popleft()
   for cb in list(self.callbacks.get(topic,[])):cb(msg)
class Node:
 bus=None
 # Real rclpy exposes these as read-only properties.
 publishers=property(lambda s:iter(s._pubs));subscriptions=property(lambda s:iter(()));services=property(lambda s:iter(()));clients=property(lambda s:iter(()));context=property(lambda s:object())
 def __init__(self,name):self.node_name=name;self._pubs=[];self.logs=[];self.bus.nodes.append(self)
 def get_clock(self):return types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=2_000_000_000_000,to_msg=lambda:Message()))
 def get_logger(self):return types.SimpleNamespace(info=self.logs.append,warning=self.logs.append,error=self.logs.append)
 def create_publisher(self,cls,topic,qos):
  pub=types.SimpleNamespace(topic=topic,sent=[])
  def publish(msg):pub.sent.append(msg);self.bus.messages.append((topic,msg))
  pub.publish=publish;self._pubs.append(pub);self.bus.pubs.append(pub);return pub
 def create_subscription(self,cls,topic,cb,qos):self.bus.callbacks.setdefault(topic,[]).append(cb);return object()
 def create_timer(self,*args):return object()
 def create_client(self,cls,topic):return types.SimpleNamespace(service_is_ready=lambda:True,topic=topic)
 def count_publishers(self,topic):return sum(p.topic==topic for p in self.bus.pubs)
 def destroy_node(self):return True
 def destroy_client(self,x):return True
class Emitter:
 def __init__(self,*args):self.sent=[]
 def submit(self,x):self.sent.append(x)
 def close(self):pass

def modules():
 out={}
 def m(name,**kw):x=types.ModuleType(name);x.__dict__.update(kw);out[name]=x
 m('rclpy',spin_once=lambda node,**kw:node.bus.drain(),spin=lambda n:None,init=lambda **kw:None,try_shutdown=lambda:None)
 m('rclpy.node',Node=Node);m('rclpy.executors',ExternalShutdownException=type('ExternalShutdownException',(Exception,),{}));m('rclpy.signals',SignalHandlerOptions=types.SimpleNamespace(NO=0));m('rclpy.duration',Duration=lambda **kw:types.SimpleNamespace(to_msg=lambda:Message()))
 m('rclpy.qos',qos_profile_sensor_data=object(),QoSProfile=lambda **kw:object(),ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT=1),HistoryPolicy=types.SimpleNamespace(KEEP_LAST=1))
 for name,types_ in [('std_msgs',['String']),('geometry_msgs',['PoseStamped']),('crazyflie_interfaces',['Position','Status']),('motion_capture_tracking_interfaces',['NamedPoseArray'])]:m(name);m(name+'.msg',**dict.fromkeys(types_,Message))
 m('crazyflie_interfaces.srv',**dict.fromkeys(('Arm','Land','Takeoff','NotifySetpointsStop'),Service));m('rcl_interfaces');m('rcl_interfaces.srv',GetParameters=Service,SetParameters=Service)
 m('rclpy.parameter',Parameter=type('Parameter',(),{'Type':types.SimpleNamespace(INTEGER=2)}))
 return out

class AdapterTests(unittest.TestCase):
 def setUp(self):
  Node.bus=Bus();self.env=patch.dict(sys.modules,modules());self.env.start();self.temp=tempfile.TemporaryDirectory();self.c=cfg();self.c['workspace']=self.temp.name;self.c['starts_file']='starts.yaml';self.p=plan(self.c)
  Path(self.temp.name,'starts.yaml').write_text(yaml.safe_dump(dict(frame_id='world',positions=dict(zip(NAMES,self.p['ground'])),yaw_rad=dict(zip(NAMES,self.p['initial_yaw'])))))
  for name in ('ros_io','agent','mission','watchdog'):
   key='safe_redcho_single_mission.'+name
   if key in sys.modules:del sys.modules[key]
 def tearDown(self):self.env.stop();self.temp.cleanup()
 def test_three_actual_agent_classes_exchange_same_cycle_and_emit_diagnostics(self):
  a=importlib.import_module('safe_redcho_single_mission.agent')
  with patch.object(a,'DiagnosticEmitter',Emitter):agents=[a.Agent(self.c,i) for i in range(3)]
  f=dict(session='session-A',seq=1,stage='WARMUP',config_hash=fingerprint(self.c),plan=self.p,q=self.p['hover'],v=np.zeros((3,3)).tolist(),previous=np.zeros((3,3)).tolist(),voltage=[4,4,4],flight_active=True,streaming=True)
  for n in agents:n.on_frame(Message(data=encode(f)))
  Node.bus.drain();packets=[json.loads(pub.sent[0].data) for pub in Node.bus.pubs if pub.topic.endswith('/commands')]
  self.assertEqual(len(packets),3);self.assertEqual({p['name'] for p in packets},set(NAMES));self.assertTrue(all(p['seq']==1 and 'error' not in p for p in packets));self.assertTrue(all(n.emitter.sent for n in agents))
  for n in agents:n.on_frame(Message(data=encode(f)))
  self.assertTrue(all(not n.failed for n in agents));self.assertTrue(all(len(n.result.sent)==1 for n in agents))
 def make_mission(self):
  m=importlib.import_module('safe_redcho_single_mission.mission');n=m.Mission(self.c,Path(self.temp.name)/'run');n.spin_for=lambda *a,**kw:None;return m,n
 def test_check_only_runs_no_arm_takeoff_position_or_land(self):
  m,n=self.make_mission();calls=[];n.flight_services.call=lambda key,reqs,**kw:calls.append(key);n.preflight=lambda:self.p;n.batteries={name:(10,4.) for name in NAMES}
  n.cycle=lambda *a,**kw:([{}]*3,np.zeros((3,3)))
  with contextlib.redirect_stdout(io.StringIO()),patch.object(m,'verify_cache',return_value={}),patch.object(m,'numerical_preview',return_value=({'status':'PASS'},[])):n.run(fly=False)
  self.assertEqual(n.result['status'],'CHECK_ONLY_PASS');self.assertEqual(calls,[]);self.assertTrue(all(not p.sent for p in n.cmd.values()))
 def test_duplicate_backend_route_records_topic_and_endpoint_counts(self):
  m,n=self.make_mission()
  backend=types.SimpleNamespace(node_namespace='/',node_name='crazyflie_server',topic_type='crazyflie_interfaces/msg/Position')
  n.get_subscriptions_info_by_topic=lambda topic:[backend,backend]
  with self.assertRaisesRegex(RuntimeError,'/cf2/cmd_position: backend subscribers=2'):n.routes()
  self.assertEqual(n.result['last_route_fault']['topic'],'/cf2/cmd_position')
  self.assertEqual(len(n.result['last_route_fault']['subscribers']),2)
 def test_failed_ground_preview_never_arms(self):
  m,n=self.make_mission();calls=[];n.flight_services.call=lambda key,reqs,**kw:calls.append(key);n.preflight=lambda:self.p;n.batteries={name:(10,4.) for name in NAMES}
  with contextlib.redirect_stdout(io.StringIO()),patch.object(m,'verify_cache',return_value={}),patch.object(m,'numerical_preview',side_effect=RuntimeError('Blocked goals')):n.run(fly=True)
  self.assertEqual(n.result['status'],'FAIL');self.assertEqual(calls,[])
 def test_incomplete_journal_close_prevents_a_successful_status(self):
  for failure in ({'errors':['disk write failed']},{'dropped_rows':1},{'pending_batches':1},{'worker_still_running':True}):
   with self.subTest(failure=failure):
    m,n=self.make_mission();n.preflight=lambda:self.p;n.batteries={name:(10,4.) for name in NAMES}
    n.cycle=lambda *a,**kw:([{}]*3,np.zeros((3,3)))
    journal=dict(rows=0,dropped_rows=0,errors=[],pending_batches=0,worker_still_running=False);journal.update(failure)
    n.command_file=types.SimpleNamespace(close=lambda:journal)
    with contextlib.redirect_stdout(io.StringIO()),patch.object(m,'verify_cache',return_value={}),patch.object(m,'numerical_preview',return_value=({'status':'PASS'},[])):n.run(fly=False)
    self.assertEqual(n.result['status'],'FAIL');self.assertIn('recording_error',n.result)
 def test_land_releases_stream_and_disarms_only_verified_aircraft(self):
  m,n=self.make_mission();clock=Clock();n.spin_for=lambda duration,*a,**kw:setattr(clock,'t',clock.t+duration);calls=[];n.flight_services.call=lambda key,reqs,**kw:calls.append((key,list(reqs)));n.command_file=object()
  def begin(reqs,release=True):
   calls.append(('handover',list(reqs)));self.assertTrue(release)
   return types.SimpleNamespace(done=True,report=lambda:{'handover_events':[]})
  n.flight_services.begin_landing=begin
  n.mc={name:types.SimpleNamespace(touchdown=lambda *args,name=name,**kw:dict(verified=name!='cf4')) for name in NAMES}
  with patch.object(m,'time',clock):report=n.land()
  self.assertEqual([x[0] for x in calls],['handover','arm']);self.assertEqual(calls[-1][1],['cf2','cf3']);self.assertEqual(report['touchdown_not_verified'],['cf4'])
 def test_publication_contains_position_and_actual_smooth_yaw(self):
  m,n=self.make_mission();n.stage='YAW';q=np.array(self.p['hover']);n.publish(q,np.zeros((3,3)),np.zeros((3,3)),[1.2,-.5,.8]);self.assertAlmostEqual(n.cmd['cf2'].sent[0].yaw,1.2);self.assertFalse(hasattr(n.cmd['cf2'].sent[0],'acc'))
  n.command_file.close()
 def test_publication_slot_prevents_bursts_after_a_slow_cycle(self):
  m,n=self.make_mission();clock=Clock(10.003);n.spin_for=lambda duration,*a,**kw:setattr(clock,'t',clock.t+duration)
  with patch.object(m,'time',clock):self.assertAlmostEqual(n.publication_slot(10.,np.array(self.p['hover'])),10.02)
 def test_publication_slot_rejects_an_expired_gap(self):
  m,n=self.make_mission();clock=Clock(10.081)
  with patch.object(m,'time',clock):
   with self.assertRaisesRegex(RuntimeError,'gap'):n.publication_slot(10.,np.array(self.p['hover']))
 def test_watchdog_rejects_old_queued_heartbeat_and_recovers_independently(self):
  w=importlib.import_module('safe_redcho_single_mission.watchdog');n=w.Watchdog(self.c);clock=Clock();sent=[];n.send=lambda key,reqs:sent.append((key,list(reqs)))
  def begin(reqs,release=True):
   sent.append(('handover',list(reqs)));self.assertTrue(release)
   return types.SimpleNamespace(done=True,tick=lambda:None,report=lambda:{'handover_events':[]})
  n.flight_services.begin_landing=begin
  def beat(stamp,active):return Message(data=encode(dict(config_hash=fingerprint(self.c),monotonic_stamp=stamp,active=active,session='S')))
  with patch.object(w,'time',clock):
   n.beat(beat(10.,True));clock.t=10.26;n.beat(beat(10.,True));self.assertEqual(n.last,10.);n.tick();self.assertTrue(n.triggered);self.assertEqual(sent,[('handover',NAMES)]);self.assertEqual(n.stage,'LAND')
   n.mc={name:types.SimpleNamespace(touchdown=lambda *args,name=name,**kw:dict(verified=name=='cf2')) for name in NAMES};clock.t=20;n.tick();self.assertEqual(sent[-1],('arm',['cf2']))
 def test_inactive_watchdog_does_not_land(self):
  w=importlib.import_module('safe_redcho_single_mission.watchdog');n=w.Watchdog(self.c);n.tick();self.assertFalse(n.triggered)

if __name__=='__main__':unittest.main()

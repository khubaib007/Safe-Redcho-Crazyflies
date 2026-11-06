"""Explicit doubles: no real ROS graph, DDS, radio or firmware in these tests."""
import importlib
import sys
import threading
import types
import unittest
from unittest.mock import patch
import numpy as np
from test_core import cfg
from test_ros_adapters import modules,Node,Bus,Message
from safe_redcho_single_mission.health import History
from safe_redcho_single_mission.state_receiver import PacketBuffer,StateReceiver
from safe_redcho_single_mission.settings import validate


class BufferTests(unittest.TestCase):
    def test_bounded_buffers_count_overflow(self):
        b=PacketBuffer({'pose/cf3':2})
        for i in range(4):b.put('pose/cf3',i,100+i,10+i)
        self.assertEqual([r[1] for r in b.drain()],[2,3])
        self.assertEqual(b.report(13)['pose/cf3']['dropped'],2)
    def test_receipt_times_are_not_replaced_at_drain(self):
        b=PacketBuffer({'pose/cf3':2});b.put('pose/cf3','message',100.,10.)
        self.assertEqual(b.drain(),[('pose/cf3','message',100.,10.)])
        self.assertAlmostEqual(b.report(10.4)['pose/cf3']['last_callback_age_s'],.4)
    def test_cross_channel_receipt_order(self):
        b=PacketBuffer({'mocap':2,'pose/cf3':2});b.put('mocap','later',100.2,10.2);b.put('pose/cf3','earlier',100.,10.)
        self.assertEqual([r[1] for r in b.drain()],['earlier','later'])
    def test_concurrent_writer_and_drain_do_not_duplicate_packets(self):
        b=PacketBuffer({'x':2000});done=threading.Event();seen=[]
        def writer():
            for i in range(1000):b.put('x',i,100+i*.001,10+i*.001)
            done.set()
        worker=threading.Thread(target=writer);worker.start()
        while not done.is_set():seen.extend(r[1] for r in b.drain())
        worker.join();seen.extend(r[1] for r in b.drain())
        self.assertEqual(seen,list(range(1000)))


class ReceiverTests(unittest.TestCase):
    def fake_runtime(self,fail=False):
        instances=[]
        class RxNode:
            def __init__(self,name,**kwargs):self.callbacks={};self.destroyed=False;instances.append(self)
            def create_subscription(self,cls,topic,cb,qos):self.callbacks[topic]=cb
            def destroy_node(self):self.destroyed=True
        class Executor:
            def __init__(self,**kwargs):self.stopped=threading.Event()
            def add_node(self,node):self.node=node
            def spin(self):
                if fail:raise RuntimeError('test executor failure')
                self.node.callbacks['/cf3/pose']('sample');self.stopped.wait(1.)
            def shutdown(self,timeout_sec):self.stopped.set();return True
            def remove_node(self,node):pass
        env={'rclpy.node':types.SimpleNamespace(Node=RxNode),
             'rclpy.executors':types.SimpleNamespace(SingleThreadedExecutor=Executor,
                        ExternalShutdownException=type('ExternalShutdownException',(Exception,),{})),
             'rclpy.qos':types.SimpleNamespace(qos_profile_sensor_data=object())}
        return env,instances
    def test_executor_receives_without_owner_spinning_and_closes(self):
        env,nodes=self.fake_runtime();owner=types.SimpleNamespace(context=object(),clocks=lambda:(100.,10.))
        with patch.dict(sys.modules,env):
            rx=StateReceiver(owner,'receiver',[('pose/cf3',object,'/cf3/pose',2)])
            # Event is only for a bounded test handshake, never control timing.
            for _ in range(1000):
                if rx.report(10.)['channels']['pose/cf3']['received']:break
                threading.Event().wait(.001)
            self.assertEqual(rx.drain(),[('pose/cf3','sample',100.,10.)])
            rx.close();self.assertFalse(rx.worker.is_alive());self.assertTrue(nodes[0].destroyed)
    def test_executor_error_is_retained(self):
        env,_=self.fake_runtime(fail=True);owner=types.SimpleNamespace(context=object(),clocks=lambda:(100.,10.))
        with patch.dict(sys.modules,env):
            rx=StateReceiver(owner,'receiver',[('pose/cf3',object,'/cf3/pose',2)])
            rx.worker.join(timeout=1.);self.assertIn('test executor failure',rx.error);rx.close()


class StateAdapterTests(unittest.TestCase):
    def setUp(self):
        Node.bus=Bus();self.env=patch.dict(sys.modules,modules());self.env.start()
        key='safe_redcho_single_mission.ros_io'
        if key in sys.modules:del sys.modules[key]
        self.io=importlib.import_module(key);self.c=cfg();self.c['safety']['maximum_estimator_age_s']=.35
        self.node=self.io.StateNode(self.c,'test_state')
        self.time=[2000.,10.];self.node.clocks=lambda:tuple(self.time)
    def tearDown(self):self.node.destroy_node();self.env.stop()
    def pose(self,stamp=2000.):
        sec=int(stamp);msg=Message();msg.header.stamp=types.SimpleNamespace(sec=sec,nanosec=int(round((stamp-sec)*1e9)))
        msg.pose=types.SimpleNamespace(position=types.SimpleNamespace(x=0,y=0,z=.5),
                                      orientation=types.SimpleNamespace(x=0,y=0,z=0,w=1))
        return msg
    def test_rejected_onboard_packets_are_counted(self):
        m=self.pose();self.node.onboard(m,'cf3');self.node.onboard(m,'cf3')
        self.assertEqual(self.node.rejected['rejected_onboard_cf3'],1)
        self.assertEqual(self.node.est['cf3'].last_rejection,'nonmonotonic_source_stamp')
    def test_delayed_drain_never_refreshes_stale_receipt(self):
        b=PacketBuffer({'pose/cf3':2});b.put('pose/cf3',self.pose(),2000.,10.)
        self.node._state_receiver=types.SimpleNamespace(error=None,drain=b.drain,
            report=lambda mono:b.report(mono),close=lambda:None)
        self.time[:]=[2000.36,10.36];self.node._drain_state()
        with self.assertRaisesRegex(RuntimeError,'Stale pose'):
            self.node.est['cf3'].latest(*self.time,.35)
    def test_failed_history_source_age_still_rejects(self):
        self.node.onboard(self.pose(1999.5),'cf3')
        self.assertEqual(self.node.rejected['rejected_onboard_cf3'],1)
        self.assertEqual(len(self.node.est['cf3'].rows),0)
    def test_receiver_failure_aborts_health_without_breaking_recovery_pump(self):
        self.node._state_receiver=types.SimpleNamespace(error='test failure',drain=lambda:[],
            report=lambda mono:dict(executor_error='test failure'),close=lambda:None)
        self.node._drain_state()  # Recovery can keep pumping service replies.
        with self.assertRaisesRegex(RuntimeError,'State receiver executor failed'):self.node.health()
    def test_buffered_battery_keeps_original_receipt(self):
        self.node.battery(types.SimpleNamespace(battery_voltage=3.8),'cf3',(1999.,9.))
        self.assertEqual(self.node.batteries['cf3'],(9.,3.8))
    def test_diagnostics_separate_received_from_accepted(self):
        m=self.pose();self.node.onboard(m,'cf3');self.time[:]=[2000.2,10.2];self.node.onboard(m,'cf3')
        d=self.node.measurement_diagnostics()
        self.assertAlmostEqual(d['onboard_input']['cf3']['callback_age_s'],0)
        self.assertAlmostEqual(d['onboard']['cf3']['receipt_age_s'],.2)
        self.assertEqual(d['onboard']['cf3']['rejected'],1)
    def test_invalid_receiver_config_types_are_rejected(self):
        for value in ['true',1,None]:
            c=cfg();c['backend']['state_receiver_thread']=value
            with self.assertRaisesRegex(ValueError,'boolean'):validate(c)
    def test_new_mode_owns_state_subscriptions_on_receiver_only(self):
        self.c['backend']['state_receiver_thread']=True;Node.bus.callbacks.clear()
        created=[]
        class Receiver:
            error=None
            def __init__(self,owner,name,specs):created.append(specs)
            def close(self):pass
        with patch('safe_redcho_single_mission.state_receiver.StateReceiver',Receiver):
            node=self.io.StateNode(self.c,'threaded_owner')
        self.assertEqual(len(created[0]),7);self.assertEqual(Node.bus.callbacks,{})
        node.destroy_node()
    def test_real_health_path_retains_stale_onboard_abort_and_diagnostics(self):
        self.time[:]=[2000.6,10.6];positions={'cf2':[-.05,.76,.55],'cf3':[-.78,0,.55],'cf4':[-.05,-.76,.55]}
        for n in self.c['names']:
            for k in range(61):
                t=k*.01;self.node.mc[n].add(positions[n],[0,0,0,1],2000+t,2000+t,10+t,.08)
            for k in range(13 if n!='cf3' else 5):
                t=k*.05;self.node.est[n].add(positions[n],[0,0,0,1],2000+t,2000+t,10+t,.35,True)
            self.node.batteries[n]=(10.6,4.)
        self.node.count_publishers=lambda topic:1
        with self.assertRaisesRegex(RuntimeError,'/cf3/pose: Stale pose'):self.node.health()
        d=self.node.last_fault['telemetry']
        self.assertAlmostEqual(d['onboard']['cf3']['receipt_age_s'],.4)
        self.assertLess(d['mocap']['cf3']['receipt_age_s'],.08)

if __name__=='__main__':unittest.main()

"""Execute the actual capture node with minimal ROS message/Node test doubles."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
from support import cfg,packet

T=1_000_000_000_000


class FakeNode:
    # Match read-only Node properties that must not be overwritten by subclasses.
    @property
    def publishers(self): return iter(self._pubs)
    @property
    def context(self): return object()
    def __init__(self,name):self._pubs=[];self.subs=[];self.logs=[]
    def get_clock(self):return types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=T))
    def get_logger(self):return types.SimpleNamespace(info=self.logs.append,warning=self.logs.append)
    def create_subscription(self,cls,topic,cb,qos):self.subs.append(topic);return object()
    def create_publisher(self,cls,topic,qos):
        p=types.SimpleNamespace(topic=topic,messages=[]);p.publish=p.messages.append;self._pubs.append(p);return p
    def create_timer(self,*args):return object()
    def destroy_node(self):return True


class CaptureAdapterTests(unittest.TestCase):
    def test_read_only_capture_constructs_receives_and_publishes_without_node_property_collisions(self):
        modules={}
        def module(name,**kw):
            m=types.ModuleType(name);m.__dict__.update(kw);modules[name]=m;return m
        module('rclpy');module('rclpy.node',Node=FakeNode)
        module('rclpy.executors',ExternalShutdownException=type('ExternalShutdownException',(Exception,),{}))
        module('rclpy.qos',QoSProfile=lambda **kw:kw,
               ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT=1),HistoryPolicy=types.SimpleNamespace(KEEP_LAST=1))
        class Message:
            def __init__(self,**kw):self.__dict__.update(kw)
        for root,names in [('std_msgs',['String']),('geometry_msgs',['PoseStamped']),
                           ('crazyflie_interfaces',['Status']),('motion_capture_tracking_interfaces',['NamedPoseArray'])]:
            module(root);module(root+'.msg',**dict.fromkeys(names,Message))
        source=Path(__file__).resolve().parents[1]/'safe_redcho_three_liveplots/capture.py'
        spec=importlib.util.spec_from_file_location('safe_redcho_three_liveplots._capture_test_double',source)
        m=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,modules),tempfile.TemporaryDirectory() as temp:
            spec.loader.exec_module(m)
            c=cfg();c.update(results_root=temp,command_file='',onboard_blocks={},record_events=False)
            n=m.Capture(c)
            try:
                for name in c['names']:n.algorithm(Message(data=json.dumps(packet(c,name))),name)
                n.onboard_log(Message(values=[.5],timestamp=0),'cf2',
                              {'fields':['position_m.z'],'recorder':'control','frequency':10})
                n.tick()
                self.assertEqual(len(n._pubs),8)
                self.assertTrue(all(p.topic.startswith('/safe_redcho/live/') for p in n._pubs))
                self.assertFalse(any('cmd_' in topic for topic in n.subs))
                for pub in n._pubs:
                    self.assertTrue(pub.messages)
                    body=json.loads(pub.messages[0].data)
                    self.assertEqual(body['schema_version'],1)
                    self.assertEqual(body['status']['stage'],'ALIGN [ACTIVE]')
                    if body['group']=='control':
                        self.assertTrue(any('cf2.firmware.position_m.z' in e['values'] for e in body['events']))
            finally:n.destroy_node()


if __name__=='__main__':unittest.main()

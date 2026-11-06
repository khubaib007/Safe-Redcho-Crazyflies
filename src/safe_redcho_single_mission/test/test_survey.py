"""Automatic ground survey tests: generated profiles never mutate source YAML."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import types
import sys
from unittest.mock import patch
import unittest
import numpy as np
import yaml
from test_core import cfg
from safe_redcho_single_mission.settings import NAMES
from safe_redcho_single_mission.survey import SurveyWindow,commit_survey,resolve_profile,quiet_graph,survey_lock,collect_survey

class SurveyTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.c=cfg();self.c['workspace']=str(self.root)
        self.c['mission']['distance_m']=.9
        self.c['safety'].update(tracking_tube_m=.12,mocap_position_uncertainty_m=.009)
        self.o=dict(name='obstacle_1',center_xy=[.9,.0],radius_m=.11,surface_clearance_m=.1,
                    position_uncertainty_m=.02,center_tolerance_m=.01,geometry_verified=False,
                    pivot_height_m=.8578,height_tolerance_m=.02,maximum_tilt_rad=.1)
        self.c['obstacles']=[self.o]
        self.positions={'cf2':[-.1,.8,.03], 'cf3':[-.7,0,.03], 'cf4':[-.1,-.8,.03], 'obstacle_1':[.6,-.35,.6078]}
        self.base=self.root/'source.yaml';self.base.write_text(yaml.safe_dump(self.c))
        self.output=self.root/'generated/combined.yaml'
    def tearDown(self):self.temp.cleanup()
    def feed(self,positions=None,window=None,start=0,count=251):
        w=SurveyWindow(self.c) if window is None else window
        p=self.positions if positions is None else positions
        for i in range(start,start+count):
            t=i*.01
            poses=[(name,xyz,[0,0,0,1]) for name,xyz in p.items()]
            w.ingest('world',100+t,poses,100+t,10+t)
        return w
    def sample(self):return self.feed().result(102.5,12.5)
    def test_detects_new_center_and_new_pivot_height(self):
        sample=self.sample();updated,_=commit_survey(self.base,self.output,sample)
        self.assertEqual(updated['obstacles'][0]['center_xy'],[.6,-.35])
        self.assertAlmostEqual(updated['obstacles'][0]['pivot_height_m'],.6078)
        self.assertFalse(updated['obstacles'][0]['geometry_verified'])
    def test_another_safe_location_is_detected_without_hardcoding(self):
        positions=deepcopy(self.positions);positions['obstacle_1']=[.55,.35,.62]
        sample=self.feed(positions).result(102.5,12.5)
        updated,_=commit_survey(self.base,self.output,sample)
        self.assertEqual(updated['obstacles'][0]['center_xy'],[.55,.35])
        self.assertEqual(updated['obstacles'][0]['pivot_height_m'],.62)
    def test_preserves_source_yaml_and_all_nonpose_settings(self):
        before=self.base.read_bytes();updated,_=commit_survey(self.base,self.output,self.sample())
        self.assertEqual(self.base.read_bytes(),before)
        for key in ('mission','control','safety','redcho','arena','backend','battery','acquisition'):
            self.assertEqual(updated[key],self.c[key])
        for key in self.o:
            if key not in ('center_xy','pivot_height_m'):
                self.assertEqual(updated['obstacles'][0][key],self.o[key])
    def test_one_snapshot_is_shared_by_all_runtime_modes(self):
        updated,directory=commit_survey(self.base,self.output,self.sample())
        self.assertEqual(resolve_profile(self.base,self.output),(directory/'profile.yaml').resolve())
        self.assertTrue((directory/'survey.json').is_file())
        starts=yaml.safe_load(Path(updated['starts_file']).read_text())
        self.assertEqual(starts['positions'],{n:self.positions[n] for n in NAMES})
    def test_changed_source_requires_resurvey(self):
        commit_survey(self.base,self.output,self.sample());self.base.write_text(self.base.read_text()+'\n# edited\n')
        with self.assertRaisesRegex(RuntimeError,'Source YAML changed'):resolve_profile(self.base,self.output)
    def test_changed_generated_profile_is_rejected(self):
        commit_survey(self.base,self.output,self.sample());c=yaml.safe_load(self.output.read_text());c['mission']['offsets_xyz']['cf2'][1]+=.01;self.output.write_text(yaml.safe_dump(c))
        with self.assertRaises((RuntimeError,ValueError)):resolve_profile(self.base,self.output)
    def test_changed_starts_is_rejected(self):
        c,_=commit_survey(self.base,self.output,self.sample());p=Path(c['starts_file']);p.write_text(p.read_text()+'\n# edited\n')
        with self.assertRaisesRegex(RuntimeError,'starts'):resolve_profile(self.base,self.output)
    def test_no_survey_gives_actionable_error(self):
        with self.assertRaisesRegex(RuntimeError,'capture'):resolve_profile(self.base,self.output)
    def test_blocked_goal_does_not_replace_previous_good_survey(self):
        commit_survey(self.base,self.output,self.sample());before=self.output.read_bytes()
        p=deepcopy(self.positions);p['obstacle_1']=[1.05,0,.60]
        bad=self.feed(p).result(102.5,12.5)
        with self.assertRaisesRegex(ValueError,'inside obstacle'):commit_survey(self.base,self.output,bad)
        self.assertEqual(self.output.read_bytes(),before)
    def test_survey_rejects_airborne_aircraft(self):
        p=deepcopy(self.positions);p['cf2'][2]=.4
        with self.assertRaisesRegex(RuntimeError,'floor'):self.feed(p).result(102.5,12.5)
    def test_missing_obstacle_is_rejected(self):
        p=deepcopy(self.positions);del p['obstacle_1']
        with self.assertRaisesRegex(RuntimeError,'obstacle_1'):self.feed(p).result(102.5,12.5)
    def test_wrong_frame_cannot_refresh_window(self):
        w=self.feed();w.ingest('other',102.51,[],102.51,12.51)
        with self.assertRaises(RuntimeError):w.result(102.51,12.51)
    def test_duplicate_obstacle_clears_window(self):
        w=self.feed();poses=[(n,p,[0,0,0,1]) for n,p in self.positions.items()];poses.append(poses[-1])
        w.ingest('world',102.51,poses,102.51,12.51)
        with self.assertRaisesRegex(RuntimeError,'obstacle_1'):w.result(102.51,12.51)
    def test_invalid_pose_clears_window(self):
        w=self.feed();poses=[(n,p,[0,0,0,1]) for n,p in self.positions.items()];poses[-1]=('obstacle_1',[float('nan'),0,0],[0,0,0,1])
        w.ingest('world',102.51,poses,102.51,12.51)
        with self.assertRaisesRegex(RuntimeError,'obstacle_1'):w.result(102.51,12.51)
    def test_invalid_timestamp_cannot_refresh_survey(self):
        w=self.feed();poses=[(n,p,[0,0,0,1]) for n,p in self.positions.items()]
        w.ingest('world',102.5,poses,102.51,12.51)
        with self.assertRaises(RuntimeError):w.result(102.51,12.51)
    def test_stale_receipts_and_stale_source_are_rejected(self):
        w=self.feed()
        for now,mono in [(102.6,12.6),(102.6,12.5),(102.5,12.6)]:
            with self.assertRaisesRegex(RuntimeError,'Stale'):w.result(now,mono)
    def test_receipt_gap_is_rejected_even_with_regular_source_stamps(self):
        w=SurveyWindow(self.c)
        for i in range(201):
            t=i*.01;mono=10+t if i<190 else 10+t+.09
            w.ingest('world',100+t,[(n,p,[0,0,0,1]) for n,p in self.positions.items()],100+t,mono)
        with self.assertRaisesRegex(RuntimeError,'coverage/rate'):w.result(102.,12.09)
    def test_moving_obstacle_is_rejected(self):
        w=SurveyWindow(self.c)
        for i in range(251):
            p=deepcopy(self.positions);p['obstacle_1'][0]+=.02*np.sin(i*.07)
            self.feed(p,w,start=i,count=1)
        with self.assertRaisesRegex(RuntimeError,'moving'):w.result(102.5,12.5)
    def test_tilted_obstacle_is_rejected(self):
        w=SurveyWindow(self.c)
        for i in range(251):
            t=i*.01;poses=[(n,p,[0,0,0,1] if n!='obstacle_1' else [np.sin(.1),0,0,np.cos(.1)]) for n,p in self.positions.items()]
            w.ingest('world',100+t,poses,100+t,10+t)
        with self.assertRaisesRegex(RuntimeError,'upright'):w.result(102.5,12.5)
    def test_active_agents_or_commands_block_survey(self):
        for name,topic in [('single_mission_agent_cf2',None),('single_mission_watchdog',None),(None,'/cf3/cmd_position')]:
            node=types.SimpleNamespace(get_node_names_and_namespaces=lambda:[(name,'/')],count_publishers=lambda t:int(t==topic))
            with self.assertRaisesRegex(RuntimeError,'Stop|Active'):quiet_graph(node,self.c)
    def test_missing_new_samples_after_old_stable_window_cannot_pass(self):
        w=self.feed();self.feed(window=w,start=300,count=10)
        with self.assertRaisesRegex(RuntimeError,'coverage/rate|survey window'):w.result(103.09,13.09)
    def test_ros_collection_entrypoint_commits_without_command_api(self):
        clock=types.SimpleNamespace(t=0.)
        fake_ros=types.ModuleType('rclpy');fake_node=types.ModuleType('rclpy.node')
        fake_qos=types.ModuleType('rclpy.qos');fake_qos.qos_profile_sensor_data=object()
        fake_interfaces=types.ModuleType('motion_capture_tracking_interfaces.msg');fake_interfaces.NamedPoseArray=object
        instances=[]
        class Node:
            def __init__(node,name):node.callback=None;node.logs=[];instances.append(node)
            def create_subscription(node,*args):node.callback=args[2];return object()
            def get_clock(node):return types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=int((100+clock.t)*1e9)))
            def get_logger(node):return types.SimpleNamespace(info=node.logs.append)
            def get_node_names_and_namespaces(node):return []
            def count_publishers(node,topic):return int(topic=='/poses')
            def destroy_node(node):pass
        fake_node.Node=Node
        fake_ros.init=lambda:None;fake_ros.try_shutdown=lambda:None
        def spin(node,timeout_sec):
            clock.t+=.01
            stamp=100+clock.t;sec=int(stamp)
            poses=[]
            for name,xyz in self.positions.items():
                poses.append(types.SimpleNamespace(name=name,pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=xyz[0],y=xyz[1],z=xyz[2]),
                    orientation=types.SimpleNamespace(x=0.,y=0.,z=0.,w=1.))))
            node.callback(types.SimpleNamespace(header=types.SimpleNamespace(frame_id='world',
                stamp=types.SimpleNamespace(sec=sec,nanosec=int((stamp-sec)*1e9))),poses=poses))
        fake_ros.spin_once=spin
        mocks={'rclpy':fake_ros,'rclpy.node':fake_node,'rclpy.qos':fake_qos,
               'motion_capture_tracking_interfaces.msg':fake_interfaces}
        args=types.SimpleNamespace(config=str(self.base),output=str(self.output),timeout=5.)
        with patch.dict(sys.modules,mocks),patch('safe_redcho_single_mission.survey.time.monotonic',lambda:10+clock.t):
            collect_survey(args)
        self.assertTrue(self.output.is_file())
        self.assertTrue(any('SURVEY PASS' in line for line in instances[0].logs))
        self.assertAlmostEqual(yaml.safe_load(self.output.read_text())['obstacles'][0]['pivot_height_m'],.6078)

    def test_immutable_snapshot_tampering_is_rejected(self):
        c,directory=commit_survey(self.base,self.output,self.sample())
        p=directory/'profile.yaml';p.write_text(p.read_text()+'\n# altered\n')
        with self.assertRaisesRegex(RuntimeError,'Immutable'):resolve_profile(self.base,self.output)
    def test_concurrent_surveys_are_rejected(self):
        with survey_lock(self.output):
            with self.assertRaisesRegex(RuntimeError,'Another ground survey'):
                with survey_lock(self.output):pass
        with survey_lock(self.output):pass

    def test_output_cannot_overwrite_source(self):
        before=self.base.read_bytes()
        with self.assertRaises(ValueError):commit_survey(self.base,self.base,self.sample())
        self.assertEqual(self.base.read_bytes(),before)

if __name__=='__main__':unittest.main()

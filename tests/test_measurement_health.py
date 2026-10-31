from collections import deque
import ast
from contextlib import redirect_stdout
from copy import deepcopy
import importlib.util
import io
import json
import math
from pathlib import Path
import shutil
import sys
import tempfile
import time
from types import ModuleType,SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
def load_module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec)
    sys.modules[name]=m;spec.loader.exec_module(m);return m
from safe_redcho_flight import health_sync as sync

def cfg():return yaml.safe_load((ROOT/'tests/fixtures/flight.yaml').read_text())
def qyaw(a):return np.array([0.,0.,math.sin(a/2),math.cos(a/2)])
def pose(p,q=None):
    q=qyaw(0) if q is None else q
    return NS(position=NS(x=p[0],y=p[1],z=p[2]),orientation=NS(x=q[0],y=q[1],z=q[2],w=q[3]))
def node():
    n=NS(names=[f'cf{i}' for i in range(1,7)],profile=cfg(),get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000)))
    sync.history(n);return n
def entry(source,receipt,p=(0,0,.03),q=None):
    return {'source_ns':int(source*1e9),'receipt_mono':receipt,'receipt_ros_ns':int(source*1e9),
            'p':np.asarray(p,float),'q':qyaw(0) if q is None else q}

class MeasurementTests(unittest.TestCase):
    def test_velocity_uses_source_time_not_compressed_receipt_time(self):
        rows=[entry(9.8+.01*i,99.91+.006*i,p=(.1*.01*i,0,.03)) for i in range(16)]
        vv,stats=sync.fit_source_velocity(rows,.16,.08,100.,.08)
        np.testing.assert_allclose(vv,[.1,0,0],atol=1e-12)
        t=np.array([r['receipt_mono'] for r in rows]);t-=t.mean()
        wrong=t@np.array([r['p'] for r in rows])/(t@t)
        self.assertGreater(wrong[0],.16)
        self.assertEqual(stats['fit_samples'],16)

    def test_source_gap_is_not_smoothed_or_extrapolated(self):
        rows=[entry(9.84+.01*i,99.85+.01*i) for i in range(5)]
        rows.extend(entry(9.98+.01*i,99.98+.01*i) for i in range(2))
        with self.assertRaisesRegex(RuntimeError,'gap'):
            sync.fit_source_velocity(rows,.2,.08,100.,.08)

    def test_stale_pose_rejected(self):
        rows=[entry(9.8+.01*i,99.6+.01*i) for i in range(16)]
        with self.assertRaisesRegex(RuntimeError,'fresh'):
            sync.fit_source_velocity(rows,.16,.08,100.,.08)

    def test_duplicate_timestamp_cannot_refresh_body(self):
        n=node();msg=NS(header=NS(frame_id='world',stamp=NS(sec=10,nanosec=0)),poses=[NS(name='cf1',pose=pose([0,0,.03]))])
        sync.observe(n,'/poses',msg);sync.observe(n,'/poses',msg)
        self.assertEqual(len(sync.history(n)['mocap']['cf1']),1)
        self.assertEqual(n._health_sync_counters['mocap_cf1_nonincreasing_header'],1)

    def test_duplicate_body_names_are_not_accepted(self):
        n=node();p=NS(name='cf1',pose=pose([0,0,.03]))
        msg=NS(header=NS(frame_id='world',stamp=NS(sec=10,nanosec=0)),poses=[p,p])
        sync.observe(n,'/poses',msg)
        self.assertFalse(sync.history(n)['mocap']['cf1'])

    def test_missing_body_is_not_refreshed_by_other_bodies(self):
        n=node()
        for i in range(2):
            p=NS(name=f'cf{i+1}',pose=pose([0,0,.03]))
            sync.observe(n,'/poses',NS(header=NS(frame_id='world',stamp=NS(sec=10,nanosec=i+1)),poses=[p]))
        self.assertEqual(len(sync.history(n)['mocap']['cf1']),1)
        self.assertEqual(len(sync.history(n)['mocap']['cf2']),1)
        self.assertFalse(sync.history(n)['mocap']['cf3'])

    def test_old_or_wrong_frame_headers_are_rejected(self):
        n=node();p=NS(name='cf1',pose=pose([0,0,.03]))
        sync.observe(n,'/poses',NS(header=NS(frame_id='world',stamp=NS(sec=8,nanosec=0)),poses=[p]))
        sync.observe(n,'/poses',NS(header=NS(frame_id='other',stamp=NS(sec=10,nanosec=0)),poses=[p]))
        self.assertFalse(sync.history(n)['mocap']['cf1'])

    def test_zero_onboard_header_retains_fresh_pose_without_alignment_claim(self):
        n=node();msg=NS(header=NS(frame_id='world',stamp=NS(sec=0,nanosec=0)),pose=pose([0,0,.03]))
        sync.observe(n,'/cf1/pose',msg)
        rows=sync.history(n)['onboard']['cf1']
        self.assertEqual(len(rows),1)
        self.assertEqual(rows[-1]['time_basis'],'receipt_fallback_unaligned')
        sync.history(n)['mocap']['cf1'].append(entry(10.,time.monotonic(),q=qyaw(.6)))
        result=sync.aligned_pair(n,'cf1')
        self.assertGreater(result['angle_rad'],.35)
        self.assertIsNone(result['header_pair_skew_s'])
        self.assertIn('Unaligned',result['timing_scope'])

    def test_relative_onboard_headers_are_explicit_and_duplicates_rejected(self):
        n=node();n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=1_792_761_000_000_000_000))
        msg=NS(header=NS(frame_id='world',stamp=NS(sec=20,nanosec=0)),pose=pose([0,0,.03]))
        sync.observe(n,'/cf1/pose',msg);sync.observe(n,'/cf1/pose',msg)
        rows=sync.history(n)['onboard']['cf1']
        self.assertEqual(len(rows),1)
        self.assertEqual(rows[-1]['raw_source_ns'],20_000_000_000)
        self.assertEqual(rows[-1]['time_basis'],'receipt_fallback_unaligned')

    def test_matching_orientation_not_current_vs_old(self):
        n=node();h=sync.history(n);now=time.monotonic()
        h['mocap']['cf1'].extend([entry(9.94,now-.06,q=qyaw(.1)),entry(9.95,now-.05,q=qyaw(.2)),
                                 entry(10.,now,q=qyaw(.7))])
        h['onboard']['cf1'].append(entry(9.945,now-.05,q=qyaw(.15)))
        r=sync.aligned_pair(n,'cf1')
        self.assertLess(r['angle_rad'],1e-7)
        self.assertGreater(r['latest_unaligned_angle_rad'],.5)

    def test_quaternion_sign_is_not_an_attitude_error(self):
        q=qyaw(.3)
        self.assertLess(sync.angle(q,-q),1e-7)

    def test_real_orientation_disagreement_stays_visible(self):
        n=node();now=time.monotonic();h=sync.history(n)
        h['mocap']['cf1'].append(entry(10.,now,q=qyaw(.6)))
        h['onboard']['cf1'].append(entry(10.,now,q=qyaw(0)))
        self.assertGreater(sync.aligned_orientation(n,'cf1'),.35)

    def test_no_attitude_extrapolation_through_loss(self):
        n=node();now=time.monotonic();h=sync.history(n)
        h['mocap']['cf1'].append(entry(9.8,now))
        h['onboard']['cf1'].append(entry(10.,now))
        with self.assertRaisesRegex(RuntimeError,'40 ms'):sync.aligned_pair(n,'cf1')

    def test_subscription_hook_handles_node_initialization_before_names_exist(self):
        callback=lambda msg:msg
        self.assertIs(sync.wrap_callback(NS(),'/clock',callback),callback)

    def test_capture_failure_never_prevents_original_callback(self):
        n=node();seen=[];cb=sync.wrap_callback(n,'/poses',lambda msg:seen.append(msg))
        cb(NS())
        self.assertEqual(len(seen),1)
        self.assertTrue(n._health_sync_counters)

class LandingTests(unittest.TestCase):
    def qualified_node(self,now=100.):
        n=node();n.config={'_starts':{name:[0,0,.03] for name in n.names}}
        for name in n.names:
            sync.history(n)['mocap'][name].extend(entry(9.51+.01*i,now-.49+.01*i) for i in range(50))
        return n

    def test_recent_ground_window_can_qualify(self):
        self.assertTrue(sync.touchdown_evidence(self.qualified_node(),'cf1',100.)['verified'])

    def test_old_floor_pose_cannot_qualify(self):
        n=self.qualified_node();self.assertFalse(sync.touchdown_evidence(n,'cf1',101.)['verified'])

    def test_airborne_pose_cannot_qualify(self):
        n=self.qualified_node()
        for row in sync.history(n)['mocap']['cf1']:row['p'][2]=.65
        self.assertFalse(sync.touchdown_evidence(n,'cf1',100.)['verified'])

    def test_latest_good_floor_pose_does_not_hide_moving_history(self):
        n=self.qualified_node();sync.history(n)['mocap']['cf1'][1]['p'][0]=.1
        self.assertFalse(sync.touchdown_evidence(n,'cf1',100.)['verified'])

    def test_no_disarm_without_recent_ground_evidence(self):
        mod=ModuleType('crazyflie_interfaces.srv');mod.Arm=NS(Request=NS)
        with patch.dict(sys.modules,{'crazyflie_interfaces':ModuleType('crazyflie_interfaces'),'crazyflie_interfaces.srv':mod}):
            n=self.qualified_node();n.spin_for=lambda seconds:None
            n.call_many=lambda *a: self.fail('must not disarm stale aircraft')
            report={'touchdown_verified':[],'disarm_requests_acknowledged':[]}
            with patch.object(sync.time,'monotonic',side_effect=[101.,105.]):
                result=sync.recheck_touchdown(n,report)
            self.assertEqual(len(result['touchdown_not_verified']),6)

    def test_reacquired_stationary_ground_evidence_allows_disarm(self):
        mod=ModuleType('crazyflie_interfaces.srv');mod.Arm=NS(Request=NS)
        with patch.dict(sys.modules,{'crazyflie_interfaces':ModuleType('crazyflie_interfaces'),'crazyflie_interfaces.srv':mod}):
            n=self.qualified_node();n.spin_for=lambda seconds:None;calls=[]
            n.call_many=lambda label,requests:calls.append((label,requests))
            with patch.object(sync.time,'monotonic',return_value=100.):
                report=sync.recheck_touchdown(n,{'touchdown_verified':[]})
            self.assertEqual(report['touchdown_verified'],n.names)
            self.assertTrue(all(not req.arm for req in calls[0][1].values()))


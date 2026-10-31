import ast
import csv
import json
import os
from pathlib import Path
import queue
import tempfile
import time
import unittest
import numpy as np
from plot_support import cfg,packet
from safe_redcho_three_mission.liveplots.engine import Engine
from safe_redcho_three_mission.liveplots.buffer import Histories
from safe_redcho_three_mission.liveplots.io import CommandTail,EventWriter
from safe_redcho_three_mission.liveplots.configuration import GROUPS

T=1_000_000_000_000


class DataTests(unittest.TestCase):
    def setUp(self): self.c=cfg(); self.e=Engine(self.c,T)
    def feed(self,p): self.e.algorithm(p,p['name'],p['producer_ros_ns'],p['producer_ros_ns'])
    def test_exact_cycle_convergence_matches_mean_input_and_not_just_agreement(self):
        for n in self.c['names']: self.feed(packet(self.c,n))
        e=self.e.out['velocity_redcho'][-1]
        self.assertAlmostEqual(e['values']['fleet.disagreement.velocity_mps'],.02)
        self.assertAlmostEqual(e['values']['fleet.average_error.velocity_mps'],.02)
        self.assertEqual(e['values']['fleet.average.velocity_mps.x'],1.)
    def test_mixed_cycles_stages_flags_never_form_convergence(self):
        a=packet(self.c,'cf2'); b=packet(self.c,'cf3',seq=2); c=packet(self.c,'cf4',stage='STOP')
        for p in (a,b,c): self.feed(p)
        self.assertFalse(any('fleet.disagreement.velocity_mps' in e['values'] for e in self.e.out['velocity_redcho']))
    def test_wrong_hash_and_duplicate_cycles_rejected(self):
        p=packet(self.c,'cf2');self.feed(p);self.feed(p)
        p=packet(self.c,'cf3');p['config_hash']='wrong';self.feed(p)
        self.assertEqual(self.e.rejected,2)
    def test_new_session_sequence_restart_is_valid(self):
        p=packet(self.c,'cf2',seq=10);self.feed(p)
        p=packet(self.c,'cf2');p['session']='next';self.feed(p)
        self.assertEqual(self.e.rejected,0)
    def test_missing_groups_are_missing_not_zero(self):
        p=packet(self.c,'cf2');p['data']={};self.feed(p)
        self.assertIsNone(self.e.out['reference'][-1]['values']['cf2.svf_v.x'])
        self.assertIsNone(self.e.out['hocbf'][-1]['values']['cf2.solver_invoked'])
    def test_false_solver_flag_is_real_zero_and_kept_distinct_from_missing(self):
        self.feed(packet(self.c,'cf2'))
        self.assertEqual(self.e.out['hocbf'][-1]['values']['cf2.solver_invoked'],0.)
    def test_partial_or_failed_diagnostics_cannot_make_convergence(self):
        for n in self.c['names']:
            p=packet(self.c,n)
            if n=='cf4':p['error']='infeasible'
            self.feed(p)
        self.assertFalse(any(k.startswith('fleet.disagreement') for e in self.e.out['velocity_redcho'] for k in e['values']))
    def test_pending_cycles_bounded(self):
        for seq in range(1,80):self.feed(packet(self.c,'cf2',seq=seq))
        self.assertEqual(len(self.e.pending),32);self.assertGreater(self.e.incomplete,0)
    def test_noisy_stream_velocity_uses_source_time(self):
        for i in range(30):
            t=T+i*8_333_333
            self.e.pose('cf2',[.1*(t-T)/1e9,0.,.5],[0,0,0,1],t,t,t)
        self.assertAlmostEqual(self.e.out['flight'][-1]['values']['cf2.mocap.speed'],.1,places=8)
    def test_duplicate_stale_or_invalid_pose_does_not_refresh(self):
        self.e.pose('cf2',[0,0,.5],[0,0,0,1],T,T,T)
        for stamp,receipt,q in [(T,T,[0,0,0,1]),(T+1,T+1_000_000_000,[0,0,0,1]),(T+2,T+2,[0,0,0,0])]:
            self.assertFalse(self.e.pose('cf2',[0,0,.5],q,stamp,receipt,receipt))
        self.assertEqual(self.e.streams['cf2.mocap']['source_ns'],T)
    def test_pair_distance_requires_complete_same_frame(self):
        self.e.pair_batch({'cf2':[0,0,0],'cf3':[0,1,0]},T,T,T)
        self.assertFalse(self.e.out['flight'])
        self.e.pair_batch({'cf2':[0,1,0],'cf3':[0,0,0],'cf4':[0,-1,0]},T,T,T)
        self.assertEqual(self.e.out['flight'][-1]['values']['pair.cf3_cf4'],1.)
    def command(self,stamp):
        return dict(name='cf2',ros_ns=str(stamp),wall_monotonic_s=str(stamp/1e9),
                    published_x_m='0.0',published_y_m='0.0',published_z_m='.5',interface='cmd_position',stage='ALIGN')
    def test_delayed_csv_matches_historical_pose_not_latest_pose(self):
        for i in range(61):
            t=T+i*10_000_000
            self.e.pose('cf2',[(t-T)/1e9,0.,.5],[0,0,0,1],t,t,t)
        self.e.command(self.command(T+400_000_000),T+600_000_000,T+600_000_000)
        self.assertAlmostEqual(self.e.out['flight'][-1]['values']['cf2.tracking.norm'],.4)
        self.e.command(self.command(T+420_000_000),T+600_000_000,T+600_000_000)
        self.assertAlmostEqual(self.e.out['timing'][-1]['values']['cf2.command_gap_ms'],20.)
    def test_tracking_missing_when_alignment_unavailable(self):
        self.e.command(self.command(T),T,T)
        self.assertIsNone(self.e.out['flight'][-1]['values']['cf2.tracking.norm'])
    def test_onboard_stale_even_when_mocap_continues(self):
        self.e.pose('cf3',[0,0,.5],[0,0,0,1],T,T,T,'onboard')
        self.e.pose('cf3',[0,0,.5],[0,0,0,1],T+301_000_000,T+301_000_000,T+301_000_000)
        states=self.e.banner(T+301_000_000,T+301_000_000)['streams']
        self.assertEqual(states['cf3.onboard'],'STALE');self.assertEqual(states['cf3.mocap'],'LIVE')
    def test_row_units_do_not_mix_squared_pair_and_linear_wall(self):
        self.feed(packet(self.c,'cf2'))
        v=self.e.out['hocbf'][-1]['values']
        self.assertEqual(v['cf2.h_squared'],.25);self.assertEqual(v['cf2.h_wall'],.5)
    def test_nonfinite_never_serializes_as_nan(self):
        self.e.emit('flight',T,{'missing':float('nan')},T,T)
        self.assertIsNone(self.e.out['flight'][-1]['values']['missing'])
        json.dumps(list(self.e.out['flight']),allow_nan=False)


class StorageTests(unittest.TestCase):
    def test_history_is_bounded_and_missing_stays_nan(self):
        h=Histories(5,10.)
        for i in range(12):
            h.accept(dict(epoch='a',origin_ns=T,batch=i+1,now_ns=T+i*10_000_000,
                     events=[dict(t_ns=T+i*10_000_000,values={'x':None if i==11 else i})],status={'stage':'ALIGN'}))
        self.assertEqual(len(h.series['x']),5);self.assertTrue(np.isnan(h.data('x')[1][-1]))
    def test_new_epoch_and_long_gap_not_joined(self):
        h=Histories()
        for i,t in enumerate([T,T+1_000_000_000]):
            h.accept(dict(epoch='a',origin_ns=T,batch=i+1,now_ns=t,
                events=[dict(t_ns=t,values={'x':i,'y':i})],status={'stage':'ALIGN'}))
        self.assertTrue(np.isnan(h.data('x')[0]).any());self.assertTrue(np.isnan(h.xy('x','y')[0]).any())
        h.accept(dict(epoch='b',origin_ns=T,batch=1,now_ns=T,events=[],status={'stage':'GROUND'}))
        self.assertFalse(h.series)
    def test_partial_csv_line_retried_without_data_loss(self):
        tail=CommandTail.__new__(CommandTail)
        tail.path=tail.handle=tail.header=tail.inode=None;tail.queue=queue.Queue(100);tail.errors=tail.dropped=0
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'position_commands.csv'
            p.write_text('name,ros_ns,published_z_m\ncf2,1000,0.')
            tail.poll(p);self.assertTrue(tail.queue.empty())
            with p.open('a') as f:f.write('5\n')
            tail.poll(p);self.assertEqual(tail.queue.get()[1]['published_z_m'],'0.5')
            tail.handle.close()
    def test_old_mission_file_not_auto_replayed(self):
        tail=CommandTail.__new__(CommandTail);tail.explicit='';tail.birth=time.time();
        with tempfile.TemporaryDirectory() as temp:
            tail.root=Path(temp);p=tail.root/'three_mission_hover_old/position_commands.csv';p.parent.mkdir();p.write_text('old')
            os.utime(p,(tail.birth-100,tail.birth-100));self.assertIsNone(tail.choose())
    def test_event_writer_and_summary(self):
        with tempfile.TemporaryDirectory() as temp:
            w=EventWriter(temp,{});w.submit('flight',{'values':{'missing':None}});summary=w.close()
            self.assertEqual(summary['written_events'],1);self.assertFalse(summary['errors'])
            self.assertIsNone(json.loads((w.directory/'flight.jsonl').read_text())['values']['missing'])
    def test_package_has_no_flight_service_calls_or_cmd_subscription(self):
        root=Path(__file__).resolve().parents[1]/'safe_redcho_three_mission/liveplots'
        for p in root.glob('*.py'):
            tree=ast.parse(p.read_text())
            for n in ast.walk(tree):
                if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute):
                    self.assertNotIn(n.func.attr,('create_client','call_async'),str(p))
                    if n.func.attr=='create_subscription':
                        self.assertNotIn('cmd_position',ast.unparse(n),str(p))


if __name__=='__main__':unittest.main()

import copy,json,math,tempfile,unittest
from pathlib import Path
import numpy as np
from safe_redcho_three_mission.settings import load,make_plan,pair_distance,virtual_box,NAMES,validate
from safe_redcho_three_mission.core import LocalController,StageMachine,Reference,accept_batch,wrap
from safe_redcho_three_mission.preview import run
from safe_redcho_three_mission.cbf_qp import solve
from safe_redcho_three_mission.health import History,rotation_error
from safe_redcho_three_mission.backend import radios

ROOT=Path(__file__).resolve().parents[1]
def cfg():return load(ROOT/'config/mission.yaml')
def plan(c):
 z=c['arena']['floor_z_m'];return make_plan(c,[[-.05,.76,z],[-.78,0,z],[-.05,-.76,z]],[2.5,-1.8,.9])

class CoreTests(unittest.TestCase):
 def test_arbitrary_yaw_is_accepted(self):
  c=cfg();p=plan(c);self.assertAlmostEqual(p['initial_yaw'][0],2.5)
 def test_start_rectangle_is_not_a_separation_exemption(self):
  c=cfg();z=c['arena']['floor_z_m']
  with self.assertRaisesRegex(ValueError,'spacing'):make_plan(c,[[-.1,.1,z],[-.2,.2,z],[-.3,.3,z]],[0,1,2])
 def test_outside_region_is_rejected(self):
  c=cfg();p=plan(c);p['ground'][0][0]=.01
  with self.assertRaisesRegex(ValueError,'region'):make_plan(c,p['ground'],p['initial_yaw'])
 def test_assignment_is_recorded_and_follows_initial_y(self):
  c=cfg();p=plan(c);g=p['ground'];g[0],g[2]=g[2],g[0];p=make_plan(c,g,p['initial_yaw']);self.assertEqual(p['offsets'][0][1],-.95);self.assertEqual(p['offsets'][2][1],.95)
 def test_final_one_meter_sweep_does_not_fit_old_point_fifteen_start(self):
  c=cfg();c['mission']['distance_m']=1
  with self.assertRaisesRegex(ValueError,'arena'):validate(c)
 def test_height_proposals_are_absolute_not_slot_offsets(self):
  c=cfg();p=plan(c);r=Reference(c,0,p);v,position,*_=r.step('HEIGHT',[4,4,4]);np.testing.assert_allclose(position[:,0],p['hover'][0]);np.testing.assert_allclose(v,0)
 def test_voltage_actually_changes_reference_phase(self):
  c=cfg();p=plan(c);fast=Reference(c,0,p);slow=Reference(c,0,p)
  for _ in range(100):fast.step('FORMATION',[4,4,4]);slow.step('FORMATION',[3.4,4,4])
  self.assertLess(slow.phase,fast.phase);self.assertLess(slow.shape_phase,fast.shape_phase)
 def test_battery_failure_is_not_replaced_with_fabricated_voltage(self):
  c=cfg();r=Reference(c,0,plan(c))
  for values in ([float('nan'),4,4],[3.2,4,4]):
   with self.assertRaises(RuntimeError):r.step('FORMATION',values)
 def test_yaw_takes_short_path_and_obeys_rate_budget(self):
  c=cfg();p=plan(c);p['initial_yaw']=[3.12,-3.12,2.];m=StageMachine(c,p);m.stage='YAW';last=m.yaw_reference();maximum=0
  for k in range(1000):
   m.elapsed=k*c['control']['dt_s'];ys=m.yaw_reference();maximum=max(maximum,float(max(abs(wrap(ys-last)))/c['control']['dt_s']));last=ys
  self.assertLessEqual(maximum,c['acquisition']['yaw_rate_limit_radps']*1.01);np.testing.assert_allclose(wrap(last),0,atol=1e-10)
 def test_complete_batch_rejects_nan(self):
  c=cfg();q=np.asarray(plan(c)['hover']);u=np.zeros((3,3));u[1,0]=np.nan
  with self.assertRaisesRegex(RuntimeError,'nonfinite'):accept_batch(q,np.zeros((3,3)),np.zeros((3,3)),u,c)
 def test_controller_requires_both_neighbors(self):
  c=cfg();p=plan(c);core=LocalController(c,0,p);q=p['hover'];frame=dict(seq=1,stage='WARMUP',q=q,v=np.zeros((3,3)).tolist(),previous=np.zeros((3,3)).tolist(),voltage=[4,4,4]);core.prepare(frame)
  with self.assertRaisesRegex(ValueError,'peers'):core.finish({})
 def test_reciprocal_qp_brakes_closing_pair(self):
  c=cfg();q=np.array([[0,.475,.53],[0,-.475,.53],[-1.0,0,.53]]);v=np.array([[0,-.05,0],[0,.05,0],[0,0,0]]);prev=np.zeros((3,3));prev[0,1]=.1;prev[1,1]=-.1
  u,info=solve(0,[0,-.2,0],q,v,prev,c,diagnostics=True);self.assertGreater(u[1],0);self.assertTrue(info['diagnostics']['inequality_solver_invoked'])
 def test_airborne_volume_refuses_unreachable_height(self):
  c=cfg();c['acquisition']['takeoff_heights_above_floor_m']['cf2']=1.2
  with self.assertRaisesRegex(ValueError,'height'):validate(c)
 def test_radio_parser_supports_three_dongles_without_editing_old_fleet(self):
  import yaml
  with tempfile.TemporaryDirectory() as temp:
   c=cfg();c['workspace']=temp;c['fleet_config_file']='fleet.yaml';p=Path(temp)/'fleet.yaml';p.write_text(yaml.safe_dump({'robots':{n:{'uri':f'radio://{i}/{40+20*i}/2M/E7E7E7E70{i+2}'} for i,n in enumerate(NAMES)}}));self.assertEqual(len(radios(c)),3)
 def test_full_mission_synthetic_preview(self):
  c=cfg();report,rows=run(c,plan(c),trace=False);self.assertEqual(report['status'],'PASS');self.assertLess(report['maximum_synthetic_tracking_error_m'],.10);self.assertEqual([x[1] for x in report['stage_events']],['WARMUP','HEIGHT','YAW','FORMATION','SETTLE','FORWARD','STOP']);self.assertLess(max(report['final_endpoint_errors_m']),.10)
 def test_acquisition_only_does_not_run_forward(self):
  c=cfg();report,rows=run(c,plan(c),acquire_only=True);self.assertEqual(report['status'],'PASS');self.assertNotIn('FORWARD',[x[1] for x in report['stage_events']])

class HistoryTests(unittest.TestCase):
 def test_old_sample_cannot_refresh_freshness(self):
  h=History();q=np.array([0,0,0,1]);self.assertTrue(h.add([0,0,0],q,10,10,1,.08));self.assertFalse(h.add([0,0,0],q,10,10.01,2,.08))
  with self.assertRaisesRegex(RuntimeError,'Stale'):h.latest(10.2,2,.08)
 def test_velocity_uses_source_time(self):
  h=History();q=[0,0,0,1]
  for i in range(20):t=10+i*.01;h.add([.2*(t-10),0,0],q,t,t,i*.01,.08)
  np.testing.assert_allclose(h.velocity(.16,.08),[.2,0,0],atol=1e-10)
 def test_quaternion_sign_does_not_change_orientation(self):self.assertAlmostEqual(rotation_error(np.array([0,0,0,1]),np.array([0,0,0,-1])),0)
 def test_stale_pose_cannot_verify_touchdown(self):
  h=History();q=[0,0,0,1]
  for i in range(70):h.add([0,0,.03],q,10+i*.01,10+i*.01,i*.01,.08)
  self.assertTrue(h.touchdown(.03,10.69,.69,.08)['verified']);self.assertFalse(h.touchdown(.03,11.,1.,.08)['verified'])

if __name__=='__main__':unittest.main()

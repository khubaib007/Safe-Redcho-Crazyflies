"""Geometry, live tracking and failure gates; these are not hardware tests."""
import copy
import unittest
import numpy as np
from test_core import cfg
from safe_redcho_three_mission.obstacles import (
    obstacle_radius, validate_obstacles, check_live_obstacles,
    check_reference_clearance, require_verified_geometry, check_measured_clearance)
from safe_redcho_three_mission.settings import validate
from safe_redcho_three_mission.health import History
from safe_redcho_three_mission.cbf_qp import constraints, minimum_held_distance
from safe_redcho_three_mission.telemetry_trace import barrier_rows

class ObstacleTests(unittest.TestCase):
    def setUp(self):
        self.c=cfg()
        self.c['safety'].update(tracking_tube_m=.12,mocap_position_uncertainty_m=.009)
        self.o=dict(name='obstacle_1',center_xy=[.6,.35],radius_m=.11,surface_clearance_m=.1,
                    position_uncertainty_m=.02,center_tolerance_m=.01,geometry_verified=False,pivot_height_m=.85,height_tolerance_m=.02,maximum_tilt_rad=.1)
        self.c['obstacles']=[self.o]
        self.p=np.array([[-.2,.95,.5],[-.4,0,.5],[-.2,-.95,.5]])
    def history(self):
        h=History('/poses[obstacle_1]')
        for i in range(60):
            t=i*.01;h.add([.6,.35,.85],[0,0,0,1],100+t,100+t,10+t,.08)
        return {'obstacle_1':h}
    def test_inflation_uses_obstacle_uncertainty(self):
        self.assertAlmostEqual(obstacle_radius(self.o,self.c),.439)
        self.assertAlmostEqual(obstacle_radius(self.o,self.c,False),.289)
    def test_schema_rejects_duplicate_name(self):
        self.c['obstacles'].append(copy.deepcopy(self.o))
        with self.assertRaises(ValueError):validate_obstacles(self.c)
    def test_schema_rejects_nonfinite_or_wrong_dimension(self):
        for center in ([float('nan'),0],[0,0,0]):
            self.o['center_xy']=center
            with self.assertRaises(ValueError):validate_obstacles(self.c)
    def test_uncertainty_must_cover_movement_and_sensor_error(self):
        self.o['position_uncertainty_m']=.01
        with self.assertRaisesRegex(ValueError,'uncertainty'):validate_obstacles(self.c)
    def test_old_obstacle_position_blocks_middle_final_goal(self):
        self.c['mission']['distance_m']=.9
        self.o['center_xy']=[.916636765,.070338063]
        with self.assertRaisesRegex(ValueError,'inside obstacle'):validate(self.c)
    def test_unverified_geometry_cannot_fly(self):
        with self.assertRaisesRegex(RuntimeError,'footprint'):require_verified_geometry(self.c)
        self.o['geometry_verified']=True;require_verified_geometry(self.c)
    def test_stationary_live_tracking_passes(self):
        result=check_live_obstacles(self.history(),self.p,self.c,100.59,10.59)
        self.assertAlmostEqual(result['obstacle_1']['center_error_m'],0)
    def test_missing_and_stale_obstacle_fail_with_name(self):
        for histories in ({'obstacle_1':History('/poses[obstacle_1]')},self.history()):
            with self.assertRaisesRegex(RuntimeError,'obstacle_1'):
                check_live_obstacles(histories,self.p,self.c,101.,11.)
    def test_height_and_tilt_changes_abort(self):
        for mode in ('height','tilt'):
            h=self.history()
            row=h['obstacle_1'].rows[-1]
            pos=row[2].copy();quat=row[3].copy()
            if mode=='height':pos[2]+=.1
            else:quat=np.array([np.sin(.1),0,0,np.cos(.1)])
            h['obstacle_1'].rows[-1]=(row[0],row[1],pos,quat)
            with self.assertRaisesRegex(RuntimeError,'height/tilt'):
                check_live_obstacles(h,self.p,self.c,100.59,10.59)

    def test_center_move_aborts(self):
        self.o['center_xy']=[.62,.35]
        with self.assertRaisesRegex(RuntimeError,'mismatch'):
            check_live_obstacles(self.history(),self.p,self.c,100.59,10.59)
    def test_measured_clearance_checks_actual_center(self):
        p=self.p.copy();p[1,:2]=[.6,.35]
        with self.assertRaisesRegex(RuntimeError,'cf3'):check_measured_clearance(p,self.c)
    def test_vertical_bypass_does_not_disable_clearance(self):
        p=self.p.copy();p[1]=[.6,.35,3.]
        with self.assertRaises(ValueError):check_reference_clearance(p,self.c)
    def test_trace_and_qp_use_identical_radius(self):
        v=np.zeros((3,3));A,b,labels=constraints(1,self.p,v,v,self.c)
        rows=barrier_rows(1,self.p,v,v,v[1],A,b,labels,self.c)
        self.assertAlmostEqual(rows['obstacle_0']['h'],np.sum((self.p[1,:2]-self.o['center_xy'])**2)-.439**2)
    def test_between_sample_clearance_catches_safe_endpoints(self):
        self.assertAlmostEqual(minimum_held_distance(np.array([-1.,0]),np.array([2.,0]),np.zeros(2),1.),0)

if __name__=='__main__':unittest.main()

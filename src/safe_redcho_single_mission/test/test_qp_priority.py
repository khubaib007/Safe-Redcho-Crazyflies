"""Priority changes reference slew only; every collision row stays hard."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from scipy.optimize import OptimizeResult
from test_core import cfg,plan
from safe_redcho_single_mission.cbf_qp import solve,validate_batch
from safe_redcho_single_mission.qp_priority import solve_priority,attempt,violation,active_set_projection
from safe_redcho_single_mission.settings import validate
from safe_redcho_single_mission.preview import run
from safe_redcho_single_mission.telemetry_trace import QPTraceError


class PriorityTests(unittest.TestCase):
    def setUp(self):
        self.c=cfg()['control'];self.c.update(allow_slew_relaxation_for_safety=True,maximum_safety_acceleration_slew_mps3=8.)
        self.A=np.array([[1.,0.,0.]])
        self.b=np.array([-.06]);self.previous=np.zeros(3);self.velocity=np.zeros(3)
    def call(self, **changes):
        c=deepcopy(self.c);c.update(changes)
        return solve_priority(np.array([.1,0.,0.]),self.previous,self.velocity,self.A,self.b,c)
    def test_original_hard_policy_rejects_conflicting_slew_box(self):
        u,result,lo,hi,info=self.call(allow_slew_relaxation_for_safety=False)
        self.assertGreater(violation(u,self.A,self.b,lo,hi),self.c['qp_tolerance'])
        self.assertFalse(info['slew_relaxation_applied'])
    def test_safety_priority_relaxes_only_necessary_slew(self):
        u,result,lo,hi,info=self.call()
        self.assertLessEqual(violation(u,self.A,self.b,lo,hi),self.c['qp_tolerance'])
        self.assertLessEqual(u[0],-.06+1e-6)
        self.assertTrue(info['slew_relaxation_applied'])
        self.assertLessEqual(abs(u[0]),.06002)
        self.assertLessEqual(abs(u[0]),self.c['maximum_reference_acceleration_mps2'])
    def test_no_relaxation_when_hard_slew_problem_is_feasible(self):
        self.b=np.array([.2]);a=self.call();b=self.call(allow_slew_relaxation_for_safety=False)
        np.testing.assert_array_equal(a[0],b[0]);self.assertFalse(a[-1]['slew_relaxation_applied'])
    def test_incompatible_safety_rows_are_not_softened(self):
        self.A=np.array([[1.,0.,0.],[-1.,0.,0.]])
        self.b=np.array([-.1,-.1]);u,_,lo,hi,info=self.call()
        self.assertGreater(violation(u,self.A,self.b,lo,hi),self.c['qp_tolerance'])
        self.assertFalse(info['hard_constraint_solution_verified'])
    def test_acceleration_limit_is_not_relaxed(self):
        self.b=np.array([-.9]);u,_,lo,hi,info=self.call()
        self.assertFalse(info['hard_constraint_solution_verified'])
    def test_explicit_safety_slew_cap_is_hard(self):
        self.b=np.array([-.2]);u,_,lo,hi,info=self.call()
        self.assertFalse(info['hard_constraint_solution_verified'])
    def test_policy_requires_explicit_valid_safety_slew_cap(self):
        c=cfg();c['control']['allow_slew_relaxation_for_safety']=True
        with self.assertRaisesRegex(ValueError,'maximum_safety'):validate(c)
        c['control']['maximum_safety_acceleration_slew_mps3']=.1
        with self.assertRaisesRegex(ValueError,'maximum_safety'):validate(c)
        c['control']['maximum_safety_acceleration_slew_mps3']=8.;validate(c)
    def test_empty_hard_velocity_slew_box_can_be_resolved(self):
        self.previous[0]=.1;self.velocity[0]=self.c['maximum_reference_speed_mps']
        self.b=np.array([0.]);u,_,lo,hi,info=self.call()
        self.assertLessEqual(u[0],1e-6);self.assertTrue(info['hard_constraint_solution_verified'])
    def test_active_set_fallback_recovers_feasible_solver_failure(self):
        failure=OptimizeResult(x=np.zeros(3),success=False,status=8,nit=1)
        with patch('safe_redcho_single_mission.qp_priority.minimize',return_value=failure):
            u,r,backend=attempt(np.array([.1,0,0]),np.zeros(3),self.A,self.b,np.full(3,-.4),np.full(3,.4),self.c)
        self.assertEqual(backend,'verified_active_set');self.assertTrue(r.success)
        self.assertLessEqual(u[0],-.06+1e-6)
    def test_active_set_checks_all_rows_and_bounds(self):
        A=np.array([[1.,1.,0.],[0.,0.,1.]])
        b=np.array([-.2,.1]);lo=np.full(3,-.4);hi=np.full(3,.4)
        u=active_set_projection(np.ones(3),A,b,lo,hi,1e-6)
        self.assertIsNotNone(u);self.assertLessEqual(violation(u,A,b,lo,hi),1e-6)
        np.testing.assert_allclose(u,[-.1,-.1,.1],atol=1e-6)
    def test_invalid_policy_types_fail_validation(self):
        for flag in [1,'true',None,[]]:
            c=cfg();c['control']['allow_slew_relaxation_for_safety']=flag
            with self.assertRaisesRegex(ValueError,'boolean'):validate(c)
    def test_batch_policy_accepts_slew_but_not_unsafe_acceleration(self):
        c=cfg();q=np.asarray(plan(c)['hover']);z=np.zeros((3,3));u=z.copy();u[:,0]=.05
        with self.assertRaisesRegex(RuntimeError,'hard constraint'):validate_batch(q,z,z,u,c)
        c['control'].update(allow_slew_relaxation_for_safety=True,maximum_safety_acceleration_slew_mps3=8.)
        validate_batch(q,z,z,u,c)
        u[:,0]=.2
        with self.assertRaisesRegex(RuntimeError,'hard constraint'):validate_batch(q,z,z,u,c)
        u[:,0]=c['control']['maximum_reference_acceleration_mps2']+.1
        with self.assertRaisesRegex(RuntimeError,'hard constraint'):validate_batch(q,z,z,u,c)
    def test_full_preview_writes_diagnostics_on_failure(self):
        c=cfg();p=plan(c);diag={'rows':{'pair_cf3':{'residual_b_minus_Au':-.01}}}
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'hocbf_rows.csv'
            with patch('safe_redcho_single_mission.preview.LocalController.finish',side_effect=QPTraceError('test conflict',diag)):
                with self.assertRaises(QPTraceError):run(c,p,trace=True,evidence_path=path)
            report=json.loads((Path(d)/'preview_failure.json').read_text())
            self.assertEqual(report['stage'],'WARMUP');self.assertEqual(report['cycle'],1)
            self.assertEqual(report['qp_diagnostics'],diag);self.assertEqual(report['plan'],p)

if __name__=='__main__':unittest.main()

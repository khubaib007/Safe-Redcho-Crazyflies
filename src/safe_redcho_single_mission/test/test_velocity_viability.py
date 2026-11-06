"""Anticipate hard speed saturation before bounded acceleration slew traps it."""
from copy import deepcopy
import numpy as np
import pytest

from test_core import cfg,plan
from safe_redcho_single_mission.cbf_qp import validate_batch
from safe_redcho_single_mission.qp_priority import solve_priority,velocity_slew_bounds,violation


def control(anticipate=True,allow_relaxation=True):
    c=deepcopy(cfg()['control'])
    c.update(maximum_reference_speed_mps=.19,maximum_reference_acceleration_mps2=.4,
        maximum_acceleration_slew_mps3=.8,maximum_safety_acceleration_slew_mps3=8.,
        allow_slew_relaxation_for_safety=allow_relaxation,anticipate_velocity_slew=anticipate)
    return c


def tick(c,velocity,previous,sign=1):
    return solve_priority(np.array([sign*.4,0,0]),previous,velocity,np.zeros((1,3)),np.ones(1),c)


def test_previous_speed_only_policy_can_enter_future_empty_speed_slew_box():
    c=control(False);v=np.array([.18,0,0]);previous=np.array([.3,0,0])
    failed=False
    for _ in range(5):
        u,_,lo,hi,info=tick(c,v,previous)
        if np.any(lo>hi) or not info['hard_constraint_solution_verified']:
            failed=True;break
        assert v[0]+c['dt_s']*u[0]<=c['maximum_reference_speed_mps']+1e-6
        v+=c['dt_s']*u;previous=u
    assert failed,'Regression fixture must expose a future speed/slew conflict'


@pytest.mark.parametrize('sign',[-1,1])
def test_anticipatory_bounds_keep_braking_ramp_feasible(sign):
    c=control();v=np.array([sign*.18,0,0]);previous=np.array([sign*.3,0,0])
    for _ in range(80):
        u,_,lo,hi,info=tick(c,v,previous,sign)
        assert info['hard_constraint_solution_verified'] and np.all(lo<=hi)
        assert violation(u,np.zeros((1,3)),np.ones(1),lo,hi)<=c['qp_tolerance']
        assert np.max(np.abs(u-previous))<=8*c['dt_s']+1e-6
        v+=c['dt_s']*u;previous=u
        assert np.max(np.abs(v))<=c['maximum_reference_speed_mps']+1e-6


def test_ordinary_hard_slew_is_used_when_relaxation_is_disabled():
    c=control(allow_relaxation=False);v=np.array([.13,0,0]);previous=np.array([.12,0,0])
    for _ in range(80):
        u,_,lo,hi,info=tick(c,v,previous)
        assert info['hard_constraint_solution_verified'] and np.all(lo<=hi)
        assert np.max(np.abs(u-previous))<=.8*c['dt_s']+1e-6
        v+=c['dt_s']*u;previous=u
        assert np.max(np.abs(v))<=c['maximum_reference_speed_mps']+1e-6


def test_component_signs_and_triangle_headroom_bound():
    c=control();v=np.array([.18,0,-.18]);lo,hi=velocity_slew_bounds(v,c)
    np.testing.assert_allclose(lo,-hi[::-1],atol=1e-14)
    J=8.;dt=c['dt_s'];vmax=c['maximum_reference_speed_mps']
    np.testing.assert_allclose(dt*hi+hi*hi/(2*J),vmax-v,atol=1e-14)
    np.testing.assert_allclose(-dt*lo+lo*lo/(2*J),vmax+v,atol=1e-14)
    at_limit=np.array([vmax,-vmax,0.]);low,high=velocity_slew_bounds(at_limit,c)
    assert high[0]==0 and low[1]==0


def test_invalid_viability_inputs_are_rejected():
    c=control()
    for v in ([np.nan,0,0],[0,0],[0,np.inf,0]):
        with pytest.raises(ValueError,match='finite'):velocity_slew_bounds(v,c)
    for J in (0,-1,float('nan')):
        c['maximum_safety_acceleration_slew_mps3']=J
        with pytest.raises(ValueError,match='positive finite'):velocity_slew_bounds([0,0,0],c)


def test_batch_validation_rejects_future_speed_slew_trap():
    c=cfg();c['control']=control(False);q=np.asarray(plan(c)['hover'])
    v=np.zeros_like(q);v[0,0]=.18;u=np.zeros_like(q);u[0,0]=.3;previous=u.copy()
    # This command satisfies the old next-speed and slew bounds now.
    validate_batch(q,v,previous,u,c)
    c['avoidance']={'predictive_barriers':True}
    with pytest.raises(RuntimeError,match='hard constraint'):
        validate_batch(q,v,previous,u,c)

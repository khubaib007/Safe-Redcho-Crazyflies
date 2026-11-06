"""Reference pacing, deterministic passing and diagnostics contracts."""
from copy import deepcopy
import numpy as np
import pytest
from test_core import cfg,plan
from safe_redcho_single_mission.avoidance import PassingGuide,motion_progress,DEFAULTS
from safe_redcho_single_mission.core import Reference,StageMachine
from safe_redcho_single_mission.settings import validate
from safe_redcho_single_mission.settings import load
from safe_redcho_single_mission.cbf_qp import solve
from safe_redcho_single_mission.telemetry.common import unit_for


def configured():
    c=cfg();c['avoidance']={**DEFAULTS,'enabled':True,'predictive_barriers':True}
    return c


def test_shipped_profile_enables_hardened_controller():
    from pathlib import Path
    c=load(Path(__file__).resolve().parents[1]/'config/mission.yaml')
    assert c['avoidance']['enabled'] is True and c['avoidance']['predictive_barriers'] is True
    assert c['avoidance']['adaptive_pair_responsibility'] is True
    assert c['safety']['hocbf_k1']==2. and c['safety']['hocbf_k2']==2.


def test_reference_pacing_keeps_filter_states_and_battery_scaling():
    c=configured();r=Reference(c,0,plan(c))
    for _ in range(60):r.step('FORMATION',[3.5,3.7,3.8])
    phase=r.phase;shape=r.shape_phase;before=r.acq.state.copy()
    *_,trace=r.step('FORMATION',[3.5,3.7,3.8],0.)
    assert r.phase==phase and r.shape_phase==shape
    assert np.linalg.norm(r.acq.state)>0 and not np.array_equal(r.acq.state,before)
    assert trace['governor']['motion_progress_scale']==0.
    assert trace['governor']['speed_scale']>0


def test_common_pacing_slows_for_guidance_and_excessive_reference_lag():
    c=configured()
    assert motion_progress([],c)==1.
    assert motion_progress([{'avoidance':{'active':True,'target_error_m':1.}}],c)==DEFAULTS['active_progress_scale']
    assert motion_progress([{'avoidance':{'active':False,'target_error_m':1.}}],c)==pytest.approx(DEFAULTS['minimum_progress_scale'])


def test_passing_does_not_replace_normal_motion_before_a_stall():
    c=configured();q=np.array([[-.6,0,.6],[.6,0,.6],[0,1.3,.6]])
    goals=q.copy();goals[:2,0]*=-1;g=PassingGuide(c,0,q,goals)
    nominal=np.array([.15,0,0]);value,report=g.apply(nominal,goals[0],[.1,0,0],q,np.zeros_like(q),'FORMATION')
    np.testing.assert_allclose(value,nominal);assert not report['active']


def test_pair_chirality_is_identical_from_the_common_start_plan():
    c=configured();q=np.array([[-.6,0,.6],[.6,0,.6],[0,1.3,.6]])
    goals=q.copy();goals[:2,0]*=-1;left,right=PassingGuide(c,0,q,goals),PassingGuide(c,1,q,goals)
    _,a=left.pair_side(0,1,q);moved=q.copy();moved[0,1]=.1
    _,b=right.pair_side(1,0,moved);assert a==b


@pytest.mark.parametrize('changes',[{'enabled':1},{'predictive_barriers':'true'},
    {'passing_speed_mps':1.},{'lookahead_s':float('nan')},{'minimum_progress_scale':1.1},
    {'activation_margin_m':.01},{'lag_stop_m':.01},{'unknown':1}])
def test_invalid_avoidance_parameters_rejected(changes):
    c=configured();c['avoidance'].update(changes)
    with pytest.raises(ValueError):validate(c)


def test_next_barrier_rows_have_correct_geometry_and_units():
    c=configured();p=plan(c);q=np.array(p['hover']);z=np.zeros_like(q)
    _,info=solve(0,z[0],q,z,z,c,diagnostics=True)
    row=info['diagnostics']['rows']['next_pair_cf3']
    assert row['row_family']=='next_step_psi1' and row['h_unit']=='m^2'
    assert unit_for('qp.rows.next_pair_cf3.h')=='m^2'
    assert unit_for('qp.rows.next_upper_0.psi1')=='m/s'
    assert unit_for('qp.rows.next_pair_cf3.residual_b_minus_Au')=='m/s^2'


def test_stationary_blocked_mission_fails_before_waiting_indefinitely():
    c=configured();c['avoidance']['stall_window_s']=.2;p=plan(c);machine=StageMachine(c,p);machine.stage='SETTLE'
    estimates=np.zeros((3,3)).tolist();rows=[dict(position_estimate=estimates,position_input=estimates) for _ in range(3)]
    with pytest.raises(RuntimeError,match='Avoidance stalled'):
        for _ in range(30):machine.advance(rows,np.asarray(p['hover']),np.zeros((3,3)),np.zeros(3))

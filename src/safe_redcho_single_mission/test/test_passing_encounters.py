"""Synthetic encounters through actual passing guide and complete hard-QP batch.

These tests isolate guidance/filter behavior with fixed goals. They do not
model real aircraft, onboard tracking, REDCHO or a general route planner.
"""
import numpy as np
import pytest

from test_core import cfg
from safe_redcho_single_mission.avoidance import PassingGuide,DEFAULTS
from safe_redcho_single_mission.cbf_qp import solve,validate_batch
from safe_redcho_single_mission.settings import pair_distance,virtual_box
from safe_redcho_single_mission.obstacles import obstacle_radius


def encounter_config():
    c=cfg();c['arena'].update(x_limits=[-2.,2.],y_limits=[-2.,2.])
    c['safety'].update(minimum_physical_xy_separation_m=.4,measured_abort_xy_separation_m=.6,
        tracking_tube_m=.12,mocap_position_uncertainty_m=.009,pair_reserve_m=.06)
    c['control'].update(maximum_reference_speed_mps=.19,maximum_reference_acceleration_mps2=.4,
        allow_slew_relaxation_for_safety=True,maximum_safety_acceleration_slew_mps3=8.)
    c['avoidance']={**DEFAULTS,'enabled':True,'predictive_barriers':True,'adaptive_pair_responsibility':True}
    c['obstacles']=[]
    return c


def xyz(points):
    return np.column_stack((np.asarray(points,float),np.full(3,.58)))


def encounter(c,initial,goals,steps=5000):
    q=initial.copy();v=np.zeros_like(q);previous=v.copy();dt=c['control']['dt_s']
    guides=[PassingGuide(c,i,initial,goals) for i in range(3)]
    minimum_pair=float('inf');minimum_obstacle=float('inf');active=0;good=0
    max_height_error=0.
    for seq in range(steps):
        nominal=c['control']['position_gain']*(goals-q)-c['control']['velocity_gain']*v
        u=[]
        for i,guide in enumerate(guides):
            guided,report=guide.apply(nominal[i],goals[i],np.zeros(3),q,v,'FORMATION')
            assert report['constraints_softened'] is False
            active+=int(report['active'])
            try:value,_=solve(i,guided,q,v,previous,c)
            except RuntimeError as exc:
                raise AssertionError(f'cycle={seq}, q={q.tolist()}, v={v.tolist()}: {exc}') from exc
            u.append(value)
        u=np.asarray(u)
        q,v=validate_batch(q,v,previous,u,c);previous=u
        minimum_pair=min(minimum_pair,min(np.linalg.norm(q[i,:2]-q[j,:2]) for i in range(3) for j in range(i)))
        for o in c['obstacles']:
            clearance=float(np.min(np.linalg.norm(q[:,:2]-o['center_xy'],axis=1)))-obstacle_radius(o,c)
            minimum_obstacle=min(minimum_obstacle,clearance)
        max_height_error=max(max_height_error,float(np.max(np.abs(q[:,2]-initial[:,2]))))
        errors=np.linalg.norm(q-goals,axis=1)
        good=good+1 if np.max(errors)<.06 and np.max(np.linalg.norm(v,axis=1))<.05 else 0
        if good>=25:
            return dict(cycles=seq+1,minimum_pair=minimum_pair,minimum_obstacle_clearance=minimum_obstacle,
                maximum_height_error=max_height_error,active_samples=active,endpoint_errors=errors.tolist())
    raise AssertionError(f'No endpoint completion after {steps*dt}s: q={q.tolist()}, errors={np.linalg.norm(q-goals,axis=1).tolist()}, active={active}')


@pytest.mark.parametrize('kind',['head_on','perpendicular','cylinder','near_wall'])
def test_fixed_goal_encounter_completes_without_softening_or_vertical_shortcut(kind):
    c=encounter_config()
    if kind=='head_on':
        initial=xyz([[-.9,0],[.9,0],[-1.35,1.3]])
        goals=xyz([[.9,0],[-.9,0],[-1.35,1.3]])
    elif kind=='perpendicular':
        initial=xyz([[-.9,0],[0,-.9],[-1.35,1.3]])
        goals=xyz([[.9,0],[0,.9],[-1.35,1.3]])
    elif kind=='near_wall':
        initial=xyz([[-.9,1.0],[.9,1.0],[0,-1.3]])
        goals=xyz([[.9,1.0],[-.9,1.0],[0,-1.3]])
    else:
        initial=xyz([[-.9,0],[-1.3,1.3],[-1.3,-1.3]])
        goals=xyz([[.9,0],[-1.3,1.3],[-1.3,-1.3]])
        c['obstacles']=[dict(name='obstacle_1',center_xy=[0.,0.],radius_m=.11,
            surface_clearance_m=.1,position_uncertainty_m=.02)]
    report=encounter(c,initial,goals)
    assert report['active_samples']>0
    assert report['minimum_pair']>=pair_distance(c)-1e-8
    assert report['maximum_height_error']<1e-10
    assert report['minimum_obstacle_clearance']>=-1e-8


def test_inadmissible_close_encounter_is_rejected_without_safety_relaxation():
    c=encounter_config();d=pair_distance(c)+.001
    q=xyz([[-d/2,0],[d/2,0],[-1.3,1.3]])
    v=np.zeros_like(q);v[0,0]=.01;v[1,0]=-.01
    with pytest.raises(RuntimeError,match='HOCBF domain invalid'):
        solve(0,[.4,0,0],q,v,np.zeros_like(q),c)


def test_three_metre_arena_near_wall_swap_with_hardware_barrier_gains():
    c=encounter_config()
    c['arena'].update(x_limits=[-1.5,1.5],y_limits=[-1.5,1.5])
    c['safety'].update(hocbf_k1=2.,hocbf_k2=2.)
    initial=xyz([[-.8,.65],[.8,.65],[0,-.9]])
    goals=initial.copy();goals[[0,1]]=goals[[1,0]]
    report=encounter(c,initial,goals)
    assert report['active_samples']>0
    assert report['minimum_pair']>=pair_distance(c)-1e-8
    assert report['maximum_height_error']<1e-10


def test_obstacle_inside_reference_buffer_cannot_be_bypassed_by_guidance():
    c=encounter_config();c['obstacles']=[dict(name='obstacle_1',center_xy=[0.,0.],radius_m=.11,
        surface_clearance_m=.1,position_uncertainty_m=.02)]
    q=xyz([[.1,0],[-1.3,1.3],[-1.3,-1.3]]);v=np.zeros_like(q)
    guide=PassingGuide(c,0,q,q+np.array([1.,0,0]))
    nominal,report=guide.apply([.4,0,0],q[0]+[1.,0,0],[0,0,0],q,v,'FORMATION')
    assert report['constraints_softened'] is False
    with pytest.raises(RuntimeError,match='HOCBF domain invalid'):
        solve(0,nominal,q,v,v,c)


def test_incompatible_equal_responsibility_rows_are_rejected_without_softening():
    c=encounter_config();hi=virtual_box(c)[1]
    c['avoidance']['adaptive_pair_responsibility']=False
    x=hi[0]-.001;d=pair_distance(c)+.001
    q=xyz([[x,0],[x-d,0],[-1.3,-1.3]]);v=np.zeros_like(q)
    guide=PassingGuide(c,0,q,q)
    nominal,report=guide.apply([0,0,0],q[0],[0,0,0],q,v,'FORMATION')
    assert report['constraints_softened'] is False
    # Initial geometric/psi1 domains can be valid while robust local pair
    # responsibility and wall rows demand incompatible accelerations.
    with pytest.raises(RuntimeError,match='hard QP infeasible'):
        solve(0,nominal,q,v,v,c)

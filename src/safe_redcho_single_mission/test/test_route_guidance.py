"""Route geometry and full-mission regressions from recorded preview stalls."""
import json
from copy import deepcopy
from pathlib import Path
import numpy as np
import pytest
from test_avoidance import configured
from test_passing_encounters import encounter_config, encounter, xyz
from safe_redcho_single_mission.avoidance import PassingGuide,options
from safe_redcho_single_mission.route_guidance import VisibilityRoute,segment_distances
from safe_redcho_single_mission.settings import validate,pair_distance,virtual_box
from safe_redcho_single_mission.obstacles import obstacle_radius
from safe_redcho_single_mission.preview import run


def geometry():
    c=encounter_config();c['avoidance']['route_guidance']=True
    c['obstacles']=[dict(name='obstacle_1',center_xy=[0.,0.],radius_m=.11,
        surface_clearance_m=.1,position_uncertainty_m=.02)]
    q=xyz([[-.9,0],[-1.3,1.3],[-1.3,-1.3]])
    return c,q


def test_visibility_path_edges_clear_all_disks_and_stay_in_arena():
    c,q=geometry();goal=np.array([.9,0]);r=VisibilityRoute(c,0,options(c))
    path,audit=r.plan(q[0,:2],goal,q)
    assert audit['status']=='found' and 1<len(path)<24 and audit['nodes']<=256
    centers,radii,_=r.planning_geometry(q[0,:2],goal,q);lo,hi=virtual_box(c)
    for start,end in zip([q[0,:2]]+path[:-1],path):
        assert np.all(segment_distances(start,end,centers)>=radii-1e-7)
        assert np.all(end>lo[:2]) and np.all(end<hi[:2])
    again,_=r.plan(q[0,:2],goal,q)
    np.testing.assert_array_equal(path,again)


def test_occupied_goal_and_budget_fall_back_without_safety_relaxation():
    c,q=geometry();r=VisibilityRoute(c,0,options(c))
    path,audit=r.plan(q[0,:2],q[1,:2],q)
    assert not path and audit['reason']=='goal_occupied_by_inflated_hazard'
    c['avoidance']['route_max_nodes']=4;r=VisibilityRoute(c,0,options(c))
    path,audit=r.plan(q[0,:2],np.array([.9,0]),q)
    assert not path and audit['reason']=='visibility_node_budget'
    guide=PassingGuide(c,0,q,q)
    _,report=guide.apply([.3,0,0],np.array([.9,0,.58]),np.zeros(3),q,np.zeros_like(q),'FORMATION')
    assert not report['constraints_softened'] and report['route_status']=='blocked'


def test_blocked_search_is_rate_limited_and_moving_goal_invalidates_cache():
    c,q=geometry();r=VisibilityRoute(c,0,options(c))
    for _ in range(10):r.velocity(q[1],q)
    assert r.searches==1
    r.velocity(q[1]+[.2,0,0],q)
    assert r.searches==2


def test_current_forward_goal_releases_route_and_recovers_original_nominal():
    c,q=geometry();guide=PassingGuide(c,0,q,q)
    value,report=guide.apply([.3,0,.07],np.array([.9,0,.58]),np.zeros(3),q,np.zeros_like(q),'FORWARD')
    assert report['active'] and report['method']=='visibility_waypoints'
    assert value[2]==.07
    target=q[0]+[-.1,0,0];nominal=np.array([-.12,.03,.07])
    value,report=guide.apply(nominal,target,np.zeros(3),q,np.zeros_like(q),'STOP')
    np.testing.assert_array_equal(value,nominal)
    assert not report['active'] and report['route_status']=='clear'


def test_route_handoff_preserves_an_in_progress_passing_encounter():
    c=encounter_config();c['avoidance']['route_guidance']=True
    q=xyz([[-.5,0],[.5,0],[0,1.3]]);goals=q.copy();goals[[0,1]]=goals[[1,0]]
    guide=PassingGuide(c,0,q,goals);v=np.zeros_like(q)
    for _ in range(40):
        _,report=guide.apply([.4,0,0],goals[0],np.zeros(3),q,v,'FORMATION')
    assert report['active'] and guide.encounters
    side=report['passing_side']
    # A momentarily clear current target releases routing, but must not erase
    # a passing decision needed again when that moving target is occupied.
    _,clear=guide.apply([-.1,0,0],q[0]+[-.1,0,0],np.zeros(3),q,v,'FORMATION')
    assert clear['route_status']=='clear'
    _,resumed=guide.apply([.4,0,0],goals[0],np.zeros(3),q,v,'FORMATION')
    assert resumed['active'] and resumed['passing_side']==side
    assert resumed['fallback']=='deterministic_local_circulation'


@pytest.mark.parametrize('changes',[{'route_guidance':1},{'route_circle_samples':24.0},
    {'route_circle_samples':7},{'route_max_nodes':257},{'route_replan_s':0.},
    {'route_margin_m':float('inf')}])
def test_route_configuration_is_strict(changes):
    c=configured();c['avoidance'].update(changes)
    with pytest.raises(ValueError):validate(c)


@pytest.mark.parametrize('kind',['head_on','perpendicular','near_wall'])
def test_rerouted_conflicts_complete_in_three_metre_arena(kind):
    c=encounter_config();c['avoidance']['route_guidance']=True
    c['arena'].update(x_limits=[-1.5,1.5],y_limits=[-1.5,1.5])
    c['safety'].update(hocbf_k1=2.,hocbf_k2=2.)
    if kind=='head_on':
        initial=xyz([[-.8,0],[.8,0],[0,.95]]);goals=initial.copy();goals[[0,1]]=goals[[1,0]]
    elif kind=='perpendicular':
        initial=xyz([[-.85,0],[0,-.85],[-.9,.95]]);goals=xyz([[.85,0],[0,.85],[-.9,.95]])
    else:
        initial=xyz([[-.8,.65],[.8,.65],[0,-.9]]);goals=initial.copy();goals[[0,1]]=goals[[1,0]]
    report=encounter(c,initial,goals)
    assert report['minimum_pair']>=pair_distance(c)-1e-8
    assert report['maximum_height_error']<1e-10


@pytest.mark.parametrize('case',['194330_544226Z','182422_163997Z','182707_600308Z'])
@pytest.mark.parametrize('voltages',[(4.,3.8,3.7),(3.55,3.45,3.35)])
def test_recorded_blocked_routes_complete_full_single_bank_mission(case,voltages):
    cases=json.loads((Path(__file__).parent/'fixtures/route_regressions.json').read_text())
    data=deepcopy(cases[case]);c=data['configuration'];validate(c)
    report,_=run(c,data['plan'],voltages=voltages,trace=False)
    assert report['status']=='PASS'
    assert report['minimum_virtual_xy_separation_m']>=pair_distance(c)-1e-8
    assert max(report['final_endpoint_errors_m'])<c['mission']['endpoint_tolerance_m']
    for metrics in report['obstacle_clearances'].values():
        assert metrics['minimum_virtual_center_distance_m']>=metrics['required_reference_center_distance_m']-1e-8

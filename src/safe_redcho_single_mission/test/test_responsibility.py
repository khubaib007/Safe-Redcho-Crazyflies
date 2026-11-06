"""Static-authority reciprocal allocation without weaker joint safety rows."""
from copy import deepcopy
import numpy as np
import pytest

from test_passing_encounters import encounter_config
from safe_redcho_single_mission.responsibility import vertices,Allocator
from safe_redcho_single_mission.cbf_qp import constraints,solve,validate_batch


def failed_encounter():
    """Frozen fast/unequal-voltage failure; no external result-file dependency."""
    c=encounter_config();c['avoidance']['adaptive_pair_responsibility']=False
    c['arena'].update(x_limits=[-1.5,1.5],y_limits=[-1.5,1.5])
    c['safety'].update(hocbf_k1=2.,hocbf_k2=2.)
    c['obstacles']=[dict(name='obstacle_1',center_xy=[.6,-.35],radius_m=.11,
        surface_clearance_m=.1,position_uncertainty_m=.02)]
    q=np.array([[.6637445388989904,.8309027773901945,.5798000000000001],
        [.6245171962730404,.09672641401287786,.5797181170418145],
        [.6626383700385846,-.8010825913428469,.579881882958186]])
    v=np.array([[.08364451352266884,-.005512002350090388,-9.44966106019596e-18],
        [.10394880771166486,-.005831611435811856,1.1207944254067844e-15],
        [.08422787072749613,.00036052769011989724,-1.3148732736381656e-15]])
    previous=np.array([[-.009434143754122918,-.02429924028915203,-1.974825333595562e-15],
        [-.08227593140824162,-.020704063554952988,2.5307862806032787e-15],
        [-.007721359442601232,-2.3941905423269962e-05,-5.354018077229966e-16]])
    nominal=np.zeros((3,3));nominal[1]=[-.005842306924707656,-.10270718299639894,1.7211811132161053e-15]
    return c,q,v,previous,nominal


def test_frozen_obstacle_pair_conflict_yields_to_the_free_peer():
    c,q,v,previous,nominal=failed_encounter()
    with pytest.raises(RuntimeError,match='hard QP infeasible'):
        solve(1,nominal[1],q,v,previous,c)
    c['avoidance']['adaptive_pair_responsibility']=True
    results=[solve(i,nominal[i],q,v,previous,c,diagnostics=True) for i in range(3)]
    u=np.array([r[0] for r in results]);validate_batch(q,v,previous,u,c)
    assert u[0,1]>.04 # cf2 yields upward rather than demanding cf3 enter obstacle
    for _,info in results:
        assert info['pair_responsibility_policy']=='static_authority_support'
        audit=info['diagnostics']['pair_responsibility']
        assert audit['policy']=='static_authority_support' and audit['allocations']
        assert all(np.isfinite(row['own_static_minimum_raw']) for row in audit['allocations'])


def test_current_and_predictive_pair_allocations_preserve_joint_bounds():
    c,q,v,previous,_=failed_encounter();c['avoidance']['adaptive_pair_responsibility']=True
    packets=[constraints(i,q,v,previous,c,audit=True) for i in range(3)]
    for i in range(3):
        for j in range(i):
            for prefix in ('pair_','next_pair_'):
                ai=next(a for a in packets[i][3]['allocations'] if a['label']==prefix+c['names'][j])
                aj=next(a for a in packets[j][3]['allocations'] if a['label']==prefix+c['names'][i])
                np.testing.assert_allclose(ai['direction'],-np.asarray(aj['direction']),atol=1e-14)
                assert abs(ai['joint_bound_raw']-aj['joint_bound_raw'])<1e-14
                assert abs(ai['own_bound_raw']+aj['own_bound_raw']-ai['joint_bound_raw'])<1e-12
                norm=np.linalg.norm(ai['direction'])
                assert abs(ai['own_effective_bound_raw']+aj['own_effective_bound_raw']-
                           (ai['joint_bound_raw']-2*norm*c['safety']['row_reserve_mps2']))<1e-12


def test_default_equal_policy_is_unchanged_when_option_is_false():
    c,q,v,previous,_=failed_encounter();c['avoidance'].pop('adaptive_pair_responsibility')
    a=constraints(0,q,v,previous,c)
    c['avoidance']['adaptive_pair_responsibility']=False;b=constraints(0,q,v,previous,c)
    for x,y in zip(a,b):np.testing.assert_array_equal(x,y)


def test_exact_xy_vertices_respect_every_static_halfspace():
    A=np.array([[-1.,0.],[0.,-1.],[1.,1.]])
    b=np.array([.1,.2,.15]);points=vertices(A,b,[-.4,-.4],[.4,.4])
    assert np.all(A@points.T<=b[:,None]+1e-9)
    assert abs(np.min(points[:,0])+.1)<1e-12
    assert abs(np.min(points[:,1])+.2)<1e-12


def test_static_support_allocates_more_pair_work_to_unconstrained_peer():
    c=encounter_config();empty=(np.empty((0,3)),np.empty(0))
    rows=[(np.array([[0.,-1.,0.]]),np.array([-.1])),empty,empty]
    allocator=Allocator(rows,np.zeros((3,3)),np.zeros((3,3)),c)
    own=allocator.allocate(0,1,[0,1,0],0.,'pair_cf3')
    peer=allocator.allocate(1,0,[0,-1,0],0.,'pair_cf2')
    assert own>.1 and peer<-.1 and abs(own+peer)<1e-14


def test_insufficient_combined_authority_and_reserves_are_not_softened():
    c=encounter_config();empty=(np.empty((0,3)),np.empty(0))
    allocator=Allocator([empty]*3,np.zeros((3,3)),np.zeros((3,3)),c)
    # Each minimum is -.16; B below -.32 is physically incompatible.
    with pytest.raises(RuntimeError,match='incompatible static pair authority'):
        allocator.allocate(0,1,[1,0,0],-.4)
    with pytest.raises(RuntimeError,match='required reserves'):
        allocator.allocate(0,1,[1,0,0],-.319)


def test_empty_xy_or_height_polytope_is_rejected():
    with pytest.raises(RuntimeError,match='Empty static feasible'):
        vertices([[1,0],[-1,0]],[-.2,-.2],[-.4,-.4],[.4,.4])
    c=encounter_config();empty=(np.empty((0,3)),np.empty(0))
    rows=[(np.array([[0.,0.,-1.]]),np.array([-.3])),empty,empty]
    with pytest.raises(RuntimeError,match='empty static feasible'):
        Allocator(rows,np.zeros((3,3)),np.zeros((3,3)),c)


def test_allocator_never_uses_height_to_pay_for_horizontal_responsibility():
    c=encounter_config();empty=(np.empty((0,3)),np.empty(0))
    allocator=Allocator([empty]*3,np.zeros((3,3)),np.zeros((3,3)),c)
    with pytest.raises(ValueError,match='XY-only'):
        allocator.allocate(0,1,[1,0,.1],1.)

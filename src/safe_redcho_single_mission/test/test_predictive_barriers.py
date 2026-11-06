"""One-hold predictive HOCBF tests: native units, tangents and joint rows."""
from copy import deepcopy
import numpy as np
import pytest

from test_core import cfg,plan
from safe_redcho_single_mission.barrier_domain import (
    check_domain,check_next_domain,predictive_rows,squared_psi1,squared_reserve)
from safe_redcho_single_mission.cbf_qp import constraints,solve,validate_batch
from safe_redcho_single_mission.settings import pair_distance,virtual_box


def enabled_cfg():
    c=cfg();c['avoidance']={'predictive_barriers':True};return c


def test_default_path_is_bitwise_unchanged_when_disabled():
    c=cfg();q=np.asarray(plan(c)['hover']);z=np.zeros_like(q)
    a=constraints(0,q,z,z,c)
    c['avoidance']={'predictive_barriers':False};b=constraints(0,q,z,z,c)
    for x,y in zip(a,b):np.testing.assert_array_equal(x,y)
    assert not any(label.startswith('next_') for label in a[2])


def test_valid_static_state_and_zero_batch_are_accepted():
    c=enabled_cfg();q=np.asarray(plan(c)['hover']);z=np.zeros_like(q)
    check_domain(q,z,c);check_next_domain(q,z,z,c)
    a,b,labels=constraints(0,q,z,z,c)
    assert 'next_pair_cf3' in labels and 'next_upper_0' in labels
    u=np.array([solve(i,[0,0,0],q,z,z,c)[0] for i in range(3)])
    validate_batch(q,z,z,u,c)


def test_geometrically_clear_but_inadmissibly_closing_pair_is_rejected():
    c=enabled_cfg();d=pair_distance(c)+.001
    q=np.array([[-d/2,0,.53],[d/2,0,.53],[0,1,.53]])
    v=np.array([[.01,0,0],[-.01,0,0],[0,0,0]])
    assert np.linalg.norm(q[0,:2]-q[1,:2])>pair_distance(c)
    with pytest.raises(RuntimeError,match='HOCBF domain invalid.*pair'):
        constraints(0,q,v,np.zeros_like(q),c)


def test_clear_endpoint_with_negative_next_psi1_is_rejected():
    c=enabled_cfg();q=np.asarray(plan(c)['hover']);q[0,0]=virtual_box(c)[1][0]-.001
    v=np.zeros_like(q);v[0,0]=.003;u=np.zeros_like(q);u[0,0]=.4
    check_domain(q,v,c)
    dt=c['control']['dt_s'];next_q=q+dt*v+.5*dt*dt*u
    assert next_q[0,0]<virtual_box(c)[1][0]
    with pytest.raises(RuntimeError,match='HOCBF domain invalid.*upper_0'):
        check_next_domain(q,v,u,c)


def test_squared_next_psi1_tangent_is_a_global_lower_bound():
    rng=np.random.default_rng(814)
    dt=.02;k1=4.;coefficient=dt**3+.25*k1*dt**4
    for _ in range(100):
        r=rng.normal(size=2);v=rng.normal(size=2)*.2
        anchor=rng.normal(size=2)*.3;a=rng.normal(size=2)*.4
        F,g=squared_psi1(r,v,anchor,.718,dt,k1)
        exact,_=squared_psi1(r,v,a,.718,dt,k1)
        tangent=F+g@(a-anchor)
        assert exact>=tangent-1e-12
        np.testing.assert_allclose(exact-tangent,coefficient*np.sum((a-anchor)**2),atol=1e-12)


def test_pair_local_predictive_rows_sum_to_the_joint_affine_bound():
    c=enabled_cfg();q=np.asarray(plan(c)['hover']);v=np.zeros_like(q)
    previous=np.array([[.1,-.03,.2],[-.06,.02,-.2],[0,0,0]])
    first=next(r for r in predictive_rows(0,q,v,previous,c) if r[2]=='next_pair_cf3')
    second=next(r for r in predictive_rows(1,q,v,previous,c) if r[2]=='next_pair_cf2')
    r=q[0,:2]-q[1,:2];rv=v[0,:2]-v[1,:2];anchor=previous[0,:2]-previous[1,:2]
    dt=c['control']['dt_s'];k1=c['safety']['hocbf_k1']
    F,g=squared_psi1(r,rv,anchor,pair_distance(c),dt,k1)
    reserve=squared_reserve(r,rv,dt,k1,2*np.sqrt(2)*c['control']['maximum_reference_acceleration_mps2'],
                            2*c['safety']['virtual_acceleration_uncertainty_mps2'])
    np.testing.assert_allclose(first[0],-second[0],atol=1e-14)
    assert first[0][2]==0 and second[0][2]==0 # vertical motion never waives XY separation
    assert abs(first[1]+second[1]-(F-g@anchor-reserve))<1e-14


def test_uncertainty_reserve_covers_bounded_acceleration_error():
    rng=np.random.default_rng(41);dt=.02;k1=4.;amax=.8;unc=.06
    for _ in range(200):
        r=rng.normal(size=2);v=rng.normal(size=2)*.2
        a=rng.normal(size=2);a*=amax/max(1.,np.linalg.norm(a))
        error=rng.normal(size=2);error*=unc/max(1.,np.linalg.norm(error))
        F,_=squared_psi1(r,v,a,.718,dt,k1);uncertain,_=squared_psi1(r,v,a+error,.718,dt,k1)
        assert uncertain>=F-squared_reserve(r,v,dt,k1,amax,unc)-1e-12


def test_obstacle_predictive_row_uses_xy_tangent_without_vertical_bypass():
    c=enabled_cfg();c['obstacles']=[dict(name='obstacle_1',center_xy=[0,0],radius_m=.11,
        surface_clearance_m=.10,position_uncertainty_m=.02)]
    q=np.array([[.7,0,.53],[-.7,.8,.53],[-.7,-.8,.53]])
    z=np.zeros_like(q);anchor=z.copy();anchor[0]=[-.05,.02,.4]
    check_domain(q,z,c)
    row=next(r for r in predictive_rows(0,q,z,anchor,c) if r[2]=='next_obstacle_0')
    assert row[0][2]==0 and np.isfinite(row[0]).all() and np.isfinite(row[1])
    check_next_domain(q,z,z,c)


def test_geometrically_clear_but_inadmissibly_closing_obstacle_is_rejected():
    from safe_redcho_single_mission.obstacles import obstacle_radius
    c=enabled_cfg();o=dict(name='obstacle_1',center_xy=[0,0],radius_m=.11,
        surface_clearance_m=.10,position_uncertainty_m=.02);c['obstacles']=[o]
    radius=obstacle_radius(o,c)
    q=np.array([[radius+.001,0,.53],[-.7,.8,.53],[-.7,-.8,.53]])
    v=np.zeros_like(q);v[0,0]=-.01
    with pytest.raises(RuntimeError,match='HOCBF domain invalid.*obstacle_0'):
        check_domain(q,v,c)


@pytest.mark.parametrize('kind',['q','v','anchor','u'])
def test_nonfinite_predictive_state_is_rejected(kind):
    c=enabled_cfg();q=np.asarray(plan(c)['hover']);v=np.zeros_like(q);anchor=np.zeros_like(q);u=np.zeros_like(q)
    value={'q':q,'v':v,'anchor':anchor,'u':u}[kind];value[0,0]=np.nan
    with pytest.raises(ValueError,match='finite'):
        if kind=='u':check_next_domain(q,v,u,c)
        else:predictive_rows(0,q,v,anchor,c)

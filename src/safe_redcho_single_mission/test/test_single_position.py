"""Baseline-specific contracts: one estimator, derivative provenance and isolation."""
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch
import numpy as np
import pytest
import yaml

from test_core import cfg, plan
from safe_redcho_single_mission.core import LocalController
from safe_redcho_single_mission.observers import Bank
from safe_redcho_single_mission.observation_config import prepare
from safe_redcho_single_mission.protocol import fingerprint
from safe_redcho_single_mission.run import command
from safe_redcho_single_mission.settings import NAMES, validate
from safe_redcho_single_mission.telemetry.configuration import load_config
from safe_redcho_single_mission.liveplots.configuration import load as plot_config


def frame(c, p, stage='FORMATION'):
    return dict(seq=1, stage=stage, q=p['hover'], v=np.zeros((3, 3)),
                previous=np.zeros((3, 3)), voltage=[3.8, 3.5, 3.4])


@pytest.mark.parametrize('stage', ['WARMUP', 'HEIGHT', 'YAW', 'FORMATION', 'SETTLE', 'FORWARD', 'STOP'])
def test_tracking_uses_only_position_bank_outputs(stage):
    c=cfg(); p=plan(c); core=LocalController(c, 0, p)
    assert len([v for v in vars(core).values() if isinstance(v, Bank)]) == 1
    core.position.X[:] = [[.004, .003, .002], [.001, -.002, .003], [.002, .001, -.001]]
    outgoing=core.prepare(frame(c, p, stage))
    assert set(outgoing) == {'position'}
    peers={n:deepcopy(outgoing) for n in core.neighbors}
    with patch('safe_redcho_single_mission.core.solve', return_value=(np.zeros(3), {'diagnostics':{}})):
        result=core.finish(peers)
    trace=result['diagnostics']; nominal=trace['nominal']
    pd=np.array(nominal['desired_position_m']); vd=np.array(nominal['desired_velocity_mps']); ad=np.array(nominal['desired_acceleration_mps2'])
    if stage == 'WARMUP':
        np.testing.assert_allclose(pd, p['hover'][0]); np.testing.assert_allclose(vd, 0); np.testing.assert_allclose(ad, 0)
    else:
        np.testing.assert_allclose([pd[2],vd[2],ad[2]], core.py[2])
        if stage in ('FORMATION','SETTLE','FORWARD','STOP'):
            np.testing.assert_allclose(pd[:2], core.py[:2,0]+core.rho[:2])
            np.testing.assert_allclose(vd[:2], core.py[:2,1]+core.rd[:2])
            np.testing.assert_allclose(ad[:2], core.py[:2,2]+core.rdd[:2])
    assert 'velocity_redcho' not in trace and 'velocity_estimate' not in result
    assert trace['architecture']['bank_count'] == 1
    np.testing.assert_allclose(result['motion_derivatives'], core.py[:,1:])


def test_dual_bank_peer_packet_cannot_enter_baseline():
    c=cfg(); p=plan(c); core=LocalController(c,0,p); outgoing=core.prepare(frame(c,p))
    with pytest.raises(ValueError, match='Single-bank peers'):
        core.finish({n:dict(outgoing, velocity=[0,0,0]) for n in core.neighbors})


def test_position_bank_preserves_mean_and_reconstructs_constant_reference():
    c=cfg(); dt=c['control']['dt_s']; banks=[Bank(c['redcho'],c['redcho']['position_L'],dt) for _ in NAMES]
    references=np.zeros((3,3,3)); references[:,:,0]=[[.1,.7,.58],[-.6,0,.43],[.1,-.7,.73]]
    for _ in range(800):
        outputs=[b.prepare(r) for b,r in zip(banks,references)]
        np.testing.assert_allclose(np.mean(outputs,axis=0),np.mean(references,axis=0),atol=1e-12)
        for i,b in enumerate(banks):b.finish({NAMES[j]:outputs[j][:,0] for j in range(3) if i!=j})
    assert max(np.linalg.norm(o[:,0]-references[:,:,0].mean(axis=0)) for o in outputs)<.04


@pytest.mark.parametrize('section,key,value', [('arena','body_radius_m',-.05),('arena','ceiling_clearance_m',-.2),
    ('backend','pose_frequency_hz',-20),('backend','status_frequency_hz',0),('backend','bvca_enabled',1)])
def test_invalid_physical_configuration_is_rejected(section,key,value):
    c=cfg();c[section][key]=value
    with pytest.raises(ValueError):validate(c)


def test_dual_profile_is_rejected_even_if_all_other_parameters_match():
    c=cfg();c['architecture']='dual_redcho'
    with pytest.raises(ValueError,match='architecture'):validate(c)


def test_observers_follow_selected_profile_without_rewriting_templates(tmp_path):
    package=Path(__file__).resolve().parents[1]
    c=cfg();c['workspace']=str(tmp_path);profile=tmp_path/'surveyed.yaml';profile.write_text(yaml.safe_dump(c))
    telemetry=yaml.safe_load((package/'config/telemetry.yaml').read_text())
    onboard=tmp_path/'onboard.yaml';onboard.write_text((package/'config/onboard.yaml').read_text())
    telemetry['onboard_config_file']=str(onboard)
    t=tmp_path/'telemetry-template.yaml';t.write_text(yaml.safe_dump(telemetry))
    g=tmp_path/'plots-template.yaml';g.write_text((package/'config/liveplots.yaml').read_text())
    before=(t.read_bytes(),g.read_bytes(),profile.read_bytes())
    tp,gp=prepare(profile,t,g,tmp_path/'generated')
    assert load_config(tp)['expected_config_hash']==fingerprint(c)
    assert plot_config(gp)['config_hash']==fingerprint(c)
    assert before==(t.read_bytes(),g.read_bytes(),profile.read_bytes())


def test_live_modes_require_new_ground_survey(tmp_path):
    for mode in ('backend','agents','check','preview','observe','acquire','fly'):
        with pytest.raises(RuntimeError,match='survey'):
            command(tmp_path,'combined',mode)


def test_runner_keeps_flight_flags_and_surveyed_profiles_separate(tmp_path):
    (tmp_path/'config').mkdir()
    c=cfg();c['workspace']=str(tmp_path)
    base=tmp_path/'config/single_mission_combined.yaml';base.write_text(yaml.safe_dump(c))
    immutable=tmp_path/'immutable-profile.yaml';immutable.write_bytes(base.read_bytes())
    capture=command(tmp_path,'combined','capture')
    assert capture[capture.index('--config')+1]==str(base)
    assert 'generated_single_hocbf' in capture[capture.index('--output')+1]
    offline=command(tmp_path,'combined','offline')
    assert '--fly' not in offline and '--captured' not in offline
    assert offline[offline.index('--starts')+1].endswith('single_mission_combined_fixture.yaml')
    with patch('safe_redcho_single_mission.survey.resolve_profile',return_value=immutable):
        for mode in ('check','acquire','fly'):
            args=command(tmp_path,'combined',mode)
            assert args[args.index('--config')+1]==str(immutable)
            assert ('--fly' in args)==(mode in ('acquire','fly'))
            assert ('--acquire-only' in args)==(mode=='acquire')

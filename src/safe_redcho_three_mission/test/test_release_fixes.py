"""Regression checks for release fixes; no ROS or hardware connection."""
from pathlib import Path
import copy,json,tempfile,unittest
from unittest.mock import patch
import yaml
from test_core import cfg
from safe_redcho_three_mission.settings import validate
from safe_redcho_three_mission.protocol import fingerprint
from safe_redcho_three_mission.observation_config import prepare
from safe_redcho_three_mission.liveplots.configuration import load as plot_load
from safe_redcho_three_mission.telemetry.configuration import load_config
import test_ros_adapters as adapters

ROOT=Path(__file__).resolve().parents[3]

class ProfileRegressionTests(unittest.TestCase):
    def test_negative_geometric_allowances_are_rejected(self):
        for key in ['body_radius_m','wall_surface_clearance_m','ceiling_clearance_m']:
            c=cfg();c['arena'][key]=-.1
            with self.subTest(key=key),self.assertRaises(ValueError):validate(c)
    def test_invalid_backend_values_are_rejected(self):
        for key in ['pose_frequency_hz','status_frequency_hz','bvca_horizontal_radius_m','bvca_vertical_radius_m','bvca_horizon_s']:
            c=cfg();c['backend'][key]=0
            with self.subTest(key=key),self.assertRaises(ValueError):validate(c)
    def test_observers_bind_to_exact_executed_profile(self):
        with tempfile.TemporaryDirectory() as d:
            c=cfg();c['workspace']=d
            profile=Path(d)/'surveyed.yaml';profile.write_text(yaml.safe_dump(c))
            t,p=prepare(profile,ROOT/'config/three_mission_telemetry.yaml',ROOT/'config/three_mission_liveplots.yaml',Path(d)/'observers')
            self.assertEqual(load_config(t)['expected_config_hash'],fingerprint(c))
            self.assertEqual(plot_load(p)['config_hash'],fingerprint(c))
            self.assertEqual(Path(yaml.safe_load(t.read_text())['algorithm_profile_file']),profile)

class FinalJournalRegressionTests(unittest.TestCase):
    setUp=adapters.AdapterTests.setUp
    tearDown=adapters.AdapterTests.tearDown
    make_mission=adapters.AdapterTests.make_mission
    def test_journal_error_at_final_drain_cannot_report_pass(self):
        from test_ros_adapters import contextlib,io,np
        m,n=self.make_mission();n.preflight=lambda:self.p;n.batteries={name:(10,4.) for name in self.c['names']}
        n.cycle=lambda *a,**kw:([{}]*3,np.zeros((3,3)))
        n.command_file=type('Journal',(),{'close':lambda _:dict(errors=['injected disk error'],pending_batches=0,worker_still_running=False,dropped_rows=0)})()
        with contextlib.redirect_stdout(io.StringIO()),patch.object(m,'verify_cache',return_value={}),patch.object(m,'numerical_preview',return_value=({'status':'PASS'},[])):
            n.run(fly=False)
        self.assertEqual(n.result['status'],'FAIL');self.assertIn('recording_error',n.result)

if __name__=='__main__':unittest.main()

"""Portable launch selection tests; no ROS initialization or aircraft commands."""
import importlib.util
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('single_release_runner', ROOT/'scripts/single_run.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class SingleRunner(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        (self.root/'config/local').mkdir(parents=True)
        for directory in ['config', 'config/local']:
            shutil.copy2(ROOT/'config/single_mission_combined.yaml', self.root/directory/'single_mission_combined.yaml')
        self.env = patch.object(runner, 'ROOT', self.root)
        self.env.start()

    def tearDown(self):
        self.env.stop()
        self.temp.cleanup()

    def test_offline_uses_example_fixture_and_never_enables_flight(self):
        command = runner.command('combined', 'offline')
        self.assertIn(str(self.root/'config/single_mission_combined.yaml'), command)
        self.assertIn(str(self.root/'config/single_mission_combined_fixture.yaml'), command)
        self.assertNotIn('--fly', command)
        self.assertNotIn('ros2', command)

    def test_capture_and_mocap_use_local_configuration(self):
        profile = str(self.root/'config/local/single_mission_combined.yaml')
        self.assertIn(profile, runner.command('combined', 'capture'))
        self.assertIn('config_file:='+profile, runner.command('combined', 'mocap'))

    def test_live_commands_resolve_the_same_immutable_survey(self):
        profile = self.root/'results/immutable_survey/profile.yaml'
        with patch('safe_redcho_single_mission.survey.resolve_profile', return_value=profile) as resolve:
            for mode in ('backend', 'agents', 'preview', 'check', 'acquire', 'fly'):
                command = runner.command('combined', mode)
                self.assertTrue(str(profile) in command or 'config_file:='+str(profile) in command)
                self.assertEqual('--fly' in command, mode in ('acquire', 'fly'))
                self.assertEqual('--acquire-only' in command, mode == 'acquire')
                self.assertEqual('--captured' in command, mode == 'preview')
            resolve.assert_called_with(self.root/'config/local/single_mission_combined.yaml',
                                       self.root/'config/generated_single_hocbf/combined.yaml')

    def test_observers_keep_plot_off_arguments_and_bind_local_settings(self):
        with patch('safe_redcho_single_mission.survey.resolve_profile', return_value=Path('captured.yaml')), \
             patch('safe_redcho_single_mission.observation_config.prepare', return_value=('telemetry.yaml', 'plots.yaml')) as prepare:
            command = runner.command('combined', 'observe', ['recorders:=true', 'control:=false'])
            self.assertIn('config_file:=captured.yaml', command)
            self.assertEqual(command[-2:], ['recorders:=true', 'control:=false'])
            self.assertEqual(prepare.call_args.args[1], self.root/'config/local/single_mission_telemetry.yaml')

    def test_missing_local_configuration_and_unexpected_flags_fail_closed(self):
        (self.root/'config/local/single_mission_combined.yaml').unlink()
        with self.assertRaisesRegex(ValueError, 'Initialize'):
            runner.command('combined', 'capture')
        with self.assertRaisesRegex(ValueError, 'Additional'):
            runner.command('combined', 'check', ['--fly'])


if __name__ == '__main__':
    unittest.main()

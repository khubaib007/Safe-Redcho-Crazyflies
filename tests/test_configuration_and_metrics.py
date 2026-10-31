from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace as S
import math
import sys
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/safe_redcho_bringup'))
from safe_redcho_bringup.configuration import load_lab, prepare
from safe_redcho_bringup.pose_metrics import PoseStream, orientation_error, tilt


def pose(x=0.0, y=0.0, z=0.0, q=(0., 0., 0., 1.)):
    return S(position=S(x=x, y=y, z=z), orientation=S(x=q[0], y=q[1], z=q[2], w=q[3]))


class Metrics(unittest.TestCase):
    def test_stationary_stream_and_dropout(self):
        stream = PoseStream()
        for i in range(201):
            stream.add(pose(), 'world', now=i/100)
        stats = stream.stats(now=2.0)
        self.assertAlmostEqual(stats['rate_hz'], 100)
        self.assertEqual(stats['span_m'], [0., 0., 0.])
        self.assertAlmostEqual(stats['maximum_gap_s'], 0.01)
        self.assertGreater(stream.stats(now=2.5)['maximum_gap_s'], 0.08)

    def test_invalid_poses_do_not_refresh_valid_sample(self):
        stream = PoseStream()
        stream.add(pose(), 'world', now=0)
        self.assertFalse(stream.add(pose(x=math.nan), 'world', now=1))
        self.assertFalse(stream.add(pose(q=(0., 0., 0., 0.)), 'world', now=2))
        self.assertEqual(stream.last_time, 0)
        self.assertEqual(stream.invalid_samples, 2)

    def test_orientation_equivalence_and_bad_alignment(self):
        self.assertAlmostEqual(orientation_error(pose(), pose(q=(0., 0., 0., -1.))), 0)
        self.assertAlmostEqual(orientation_error(pose(), pose(q=(0., 0., 1., 0.))), math.pi)
        self.assertAlmostEqual(tilt(pose(q=(1., 0., 0., 0.))), math.pi)


class Configuration(unittest.TestCase):
    def test_placeholder_is_usable_only_for_pose_reception(self):
        path = ROOT/'config/lab_config.yaml'
        load_lab(path, require_radio=False)
        with self.assertRaisesRegex(ValueError, 'actual complete'):
            load_lab(path, require_radio=True)

    def test_generation_keeps_vendor_mode_and_one_robot_without_source_edits(self):
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'config').mkdir()
            lab = yaml.safe_load((ROOT/'config/lab_config.yaml').read_text())
            lab['robot']['uri'] = 'radio://0/80/2M/E7E7E7E701'
            config = root/'config/lab_config.yaml'
            config.write_text(yaml.safe_dump(lab))
            upstream = root/'src/crazyswarm2/crazyflie/config'
            upstream.mkdir(parents=True)
            # Minimal schema fixture, not a claim of a ROS/upstream integration build.
            mocap = {'/motion_capture_tracking': {'ros__parameters': {
                'type': 'vicon', 'hostname': 'unused', 'rigid_bodies': {'old': {}},
                'marker_configurations': {'keep': {'offset': [0., 0., 0.]}}}}}
            robots = {'fileversion': 3, 'robots': {'old': {'enabled': True}},
                      'robot_types': {'cf21': {'motion_capture': {'tracking': 'librigidbodytracker'},
                                              'big_quad': False}},
                      'all': {'firmware_params': {'locSrv': {'extQuatStdDev': .05}}}}
            for name, value in [('motion_capture.yaml', mocap), ('crazyflies.yaml', robots)]:
                (upstream/name).write_text(yaml.safe_dump(value))
            before = {p.name: p.read_bytes() for p in upstream.iterdir()}
            _, generated = prepare(config, require_radio=True)
            actual = yaml.safe_load((generated/'crazyflies.yaml').read_text())
            self.assertEqual(list(actual['robots']), ['cf1'])
            self.assertEqual(actual['robot_types']['cf21']['motion_capture']['tracking'], 'vendor')
            self.assertEqual(actual['all']['firmware_params']['locSrv']['extQuatStdDev'], .05)
            params = yaml.safe_load((generated/'motion_capture.yaml').read_text())['/motion_capture_tracking']['ros__parameters']
            self.assertEqual(params['hostname'], '192.168.2.5')
            self.assertEqual(params['topics']['frame_id'], 'world')
            self.assertNotIn('rigid_bodies', params)
            self.assertIn('keep', params['marker_configurations'])
            self.assertEqual(before, {p.name: p.read_bytes() for p in upstream.iterdir()})

    def test_mismatched_vendor_name_rejected(self):
        with TemporaryDirectory() as tmp:
            lab = yaml.safe_load((ROOT/'config/lab_config.yaml').read_text())
            lab['robot']['rigid_body_name'] = 'different'
            path = Path(tmp)/'lab.yaml'
            path.write_text(yaml.safe_dump(lab))
            with self.assertRaisesRegex(ValueError, 'match exactly'):
                load_lab(path)


if __name__ == '__main__':
    unittest.main()

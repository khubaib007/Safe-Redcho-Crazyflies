# Release test fixtures are independent of actual lab settings.
from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory
import math
import sys
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/safe_redcho_bringup'))
from safe_redcho_bringup.backend_audit import analyze_source
from safe_redcho_bringup.fleet_configuration import load_fleet, prepare_fleet
from safe_redcho_bringup.onboard_avoidance import firmware_parameters
from safe_redcho_bringup.swap_planner import build_plan


def stage3_config():
    cfg = load_fleet(ROOT/'config/fleet_config.yaml')
    cfg['mission'].update(minimum_separation_m=.5,abort_separation_m=.65,tracking_tolerance_m=.1,planning_reserve_m=.05,maximum_flight_duration_s=300.)
    cfg['arena']['wall_clearance_m']=.3
    cfg['swap']['buffer_xy']=[0.,0.]
    for i,n in enumerate(cfg['robots']):cfg['robots'][n]['uri']=f'radio://{0 if i<3 else 1}/{80 if i<3 else 60}/2M/E7E7E7E70{i+1}'
    cfg['arena'].update({'floor_z_m': 0., 'ceiling_z_m': 1.8})
    cfg['_starts'] = {f'cf{i+1}': [.95*math.cos(i*math.pi/3), .95*math.sin(i*math.pi/3), .02]
                      for i in range(6)}
    return cfg


GLOBAL_SOURCE = '''
void poses_changed(const motion_capture_tracking_interfaces::msg::NamedPoseArray::SharedPtr msg) {
  std::vector<Crazyflie::externalPose> externalPoses;
  for (auto const & pose : msg->poses) { externalPoses.push_back(convert(pose)); }
  for (auto & broadcaster : m_cfBroadcasters) {
    broadcaster.second->sendExternalPoses(externalPoses);
  }
}
'''


class Avoidance(unittest.TestCase):
    def test_two_radios_and_distinct_channels(self):
        cfg = stage3_config()
        self.assertEqual(cfg['robots']['cf1']['uri'], 'radio://0/80/2M/E7E7E7E701')
        self.assertEqual(cfg['robots']['cf4']['uri'], 'radio://1/60/2M/E7E7E7E704')
        self.assertEqual(cfg['robots']['cf6']['uri'], 'radio://1/60/2M/E7E7E7E706')

    def test_3m_arena_and_half_metre_threshold_fit_buffered_swap(self):
        plan = build_plan(stage3_config())
        self.assertAlmostEqual(plan['minimum_required_planned_xy_separation_m'], .75)
        self.assertAlmostEqual(plan['minimum_nominal_xy_separation_m'], .95*math.sqrt(3)/2)
        self.assertEqual(len(plan['legs']), 7)

    def test_radius_is_half_pair_distance_plus_extra_per_robot(self):
        values = firmware_parameters(stage3_config())
        self.assertEqual(values['enable'], 1)
        self.assertAlmostEqual(values['ellipsoidX'], .30)
        self.assertAlmostEqual(values['ellipsoidY'], .30)
        self.assertAlmostEqual(values['ellipsoidZ'], .40)
        self.assertAlmostEqual(values['bboxMinX'], -1.15)
        self.assertAlmostEqual(values['bboxMaxX'], 1.15)
        self.assertEqual(values['maxPeerLocAge'], 150)

    def test_generated_fleet_uses_pid_and_bvca(self):
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'config').mkdir()
            (root/'config/lab_config.yaml').write_text((ROOT/'config/lab_config.yaml').read_text())
            cfg = stage3_config()
            plain = {k: v for k, v in cfg.items() if not k.startswith('_')}
            (root/'config/fleet_config.yaml').write_text(yaml.safe_dump(plain))
            snapshot = {'frame_id': 'world', 'positions': cfg['_starts']}
            (root/'config/measured_starts.yaml').write_text(yaml.safe_dump(snapshot))
            upstream = root/'src/crazyswarm2/crazyflie/config'
            upstream.mkdir(parents=True)
            (upstream/'motion_capture.yaml').write_text(yaml.safe_dump({'/motion_capture_tracking': {'ros__parameters': {}}}))
            (upstream/'crazyflies.yaml').write_text(yaml.safe_dump({
                'fileversion': 3, 'robot_types': {'cf21': {'motion_capture': {'tracking': 'vendor'}}}, 'all': {}}))
            _, _, outfile = prepare_fleet(root/'config/fleet_config.yaml')
            generated = yaml.safe_load(outfile.read_text())
            self.assertEqual(generated['all']['firmware_params']['stabilizer']['controller'], 1)
            self.assertEqual(generated['all']['firmware_params']['colAv']['enable'], 1)
            self.assertEqual(len(generated['robots']), 6)
            self.assertIn('radio://1/60/', generated['robots']['cf6']['uri'])

    def test_overlapping_radio_channels_are_rejected(self):
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'lab_config.yaml').write_text((ROOT/'config/lab_config.yaml').read_text())
            cfg = {k:v for k,v in stage3_config().items() if not k.startswith('_')}
            for i in (4, 5, 6):
                cfg['robots'][f'cf{i}']['uri'] = cfg['robots'][f'cf{i}']['uri'].replace('/60/', '/80/')
            (root/'fleet_config.yaml').write_text(yaml.safe_dump(cfg))
            with self.assertRaisesRegex(ValueError, 'channels'):
                load_fleet(root/'fleet_config.yaml')

    def test_global_source_pattern_recognized(self):
        self.assertEqual(analyze_source(GLOBAL_SOURCE)['status'], 'PASS_SOURCE_PATTERN')

    def test_channel_filtered_pose_collection_rejected(self):
        source = GLOBAL_SOURCE.replace('externalPoses.push_back(convert(pose));',
                                       'if (pose.channel == radio.channel) externalPoses.push_back(convert(pose));')
        self.assertEqual(analyze_source(source)['status'], 'UNVERIFIED')

    def test_channel_scoped_vector_rejected(self):
        source = '''void callback(const NamedPoseArray::SharedPtr msg) {
          for (auto & b : m_cfBroadcasters) {
            std::vector<Crazyflie::externalPose> poses;
            for (auto & p : msg->poses) { poses.push_back(p); }
            b.second->sendExternalPoses(poses);
          }
        }'''
        self.assertEqual(analyze_source(source)['status'], 'UNVERIFIED')

    def test_comment_cannot_fake_routing_evidence(self):
        self.assertEqual(analyze_source('/* '+GLOBAL_SOURCE+' */')['status'], 'UNVERIFIED')

    def test_unknown_source_rejected(self):
        self.assertEqual(analyze_source('void alternate() {}')['status'], 'UNVERIFIED')


if __name__ == '__main__':
    unittest.main()

# Release test fixtures are independent of actual lab settings.
from copy import deepcopy
import math
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/safe_redcho_bringup'))
from safe_redcho_bringup.fleet_configuration import load_fleet
from safe_redcho_bringup.swap_planner import build_plan, point_segment_distance, move_duration


def example_config(radius=1.2):
    cfg = load_fleet(ROOT/'config/fleet_config.yaml')
    # Retain stage-2 geometry regression cases independently of stage-3 live defaults.
    cfg['mission'].update({'minimum_separation_m': .7, 'abort_separation_m': .9,
                           'planning_reserve_m': .05, 'tracking_tolerance_m': .1, 'maximum_flight_duration_s': 300.})
    cfg['arena'].update({'x_limits': [-1.8, 1.8], 'y_limits': [-1.7, 1.7],
                         'floor_z_m': 0., 'ceiling_z_m': 2.})
    cfg['swap']['buffer_xy'] = [0.,0.]
    cfg['_starts'] = {f'cf{i+1}': [radius*math.cos(i*math.pi/3),
                                  radius*math.sin(i*math.pi/3), .02] for i in range(6)}
    return cfg


class Geometry(unittest.TestCase):
    def test_buffered_swap_moves_only_one_aircraft_and_hits_correct_destinations(self):
        cfg = example_config()
        plan = build_plan(cfg)
        self.assertEqual(len(plan['legs']), 7)
        self.assertEqual([leg['robot'] for leg in plan['legs']],
                         ['cf1', 'cf6', 'cf5', 'cf4', 'cf3', 'cf2', 'cf1'])
        expected = 1.2*math.sqrt(3)/2
        self.assertAlmostEqual(plan['minimum_nominal_xy_separation_m'], expected)
        self.assertAlmostEqual(plan['minimum_required_planned_xy_separation_m'], .95)
        for i in range(6):
            goal = plan['final_hover_positions'][f'cf{i+1}']
            self.assertEqual(goal[:2], cfg['_starts'][f'cf{(i+1)%6+1}'][:2])

    def test_each_entire_segment_has_margin_against_every_holding_aircraft(self):
        plan = build_plan(example_config())
        for leg in plan['legs']:
            for p in leg['stationary_positions'].values():
                self.assertGreaterEqual(point_segment_distance(p[:2], leg['start'][:2], leg['goal'][:2]), .95)

    def test_one_metre_hexagon_rejected_with_current_recovery_allowance(self):
        with self.assertRaisesRegex(ValueError, 'planned XY separation'):
            build_plan(example_config(radius=1.0))

    def test_buffer_collision_is_rejected(self):
        cfg = example_config()
        cfg['swap']['buffer_xy'] = cfg['_starts']['cf2'][:2]
        with self.assertRaises(ValueError):
            build_plan(cfg)

    def test_obstacle_blocking_buffer_path_rejected(self):
        cfg = example_config()
        # Far enough from all initial points, directly in cf1's buffer path.
        cfg['obstacles'] = [{'center_xy': [0., 0.], 'radius_m': .05}]
        build_plan(cfg, mode='hover')
        with self.assertRaisesRegex(ValueError, 'obstacle'):
            build_plan(cfg, mode='swap')

    def test_wall_ceiling_and_airborne_starts_rejected(self):
        cfg = example_config()
        cfg['arena']['x_limits'] = [-1.4, 1.4]
        with self.assertRaisesRegex(ValueError, 'endpoint'):
            build_plan(cfg)
        cfg = example_config()
        cfg['arena']['ceiling_z_m'] = cfg['arena']['floor_z_m']+cfg['mission']['hover_height_above_floor_m']+cfg['arena']['ceiling_clearance_m']-.01
        with self.assertRaisesRegex(ValueError, 'ceiling'):
            build_plan(cfg)
        cfg = example_config()
        cfg['_starts']['cf1'][2] = .4
        with self.assertRaisesRegex(ValueError, 'floor'):
            build_plan(cfg)

    def test_hover_does_not_require_a_valid_swap_buffer(self):
        cfg = example_config()
        cfg['swap']['buffer_xy'] = cfg['_starts']['cf2'][:2]
        self.assertEqual(build_plan(cfg, mode='hover')['legs'], [])

    def test_duration_bounds_for_seventh_degree_trajectory(self):
        cfg = example_config()
        for length in (.01, .5, 1.2, 3.):
            duration = move_duration(length, cfg)
            self.assertLessEqual((35/16)*length/duration, cfg['mission']['maximum_speed_mps']+1e-12)
            self.assertLessEqual(8*length/duration**2, cfg['mission']['maximum_acceleration_mps2']+1e-12)

    def test_near_point_uses_segment_interior_not_only_endpoints(self):
        self.assertAlmostEqual(point_segment_distance([0, .1], [-1, 0], [1, 0]), .1)
        self.assertAlmostEqual(point_segment_distance([2, 0], [-1, 0], [1, 0]), 1.)

    def test_empty_arena_bounds_rejected_by_loader(self):
        with TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/'lab_config.yaml').write_text((ROOT/'config/lab_config.yaml').read_text())
            cfg=yaml.safe_load((ROOT/'config/fleet_config.yaml').read_text())
            cfg['arena']['x_limits']=[0.,0.]
            (root/'fleet_config.yaml').write_text(yaml.safe_dump(cfg))
            with self.assertRaises(ValueError):load_fleet(root/'fleet_config.yaml',require_arena=True)


if __name__ == '__main__':
    unittest.main()

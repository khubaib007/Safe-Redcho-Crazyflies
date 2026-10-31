"""Offline tests of the shipped patch: real functions, fake ROS/clock only."""
import ast
import copy
import importlib.util
from pathlib import Path
import sys
import types
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CODE = ROOT / 'src/safe_redcho_three/safe_redcho_three'
NAMES = ['cf2', 'cf3', 'cf4']


def profile():
    return dict(names=NAMES.copy(), obstacles=[],
        arena=dict(x_limits=[-1.5, 1.5], y_limits=[-1.5, 1.5], floor_z_m=.0298,
                   ceiling_z_m=1.3, ceiling_clearance_m=.2,
                   body_radius_m=.05, wall_surface_clearance_m=.15),
        safety=dict(tracking_tube_m=.14, mocap_position_uncertainty_m=.02,
                    pair_reserve_m=.09, wall_reserve_m=.03,
                    minimum_physical_xy_separation_m=.5,
                    airborne_minimum_height_above_floor_m=.35),
        mission=dict(center_start_xy=[-.325, 0.],
                     offsets_xyz={n: [.475, y, 0.] for n, y in zip(NAMES, [.95, 0., -.95])},
                     hover_height_above_floor_m=.5, endpoint_tolerance_m=.11,
                     endpoint_speed_mps=.1, takeoff_duration_s=5.),
        control=dict(dt_s=.02, maximum_publish_gap_s=.08),
        commissioning=dict(handover_duration_s=5., settle_timeout_s=8., settle_dwell_s=2.))


# Geometry fixture copied from the previously reviewed settings.py. The ROS-free
# tests load the actual patched commissioning module against these unchanged APIs.
def slots(c):
    center = np.r_[c['mission']['center_start_xy'],
                   c['arena']['floor_z_m']+c['mission']['hover_height_above_floor_m']]
    return np.array([center+c['mission']['offsets_xyz'][n] for n in c['names']])


def pair_distance(c):
    s = c['safety']
    return s['minimum_physical_xy_separation_m']+2*s['tracking_tube_m']+2*s['mocap_position_uncertainty_m']+s['pair_reserve_m']


def virtual_box(c):
    a, s = c['arena'], c['safety']
    margin = a['body_radius_m']+a['wall_surface_clearance_m']+s['tracking_tube_m']+s['mocap_position_uncertainty_m']+s['wall_reserve_m']
    return (np.array([a['x_limits'][0]+margin, a['y_limits'][0]+margin,
                      a['floor_z_m']+s['airborne_minimum_height_above_floor_m']]),
            np.array([a['x_limits'][1]-margin, a['y_limits'][1]-margin,
                      a['ceiling_z_m']-a['ceiling_clearance_m']-s['tracking_tube_m']-
                      s['mocap_position_uncertainty_m']-s['wall_reserve_m']]))


pkg = types.ModuleType('_handover_patch_test'); pkg.__path__ = [str(CODE)]
sys.modules[pkg.__name__] = pkg
settings = types.ModuleType(pkg.__name__+'.settings')
settings.slots, settings.pair_distance, settings.virtual_box = slots, pair_distance, virtual_box
sys.modules[settings.__name__] = settings
fleet = types.ModuleType(pkg.__name__+'.fleet_adapter')
fleet.ACTIVE, fleet.INACTIVE = tuple(NAMES), ('cf1', 'cf5', 'cf6')
def require_active(names):
    if list(names) != NAMES: raise RuntimeError('Unexpected active names')
fleet.require_active = require_active
sys.modules[fleet.__name__] = fleet
spec = importlib.util.spec_from_file_location(pkg.__name__+'.commissioning', CODE/'commissioning.py')
mod = importlib.util.module_from_spec(spec); sys.modules[spec.name] = mod; spec.loader.exec_module(mod)


class Clock:
    def __init__(self): self.now = 0.
    def monotonic(self): return self.now


class Node:
    def __init__(self, cfg=None):
        self.profile = cfg or profile()
        self.names = NAMES.copy()
        self.ground = np.array([[.1575, .9503, .0295], [.1635, .0121, .0281], [.1515, -.9064, .0266]])
        self.result, self.logs, self.packets, self.calls = {}, [], [], []
        self.expected, self.streaming, self.flight_active = None, False, False
        self.arm_attempted, self.airborne_possible = False, False
        self.stage_name = 'GROUND'
        self.clock = Clock(); mod.time = self.clock
        self.extra_error = np.zeros((3, 3)); self.speed = np.zeros((3, 3))
        self.fail_health, self.publication_span, self.extra_wait = None, .0001, 0.
    def get_logger(self): return types.SimpleNamespace(info=self.logs.append)
    def pulse(self): pass
    def spin_for(self, duration, guard=None):
        self.clock.now += max(duration, .001)+self.extra_wait
        if guard: guard()
    def health(self, ground=False, airborne=False):
        if self.fail_health: raise RuntimeError(self.fail_health)
        if ground: p = self.ground.copy()
        elif self.expected is not None: p = self.expected.copy()+self.extra_error
        else: p = self.takeoff_hover_reference.copy()+self.extra_error
        if airborne and self.expected is not None:
            if np.max(np.linalg.norm(p-self.expected, axis=1)) > self.profile['safety']['tracking_tube_m']:
                raise RuntimeError('tracking tube')
        return p, self.speed.copy(), min(np.linalg.norm(p[i, :2]-p[j, :2]) for i in range(3) for j in range(i))
    def publish_setpoints(self, q, v, a):
        self.packets.append((self.clock.now, q.copy(), v.copy(), a.copy()))
        self.clock.now += self.publication_span
        return self.publication_span
    def call_many(self, label, requests): self.calls.append((label, requests))


class HandoverTests(unittest.TestCase):
    def test_existing_floor_positions_plan_without_repositioning(self):
        n = Node(); ground = n.ground.copy(); mod.prepare_handover(n)
        np.testing.assert_array_equal(n.ground, ground)
        np.testing.assert_array_equal(n.takeoff_hover_reference[:, :2], ground[:, :2])
        np.testing.assert_allclose(n.takeoff_hover_reference[:, 2], .5298)
        self.assertAlmostEqual(pair_distance(n.profile), .91)
        self.assertEqual(n.result['handover_plan']['status'], 'PASS_BEFORE_ARM')
        self.assertFalse(n.calls)

    def test_clear_reference_with_inward_measured_drift_is_accepted(self):
        n = Node(); mod.prepare_handover(n)
        n.extra_error[2, 1] = .03
        actual = n.takeoff_hover_reference+n.extra_error
        with self.assertRaisesRegex(RuntimeError, 'clearance'):
            mod.bridge_clear(actual, slots(n.profile), n.profile)
        mod.handover(n)
        self.assertEqual(n.result['position_handover']['status'], 'PASS')
        self.assertFalse(n.result['position_handover']['first_command_matches_measured_pose'])
        np.testing.assert_array_equal(n.packets[0][1], n.takeoff_hover_reference)
        np.testing.assert_allclose(n.packets[-1][1], slots(n.profile))
        for _, q, _, _ in n.packets:
            mod.bridge_clear(q, slots(n.profile), n.profile)
        self.assertGreaterEqual(n.clock.now, 7.5)

    def test_unsafe_ground_reference_is_rejected_before_arm(self):
        n = Node(); n.ground[2, 1] = -.88
        with self.assertRaisesRegex(RuntimeError, 'clearance'): mod.prepare_handover(n)
        self.assertFalse(n.calls); self.assertFalse(n.packets)

    def test_crossing_between_clear_endpoints_is_rejected(self):
        c = profile(); start = slots(c); goal = start.copy(); goal[[0, 2]] = goal[[2, 0]]
        with self.assertRaisesRegex(RuntimeError, 'clearance'): mod.bridge_clear(start, goal, c)

    def test_nonfinite_and_wrong_shape_rejected(self):
        c = profile(); goal = slots(c)
        for start in [np.full((3, 3), np.nan), np.zeros((2, 3)), np.full((3, 3), np.inf)]:
            with self.assertRaisesRegex(RuntimeError, 'finite'): mod.bridge_clear(start, goal, c)

    def test_wall_and_obstacle_checks_retained(self):
        c = profile(); goal = slots(c); start = goal.copy(); start[0, 0] = 1.2
        with self.assertRaisesRegex(RuntimeError, 'volume'): mod.bridge_clear(start, goal, c)
        c['obstacles'] = [{'test': True}]
        with self.assertRaisesRegex(RuntimeError, 'obstacle'): mod.bridge_clear(goal, goal, c)

    def test_missing_plan_cannot_be_created_in_air(self):
        n = Node()
        with self.assertRaisesRegex(RuntimeError, 'pre-arm'): mod.handover(n)
        self.assertFalse(n.packets)

    def test_excessive_bridge_speed_rejected_before_arm(self):
        n = Node(); n.profile['commissioning']['handover_duration_s'] = .1
        with self.assertRaisesRegex(RuntimeError, 'speed'): mod.prepare_handover(n)
        self.assertFalse(n.calls)

    def test_stale_pose_failure_propagates_without_publication(self):
        n = Node(); mod.prepare_handover(n); n.fail_health = 'stale mocap'
        with self.assertRaisesRegex(RuntimeError, 'stale'): mod.handover(n)
        self.assertFalse(n.packets)

    def test_large_measured_descent_not_hidden_by_planned_reference(self):
        n = Node(); mod.prepare_handover(n); n.extra_error[2, 2] = -.15
        with self.assertRaisesRegex(RuntimeError, 'tracking tube'): mod.handover(n)
        self.assertFalse(n.packets)

    def test_unsettled_speed_times_out_without_publication(self):
        n = Node(); mod.prepare_handover(n); n.speed[2, 2] = -.12
        with self.assertRaisesRegex(RuntimeError, 'within 8.0 s'): mod.handover(n)
        self.assertFalse(n.packets)
        self.assertLess(n.clock.now, 8.1)

    def test_publication_span_and_gap_still_abort(self):
        n = Node(); mod.prepare_handover(n); n.publication_span = .03
        with self.assertRaisesRegex(RuntimeError, 'control period'): mod.handover(n)
        n = Node(); mod.prepare_handover(n); n.extra_wait = .09
        with self.assertRaisesRegex(RuntimeError, 'publication gap'): mod.handover(n)

    def test_quintic_bridge_has_stationary_endpoints_and_speed_bound(self):
        n = Node(); mod.prepare_handover(n); start = n.takeoff_hover_reference; goal = slots(n.profile)
        samples = [mod.bridge_sample(start, goal, t, 5.) for t in np.linspace(0., 5., 1001)]
        np.testing.assert_allclose(samples[0][0], start); np.testing.assert_allclose(samples[-1][0], goal)
        for k in (0, -1):
            np.testing.assert_allclose(samples[k][1], 0.); np.testing.assert_allclose(samples[k][2], 0.)
        self.assertLess(max(np.max(np.linalg.norm(v, axis=1)) for _, v, _ in samples), .04)

    def test_takeoff_calls_plan_before_arm_and_uses_local_hover_gate(self):
        # Execute the ACTUAL shipped takeoff method with fake service request types.
        tree = ast.parse((CODE/'mission.py').read_text())
        cls = next(x for x in tree.body if isinstance(x, ast.ClassDef) and x.name == 'Mission')
        fn = copy.deepcopy(next(x for x in cls.body if isinstance(x, ast.FunctionDef) and x.name == 'takeoff'))
        requests = types.SimpleNamespace(Request=lambda: types.SimpleNamespace())
        env = dict(__package__=pkg.__name__, np=np, time=None, slots=slots,
                   Arm=requests, Takeoff=requests,
                   Duration=lambda **kw: types.SimpleNamespace(to_msg=lambda: kw))
        exec(compile(ast.Module(body=[fn], type_ignores=[]), '<actual takeoff>', 'exec'), env)
        n = Node(); env['time'] = n.clock
        n.ground[2, 1] = -.88
        with self.assertRaisesRegex(RuntimeError, 'clearance'): env['takeoff'](n)
        self.assertFalse(n.arm_attempted); self.assertFalse(n.calls)
        n = Node(); env['time'] = n.clock; env['takeoff'](n)
        self.assertEqual([x[0] for x in n.calls], ['arm', 'takeoff'])
        self.assertEqual(n.stage_name, 'TAKEOFF_SETTLE')
        np.testing.assert_allclose(n.expected, n.takeoff_hover_reference)

    def test_tracking_fault_reports_cf4_xyz_and_includes_failed_sample_in_maximum(self):
        # Execute the actual changed tracking block inside FlightTransport.health.
        tree = ast.parse((CODE/'transport.py').read_text())
        guard = next(x for x in ast.walk(tree) if isinstance(x, ast.If)
                     and ast.unparse(x.test) == 'self.expected is not None')
        n = Node(); n.expected = slots(n.profile); p = n.expected.copy(); p[2, 2] -= .15
        details = {}
        def fault(message, **kw):
            details.update(kw)
            raise RuntimeError(message)
        n.guard_fault = fault
        env = dict(self=n, p=p, velocity=np.zeros((3, 3)), s=n.profile['safety'], np=np)
        with self.assertRaisesRegex(RuntimeError, 'tracking tube'):
            exec(compile(ast.Module(body=[guard], type_ignores=[]), '<actual tracking guard>', 'exec'), env)
        self.assertEqual(details['name'], 'cf4')
        self.assertAlmostEqual(details['error_xyz_m'][2], -.15)
        self.assertAlmostEqual(n.maximum_tracking, .15)


if __name__ == '__main__': unittest.main()

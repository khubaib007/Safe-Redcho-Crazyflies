# Release tests exercise current service names and tracking thresholds.
"""Flight-command/recovery ordering tests with ROS transports stubbed."""
import importlib.util
import math
from pathlib import Path
import sys
import time
from tempfile import TemporaryDirectory
from types import ModuleType, SimpleNamespace as S, MethodType
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/safe_redcho_bringup'))
from test_swap_planner import example_config
from safe_redcho_bringup.pose_metrics import PoseStream


def load_mission():
    ros = ModuleType('rclpy')
    ros.spin_once = lambda *a, **k: None
    duration = ModuleType('rclpy.duration')
    class Duration:
        def __init__(self, seconds):
            self.seconds = seconds
        def to_msg(self):
            return self.seconds
    duration.Duration = Duration
    sig = ModuleType('rclpy.signals')
    sig.SignalHandlerOptions = S(NO=0)
    srv = ModuleType('crazyflie_interfaces.srv')
    class Request:
        def __init__(self):
            self.goal = S(x=0., y=0., z=0.)
    for name in ('Arm', 'Takeoff', 'Land', 'GoTo'):
        setattr(srv, name, type(name, (), {'Request': Request}))
    receiver = ModuleType('safe_redcho_bringup.fleet_receiver')
    receiver.FleetReceiver = type('FleetReceiver', (), {})
    parameters = ModuleType('rcl_interfaces.srv')
    parameters.GetParameters = type('GetParameters', (), {'Request': Request})
    modules = {'rclpy': ros, 'rclpy.duration': duration, 'rclpy.signals': sig,
               'crazyflie_interfaces': ModuleType('crazyflie_interfaces'),
               'crazyflie_interfaces.srv': srv, 'safe_redcho_bringup.fleet_receiver': receiver,
               'rcl_interfaces': ModuleType('rcl_interfaces'), 'rcl_interfaces.srv': parameters}
    spec = importlib.util.spec_from_file_location('safe_redcho_bringup._fleet_transport_test',
                                                 ROOT/'src/safe_redcho_bringup/safe_redcho_bringup/fleet_mission.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module.FleetMission


Mission = load_mission()


def pose(p):
    return S(position=S(x=p[0], y=p[1], z=p[2]), orientation=S(x=0., y=0., z=0., w=1.))


class Harness(Mission):
    def __init__(self, output, fail=None):
        self.config = example_config()
        self.m = self.config['mission']
        self.names = list(self.config['robots'])
        self.output = Path(output)
        self.mocap = {n: S(pose=pose(p)) for n, p in self.config['_starts'].items()}
        self.airborne_possible = set()
        self.arm_attempted = False
        self.expected, self.active, self.recovery = {}, {}, {}
        self.flight_start = None
        self.minimum_measured_xy_separation = float('inf')
        self.avoidance_configuration = {}
        self.csv_handle = (self.output/'stub.csv').open('w')
        self.calls = []
        self.fail = fail
        self.get_logger = lambda: S(info=lambda *_:None,error=lambda *_:None,warning=lambda *_:None)
    def wait_ready(self):
        pass
    def verify_avoidance(self):
        pass
    def check_health(self, stationary=False):
        pass
    def guard(self):
        pass
    def spin_for(self, seconds, guard=None):
        pass
    def settle(self, targets, duration=None):
        for n, p in targets.items():
            self.mocap[n].pose = pose(p)
    def get_logger(self):
        return S(info=lambda *_: None, error=lambda *_: None)
    def call_many(self, action, requests, monitor=True, timeout=2.):
        self.calls.append((action, list(requests)))
        if action == self.fail:
            raise RuntimeError('Injected uncertain transport result')
    def recover(self):
        self.calls.append(('recover', sorted(self.airborne_possible)))
        if self.airborne_possible:
            self.recovery['touchdown_verified'] = self.names
            self.recovery['touchdown_not_verified'] = []


class FleetRecovery(unittest.TestCase):
    def setUp(self):
        quiet = patch('builtins.print')
        quiet.start()
        self.addCleanup(quiet.stop)

    def test_fleet_check_only_sends_no_commands(self):
        with TemporaryDirectory() as tmp:
            h = Harness(tmp)
            h.run('swap', check_only=True)
            self.assertTrue(h.success)
            self.assertEqual(h.calls, [('recover', [])])

    def test_takeoff_timeout_marks_all_as_potentially_airborne_then_recovers_all(self):
        with TemporaryDirectory() as tmp:
            h = Harness(tmp, fail='takeoff')
            h.run('hover')
            self.assertFalse(h.success)
            self.assertEqual(h.calls[-1], ('recover', h.names))

    def test_nominal_swap_has_seven_single_aircraft_moves(self):
        with TemporaryDirectory() as tmp:
            h = Harness(tmp)
            h.run('swap')
            self.assertTrue(h.success)
            moves = [names for action, names in h.calls if action == 'go_to']
            self.assertEqual(moves, [['cf1'], ['cf6'], ['cf5'], ['cf4'], ['cf3'], ['cf2'], ['cf1']])

    def test_move_timeout_recovers_whole_fleet(self):
        with TemporaryDirectory() as tmp:
            h = Harness(tmp, fail='go_to')
            h.run('swap')
            self.assertFalse(h.success)
            self.assertEqual(h.calls[-1], ('recover', h.names))

    def test_arm_failure_never_sends_takeoff(self):
        with TemporaryDirectory() as tmp:
            h = Harness(tmp, fail='arm')
            h.run('hover')
            self.assertFalse(h.success)
            self.assertFalse(any(action == 'takeoff' for action, _ in h.calls))

    def test_unverified_avoidance_never_arms(self):
        with TemporaryDirectory() as tmp:
            h = Harness(tmp)
            def fail():
                raise RuntimeError('Routing/avoidance configuration unverified')
            h.verify_avoidance = fail
            h.run('swap')
            self.assertFalse(h.success)
            self.assertFalse(any(action in ('arm', 'takeoff', 'go_to') for action, _ in h.calls))

    def test_one_failed_request_does_not_prevent_others_being_sent(self):
        sent = []
        class Client:
            def __init__(self, name, broken=False):
                self.name, self.broken = name, broken
            def service_is_ready(self):
                return True
            def call_async(self, request):
                sent.append(self.name)
                if self.broken:
                    raise RuntimeError('transport failure')
                return S(done=lambda: True, result=lambda: S())
        obj = S(flight_service_clients={n: {'land': Client(n, n == 'cf1')} for n in ('cf1', 'cf2', 'cf3')})
        with self.assertRaisesRegex(RuntimeError, 'request errors'):
            Mission.call_many(obj, 'land', {n: S() for n in obj.flight_service_clients}, monitor=False)
        self.assertEqual(sent, ['cf1', 'cf2', 'cf3'])

    def ready_guard(self, tmp):
        h = Harness(tmp)
        now = time.monotonic()
        h.mocap, h.onboard = {}, {}
        h.expected = {}
        for name, p in h.config['_starts'].items():
            q = [*p[:2], .65]
            h.expected[name] = q
            for streams, rate in ((h.mocap, 100), (h.onboard, 20)):
                streams[name] = PoseStream()
                for i in range(2*rate+1):
                    streams[name].add(pose(q), 'world', now=now-2+i/rate)
        h.names_seen = h.names.copy()
        h.count_publishers = lambda *_: 1
        h.check_health = MethodType(Mission.check_health, h)
        h.last_log = h.last_row = now
        h.phase = 'HOVER'
        h.flight_start = now
        return h

    def test_full_guard_accepts_healthy_separated_fleet(self):
        with TemporaryDirectory() as tmp:
            h = self.ready_guard(tmp)
            Mission.guard(h)
            h.csv_handle.close()

    def test_full_guard_rejects_stale_mocap(self):
        with TemporaryDirectory() as tmp:
            h = self.ready_guard(tmp)
            h.mocap['cf2'].last_time -= 1
            with self.assertRaisesRegex(RuntimeError, 'stale/missing mocap'):
                Mission.guard(h)
            h.csv_handle.close()

    def test_full_guard_rejects_missing_rigid_body(self):
        with TemporaryDirectory() as tmp:
            h = self.ready_guard(tmp)
            h.names_seen = h.names_seen[:-1]
            with self.assertRaisesRegex(RuntimeError, 'Missing or duplicate'):
                Mission.guard(h)
            h.csv_handle.close()

    def test_full_guard_rejects_close_pair_even_when_estimates_match(self):
        with TemporaryDirectory() as tmp:
            h = self.ready_guard(tmp)
            near = h.expected['cf1'].copy()
            near[1] += .5
            h.mocap['cf2'].pose = h.onboard['cf2'].pose = pose(near)
            with self.assertRaisesRegex(RuntimeError, 'XY separation'):
                Mission.guard(h)
            h.csv_handle.close()

    def test_full_guard_rejects_tracking_corridor_departure(self):
        with TemporaryDirectory() as tmp:
            h = self.ready_guard(tmp)
            moved = h.expected['cf1'].copy()
            moved[0] += h.m['tracking_tolerance_m']+.01
            h.mocap['cf1'].pose = h.onboard['cf1'].pose = pose(moved)
            with self.assertRaisesRegex(RuntimeError, 'tracking corridor'):
                Mission.guard(h)
            h.csv_handle.close()


if __name__ == '__main__':
    unittest.main()

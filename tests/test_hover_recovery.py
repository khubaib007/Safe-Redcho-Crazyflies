"""Service ordering/recovery tests with ROS transport stubbed; no hardware simulation."""
from pathlib import Path
from types import ModuleType, SimpleNamespace as S
import importlib.util
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/safe_redcho_bringup'))
from safe_redcho_bringup.configuration import load_lab


def load_hover():
    rclpy = ModuleType('rclpy')
    duration = ModuleType('rclpy.duration')
    class Duration:
        def __init__(self, seconds):
            self.seconds = seconds
        def to_msg(self):
            return self.seconds
    duration.Duration = Duration
    signals = ModuleType('rclpy.signals')
    signals.SignalHandlerOptions = S(NO=0)
    services = ModuleType('crazyflie_interfaces.srv')
    for name in ('Arm', 'Land', 'Takeoff'):
        setattr(services, name, type(name, (), {'Request': type('Request', (), {})}))
    receiver = ModuleType('safe_redcho_bringup.receiver')
    receiver.Receiver = type('Receiver', (), {})
    overrides = {'rclpy': rclpy, 'rclpy.duration': duration, 'rclpy.signals': signals,
                 'crazyflie_interfaces': ModuleType('crazyflie_interfaces'),
                 'crazyflie_interfaces.srv': services,
                 'safe_redcho_bringup.receiver': receiver}
    spec = importlib.util.spec_from_file_location(
        'safe_redcho_bringup._hover_transport_test',
        ROOT/'src/safe_redcho_bringup/safe_redcho_bringup/single_hover.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, overrides):
        spec.loader.exec_module(module)
    return module.Hover


Hover = load_hover()


class Harness(Hover):
    def __init__(self, failure=None):
        self.f = load_lab(ROOT/'config/lab_config.yaml')[0]['flight']
        self.events = []
        self.failure = failure
        self.airborne_possible = False
        self.arm_client, self.takeoff_client, self.land_client = 'arm', 'takeoff', 'land'
        self.start_position = (1., -2., .07)
        self.mocap = S(pose=S(position=S(x=1., y=-2., z=.57)))
    def preflight(self):
        self.events.append('preflight')
    def spin_for(self, seconds):
        pass
    def health(self, stationary=False):
        return None
    def call(self, client, request, timeout=3):
        self.events.append((client, getattr(request, 'arm', None),
                            getattr(request, 'height', None)))
        if self.failure == client:
            self.failure = None
            raise RuntimeError('Injected uncertain service outcome')
    def monitor(self, seconds, hover=False):
        if self.failure == 'monitor':
            raise RuntimeError('Injected stale mocap')
    def land(self):
        self.events.append('recovery_land')
        self.airborne_possible = False
    def get_logger(self):
        return S(info=lambda *_: None, warning=lambda *_: None)


class Recovery(unittest.TestCase):
    def test_check_only_never_arms(self):
        h = Harness()
        h.run(check_only=True)
        self.assertEqual(h.events, ['preflight'])

    def test_nominal_absolute_height_and_landing(self):
        h = Harness()
        h.run(check_only=False)
        self.assertEqual(h.events[1], ('arm', True, None))
        self.assertEqual(h.events[2][0], 'takeoff')
        self.assertAlmostEqual(h.events[2][2], .57)
        self.assertEqual(h.events[-1], 'recovery_land')

    def test_uncertain_takeoff_outcome_still_requests_landing(self):
        h = Harness(failure='takeoff')
        with self.assertRaises(RuntimeError):
            h.run(check_only=False)
        self.assertEqual(h.events[-1], 'recovery_land')

    def test_stale_mocap_during_flight_requests_landing(self):
        h = Harness(failure='monitor')
        with self.assertRaises(RuntimeError):
            h.run(check_only=False)
        self.assertEqual(h.events[-1], 'recovery_land')

    def test_uncertain_arming_attempt_is_followed_by_disarm_without_takeoff(self):
        h = Harness(failure='arm')
        with self.assertRaises(RuntimeError):
            h.run(check_only=False)
        self.assertEqual(h.events[-1], ('arm', False, None))
        self.assertFalse(any(isinstance(e, tuple) and e[0] == 'takeoff' for e in h.events))

    def test_real_landing_does_not_disarm_without_touchdown(self):
        h = Harness()
        h.mocap.stats = lambda: {'age_s': 0.0}
        with self.assertRaisesRegex(RuntimeError, 'Touchdown not verified'):
            Hover.land(h)
        self.assertFalse(any(isinstance(e, tuple) and e[0] == 'arm' for e in h.events))

    def test_real_landing_disarms_after_fresh_ground_measurement(self):
        h = Harness()
        h.mocap.pose.position.z = .08
        h.mocap.stats = lambda: {'age_s': 0.0}
        Hover.land(h)
        self.assertEqual(h.events[-1], ('arm', False, None))


if __name__ == '__main__':
    unittest.main()

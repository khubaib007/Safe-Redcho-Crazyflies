import types
import unittest
from concurrent.futures import Future
from safe_redcho_three_mission.landing import LandingRequests


NAMES = ('cf2', 'cf3', 'cf4')


class Service:
    Request = types.SimpleNamespace


class Client:
    def __init__(self, name, kind, calls):
        self.name, self.kind, self.calls = name, kind, calls
        self.futures = []
        self.available = True

    def service_is_ready(self):
        return self.available

    def call_async(self, request):
        self.calls.append((self.name, self.kind, request))
        future = Future()
        self.futures.append(future)
        return future


class LandingTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.calls = []
        self.clients = {n: {k: Client(n, k, self.calls)
                           for k in ('notify_setpoints_stop', 'land')} for n in NAMES}
        self.requests = {n: object() for n in NAMES}

    def start(self, release=True):
        return LandingRequests(self.clients, self.requests, Service,
                               release=release, clock=lambda: self.now)

    def reply(self, n, kind, success=True):
        self.clients[n][kind].futures[-1].set_result(types.SimpleNamespace(success=success))

    def test_first_aircraft_lands_without_waiting_for_other_release_replies(self):
        h = self.start()
        self.reply('cf2', 'notify_setpoints_stop')
        h.tick()
        self.assertEqual([(n, k) for n, k, _ in self.calls[-1:]], [('cf2', 'land')])
        self.assertEqual(h.events[-1]['monotonic_s'], 10.)
        self.assertFalse(self.clients['cf3']['land'].futures)

    def test_all_release_replies_dispatch_all_lands_in_same_tick(self):
        h = self.start()
        for n in NAMES:
            self.reply(n, 'notify_setpoints_stop')
        h.tick()
        for n in NAMES:
            self.assertIs(self.calls[[x[:2] for x in self.calls].index((n, 'land'))][2], self.requests[n])
            self.reply(n, 'land')
        h.tick()
        self.assertTrue(h.done)
        self.assertNotIn('handover_error', h.report())

    def test_missing_release_reply_is_bounded_and_land_is_still_attempted(self):
        h = self.start()
        self.now += .151
        h.tick()
        self.assertTrue(all(self.clients[n]['land'].futures for n in NAMES))
        self.assertIn('physical outcome uncertain', h.report()['handover_error'])

    def test_release_failure_still_dispatches_land_without_delay(self):
        h = self.start()
        self.reply('cf2', 'notify_setpoints_stop', False)
        h.tick()
        self.assertTrue(self.clients['cf2']['land'].futures)
        self.assertIn('cf2/notify_setpoints_stop', h.report()['handover_error'])

    def test_unavailable_release_still_attempts_land(self):
        self.clients['cf2']['notify_setpoints_stop'].available = False
        h = self.start()
        self.assertTrue(self.clients['cf2']['land'].futures)
        self.assertIn('Service unavailable', h.report()['handover_error'])

    def test_land_timeout_terminates_and_is_reported_without_retrying_trajectory(self):
        h = self.start(False)
        self.now += 2.01
        h.tick()
        self.assertTrue(h.done)
        self.assertEqual(len(self.calls), 3)
        self.assertIn('cf2/land', h.report()['handover_error'])

    def test_takeoff_recovery_needs_no_stream_release(self):
        h = self.start(False)
        self.assertEqual([k for _, k, _ in self.calls], ['land'] * 3)
        self.assertFalse(h.done)

    def test_no_request_is_reissued_after_completion(self):
        h = self.start(False)
        for n in NAMES:
            self.reply(n, 'land')
        for _ in range(10):
            h.tick()
        self.assertTrue(h.done)
        self.assertEqual(len(self.calls), 3)


if __name__ == '__main__':
    unittest.main()

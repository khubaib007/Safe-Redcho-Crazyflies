"""Bounded, per-aircraft release -> land requests. No ROS import or sleeps.

Service completion is host-side evidence, not proof of firmware/RF delivery.
Call tick() while continuing to spin the node so heartbeats remain live.
"""
import time


class LandingRequests:
    def __init__(self, clients, requests, notify_type, release=True,
                 clock=time.monotonic, notify_timeout=.15, land_timeout=2.):
        self.clients, self.requests, self.clock = clients, requests, clock
        self.notify_type = notify_type
        self.notify_timeout, self.land_timeout = notify_timeout, land_timeout
        self.pending, self.completed, self.errors, self.events = {}, set(), [], []
        for name in requests:
            self._start(name, 'notify_setpoints_stop' if release else 'land')

    @property
    def done(self):
        return len(self.completed) == len(self.requests)

    def _start(self, name, kind):
        started = self.clock()
        self.events.append(dict(name=name, service=kind, monotonic_s=started))
        try:
            client = self.clients[name][kind]
            if not client.service_is_ready():
                raise RuntimeError('Service unavailable')
            request = self.requests[name]
            if kind == 'notify_setpoints_stop':
                request = self.notify_type.Request()
                request.group_mask = 0
                request.remain_valid_millisecs = 100  # Legacy field; not a sleep.
            future = client.call_async(request)
            timeout = self.notify_timeout if kind == 'notify_setpoints_stop' else self.land_timeout
            self.pending[name] = (kind, future, started + timeout)
        except Exception as exc:
            self._finish(name, kind, str(exc))

    def _finish(self, name, kind, error=None):
        self.pending.pop(name, None)
        if error:
            self.errors.append(name + '/' + kind + ': ' + error)
        if kind == 'notify_setpoints_stop':
            # Do not wait for the other aircraft, or add a release dwell.
            # Even after a release error, attempt the controlled land request.
            self._start(name, 'land')
        else:
            self.completed.add(name)

    def tick(self):
        for name, (kind, future, deadline) in list(self.pending.items()):
            if future.done():
                error = None
                try:
                    reply = future.result()
                    if reply is None or (hasattr(reply, 'success') and not reply.success):
                        raise RuntimeError('Service returned failure')
                except Exception as exc:
                    error = str(exc)
                self._finish(name, kind, error)
            elif self.clock() >= deadline:
                future.cancel()
                self._finish(name, kind, 'Timed out; physical outcome uncertain')

    def report(self):
        result = dict(handover_events=self.events,
                      handover_scope='Host service requests/replies; RF and firmware execution unverified')
        if self.errors:
            result['handover_error'] = '; '.join(self.errors)
        return result

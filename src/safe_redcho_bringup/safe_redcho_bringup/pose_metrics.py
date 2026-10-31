"""Small ROS-independent checks; rates/gaps refer to local reception times."""
from collections import deque
import math
import time


def coordinates(pose):
    return (pose.position.x, pose.position.y, pose.position.z)


def quaternion(pose):
    q = pose.orientation
    return (q.x, q.y, q.z, q.w)


def valid_pose(pose):
    values = coordinates(pose) + quaternion(pose)
    norm = sum(v*v for v in quaternion(pose))**0.5
    return all(math.isfinite(v) for v in values) and 0.95 <= norm <= 1.05


def distance(a, b):
    return math.sqrt(sum((x-y)**2 for x, y in zip(a, b)))


def orientation_error(a, b):
    qa, qb = quaternion(a), quaternion(b)
    dot = abs(sum(x*y for x, y in zip(qa, qb)))
    norm = math.sqrt(sum(x*x for x in qa)*sum(x*x for x in qb))
    return 2*math.acos(min(1.0, dot/norm))


def tilt(pose):
    x, y, z, w = quaternion(pose)
    return math.acos(max(-1.0, min(1.0, 1-2*(x*x+y*y)/(x*x+y*y+z*z+w*w))))


class PoseStream:
    def __init__(self):
        self.samples = deque(maxlen=5000)
        self.pose = None
        self.last_time = None
        self.frame_id = None
        self.invalid_samples = 0
        self.count = 0

    def add(self, pose, frame_id, now=None):
        if not valid_pose(pose):
            self.invalid_samples += 1
            return False
        now = time.monotonic() if now is None else now
        self.pose, self.last_time, self.frame_id = pose, now, frame_id
        self.samples.append((now, coordinates(pose)))
        self.count += 1
        return True

    def stats(self, now=None, window=2.0):
        now = time.monotonic() if now is None else now
        rows = [(t, p) for t, p in self.samples if t >= now-window]
        age = math.inf if self.last_time is None else now-self.last_time
        gaps = [rows[i+1][0]-rows[i][0] for i in range(len(rows)-1)]
        rate = ((len(rows)-1)/(rows[-1][0]-rows[0][0])
                if len(rows) > 1 and rows[-1][0] > rows[0][0] else 0.0)
        spans = ([max(p[k] for _, p in rows)-min(p[k] for _, p in rows)
                  for k in range(3)] if rows else [math.inf]*3)
        return {'rate_hz': rate, 'age_s': age,
                'maximum_gap_s': max(gaps+[age]), 'span_m': spans,
                'samples': len(rows),
                'coverage_s': rows[-1][0]-rows[0][0] if len(rows) > 1 else 0.0}

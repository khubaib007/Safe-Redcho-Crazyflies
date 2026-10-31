"""Timestamped measurements; no threshold relaxation or fabricated observations."""
from collections import deque
import math
import numpy as np


def unit_quaternion(value):
    q = np.asarray(value, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise RuntimeError('Invalid quaternion components')
    norm = float(np.linalg.norm(q))
    if not 0.9 <= norm <= 1.1:
        raise RuntimeError(f'Invalid quaternion norm {norm:.6f}')
    return q / norm


def rotation_error(a, b):
    a, b = unit_quaternion(a), unit_quaternion(b)
    # q and -q describe the same rotation. atan2 remains accurate near zero.
    if float(a @ b) < 0:
        b = -b
    return float(4 * math.atan2(np.linalg.norm(a-b), np.linalg.norm(a+b)))


def slerp(a, b, fraction):
    a, b = unit_quaternion(a), unit_quaternion(b)
    dot = float(a @ b)
    if dot < 0:
        b, dot = -b, -dot
    dot = float(np.clip(dot, -1., 1.))
    if dot > .9995:
        return unit_quaternion((1-fraction)*a + fraction*b)
    angle = math.acos(dot)
    return (math.sin((1-fraction)*angle)*a + math.sin(fraction*angle)*b)/math.sin(angle)


class PoseHistory:
    def __init__(self):
        self.rows = deque(maxlen=1024)

    def add(self, stamp, received, position, orientation):
        p = np.asarray(position, dtype=float)
        q = unit_quaternion(orientation)
        if p.shape != (3,) or not np.isfinite(p).all():
            raise RuntimeError('Nonfinite/invalid position')
        if not math.isfinite(stamp) or not math.isfinite(received) or stamp <= 0:
            raise RuntimeError('Invalid measurement time')
        if self.rows and stamp <= self.rows[-1][0]:
            return False  # repeated/reordered samples never renew history
        self.rows.append((float(stamp), float(received), p.copy(), q.copy()))
        while self.rows and stamp-self.rows[0][0] > 3.:
            self.rows.popleft()
        return True

    def latest(self, source_now, receive_now, maximum_age):
        if not self.rows:
            raise RuntimeError('No timestamped pose history')
        row = self.rows[-1]
        if receive_now-row[1] > maximum_age or source_now-row[0] > maximum_age:
            raise RuntimeError('Timestamped pose history is stale')
        if row[0]-source_now > .05:
            raise RuntimeError('Pose timestamp is in the future')
        return row

    def at(self, stamp, maximum_skew, maximum_gap):
        if not self.rows:
            raise RuntimeError('No pose history for alignment')
        rows = list(self.rows)
        for i, row in enumerate(rows):
            if row[0] == stamp:
                return row[2], row[3], 0., 'exact'
            if row[0] > stamp:
                if i:
                    left = rows[i-1]
                    gap = row[0]-left[0]
                    if gap > maximum_gap:
                        raise RuntimeError(f'Pose alignment crosses {gap:.3f}s gap')
                    f = (stamp-left[0])/gap
                    return ((1-f)*left[2]+f*row[2], slerp(left[3], row[3], f),
                            0., 'interpolated')
                break
        nearest = min(rows, key=lambda r: abs(r[0]-stamp))
        skew = abs(nearest[0]-stamp)
        if skew > maximum_skew:
            raise RuntimeError(f'Pose alignment skew {skew:.3f}s exceeds {maximum_skew:.3f}s')
        # Bounded nearest sample, explicitly identified; no extrapolation.
        return nearest[2], nearest[3], float(skew), 'nearest'

    def velocity(self, window, maximum_gap):
        if not self.rows:
            raise RuntimeError('No mocap velocity history')
        end = self.rows[-1][0]
        rows = [r for r in self.rows if r[0] >= end-window-1e-9]
        if len(rows) < 5 or rows[-1][0]-rows[0][0] < window*.5:
            raise RuntimeError('Insufficient timestamped mocap velocity history')
        times = np.array([r[0]-rows[0][0] for r in rows])
        if float(np.max(np.diff(times))) > maximum_gap:
            raise RuntimeError('Mocap velocity fit crosses a reception/source gap')
        positions = np.array([r[2] for r in rows])
        centered = times-times.mean()
        velocity = centered @ positions / float(centered @ centered)
        fitted = positions.mean(axis=0)+centered[:, None]*velocity
        residual = float(np.max(np.linalg.norm(positions-fitted, axis=1)))
        if not np.isfinite(velocity).all():
            raise RuntimeError('Nonfinite mocap velocity')
        return velocity, {'samples': len(rows), 'span_s': float(times[-1]),
                          'max_gap_s': float(np.max(np.diff(times))),
                          'maximum_fit_residual_m': residual}


def touchdown_evidence(stream, start_z, now, maximum_age):
    """Keep the original 6 cm height/2 cm span limits, require real coverage."""
    rows = [(t, np.asarray(p, float)) for t, p in stream.samples if now-.5 <= t <= now]
    result = {'verified': False, 'samples': len(rows)}
    if len(rows) < 10:
        result['reason'] = 'insufficient samples'; return result
    times = np.array([r[0] for r in rows])
    positions = np.array([r[1] for r in rows])
    if positions.shape != (len(rows), 3) or not np.isfinite(positions).all():
        result['reason'] = 'invalid positions'; return result
    span = float(times[-1]-times[0])
    gap = float(np.max(np.diff(times)))
    height = float(np.max(np.abs(positions[:, 2]-start_z)))
    axis_span = float(np.max(np.ptp(positions, axis=0)))
    result.update(coverage_s=span, maximum_gap_s=gap,
                  maximum_height_error_m=height, maximum_axis_span_m=axis_span,
                  age_s=float(now-times[-1]))
    result['verified'] = bool(span >= .4 and gap <= maximum_age and
                              now-times[-1] <= maximum_age and height < .06 and axis_span < .02)
    result['reason'] = 'qualified' if result['verified'] else 'ground/stability/coverage condition failed'
    return result

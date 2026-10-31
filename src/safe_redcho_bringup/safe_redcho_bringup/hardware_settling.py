"""Bounded commissioning endpoint checks; uses the existing mission safety guard.

No ROS dependencies: this module does not publish commands or change coordinates.
"""
import json
import math
from pathlib import Path
import time


DEFAULTS = {
    'settle_timeout_s': 10.0,
    'settle_window_s': 0.8,
    'settle_minimum_coverage_s': 0.6,
    'settle_minimum_samples': 20,
    'settle_endpoint_tolerance_m': 0.085,
    'settle_maximum_axis_span_m': 0.06,
    'settle_maximum_speed_mps': 0.10,
    'settle_inlier_fraction': 0.90,
    'heading_world_rad': 0.0,
    'maximum_start_heading_error_rad': 0.35,
}


def settings(config):
    options = dict(DEFAULTS)
    supplied = config.get('hardware_runtime', {})
    if not isinstance(supplied, dict):
        raise ValueError('hardware_runtime must be a mapping')
    unknown = set(supplied)-set(options)
    if unknown:
        raise ValueError(f'Unknown hardware_runtime settings: {sorted(unknown)}')
    options.update(supplied)
    for key, value in options.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'hardware_runtime.{key} must be finite numeric')
        if key != 'heading_world_rad' and value <= 0:
            raise ValueError(f'hardware_runtime.{key} must be positive')
    if not 0.8 <= options['settle_inlier_fraction'] <= 1.0:
        raise ValueError('settle_inlier_fraction must be between 0.8 and 1')
    count = options['settle_minimum_samples']
    if int(count) != count or count < 3:
        raise ValueError('settle_minimum_samples must be an integer >= 3')
    m = config['mission']
    if options['settle_endpoint_tolerance_m'] >= m['tracking_tolerance_m']:
        raise ValueError('Settle endpoint tolerance must be strictly inside the tracking corridor')
    if options['settle_minimum_coverage_s'] >= options['settle_window_s']:
        raise ValueError('Settle coverage must be shorter than its window')
    if options['settle_timeout_s'] <= options['settle_window_s']+m['settle_duration_s']:
        raise ValueError('Settle timeout must cover the observation window and dwell')
    if options['settle_maximum_speed_mps'] > m['maximum_speed_mps']:
        raise ValueError('Settling speed limit must not exceed planned maximum speed')
    if options['maximum_start_heading_error_rad'] > math.pi/2:
        raise ValueError('Start heading tolerance must be <= pi/2')
    # The same larger corridor is used by both the planner and live guard.
    required = m['minimum_separation_m']+2*m['tracking_tolerance_m']+m['planning_reserve_m']
    if not m['minimum_separation_m'] < m['abort_separation_m'] < required:
        raise ValueError('Invalid minimum/abort/planning separation ordering')
    return options


def heading_error(yaw, target):
    return math.atan2(math.sin(yaw-target), math.cos(yaw-target))


def configure_runtime(node):
    options = settings(node.config)  # Runs before arming, including --check-only.
    headings = {}
    for name in node.names:
        q = node.mocap[name].pose.orientation
        norm2 = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        if not math.isfinite(norm2) or norm2 <= 0:
            raise RuntimeError(f'{name}: invalid heading quaternion')
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y)/norm2,
                         1-2*(q.y*q.y+q.z*q.z)/norm2)
        error = heading_error(yaw, options['heading_world_rad'])
        headings[name] = {'yaw_rad': yaw, 'target_rad': options['heading_world_rad'],
                          'error_rad': error}
        if abs(error) > options['maximum_start_heading_error_rad']:
            raise RuntimeError(f'{name}: heading differs from configured +X direction by '
                               f'{math.degrees(error):.1f} deg; check physical heading and rigid-body axes')
    node.hardware_runtime_headings = headings
    node.get_logger().info('Hardware endpoint checks enabled; world yaw target='
                           f"{options['heading_world_rad']:.3f} rad; "
                           f"endpoint={options['settle_endpoint_tolerance_m']:.3f} m, "
                           f"tracking corridor={node.m['tracking_tolerance_m']:.3f} m")


def command_yaw(node, name, previous_yaw):
    # GoTo positions are already world-frame coordinates; no heading rotation.
    return settings(node.config)['heading_world_rad']


def evaluate_samples(samples, target, now, options):
    rows = [(float(t), tuple(float(x) for x in p)) for t, p in samples
            if now-options['settle_window_s'] <= t <= now]
    reasons = []
    if len(rows) < options['settle_minimum_samples']:
        return {'stable': False, 'reasons': ['insufficient_samples'], 'samples': len(rows)}
    if any(len(p) != 3 or not math.isfinite(t) or not all(math.isfinite(v) for v in p)
           for t, p in rows):
        return {'stable': False, 'reasons': ['nonfinite_sample'], 'samples': len(rows)}
    if any(rows[i][0] >= rows[i+1][0] for i in range(len(rows)-1)):
        return {'stable': False, 'reasons': ['unordered_samples'], 'samples': len(rows)}
    coverage = rows[-1][0]-rows[0][0]
    errors = [math.dist(p, target) for _, p in rows]
    inside = sum(e <= options['settle_endpoint_tolerance_m'] for e in errors)/len(errors)
    span = [max(p[k] for _, p in rows)-min(p[k] for _, p in rows) for k in range(3)]
    # Least-squares velocity over the whole window rejects drift without
    # differentiating consecutive noisy mocap samples.
    ts = [t-rows[0][0] for t, _ in rows]
    mean_t = sum(ts)/len(ts)
    centered = [t-mean_t for t in ts]
    denominator = sum(t*t for t in centered)
    velocity = [sum(t*p[k] for t, (_, p) in zip(centered, rows))/denominator
                if denominator > 0 else 0.0 for k in range(3)]
    speed = math.sqrt(sum(v*v for v in velocity))
    if coverage < options['settle_minimum_coverage_s']:
        reasons.append('insufficient_coverage')
    if errors[-1] > options['settle_endpoint_tolerance_m']:
        reasons.append('current_endpoint_error')
    if inside < options['settle_inlier_fraction']:
        reasons.append('endpoint_window_error')
    if max(span) > options['settle_maximum_axis_span_m']:
        reasons.append('hover_motion_span')
    if speed > options['settle_maximum_speed_mps']:
        reasons.append('drifting')
    return {'stable': not reasons, 'reasons': reasons, 'samples': len(rows),
            'coverage_s': coverage, 'current_error_m': errors[-1],
            'maximum_error_m': max(errors), 'endpoint_inlier_fraction': inside,
            'axis_span_m': span, 'fitted_velocity_mps': velocity, 'fitted_speed_mps': speed}


def _save(node, record):
    records = getattr(node, '_hardware_settle_records', [])
    if not hasattr(node, '_hardware_settle_records'):
        node._hardware_settle_records = records
    if not records or records[-1] is not record:
        records.append(record)
    destination = Path(node.output)/'settle_diagnostics.json'
    temporary = destination.with_suffix('.json.tmp')
    temporary.write_text(json.dumps({'version': 1, 'checks': records}, indent=2, allow_nan=False)+'\n')
    temporary.replace(destination)


def wait_for_settle(node, targets, duration=None):
    options = settings(node.config)
    needed = node.m['settle_duration_s'] if duration is None else float(duration)
    if not math.isfinite(needed) or needed <= 0 or needed >= options['settle_timeout_s']:
        raise ValueError('Invalid settle dwell duration')
    if set(targets) != set(node.names) or any(len(p) != 3 or not all(math.isfinite(v) for v in p)
                                            for p in targets.values()):
        raise ValueError('Settle targets must contain finite world positions for all six aircraft')
    started = time.monotonic()
    deadline = started+options['settle_timeout_s']
    good_since, last_notice = None, -math.inf
    record = {'phase': node.phase, 'status': 'WAITING', 'targets': {n: list(p) for n, p in targets.items()},
              'settings': options, 'dwell_s': needed, 'agents': {}}
    try:
        while time.monotonic() < deadline:
            # Collision, corridor, frame, freshness and arena guards still run
            # every iteration. A rolling settling statistic never bypasses them.
            node.spin_for(0.02, node.guard)
            now = time.monotonic()
            metrics = {n: evaluate_samples(list(node.mocap[n].samples), targets[n], now, options)
                       for n in node.names}
            record.update(agents=metrics, elapsed_s=now-started)
            if all(value['stable'] for value in metrics.values()):
                good_since = now if good_since is None else good_since
                if now-good_since >= needed:
                    record['status'] = 'PASS'
                    _save(node, record)
                    node.get_logger().info(f'Endpoints settled: all six aircraft; elapsed={now-started:.2f}s')
                    return
            else:
                good_since = None
            if now-last_notice >= 1.0:
                blocked = {n: {k: v for k, v in value.items()
                               if k in ('reasons', 'current_error_m', 'axis_span_m', 'fitted_speed_mps')}
                           for n, value in metrics.items() if not value['stable']}
                node.get_logger().info('Waiting for endpoints: '+json.dumps(blocked))
                _save(node, record)
                last_notice = now
        record['status'] = 'TIMEOUT'
        blocked = {n: value['reasons'] for n, value in record['agents'].items() if not value['stable']}
        raise RuntimeError(f'Endpoint settle timeout after {options["settle_timeout_s"]:.1f}s: '
                           f'{blocked}; inspect settle_diagnostics.json')
    except BaseException as exc:
        record['status'] = 'TIMEOUT' if record['status'] == 'TIMEOUT' else 'ABORTED'
        record.update(error=f'{type(exc).__name__}: {exc}', elapsed_s=time.monotonic()-started)
        _save(node, record)
        raise

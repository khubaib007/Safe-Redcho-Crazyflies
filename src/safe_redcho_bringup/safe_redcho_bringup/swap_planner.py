"""Exact line-to-point XY clearance checks for a serial buffered cyclic swap.

This plans reference geometry, not HOCBF or a guarantee on a disturbed aircraft.
The tracking tube and reserve must be validated experimentally.
"""
from copy import deepcopy
import argparse
import json
import math
from pathlib import Path
from .fleet_configuration import load_fleet


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def point_segment_distance(point, start, end):
    delta = [b-a for a, b in zip(start, end)]
    length2 = sum(v*v for v in delta)
    fraction = (sum((p-a)*d for p, a, d in zip(point, start, delta))/length2
                if length2 > 0 else 0.)
    fraction = min(1., max(0., fraction))
    return norm([p-a-fraction*d for p, a, d in zip(point, start, delta)])


def move_duration(distance, config):
    m = config['mission']
    # Rest-to-rest seventh-degree smoothstep: exact normalized peak velocity 35/16.
    # 8 is a conservative bound on its normalized peak acceleration (~7.5132).
    return max(m['move_min_duration_s'], (35/16)*distance/m['maximum_speed_mps'],
               math.sqrt(8*distance/m['maximum_acceleration_mps2']))


def check_segment(start, end, others, config):
    a, m = config['arena'], config['mission']
    required = m['minimum_separation_m']+2*m['tracking_tolerance_m']+m['planning_reserve_m']
    wall = a['wall_clearance_m']+a['body_radius_m']+m['tracking_tolerance_m']+m['planning_reserve_m']
    for p in (start, end):
        for index, key in ((0, 'x_limits'), (1, 'y_limits')):
            if not a[key][0]+wall <= p[index] <= a[key][1]-wall:
                raise ValueError(f'Path endpoint {p} violates {key} including tracking/body margins')
    minimum = math.inf
    for name, p in others.items():
        d = point_segment_distance(p[:2], start[:2], end[:2])
        minimum = min(minimum, d)
        if d < required-1e-10:
            raise ValueError(f'Path passes {name} at {d:.4f} m; need >= {required:.4f} m planned XY separation')
    for obs in config['obstacles']:
        d = point_segment_distance(obs['center_xy'], start[:2], end[:2])-obs['radius_m']
        required_obs = (m['obstacle_surface_clearance_m']+a['body_radius_m']+
                        m['tracking_tolerance_m']+m['planning_reserve_m'])
        if d < required_obs-1e-10:
            raise ValueError(f'Path obstacle surface clearance {d:.4f} m is below {required_obs:.4f} m')
    return minimum


def build_plan(config, positions=None, mode='swap'):
    if mode not in ('hover', 'swap'):
        raise ValueError('mode must be hover or swap')
    starts = deepcopy(positions or config['_starts'])
    names = list(config['robots'])
    a, m = config['arena'], config['mission']
    height = a['floor_z_m']+m['hover_height_above_floor_m']
    if not a['floor_z_m']+0.30 <= height <= a['ceiling_z_m']-a['ceiling_clearance_m']:
        raise ValueError('Hover plane does not fit usable floor/ceiling bounds')
    for name in names:
        if not a['floor_z_m']-0.03 <= starts[name][2] <= a['floor_z_m']+0.10:
            raise ValueError(f'{name} must start on the calibrated floor, not already airborne')
    current = {name: [*starts[name][:2], height] for name in names}
    minima = []
    for name in names:
        minima.append(check_segment(current[name], current[name],
                                    {n: p for n, p in current.items() if n != name}, config))
    legs = []
    destinations = deepcopy(current)
    if mode == 'swap':
        cycle = config['swap']['cycle']
        buffer = config['swap']['buffer_xy']
        if buffer is None:
            buffer = [sum(starts[n][k] for n in names)/6 for k in (0, 1)]
        if len(buffer) != 2 or not all(math.isfinite(float(x)) for x in buffer):
            raise ValueError('swap.buffer_xy must be null or two finite world coordinates')
        buffer = [*buffer, height]
        for i, name in enumerate(cycle):
            destinations[name] = [*starts[cycle[(i+1)%6]][:2], height]

        def add(name, target):
            start = current[name]
            minimum = check_segment(start, target,
                                    {n: p for n, p in current.items() if n != name}, config)
            distance = norm([x-y for x, y in zip(start, target)])
            legs.append({'robot': name, 'start': list(start), 'goal': list(target),
                         'duration_s': move_duration(distance, config),
                         'min_nominal_xy_separation_m': minimum,
                         'stationary_positions': {n: list(p) for n, p in current.items() if n != name}})
            minima.append(minimum)
            current[name] = list(target)

        add(cycle[0], buffer)
        for name in reversed(cycle[1:]):
            add(name, destinations[name])
        add(cycle[0], destinations[cycle[0]])
    estimate = (m['takeoff_duration_s']+m['settle_duration_s']+2*m['hover_duration_s']+
                sum(leg['duration_s']+2*m['settle_duration_s'] for leg in legs)+
                m['landing_duration_s']+4.0)
    if estimate > m['maximum_flight_duration_s']-10:
        raise ValueError(f'Planned flight ~{estimate:.1f}s leaves insufficient time for recovery within flight timeout')
    return {'mode': mode, 'frame_id': config['_lab']['mocap']['frame_id'],
            'ground_starts': starts, 'hover_height_world_m': height,
            'initial_hover_positions': {n: [*starts[n][:2], height] for n in names},
            'final_hover_positions': destinations, 'legs': legs,
            'minimum_nominal_xy_separation_m': min(minima),
            'minimum_required_planned_xy_separation_m': (
                m['minimum_separation_m']+2*m['tracking_tolerance_m']+m['planning_reserve_m']),
            'estimated_flight_duration_s': estimate,
            'scope': 'Prechecked serial reference paths with tracking allowance; no REDCHO or HOCBF-QP'}


def main():
    parser = argparse.ArgumentParser(description='Offline geometry preview; no radio or flight commands')
    parser.add_argument('--config')
    parser.add_argument('--mode', choices=('hover', 'swap'), default='swap')
    parser.add_argument('--output', default=None)
    args = parser.parse_args()
    cfg = load_fleet(args.config, require_starts=True, require_arena=True)
    plan = build_plan(cfg, mode=args.mode)
    destination = Path(args.output or Path(cfg['_path']).parent.parent/'results/six_swap_plan.json')
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(plan, indent=2)+'\n')
    print(json.dumps(plan, indent=2))
    print('Plan written:', destination)

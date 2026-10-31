from pathlib import Path
import math
import numpy as np
import yaml


def load(path):
    path = Path(path).expanduser().resolve()
    cfg = yaml.safe_load(path.read_text())
    validate(cfg)
    cfg['_path'] = str(path)
    return cfg


def slots(cfg):
    center = np.r_[cfg['mission']['center_start_xy'],
                   cfg['arena']['floor_z_m']+cfg['mission']['hover_height_above_floor_m']]
    return np.array([center+np.asarray(cfg['mission']['offsets_xyz'][n], float) for n in cfg['names']])


def pair_distance(cfg):
    s = cfg['safety']
    return (s['minimum_physical_xy_separation_m']+2*s['tracking_tube_m']+
            2*s['mocap_position_uncertainty_m']+s['pair_reserve_m'])


def virtual_box(cfg):
    a, s = cfg['arena'], cfg['safety']
    margin = (a['body_radius_m']+a['wall_surface_clearance_m']+s['tracking_tube_m']+
              s['mocap_position_uncertainty_m']+s['wall_reserve_m'])
    lower = np.array([a['x_limits'][0]+margin, a['y_limits'][0]+margin,
                      a['floor_z_m']+s['airborne_minimum_height_above_floor_m']])
    upper = np.array([a['x_limits'][1]-margin, a['y_limits'][1]-margin,
                      a['ceiling_z_m']-a['ceiling_clearance_m']-s['tracking_tube_m']-
                      s['mocap_position_uncertainty_m']-s['wall_reserve_m']])
    return lower, upper


def validate(cfg):
    def finite_numbers(obj, path=''):
        if isinstance(obj, dict):
            for k, v in obj.items(): finite_numbers(v, path+'.'+k)
        elif isinstance(obj, list):
            for v in obj: finite_numbers(v, path)
        elif isinstance(obj, (int, float)) and not math.isfinite(obj):
            raise ValueError('Nonfinite setting '+path)
    finite_numbers(cfg)
    if cfg['names'] != ['cf2', 'cf3', 'cf4']:
        raise ValueError('This separate commissioning profile requires cf2, cf3, cf4')
    if cfg['frame_id'] != 'world':
        raise ValueError('This profile requires the already-calibrated world frame')
    for section in ('arena', 'mission', 'control', 'redcho', 'safety', 'battery'):
        if not isinstance(cfg[section], dict): raise ValueError('Missing '+section)
    for key in ('floor_z_m', 'ceiling_z_m'):
        if not isinstance(cfg['arena'][key], (int, float)):
            raise ValueError('Install the profile to inherit calibrated '+key)
    m, c, s, b = (cfg[x] for x in ('mission', 'control', 'safety', 'battery'))
    positive = {
        'arena': ['ceiling_clearance_m', 'body_radius_m', 'wall_surface_clearance_m'],
        'mission': ['distance_m', 'nominal_forward_duration_s', 'maximum_wall_duration_s',
                    'takeoff_duration_s', 'observer_warmup_s', 'final_hold_s', 'landing_duration_s',
                    'endpoint_tolerance_m', 'endpoint_speed_mps', 'endpoint_dwell_s',
                    'initial_xy_tolerance_m', 'hover_height_above_floor_m', 'height_spread_tolerance_m'],
        'control': list(c), 'safety': list(s),
        'redcho': ['theta', 'velocity_L', 'position_L', 'epsilon', 'svf_omega'],
        'battery': list(b),
    }
    for section, keys in positive.items():
        for key in keys:
            value = cfg[section][key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
                raise ValueError(f'{section}.{key} must be positive finite numeric')
    for key in ('gamma', 'gains'):
        values = cfg['redcho'][key]
        if len(values) != 3 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or v <= 0 for v in values):
            raise ValueError('redcho.'+key+' requires three positive finite numbers')
    if not 0.005 <= c['dt_s'] <= 0.025:
        raise ValueError('Control period must lie in [0.005, 0.025] s')
    # if c['maximum_reference_speed_mps'] > 0.12:
    #     raise ValueError('This commissioning profile caps reference speed at 0.12 m/s')
    # if 1.875*m['distance_m']/m['nominal_forward_duration_s'] > 0.04:
    #     raise ValueError('Slow mission peak nominal velocity must not exceed 0.04 m/s')
    if c['maximum_reference_speed_mps'] > 0.22:
        raise ValueError('Reference speed must not exceed 0.22 m/s')

    peak_nominal_speed = (1.875 * m['distance_m'] / m['nominal_forward_duration_s'])
    if peak_nominal_speed > 0.20:
        raise ValueError('Peak nominal trajectory speed must not exceed 0.20 m/s')

    if peak_nominal_speed > c['maximum_reference_speed_mps']:
        raise ValueError('Peak nominal trajectory speed exceeds the reference speed limit')


    if m['endpoint_tolerance_m'] >= s['tracking_tube_m']:
        raise ValueError('Endpoint tolerance must be inside the tracking tube')
    if not 0 < b['minimum_speed_scale'] <= 1 or not b['land_voltage_v'] < b['full_speed_voltage_v'] <= b['minimum_ground_voltage_v']:
        raise ValueError('Invalid battery governor thresholds')
    if not s['minimum_physical_xy_separation_m'] < s['measured_abort_xy_separation_m'] < pair_distance(cfg):
        raise ValueError('Invalid physical/abort/virtual separation ordering')
    if not c['dt_s'] < c['cycle_deadline_s'] < s['watchdog_timeout_s']:
        raise ValueError('Cycle deadline must precede watchdog timeout')
    if set(m['offsets_xyz']) != set(cfg['names']): raise ValueError('One offset per active name required')
    if len(m['height_proposal_offsets_m']) != len(cfg['names']) or max(abs(float(v)) for v in m['height_proposal_offsets_m']) > 0.03:
        raise ValueError('Active height proposals within 3 cm required')
    handover = cfg.get('commissioning', {})
    for key in ('handover_duration_s', 'settle_timeout_s', 'settle_dwell_s'):
        if not isinstance(handover.get(key), (int, float)) or isinstance(handover.get(key), bool) or not math.isfinite(handover[key]) or handover[key] <= 0:
            raise ValueError('Positive finite commissioning.'+key+' required')
    if handover['handover_duration_s'] < 5. or handover['settle_dwell_s'] < 2.:
        raise ValueError('Commissioning requires >=5s bridge and >=2s settled hover')
    if cfg.get('command_interface') != 'position':
        raise ValueError('This stage commissions the streamed position interface only')
    q = slots(cfg)
    if q.shape != (len(cfg['names']), 3) or not np.isfinite(q).all(): raise ValueError('Invalid slots')
    lo, hi = virtual_box(cfg)
    if np.any(lo >= hi): raise ValueError('Empty usable airborne volume')
    for displacement in (0, m['distance_m']):
        endpoints = q+np.array([displacement, 0, 0])
        if np.any(endpoints <= lo) or np.any(endpoints >= hi):
            raise ValueError('The configured formation sweep does not fit the inflated arena')
        for i in range(len(cfg['names'])):
            for j in range(i):
                if np.linalg.norm(endpoints[i, :2]-endpoints[j, :2]) <= pair_distance(cfg):
                    raise ValueError('Formation spacing is insufficient for the complete tracking tube')
    for obstacle in cfg['obstacles']:
        if len(obstacle['center_xy']) != 2 or obstacle['radius_m'] <= 0 or obstacle['surface_clearance_m'] < 0.5:
            raise ValueError('Obstacle needs measured XY, positive radius and >= 0.5 m surface clearance')
        extra = (cfg['arena']['body_radius_m']+s['tracking_tube_m']+
                 s['mocap_position_uncertainty_m']+s['wall_reserve_m'])
        radius = obstacle['radius_m']+obstacle['surface_clearance_m']+extra
        for displacement in (0., m['distance_m']):
            endpoints = q+np.array([displacement, 0., 0.])
            if np.min(np.linalg.norm(endpoints[:, :2]-obstacle['center_xy'], axis=1)) <= radius:
                raise ValueError('An initial or final formation slot lies inside an inflated obstacle')

"""Stationary XY cylinders. Geometry is common to QP, preview and live guards.

center_xy is an immutable surveyed world center, not updated from live packets.
Live poses validate that assumption. No moving-obstacle or vertical bypass model.
"""
import math
import numpy as np


def obstacle_radius(obstacle, cfg, virtual=True):
    a, s = cfg['arena'], cfg['safety']
    radius = (obstacle['radius_m'] + obstacle['surface_clearance_m'] +
              a['body_radius_m'] + s['mocap_position_uncertainty_m'] +
              obstacle['position_uncertainty_m'])
    return radius + s['tracking_tube_m'] + s['wall_reserve_m'] if virtual else radius


def validate_obstacles(cfg):
    if not isinstance(cfg.get('obstacles'), list):
        raise ValueError('obstacles must be a list')
    names = set()
    required = {'name', 'center_xy', 'radius_m', 'surface_clearance_m',
                'position_uncertainty_m', 'center_tolerance_m', 'geometry_verified',
                'pivot_height_m', 'height_tolerance_m', 'maximum_tilt_rad'}
    for o in cfg['obstacles']:
        if not isinstance(o, dict) or set(o) != required:
            raise ValueError('Static obstacle fields must be exactly: '+', '.join(sorted(required)))
        if not isinstance(o['name'], str) or not o['name'] or o['name'] in names or o['name'] in cfg['names']:
            raise ValueError('Obstacle mocap names must be unique and separate from aircraft')
        names.add(o['name'])
        center = np.asarray(o['center_xy'], float)
        if center.shape != (2,) or not np.isfinite(center).all():
            raise ValueError('Obstacle center_xy must be two finite world coordinates')
        for key in ('radius_m', 'surface_clearance_m', 'position_uncertainty_m', 'center_tolerance_m', 'pivot_height_m', 'height_tolerance_m', 'maximum_tilt_rad'):
            v = o[key]
            if isinstance(v, bool) or not isinstance(v, (int,float)) or not math.isfinite(v) or v <= 0:
                raise ValueError('Invalid obstacle '+key)
        if type(o['geometry_verified']) is not bool:
            raise ValueError('geometry_verified must be a boolean')
        if o['position_uncertainty_m'] < o['center_tolerance_m'] + cfg['safety']['mocap_position_uncertainty_m']:
            raise ValueError('Obstacle uncertainty must cover center tolerance plus mocap uncertainty')
        for axis,key in enumerate(('x_limits','y_limits')):
            if not cfg['arena'][key][0] < center[axis] < cfg['arena'][key][1]:
                raise ValueError('Obstacle center outside arena')


def require_verified_geometry(cfg):
    for o in cfg['obstacles']:
        if not o['geometry_verified']:
            raise RuntimeError(o['name']+': measure the complete footprint including tripod feet; geometry_verified is false')


def check_reference_clearance(q, cfg):
    for o in cfg.get('obstacles', []):
        distances = np.linalg.norm(np.asarray(q)[:,:2] - o['center_xy'], axis=1)
        if np.any(distances <= obstacle_radius(o,cfg)):
            name = cfg['names'][int(np.argmin(distances))]
            raise ValueError(name+': reference start/goal inside obstacle buffer '+o['name'])


def check_measured_clearance(p, cfg, centers=None):
    evidence = {}
    for o in cfg.get('obstacles', []):
        center = o['center_xy'] if centers is None else centers[o['name']]
        distances = np.linalg.norm(np.asarray(p)[:,:2] - center, axis=1)
        limit = obstacle_radius(o, cfg, virtual=False)
        if np.any(distances <= limit):
            name = cfg['names'][int(np.argmin(distances))]
            raise RuntimeError(name+': measured obstacle clearance violated: '+o['name'])
        evidence[o['name']] = dict(center_xy=list(center), measured_abort_center_distance_m=limit,
            required_reference_center_distance_m=obstacle_radius(o,cfg),
            center_distances_m=dict(zip(cfg['names'], map(float,distances))))
    return evidence


def check_live_obstacles(histories, p, cfg, ros_now, mono):
    centers, tracking = {}, {}
    for o in cfg.get('obstacles', []):
        hist = histories[o['name']]
        row = hist.latest(ros_now, mono, cfg['safety']['mocap_maximum_age_s'])
        rate, coverage = hist.stats()
        if rate < cfg['safety']['minimum_mocap_rate_hz'] or coverage < .4:
            raise RuntimeError(o['name']+': insufficient obstacle pose rate/history')
        from .health import tilt
        if abs(row[2][2]-o['pivot_height_m']) > o['height_tolerance_m'] or tilt(row[3]) > o['maximum_tilt_rad']:
            raise RuntimeError(o['name']+': obstacle height/tilt changed; static upright model invalid')
        error = float(np.linalg.norm(row[2][:2] - o['center_xy']))
        if error > o['center_tolerance_m']:
            raise RuntimeError(o['name']+f': surveyed center mismatch/movement {error:.4f} m')
        # Detect motion within the recent accepted history, even if the current
        # sample has returned to the surveyed center. Radius encloses all yaw.
        recent = [r[2][:2] for r in hist.rows if mono-r[1] <= .5]
        if recent and np.max(np.linalg.norm(np.asarray(recent)-o['center_xy'],axis=1)) > o['center_tolerance_m']:
            raise RuntimeError(o['name']+': obstacle moved within the last 0.5 s')
        centers[o['name']] = row[2][:2].tolist()
        tracking[o['name']] = dict(center_error_m=error, source_age_s=ros_now-row[0],
                                  receipt_age_s=mono-row[1], accepted_rate_hz=rate)
    evidence = check_measured_clearance(p, cfg, centers)
    for name in evidence:evidence[name].update(tracking[name])
    return evidence

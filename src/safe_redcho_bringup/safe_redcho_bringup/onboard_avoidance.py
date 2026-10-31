"""Firmware BVCA configuration, separate from the future HOCBF-QP controller."""


def firmware_parameters(config):
    a, m, ca = config['arena'], config['mission'], config['collision_avoidance']
    radius = m['minimum_separation_m']/2+ca['horizontal_extra_per_robot_m']
    margin = a['wall_clearance_m']+a['body_radius_m']
    # Firmware boxes constrain centres, so shrink XY by the physical body allowance.
    # The lower Z permits ground start and landing; use the ceiling tracking limit.
    return {'enable': 1, 'ellipsoidX': float(radius), 'ellipsoidY': float(radius),
            'ellipsoidZ': float(ca['vertical_radius_m']),
            'bboxMinX': float(a['x_limits'][0]+margin),
            'bboxMaxX': float(a['x_limits'][1]-margin),
            'bboxMinY': float(a['y_limits'][0]+margin),
            'bboxMaxY': float(a['y_limits'][1]-margin),
            'bboxMinZ': float(a['floor_z_m']-.03),
            'bboxMaxZ': float(a['ceiling_z_m']-a['ceiling_clearance_m']),
            'horizon': float(ca['horizon_s']), 'maxSpeed': float(ca['maximum_speed_mps']),
            'maxPeerLocAge': int(ca['maximum_peer_age_ms'])}

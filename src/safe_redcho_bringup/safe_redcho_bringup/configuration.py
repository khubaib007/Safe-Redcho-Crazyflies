# Release preparation: support checkout-local editable config directory.
"""Generate local upstream configuration without editing third-party sources."""
from copy import deepcopy
from pathlib import Path
import ipaddress
import math
import os
import re
import yaml


def default_config():
    return str(Path(os.environ.get('SAFE_REDCHO_HW_ROOT',
                                   '~/safe_redcho_hardware')).expanduser()
               / 'config/lab_config.yaml')


def load_lab(path=None, require_radio=False):
    path = Path(path or default_config()).expanduser().resolve()
    with path.open() as f:
        lab = yaml.safe_load(f)
    for key in ('motive_ip', 'ubuntu_ip', 'multicast_group'):
        ipaddress.IPv4Address(lab['network'][key])
    if not ipaddress.ip_address(lab['network']['multicast_group']).is_multicast:
        raise ValueError('network.multicast_group must be a multicast IP')
    # Current upstream NatNet client uses the default transport parameters.
    if (lab['network']['multicast_group'], lab['network']['command_port'],
            lab['network']['data_port']) != ('239.255.42.99', 1510, 1511):
        raise ValueError('This starter requires NatNet default multicast group/ports')
    robot = lab['robot']
    if not re.fullmatch(r'cf[1-9][0-9]*', robot['name']):
        raise ValueError('Use a name such as cf1; it must match the Motive rigid body')
    if robot['rigid_body_name'] != robot['name']:
        raise ValueError('robot.name and robot.rigid_body_name must match exactly')
    p = robot['initial_position']
    if len(p) != 3 or not all(math.isfinite(float(v)) for v in p):
        raise ValueError('initial_position must contain three finite coordinates in metres')
    if require_radio and not re.fullmatch(
            r'radio://[0-9]+/[0-9]+/(?:250K|1M|2M)/[0-9A-Fa-f]{10}', robot['uri']):
        raise ValueError('Set robot.uri to the actual complete Crazyflie radio URI')
    if lab['mocap']['backend'] not in ('optitrack', 'optitrack_closed_source'):
        raise ValueError('Unsupported OptiTrack backend')
    flight = lab['flight']
    for key, value in flight.items():
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'flight.{key} must be a finite positive number')
    if not 0.3 <= flight['height_above_start_m'] <= 0.8:
        raise ValueError('First-hover height must be between 0.3 and 0.8 m above start')
    if flight['hover_duration_s'] > 15 or min(
            flight['takeoff_duration_s'], flight['landing_duration_s']) < 2:
        raise ValueError('First-hover durations outside commissioning limits')
    return lab, path


def prepare(path=None, require_radio=False):
    lab, path = load_lab(path, require_radio)
    root = path.parents[2] if path.parent.name == "local" and path.parent.parent.name == "config" else path.parent.parent
    upstream = root / 'src/crazyswarm2/crazyflie/config'
    with (upstream / 'motion_capture.yaml').open() as f:
        mocap = yaml.safe_load(f)
    params = mocap['/motion_capture_tracking']['ros__parameters']
    params['type'] = lab['mocap']['backend']
    params['hostname'] = lab['network']['motive_ip']
    params.pop('rigid_bodies', None)  # Vendor solves all enabled Motive rigid bodies.
    params.setdefault('topics', {})['frame_id'] = lab['mocap']['frame_id']
    topics = params['topics']
    topics.setdefault('poses', {})['qos'] = {
        'mode': 'sensor', 'deadline': float(lab['mocap']['expected_rate_hz'])}
    topics.setdefault('tf', {})['child_frame_id'] = '{}_mocap'
    # interface_ip is not exposed in every upstream release. Use it only when
    # declared by the checked-out node; route selection also works on older versions.
    mocap_source = root / 'src/motion_capture_tracking/motion_capture_tracking/src'
    exposes_interface = any('"interface_ip"' in p.read_text(errors='replace')
                            for p in mocap_source.glob('*.cpp'))
    if exposes_interface:
        params['interface_ip'] = lab['network']['ubuntu_ip']
    else:
        params.pop('interface_ip', None)
    with (upstream / 'crazyflies.yaml').open() as f:
        robots = yaml.safe_load(f)
    robot = lab['robot']
    robot_type = deepcopy(robots['robot_types']['cf21'])
    robot_type['motion_capture']['tracking'] = 'vendor'
    robots['robot_types'] = {'cf21': robot_type}
    robots['robots'] = {robot['name']: {
        'enabled': True, 'uri': robot['uri'],
        'initial_position': robot['initial_position'], 'type': 'cf21'}}
    all_cfg = robots.setdefault('all', {})
    firmware = all_cfg.setdefault('firmware_params', {})
    firmware.setdefault('commander', {})['enHighLevel'] = 1
    firmware.setdefault('stabilizer', {}).update({
        'estimator': robot['estimator'], 'controller': robot['controller']})
    all_cfg['reference_frame'] = lab['mocap']['frame_id']
    all_cfg['firmware_logging'] = {'enabled': True, 'default_topics': {
        'pose': {'frequency': int(robot['pose_log_rate_hz'])},
        'status': {'frequency': 1}}}
    generated = root / 'config/generated'
    generated.mkdir(exist_ok=True)
    for name, obj in [('motion_capture.yaml', mocap), ('crazyflies.yaml', robots)]:
        (generated / name).write_text(yaml.safe_dump(obj, sort_keys=False))
    return lab, generated

"""Six-aircraft configuration, separate from the original one-aircraft settings."""
from datetime import datetime, timezone
from pathlib import Path
import math
import re
import yaml
from .configuration import default_config, load_lab, prepare


def default_fleet_config():
    return str(Path(default_config()).with_name('fleet_config.yaml'))


def load_fleet(path=None, require_starts=False, require_arena=False):
    path = Path(path or default_fleet_config()).expanduser().resolve()
    lab, _ = load_lab(path.with_name('lab_config.yaml'))
    config = yaml.safe_load(path.read_text())
    names = list(config['robots'])
    if set(names) != {f'cf{i}' for i in range(1, 7)} or len(names) != 6:
        raise ValueError('This commissioning mission requires exactly cf1 through cf6')
    uris = [config['robots'][name]['uri'] for name in names]
    if len(set(uris)) != 6:
        raise ValueError('Each Crazyflie needs a unique radio URI')
    radio_channels = {}
    addresses = []

    for name, uri in zip(names, uris):
        match = re.fullmatch(
            r'radio://([012])/(\d{1,3})/2M/([0-9A-Fa-f]{10})',
            uri,
        )
        if not match or not 0 <= int(match[2]) <= 125:
            raise ValueError(
                f'{name}: use radio index 0, 1 or 2, '
                'channel 0–125, 2M and a full 10-digit hexadecimal address'
            )

        radio_index = int(match[1])
        channel = int(match[2])
        addresses.append(match[3].lower())

        previous_channel = radio_channels.setdefault(radio_index, channel)
        if previous_channel != channel:
            raise ValueError(
                f'Radio {radio_index} has multiple channels: '
                f'{previous_channel} and {channel}. '
                'Configure one channel per dongle.'
            )

    if len(set(addresses)) != len(names):
        raise ValueError('Radio addresses must remain unique across all aircraft')

    radios = sorted(radio_channels)
    for i, first in enumerate(radios):
        for second in radios[i + 1:]:
            if abs(radio_channels[first] - radio_channels[second]) < 2:
                raise ValueError(
                    f'Radios {first} and {second}: '
                    '2M channels must be separated by at least 2'
                )
    if set(config['swap']['cycle']) != set(names) or len(config['swap']['cycle']) != 6:
        raise ValueError('swap.cycle must contain all six names exactly once')
    m = config['mission']
    for key, value in m.items():
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'mission.{key} must be finite and positive')
    required = m['minimum_separation_m']+2*m['tracking_tolerance_m']+m['planning_reserve_m']
    if not m['minimum_separation_m'] < m['abort_separation_m'] < required:
        raise ValueError('Require minimum separation < abort separation < planning separation')
    ca = config['collision_avoidance']
    if ca.get('enabled') is not True:
        raise ValueError('This stage requires onboard BVCA to remain enabled')
    for key in ('horizontal_extra_per_robot_m', 'vertical_radius_m', 'horizon_s',
                'maximum_speed_mps', 'maximum_peer_age_ms'):
        value = ca[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'collision_avoidance.{key} must be positive and finite')
    if ca['maximum_speed_mps'] < m['maximum_speed_mps']:
        raise ValueError('Firmware maximum speed must cover the planned reference maximum speed')
    starts_path = path.parent/config['starts_file']
    config['_path'] = str(path)
    config['_lab'] = lab
    config['_starts_path'] = str(starts_path)
    if require_starts:
        snapshot = yaml.safe_load(starts_path.read_text())
        if snapshot.get('frame_id') != lab['mocap']['frame_id']:
            raise ValueError('Captured starts must use the configured mocap world frame')
        if set(snapshot['positions']) != set(names):
            raise ValueError('Captured starts do not contain exactly the configured six names')
        for p in snapshot['positions'].values():
            if len(p) != 3 or not all(math.isfinite(float(v)) for v in p):
                raise ValueError('Captured starts must be finite 3D positions in metres')
        config['_starts'] = snapshot['positions']
        config['_snapshot'] = snapshot
    if require_arena:
        a = config['arena']
        for key in ('x_limits', 'y_limits'):
            value = a[key]
            if (not isinstance(value, list) or len(value) != 2 or
                not all(isinstance(v, (float, int)) and math.isfinite(v) for v in value)
                or value[0] >= value[1]):
                raise ValueError(f'Set arena.{key} to your measured world bounds before flight')
        for key in ('floor_z_m', 'ceiling_z_m', 'wall_clearance_m',
                    'ceiling_clearance_m', 'body_radius_m'):
            if not isinstance(a[key], (float, int)) or not math.isfinite(a[key]):
                raise ValueError(f'Set arena.{key} to a finite measured value')
        if a['ceiling_z_m'] <= a['floor_z_m']:
            raise ValueError('Arena ceiling must be above floor')
        if min(a['wall_clearance_m'], a['ceiling_clearance_m'], a['body_radius_m']) <= 0:
            raise ValueError('Arena clearance and body-radius allowances must be positive')
        for obstacle in config['obstacles']:
            if (len(obstacle['center_xy']) != 2 or
                not all(math.isfinite(float(v)) for v in obstacle['center_xy']) or
                not math.isfinite(float(obstacle['radius_m'])) or obstacle['radius_m'] <= 0):
                raise ValueError('Obstacle needs finite center_xy and positive radius_m')
    return config


def prepare_fleet(path=None):
    config = load_fleet(path, require_starts=True, require_arena=True)
    source = Path(config['_path']).with_name('lab_config.yaml')
    _, generated = prepare(source, require_radio=False)
    outfile = generated/'crazyflies.yaml'
    robots = yaml.safe_load(outfile.read_text())
    robots['robots'] = {name: {'enabled': True, 'type': 'cf21', 'uri': info['uri'],
                              'initial_position': config['_starts'][name]}
                        for name, info in config['robots'].items()}
    robots['all']['firmware_logging']['default_topics']['pose']['frequency'] = int(
        config['mission']['pose_log_rate_hz'])
    from .onboard_avoidance import firmware_parameters
    # Official BVCA documentation confirms PID, and warns about Mellinger.
    robots['all']['firmware_params']['stabilizer'].update({'controller': 1, 'estimator': 2})
    robots['all']['firmware_params']['colAv'] = firmware_parameters(config)
    outfile = generated/'six_crazyflies.yaml'
    outfile.write_text(yaml.safe_dump(robots, sort_keys=False))
    return config, generated, outfile

"""Validate the original fleet, then expose ONLY cf2/cf3/cf4 to this package."""
from copy import deepcopy
from pathlib import Path
import math
import yaml

ACTIVE = ('cf2', 'cf3', 'cf4')
INACTIVE = ('cf1', 'cf5', 'cf6')


def require_active(names):
    if tuple(names) != ACTIVE:
        raise ValueError('This commissioning package only permits ordered cf2, cf3, cf4')


def load_fleet(profile, require_starts=False, require_arena=False):
    from safe_redcho_bringup.fleet_configuration import load_fleet as original_load
    require_active(profile['names'])
    # Preserve the known two-radio URI/calibration validator and original files.
    cfg = original_load(profile['fleet_config_file'], require_starts=False,
                        require_arena=require_arena)
    cfg = deepcopy(cfg)
    cfg['robots'] = {n: cfg['robots'][n] for n in ACTIVE}
    path = Path(profile['three_starts_file']).expanduser().resolve()
    cfg['_starts_path'] = str(path)
    if require_starts:
        snapshot = yaml.safe_load(path.read_text())
        if snapshot.get('frame_id') != profile['frame_id'] or set(snapshot.get('positions', {})) != set(ACTIVE):
            raise ValueError('Recapture the three grounded active aircraft in the world frame')
        for n, p in snapshot['positions'].items():
            if len(p) != 3 or not all(math.isfinite(float(x)) for x in p):
                raise ValueError('Invalid captured start for '+n)
        cfg['_starts'], cfg['_snapshot'] = snapshot['positions'], snapshot
    return cfg


def restrict_backend(robots, profile):
    require_active(profile['names'])
    fleet = load_fleet(profile, require_starts=True, require_arena=True)
    result = deepcopy(robots)
    result['robots'] = {n: deepcopy(robots['robots'][n]) for n in ACTIVE}
    for n in ACTIVE:
        result['robots'][n]['enabled'] = True
        result['robots'][n]['initial_position'] = fleet['_starts'][n]
    # Absence from this server prevents it creating command services for inactive IDs.
    return result

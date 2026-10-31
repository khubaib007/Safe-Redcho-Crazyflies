import hashlib
import json
import math
from pathlib import Path
import yaml

GROUPS = ('flight', 'battery', 'timing', 'reference', 'velocity_redcho',
          'position_redcho', 'control', 'hocbf')
NAMES = ['cf2', 'cf3', 'cf4']


def fingerprint(profile):
    return hashlib.sha256(json.dumps({k: v for k, v in profile.items() if not k.startswith('_')},
        sort_keys=True, allow_nan=False).encode()).hexdigest()


def default_config():
    from ament_index_python.packages import get_package_share_directory
    return str(Path(get_package_share_directory('safe_redcho_three_mission'))/'config/liveplots.yaml')


def load(path, profile_override=None):
    cfg = yaml.safe_load(Path(path).expanduser().read_text())
    if profile_override: cfg['profile_file'] = profile_override
    p = yaml.safe_load(Path(cfg['profile_file']).expanduser().read_text())
    if p.get('names') != NAMES or p.get('command_interface') != 'position':
        raise ValueError('Expected cf2/cf3/cf4 and the Position command interface')
    cfg['names'], cfg['profile'], cfg['config_hash'] = NAMES.copy(), p, fingerprint(p)
    cfg['profile_file'] = str(Path(cfg['profile_file']).expanduser())
    for key in ('refresh_hz', 'history_s', 'command_max_age_s', 'alignment_max_skew_s'):
        v = cfg[key]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
            raise ValueError('Invalid '+key)
    if not 1 <= cfg['refresh_hz'] <= 10: raise ValueError('Use 1–10 Hz drawing/transport refresh')
    if not 100 <= cfg['max_points_per_series'] <= 50000: raise ValueError('Invalid history bound')
    if not isinstance(cfg['record_events'], bool): raise ValueError('record_events must be boolean')
    cfg['onboard_blocks'] = {}
    # Read mappings only. Never call the telemetry package's logging-enablement code.
    tpath = Path(cfg['telemetry_config_file']).expanduser()
    if tpath.is_file():
        telemetry = yaml.safe_load(tpath.read_text()) or {}
        if telemetry.get('onboard_config_file'):
            onboard_path = Path(telemetry['onboard_config_file']).expanduser()
            if onboard_path.is_file():
                onboard = yaml.safe_load(onboard_path.read_text()) or {}
                if onboard.get('enabled'):
                    cfg['onboard_blocks'] = {k: v for k, v in onboard.get('blocks', {}).items() if v.get('enabled')}
    return cfg


def limits(cfg):
    p = cfg['profile']; a, s, c, b, m = [p[k] for k in ('arena', 'safety', 'control', 'battery', 'mission')]
    slots = {n: [m['center_start_xy'][0]+m['offsets_xyz'][n][0],
                 m['center_start_xy'][1]+m['offsets_xyz'][n][1],
                 a['floor_z_m']+m['hover_height_above_floor_m']+m['offsets_xyz'][n][2]] for n in NAMES}
    return dict(arena=a, safety=s, control=c, battery=b, slots=slots,
                geometry_note='Reference and measured clearances are different quantities')

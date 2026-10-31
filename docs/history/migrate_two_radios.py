#!/usr/bin/env python3
"""Apply the requested stage-3 changes, preserving a backup and lab calibration."""
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
import shutil
import yaml

root = Path(__file__).resolve().parents[1]
path = root/'config/fleet_config.yaml'
config = yaml.safe_load(path.read_text())
backup = path.with_name('fleet_config.before_stage3_'+datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%f')+'.yaml')
shutil.copy2(path, backup)
for i in range(1, 7):
    radio, channel = (0, 80) if i <= 3 else (1, 60)
    config['robots'][f'cf{i}']['uri'] = f'radio://{radio}/{channel}/2M/E7E7E7E7{i:02X}'
for key in ('x_limits', 'y_limits'):
    old = config['arena'].get(key)
    centre = (old[0]+old[1])/2 if isinstance(old, list) and len(old) == 2 else 0.
    config['arena'][key] = [centre-1.5, centre+1.5]
config['mission']['minimum_separation_m'] = .5
config['mission']['abort_separation_m'] = .65
config['collision_avoidance'] = {
    'enabled': True, 'horizontal_extra_per_robot_m': .05,
    'vertical_radius_m': .40, 'horizon_s': 1.0,
    'maximum_speed_mps': max(.25, config['mission']['maximum_speed_mps']),
    'maximum_peer_age_ms': 150}
path.write_text(yaml.safe_dump(config, sort_keys=False))
print('Updated:', path)
print('Backup:', backup)
print('Preserved lab_config.yaml, captured starts, floor/ceiling calibration and obstacle list.')
print('Verify XY world bounds: a missing previous origin was assumed to be arena centre (0,0).')
print('Set cf4/cf5/cf6 actual aircraft radio channel to 60 before connecting.')

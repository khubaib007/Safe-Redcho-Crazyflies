"""Read telemetry settings and the algorithm profile; never write control settings."""
import hashlib
import json
import math
from pathlib import Path
import yaml
from .onboard import enabled_blocks

NAMES = ['cf2', 'cf3', 'cf4']


def profile_hash(profile):
    return hashlib.sha256(json.dumps({k:v for k,v in profile.items() if not k.startswith('_')},
        sort_keys=True,allow_nan=False).encode()).hexdigest()


def validate(cfg):
    if cfg['names'] != NAMES: raise ValueError('This package records cf2, cf3, cf4 only')
    for section in ('flight','reference','redcho','control'):
        options=cfg['nodes'][section]
        for k in ('record_enabled','terminal_enabled'):
            if not isinstance(options[k],bool): raise ValueError(section+'.'+k+' must be boolean')
        period=options['terminal_period_s']
        if isinstance(period,bool) or not isinstance(period,(int,float)) or not math.isfinite(period) or period<.2:
            raise ValueError('Terminal period must be finite and >=0.2 seconds')
        if any(not isinstance(v,bool) for v in options['variables'].values()): raise ValueError('Variable switches must be boolean')
        for k in ('include_variables','exclude_variables','terminal_variables'):
            if not isinstance(options[k],list) or not options[k] or any(not isinstance(x,str) for x in options[k]):
                raise ValueError(section+'.'+k+' requires a nonempty list of patterns')
    for value in (cfg['writer']['queue_batches'],cfg['subscription_depth']):
        if isinstance(value,bool) or not isinstance(value,int) or value<1: raise ValueError('Invalid queue/depth')
    if cfg['writer']['layout'] not in ('wide','long'): raise ValueError('Invalid CSV layout')
    if not math.isfinite(cfg['writer']['flush_s']) or cfg['writer']['flush_s']<=0: raise ValueError('Invalid flush interval')
    for key in ('mocap_maximum_source_age_s','mocap_maximum_source_gap_s'):
        value=cfg[key]
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('Positive finite '+key+' required')
    for bank,thresholds in cfg['convergence_thresholds'].items():
        for key,value in thresholds.items():
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
                raise ValueError('Invalid convergence threshold '+bank+'.'+key)
    enabled_blocks(cfg['onboard'])
    return cfg


def load_config(path):
    cfg=yaml.safe_load(Path(path).expanduser().read_text())
    cfg['onboard']=yaml.safe_load(Path(cfg['onboard_config_file']).expanduser().read_text())
    profile=yaml.safe_load(Path(cfg['algorithm_profile_file']).expanduser().read_text())
    if profile.get('names')!=NAMES: raise ValueError('Algorithm profile is not cf2/cf3/cf4')
    cfg['expected_config_hash']=profile_hash(profile)
    if not cfg['onboard'].get('enabled',False):
        for block in cfg['onboard'].get('blocks',{}).values(): block['enabled']=False
    return validate(cfg)

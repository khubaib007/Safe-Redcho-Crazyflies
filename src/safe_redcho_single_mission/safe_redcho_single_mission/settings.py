"""Strict validation for the new profile; no legacy hardcoded speed ceiling."""
from pathlib import Path
import math
import numpy as np
import yaml

NAMES = ['cf2', 'cf3', 'cf4']

def path_of(cfg, key):
    p = Path(cfg[key]).expanduser()
    return p if p.is_absolute() else Path(cfg['workspace']).expanduser()/p

def load(path):
    p=Path(path).expanduser().resolve(); c=yaml.safe_load(p.read_text()); validate(c)
    c['_path']=str(p); return c

def pair_distance(c):
    s=c['safety']; return s['minimum_physical_xy_separation_m']+2*s['tracking_tube_m']+2*s['mocap_position_uncertainty_m']+s['pair_reserve_m']

def virtual_box(c):
    a,s=c['arena'],c['safety']; e=a['body_radius_m']+a['wall_surface_clearance_m']+s['tracking_tube_m']+s['mocap_position_uncertainty_m']+s['wall_reserve_m']
    return (np.array([a['x_limits'][0]+e,a['y_limits'][0]+e,a['floor_z_m']+s['airborne_minimum_height_above_floor_m']]),
            np.array([a['x_limits'][1]-e,a['y_limits'][1]-e,a['ceiling_z_m']-a['ceiling_clearance_m']-s['tracking_tube_m']-s['mocap_position_uncertainty_m']-s['wall_reserve_m']]))

def slots(c):
    m=c['mission']; base=np.r_[m['center_start_xy'],c['arena']['floor_z_m']+m['hover_height_above_floor_m']]
    return np.array([base+np.asarray(m['offsets_xyz'][n]) for n in NAMES])

def check_volume(q,c):
    q=np.asarray(q,float); lo,hi=virtual_box(c)
    if q.shape!=(3,3) or not np.isfinite(q).all() or np.any(q<=lo) or np.any(q>=hi): raise ValueError('Reference positions outside inflated airborne arena')
    from .obstacles import check_reference_clearance
    check_reference_clearance(q,c)
    for i in range(3):
        for j in range(i):
            d=np.linalg.norm(q[i,:2]-q[j,:2])
            if d<pair_distance(c): raise ValueError(f'{NAMES[j]} / {NAMES[i]} XY spacing {d:.3f} m requires >= {pair_distance(c):.3f} m')

def validate(c):
    def finite(x):
        if isinstance(x,dict):
            for v in x.values(): finite(v)
        elif isinstance(x,list):
            for v in x: finite(v)
        elif isinstance(x,(int,float)) and not math.isfinite(x): raise ValueError('Nonfinite configuration')
    finite(c)
    from .avoidance import DEFAULTS, options
    if 'avoidance' in c:
        if not isinstance(c['avoidance'],dict) or set(c['avoidance'])-set(DEFAULTS):
            raise ValueError('Invalid avoidance settings')
        av=options(c)
        boolean_keys=('enabled','predictive_barriers','adaptive_pair_responsibility','route_guidance')
        for key in boolean_keys:
            if type(av[key]) is not bool:raise ValueError('avoidance.'+key+' must be boolean')
        for key,value in av.items():
            if key not in boolean_keys and (isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0):
                raise ValueError('avoidance.'+key+' must be positive finite numeric')
        if type(av['route_circle_samples']) is not int or not 8<=av['route_circle_samples']<=48:
            raise ValueError('avoidance.route_circle_samples must be an integer in [8,48]')
        if type(av['route_max_nodes']) is not int or not 4<=av['route_max_nodes']<=256:
            raise ValueError('avoidance.route_max_nodes must be an integer in [4,256]')
        if not 0<av['minimum_progress_scale']<=av['active_progress_scale']<=1:
            raise ValueError('Invalid avoidance progress scales')
        if not 0<av['orbit_margin_m']<av['activation_margin_m']<av['release_margin_m']:
            raise ValueError('Require orbit < activation < release avoidance margins')
        if not av['lag_slowdown_m']<av['lag_stop_m']:
            raise ValueError('Invalid avoidance lag thresholds')
        if av['passing_speed_mps']>c['control']['maximum_reference_speed_mps']:
            raise ValueError('Passing speed must fit reference speed bounds')
    if c.get('architecture') != 'single_position_redcho':
        raise ValueError('Require the explicit single_position_redcho architecture')
    if 'velocity_L' in c['redcho']:
        raise ValueError('Single-position baseline has no velocity bank or velocity_L')
    if c['names']!=NAMES or c['frame_id']!='world' or c['command_interface']!='position' or c['namespace']!='/safe_redcho_single_mission': raise ValueError('Require cf2/cf3/cf4, world, position and isolated /safe_redcho_single_mission namespace')
    nonnumeric={'control':{'allow_slew_relaxation_for_safety'},'safety':set(),'battery':set(),'redcho':{'gamma','gains'},'mission':{'center_start_xy','offsets_xyz'},'acquisition':{'start_x_limits','start_y_limits','takeoff_heights_above_floor_m','assignment'}}
    for section in ('control','redcho','safety','battery','mission','acquisition'):
        for k,v in c[section].items():
            if k not in nonnumeric[section] and (isinstance(v,bool) or not isinstance(v,(int,float)) or v<=0): raise ValueError(f'{section}.{k} must be positive finite numeric')
    for section,keys in [('arena',('x_limits','y_limits')),('acquisition',('start_x_limits','start_y_limits'))]:
        for k in keys:
            v=c[section][k]
            if len(v)!=2 or not v[0]<v[1]: raise ValueError('Invalid '+k)
    from .obstacles import validate_obstacles
    validate_obstacles(c)
    a,m,s,b,ctrl=c['acquisition'],c['mission'],c['safety'],c['battery'],c['control']
    for key in ('body_radius_m','wall_surface_clearance_m','ceiling_clearance_m'):
        value=c['arena'][key]
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('arena.'+key+' must be positive finite numeric')
    for key in ('pose_frequency_hz','status_frequency_hz','bvca_horizontal_radius_m','bvca_vertical_radius_m','bvca_horizon_s'):
        value=c['backend'][key]
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('backend.'+key+' must be positive finite numeric')
    if type(c['backend']['bvca_enabled']) is not bool:
        raise ValueError('backend.bvca_enabled must be boolean')
    if type(c.get('backend',{}).get('state_receiver_thread',False)) is not bool:
        raise ValueError('backend.state_receiver_thread must be a boolean')
    if type(ctrl.get('allow_slew_relaxation_for_safety',False)) is not bool:
        raise ValueError('control.allow_slew_relaxation_for_safety must be a boolean')
    if ctrl.get('allow_slew_relaxation_for_safety',False):
        if 'maximum_safety_acceleration_slew_mps3' not in ctrl or ctrl['maximum_safety_acceleration_slew_mps3']<ctrl['maximum_acceleration_slew_mps3']:
            raise ValueError('Set maximum_safety_acceleration_slew_mps3 >= maximum_acceleration_slew_mps3 for safety-priority mode')
    for k in ('gamma','gains'):
        if len(c['redcho'][k])!=3 or min(c['redcho'][k])<=0: raise ValueError('Invalid REDCHO '+k)
    if not .005<=ctrl['dt_s']<=.025 or not ctrl['dt_s']<ctrl['cycle_deadline_s']<s['watchdog_timeout_s']: raise ValueError('Invalid period/deadline/watchdog ordering')
    if ctrl['maximum_publish_gap_s']>=s['watchdog_timeout_s']: raise ValueError('Publish gap must precede watchdog')
    if not isinstance(ctrl['qp_max_iterations'],int) or ctrl['qp_max_iterations']<1: raise ValueError('QP iterations must be an integer')
    if np.sqrt(3)*ctrl['maximum_reference_speed_mps']>=s['measured_maximum_speed_mps']: raise ValueError('Measured speed guard must cover the componentwise reference speed box')
    if not s['minimum_physical_xy_separation_m']<s['measured_abort_xy_separation_m']<pair_distance(c): raise ValueError('Invalid separation ordering')
    if m['endpoint_tolerance_m']>=s['tracking_tube_m']: raise ValueError('Endpoint tolerance must fit tracking tube')
    if not b['land_voltage_v']<b['full_speed_voltage_v']<=b['minimum_ground_voltage_v'] or not 0<b['minimum_speed_scale']<=1: raise ValueError('Invalid battery thresholds')
    if set(a['takeoff_heights_above_floor_m'])!=set(NAMES) or a['assignment'] not in ('sorted_y','fixed_names'): raise ValueError('Invalid takeoff heights / assignment')
    if set(m['offsets_xyz'])!=set(NAMES): raise ValueError('Require three formation offsets')
    offsets=np.array([m['offsets_xyz'][n] for n in NAMES],float)
    if offsets.shape!=(3,3) or np.max(np.abs(offsets[:,2]))>1e-12 or np.max(np.abs(offsets.mean(axis=0)))>1e-10: raise ValueError('Formation offsets must be centered, with zero vertical offsets')
    lo,hi=virtual_box(c)
    if np.any(lo>=hi): raise ValueError('Empty inflated arena')
    for h in a['takeoff_heights_above_floor_m'].values():
        z=c['arena']['floor_z_m']+h
        if not lo[2]<z<hi[2]: raise ValueError('Individual hover height outside inflated arena')
    check_volume(slots(c),c); check_volume(slots(c)+[m['distance_m'],0,0],c)
    if 1.875*m['distance_m']/m['nominal_forward_duration_s']>ctrl['maximum_reference_speed_mps']: raise ValueError('Forward nominal peak exceeds reference speed budget')
    return c

def make_plan(c,ground,yaws):
    ground=np.asarray(ground,float); yaws=np.asarray(yaws,float); a=c['acquisition']
    if ground.shape!=(3,3) or yaws.shape!=(3,) or not np.isfinite(ground).all() or not np.isfinite(yaws).all(): raise ValueError('Require three finite ground poses and headings')
    for k,key in enumerate(('start_x_limits','start_y_limits')):
        if np.any(ground[:,k]<a[key][0]) or np.any(ground[:,k]>a[key][1]): raise ValueError('Captured starts outside requested left-side region: '+key)
    if np.max(np.abs(ground[:,2]-c['arena']['floor_z_m']))>.08: raise ValueError('Capture requires aircraft on the floor')
    hover=ground.copy(); hover[:,2]=[c['arena']['floor_z_m']+a['takeoff_heights_above_floor_m'][n] for n in NAMES]
    check_volume(hover,c)
    offsets=np.array([c['mission']['offsets_xyz'][n] for n in NAMES],float)
    assigned=offsets.copy()
    if a['assignment']=='sorted_y':
        aircraft=np.argsort(ground[:,1],kind='stable'); targets=np.argsort(offsets[:,1],kind='stable')
        for i,j in zip(aircraft,targets): assigned[i]=offsets[j]
    center=hover.mean(axis=0); target=np.r_[c['mission']['center_start_xy'],center[2]]
    qgoal=target+assigned
    check_volume(qgoal,c); check_volume(qgoal+[c['mission']['distance_m'],0,0],c)
    delta=target-center
    if 1.875*np.linalg.norm(delta)/a['nominal_duration_s']>c['control']['maximum_reference_speed_mps']*.8: raise ValueError('Acquisition center speed needs a longer duration')
    return dict(ground=ground.tolist(),hover=hover.tolist(),initial_yaw=yaws.tolist(),center=center.tolist(),target_center=target.tolist(),offsets=assigned.tolist(),delta=delta.tolist(),
                assignment={n:assigned[i].tolist() for i,n in enumerate(NAMES)},scope='Planned references; not measured onboard setpoints')

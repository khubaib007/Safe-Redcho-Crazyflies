"""Edition-specific commands. Offline/init never initialize ROS or access radios."""
import argparse,json,os,subprocess,sys,time
from pathlib import Path
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
EDITION=json.loads((ROOT/'release.json').read_text())['edition']
NAMES=['cf2','cf3','cf4']
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('action',nargs='?',default='help')
parser.add_argument('--scenario',choices=['default','combined','obstacle','interagent'],default='default')
parser.add_argument('--output',type=Path)
args,extra=parser.parse_known_args()
output=args.output or ROOT/'results'/(args.action+'_'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime()))

def execute(command):
    print('Running:',*command,flush=True)
    os.execvp(command[0],command)

def plain_profile(name,local=False):return ROOT/'config'/('local' if local else '')/name

def profile_name():
    if EDITION=='mission':return 'three_mission'+('' if args.scenario=='default' else '_'+args.scenario)+'.yaml'
    return 'redcho_three.yaml' if EDITION=='three' else 'redcho_hardware.yaml'

def numerical_controller(c,package):
    import importlib
    settings=importlib.import_module(package+'.settings');control=importlib.import_module(package+'.controller');qp=importlib.import_module(package+'.cbf_qp')
    settings.validate(c);n=len(c['names']);dt=c['control']['dt_s'];agents=[control.LocalController(c,i,trace=True) for i in range(n)]
    q=settings.slots(c);v=np.zeros_like(q);previous=v.copy();history=[];minimum=float('inf');worst=0.;done=None
    for cycle in range(1,int(160/dt)+1):
        t=(cycle-1)*dt;frame=dict(seq=cycle,stage='ALIGN' if t<c['mission']['observer_warmup_s'] else 'FORWARD',q=q.tolist(),v=v.tolist(),previous=previous.tolist(),voltage=np.linspace(4.,3.6,n).tolist())
        outgoing={a.name:a.prepare(frame) for a in agents}
        results=[a.finish({name:outgoing[name] for name in a.neighbors}) for a in agents]
        u=np.array([r['u'] for r in results]);q,v=qp.validate_batch(q,v,previous,u,c);previous=u
        minimum=min(minimum,min(float(np.linalg.norm(q[i,:2]-q[j,:2])) for i in range(n) for j in range(i)))
        worst=max(worst,max(r['qp_violation'] for r in results));history.append((t,q.copy(),v.copy(),u.copy()))
        goal=settings.slots(c)+[c['mission']['distance_m'],0,0]
        settled=all(r['local_phase']>=1 for r in results) and np.max(np.linalg.norm(q-goal,axis=1))<.01 and np.max(np.linalg.norm(v,axis=1))<.01
        done=t if settled and done is None else done if settled else None
        if done is not None and t-done>=3:break
    else:raise RuntimeError('Offline reference mission failed to settle within 160 logical seconds')
    report=dict(status='PASS',edition=EDITION,duration_s=t,cycles=cycle,minimum_virtual_xy_separation_m=minimum,required_virtual_xy_separation_m=settings.pair_distance(c),maximum_qp_violation=worst,final_reference_endpoint_errors_m=np.linalg.norm(q-goal,axis=1).tolist(),scope='Virtual-reference numerical controller only; no ROS, firmware, RF or physical tracking validation')
    output.mkdir(parents=True,exist_ok=False);np.savez_compressed(output/'offline_trace.npz',**{k:np.array([row[i] for row in history]) for i,k in enumerate(['time','reference_position','reference_velocity','safe_acceleration'])})
    (output/'offline_report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))

if args.action=='help':
    print('Edition:',EDITION)
    print('Offline: init | offline. Live ROS: mocap | backend | capture | check | agents | observe | hover | acquire | fly. Commissioning additionally: single-backend | single-check | single-hover | fleet-hover | swap. See README for applicable modes.');raise SystemExit(0)
if args.action=='init':execute(['/usr/bin/python3',str(ROOT/'scripts/configure.py')])
if args.action=='offline':
    if output.exists():raise SystemExit('Choose a new output directory: '+str(output))
    if EDITION=='commissioning':
        from safe_redcho_bringup.fleet_configuration import load_fleet
        from safe_redcho_bringup.swap_planner import build_plan
        import math
        c=load_fleet(ROOT/'config/fleet_config.yaml')
        # A named synthetic geometry fixture; never used as live captured starts.
        c['arena'].update(x_limits=[-1.8,1.8],y_limits=[-1.7,1.7],floor_z_m=0.,ceiling_z_m=2.)
        c['mission'].update(minimum_separation_m=.7,abort_separation_m=.9,planning_reserve_m=.05,tracking_tolerance_m=.1,maximum_flight_duration_s=300.)
        c['swap']['buffer_xy']=[0.,0.];c['_starts']={f'cf{i+1}':[1.2*math.cos(i*math.pi/3),1.2*math.sin(i*math.pi/3),.02] for i in range(6)}
        report=build_plan(c);report.update(status='PASS',scope='Synthetic swap geometry; no ROS/radio/aircraft commands');output.mkdir(parents=True);(output/'offline_report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    elif EDITION in ['flight','three']:
        c=yaml.safe_load(plain_profile(profile_name()).read_text());numerical_controller(c,'safe_redcho_flight' if EDITION=='flight' else 'safe_redcho_three')
    else:
        command=['/usr/bin/python3','-m','safe_redcho_three_mission.preview','--config',str(plain_profile(profile_name())),'--output',str(output)]
        if args.scenario!='default':command+=['--starts',str(plain_profile('three_mission_'+args.scenario+'_fixture.yaml'))]
        execute(command+extra)
    raise SystemExit(0)

# All remaining commands are deliberate live operations. Do not run them in CI.
local=ROOT/'config/local'
if not (local/'lab_config.yaml').is_file():raise SystemExit('Initialize and edit local hardware settings first: bash scripts/run.sh init')
if EDITION=='commissioning':
    lab=str(local/'lab_config.yaml');fleet=str(local/'fleet_config.yaml')
    commands={
      'mocap':['ros2','launch','safe_redcho_bringup','mocap.launch.py','lab_config_file:='+lab],
      'single-backend':['ros2','launch','safe_redcho_bringup','single_cf.launch.py','lab_config_file:='+lab],
      'backend':['ros2','launch','safe_redcho_bringup','six_cf.launch.py','fleet_config_file:='+fleet],
      'capture':['ros2','run','safe_redcho_bringup','fleet_capture','--config',fleet],
      'single-check':['ros2','run','safe_redcho_bringup','single_hover','--config',lab,'--check-only'],
      'single-hover':['ros2','run','safe_redcho_bringup','single_hover','--config',lab],
      'check':['ros2','run','safe_redcho_bringup','fleet_mission','--config',fleet,'--mode','hover','--check-only','--output',str(output)],
      'fleet-hover':['ros2','run','safe_redcho_bringup','fleet_mission','--config',fleet,'--mode','hover','--output',str(output)],
      'swap':['ros2','run','safe_redcho_bringup','fleet_mission','--config',fleet,'--mode','swap','--output',str(output)]}
elif EDITION in ['flight','three']:
    package='safe_redcho_flight' if EDITION=='flight' else 'safe_redcho_three';profile=str(local/profile_name())
    commands={a:['ros2','launch',package,a+'.launch.py','config_file:='+profile] for a in ['mocap','backend','agents']}
    commands['capture']=(['ros2','run','safe_redcho_bringup','fleet_capture','--config',str(local/'fleet_redcho.yaml')] if EDITION=='flight' else ['ros2','run',package,'redcho_capture','--config',profile])
    for action,flags in [('check',[]),('hover',['--fly','--hover-only']),('fly',['--fly'])]:commands[action]=['ros2','run',package,'redcho_mission','--config',profile,'--output',str(output),*flags]
    telemetry='safe_redcho_telemetry' if EDITION=='flight' else 'safe_redcho_three_telemetry'
    commands['observe']=['ros2','launch',telemetry,'all.launch.py','config_file:='+str(local/('telemetry.yaml' if EDITION=='flight' else 'telemetry_three.yaml'))]
else:
    from safe_redcho_three_mission.survey import resolve_profile
    from safe_redcho_three_mission.observation_config import prepare
    base=local/profile_name();profile=base
    if args.scenario!='default' and args.action not in ['mocap','capture']:
        profile=resolve_profile(base,ROOT/'config/generated_hocbf'/profile_name())
    package='safe_redcho_three_mission'
    commands={a:['ros2','launch',package,a+'.launch.py','config_file:='+str(profile)] for a in ['mocap','backend','agents']}
    if args.scenario!='default':
        commands['mocap']=['ros2','launch',package,'survey_mocap.launch.py','config_file:='+str(base)]
        commands['capture']=['/usr/bin/python3','-m',package+'.survey','--config',str(base),'--output',str(ROOT/'config/generated_hocbf'/profile_name())]
    else:commands['capture']=['ros2','run',package,'mission_capture','--config',str(profile)]
    for action,flags in [('check',[]),('acquire',['--fly','--acquire-only']),('fly',['--fly'])]:commands[action]=['ros2','run',package,'mission_coordinator','--config',str(profile),'--output',str(output),*flags]
    if args.action=='observe':
        telemetry,plots=prepare(profile,local/'three_mission_telemetry.yaml',local/'three_mission_liveplots.yaml',ROOT/'results/observation_configs')
        commands['observe']=['ros2','launch',package,'observe.launch.py','telemetry_config:='+str(telemetry),'plot_config:='+str(plots)]
if args.action not in commands:raise SystemExit('Action not supported by this edition; see README')
execute(commands[args.action]+extra)

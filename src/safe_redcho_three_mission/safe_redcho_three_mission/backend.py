"""Read local upstream templates, write ONLY new generated configuration."""
from copy import deepcopy
from pathlib import Path
import re,json
import yaml
from .settings import NAMES,path_of,make_plan

def firmware_parameters(c):
    a,b=c['arena'],c['backend'];margin=a['body_radius_m']+a['wall_surface_clearance_m']
    return dict(enable=int(b['bvca_enabled']),ellipsoidX=b['bvca_horizontal_radius_m'],ellipsoidY=b['bvca_horizontal_radius_m'],ellipsoidZ=b['bvca_vertical_radius_m'],
                bboxMinX=a['x_limits'][0]+margin,bboxMaxX=a['x_limits'][1]-margin,bboxMinY=a['y_limits'][0]+margin,bboxMaxY=a['y_limits'][1]-margin,
                bboxMinZ=a['floor_z_m']-.03,bboxMaxZ=a['ceiling_z_m']-a['ceiling_clearance_m'],horizon=b['bvca_horizon_s'],maxSpeed=c['safety']['measured_maximum_speed_mps'],maxPeerLocAge=150)

def starts(c):
    s=yaml.safe_load(path_of(c,'starts_file').read_text())
    if s.get('frame_id')!=c['frame_id'] or set(s.get('positions',{}))!=set(NAMES) or set(s.get('yaw_rad',{}))!=set(NAMES):raise ValueError('Recapture all three aircraft in world frame')
    make_plan(c,[s['positions'][n] for n in NAMES],[s['yaw_rad'][n] for n in NAMES]);return s

def radios(c):
    fleet=yaml.safe_load(path_of(c,'fleet_config_file').read_text());result={n:fleet['robots'][n]['uri'] for n in NAMES};addresses=set();groups={}
    for n,uri in result.items():
        m=re.fullmatch(r'radio://(\d+)/(\d{1,3})/2M/([0-9a-fA-F]{10})',uri)
        if not m or not 0<=int(m[2])<=125:raise ValueError(n+': invalid radio URI')
        index,ch,addr=int(m[1]),int(m[2]),m[3].lower()
        if addr in addresses:raise ValueError('Radio addresses must be unique')
        addresses.add(addr)
        if index in groups and groups[index]!=ch:raise ValueError('One channel per dongle required')
        groups[index]=ch
    channels=list(groups.values())
    if any(abs(channels[i]-channels[j])<2 for i in range(len(channels)) for j in range(i)):raise ValueError('Dongle channels must not overlap')
    return result

def prepare(c,radio=True):
    from ament_index_python.packages import get_package_share_directory
    workspace=Path(c['workspace']).expanduser();lab=yaml.safe_load(path_of(c,'lab_config_file').read_text());share=Path(get_package_share_directory('crazyflie'))
    local=workspace/'src/crazyswarm2/crazyflie/config';source=local if (local/'motion_capture.yaml').is_file() else share/'config'
    mocap=yaml.safe_load((source/'motion_capture.yaml').read_text());mp=mocap['/motion_capture_tracking']['ros__parameters'];mp['type']=lab['mocap']['backend'];mp['hostname']=lab['network']['motive_ip'];mp['rigid_bodies']={}
    mp.setdefault('topics',{})['frame_id']=c['frame_id'];mp['topics'].setdefault('poses',{})['qos']={'mode':'sensor','deadline':float(lab['mocap']['expected_rate_hz'])};mp['topics'].setdefault('tf',{})['child_frame_id']='{}_mocap'
    mocap_src=workspace/'src/motion_capture_tracking/motion_capture_tracking/src'
    if any('"interface_ip"' in p.read_text(errors='replace') for p in mocap_src.glob('*.cpp')):mp['interface_ip']=lab['network']['ubuntu_ip']
    else:mp.pop('interface_ip',None)
    generated=workspace/'config/generated_three_mission';generated.mkdir(parents=True,exist_ok=True);(generated/'motion_capture.yaml').write_text(yaml.safe_dump(mocap,sort_keys=False))
    if not radio:return mp,None,None
    snapshot=starts(c);uris=radios(c);robots=yaml.safe_load((source/'crazyflies.yaml').read_text());typ=deepcopy(robots['robot_types']['cf21']);typ.setdefault('motion_capture',{})['tracking']='vendor';robots['robot_types']={'cf21':typ}
    robots['robots']={n:dict(enabled=True,type='cf21',uri=uris[n],initial_position=snapshot['positions'][n]) for n in NAMES}
    allcfg=robots.setdefault('all',{});allcfg['reference_frame']=c['frame_id'];fw=allcfg.setdefault('firmware_params',{});fw.pop('commander',None);fw.setdefault('stabilizer',{}).update(controller=1,estimator=2);fw['colAv']=firmware_parameters(c)
    allcfg['firmware_logging']={'enabled':True,'default_topics':{'pose':{'frequency':c['backend']['pose_frequency_hz']},'status':{'frequency':c['backend']['status_frequency_hz']}}}
    server=deepcopy(yaml.safe_load((share/'config/server.yaml').read_text())['/crazyflie_server']['ros__parameters']);server.setdefault('firmware_params',{})['query_all_values_on_connect']=True
    server['poses_qos_deadline']=float(mp['topics']['poses']['qos']['deadline']);server['robot_description']=(Path(get_package_share_directory('crazyflie_description'))/'urdf/crazyflie_description.urdf').read_text()
    (generated/'crazyflies.yaml').write_text(yaml.safe_dump(robots,sort_keys=False));(generated/'server.yaml').write_text(yaml.safe_dump(server,sort_keys=False));return mp,robots,server

def verify_cache(node,c):
    from rcl_interfaces.srv import GetParameters
    import rclpy,time,math
    expected={}
    for n in NAMES:
        expected[n+'.params.stabilizer.controller']=1;expected[n+'.params.stabilizer.estimator']=2
        expected.update({n+'.params.colAv.'+k:v for k,v in firmware_parameters(c).items()})
    client=node.create_client(GetParameters,'/crazyflie_server/get_parameters');report={'scope':'ROS parameter cache, not fresh RF readback or peer-table verification','values':{}}
    try:
        if not client.wait_for_service(timeout_sec=3):raise RuntimeError('Backend get_parameters unavailable')
        for name,value in expected.items():
            actual=None
            for attempt in range(3):
                req=GetParameters.Request();req.names=[name];future=client.call_async(req);end=time.monotonic()+1.5
                while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
                if not future.done():
                    future.cancel()
                    if hasattr(client,'remove_pending_request'):client.remove_pending_request(future)
                    continue
                response=future.result()
                if response and len(response.values)==1:
                    v=response.values[0];actual=v.integer_value if v.type==2 else v.double_value if v.type==3 else None
                break
            report['values'][name]=dict(expected=value,actual=actual)
            if actual is None or not math.isfinite(actual) or abs(actual-value)>1e-5:raise RuntimeError('Backend cached firmware setting missing/mismatched: '+name+' '+str(report['values'][name]))
        report['status']='PASS_ROS_CACHE';return report
    finally:
        if hasattr(node,'output'):(node.output/'parameter_preflight.json').write_text(json.dumps(report,indent=2))
        node.destroy_client(client)

"""Explicit real-DDS check of three agent processes, without aircraft interfaces.

Only the new algorithm namespace is used. No Mission, StateNode, backend,
watchdog, Crazyflie command publisher or flight service is instantiated.
Run this manually on a separate ROS domain; it is not part of pytest.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from std_msgs.msg import String
import yaml

from safe_redcho_single_mission.core import accept_batch,StageMachine,wrap
from safe_redcho_single_mission.avoidance import motion_progress
from safe_redcho_single_mission.protocol import encode, decode, fingerprint
from safe_redcho_single_mission.settings import load, make_plan, NAMES


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True)
    parser.add_argument('--starts',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--cycles',type=int,default=250)
    parser.add_argument('--full-mission',action='store_true',help='Run all stages with synthetic tracking and the same hard-QP checks')
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None,'0','43'):
        parser.error('Set an isolated non-production ROS_DOMAIN_ID (for example 173)')
    if not 10 <= args.cycles <= (10000 if args.full_mission else 1000):
        parser.error('Use 10 through 1000 cycles, or up to 10000 with --full-mission')
    c=load(args.config);s=yaml.safe_load(Path(args.starts).read_text())
    p=make_plan(c,[s['positions'][n] for n in NAMES],[s['yaw_rad'][n] for n in NAMES])
    output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
    binary=Path(get_package_prefix('safe_redcho_single_mission'))/'lib/safe_redcho_single_mission/mission_agent'
    processes=[];logs=[];rclpy.init();node=Node('single_position_inert_transport_probe')
    ready={};responses={};telemetry={n:0 for n in NAMES};session=uuid.uuid4().hex
    route_stats=dict(active_agent_samples=0,search_samples=0,maximum_search_wall_s=0.,maximum_nodes=0)
    pub=node.create_publisher(String,c['namespace']+'/frame',8)
    def receive_ready(msg):
        v=decode(msg.data)
        if v.get('config_hash')==fingerprint(c):ready[v['name']]=v
    def receive_command(msg):
        v=decode(msg.data)
        if v.get('session')==session:responses[v['name']]=v
    def receive_trace(msg,name):
        v=decode(msg.data)
        if v.get('session')==session:
            assert v['flight_active'] is False and v['streaming'] is False
            assert v['data']['architecture']['bank_count']==1
            assert 'velocity_redcho' not in v['data']
            guide=v['data'].get('avoidance',{})
            route=guide.get('route',{})
            route_stats['active_agent_samples']+=int(guide.get('active',False) and guide.get('method')=='visibility_waypoints')
            if 'search_wall_s' in route and not route.get('cached',False):
                route_stats['search_samples']+=1
                route_stats['maximum_search_wall_s']=max(route_stats['maximum_search_wall_s'],route['search_wall_s'])
                route_stats['maximum_nodes']=max(route_stats['maximum_nodes'],route.get('nodes',0))
            telemetry[name]+=1
    node.create_subscription(String,c['namespace']+'/agent_ready',receive_ready,8)
    node.create_subscription(String,c['namespace']+'/commands',receive_command,16)
    for name in NAMES:
        node.create_subscription(String,c['namespace']+'/'+name+'/algorithm_telemetry',
            lambda msg,name=name:receive_trace(msg,name),qos_profile_sensor_data)
    report={'status':'FAIL','scope':'Real DDS, three synthetic agent processes; no aircraft interfaces or commands',
            'domain':os.environ['ROS_DOMAIN_ID'],'maximum_cycles':args.cycles,'full_mission':args.full_mission}
    try:
        for name in NAMES:
            log=(output/(name+'.log')).open('w');logs.append(log)
            processes.append(subprocess.Popen([str(binary),'--config',str(Path(args.config).resolve()),'--agent',name],stdout=log,stderr=subprocess.STDOUT))
        end=time.monotonic()+12
        while set(ready)!=set(NAMES) and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        assert set(ready)==set(NAMES),'Agent discovery failed'
        assert len({r['pid'] for r in ready.values()})==3
        # Let all frame/peer endpoints discover before starting the first session.
        end=time.monotonic()+1
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        q=np.asarray(p['hover']);v=np.zeros((3,3));previous=v.copy();latencies=[]
        measured=q.copy();measured_velocity=v.copy();yaw=np.asarray(p['initial_yaw']);machine=StageMachine(c,p);progress=1.;complete=False
        for seq in range(1,args.cycles+1):
            responses.clear();start=time.monotonic()
            pub.publish(String(data=encode(dict(session=session,seq=seq,config_hash=fingerprint(c),stage=machine.stage if args.full_mission else 'WARMUP',motion_progress_scale=progress,
                q=q.tolist(),v=v.tolist(),previous=previous.tolist(),voltage=[3.55,3.45,3.35],
                flight_active=False,streaming=False,plan=p))))
            while set(responses)!=set(NAMES) and time.monotonic()-start<c['control']['cycle_deadline_s']:
                rclpy.spin_once(node,timeout_sec=.001)
            assert set(responses)==set(NAMES),f'Incomplete cycle {seq}'
            assert all(r.get('seq')==seq and not r.get('error') for r in responses.values()),str(responses)
            assert time.monotonic()-start<c['control']['cycle_deadline_s'],f'Cycle deadline exceeded: {seq}'
            u=np.array([responses[n]['u'] for n in NAMES]);q,v=accept_batch(q,v,previous,u,c);previous=u
            if args.full_mission:
                progress=motion_progress(list(responses.values()),c)
                measured_velocity=(q-measured)/c['acquisition']['preview_tracking_time_constant_s'];measured+=c['control']['dt_s']*measured_velocity
                yaw+=c['control']['dt_s']*wrap(machine.yaw_reference()-yaw)/.15
                complete=machine.advance([responses[n] for n in NAMES],measured,measured_velocity,yaw)
            latencies.append(time.monotonic()-start)
            while time.monotonic()-start<c['control']['dt_s']:rclpy.spin_once(node,timeout_sec=.001)
            if complete:break
        end=time.monotonic()+.3
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        assert all(n>0 for n in telemetry.values()),'Missing single-bank telemetry'
        assert not args.full_mission or complete,'Synthetic full mission did not finish within the cycle budget'
        report.update(status='PASS',distinct_agent_pids=[ready[n]['pid'] for n in NAMES],
            cycles=seq,logical_duration_s=machine.total,stage_events=machine.events,
            maximum_complete_cycle_s=max(latencies),median_complete_cycle_s=float(np.median(latencies)),
            telemetry_packets=telemetry,route_guidance=route_stats)
    finally:
        for process in processes:
            if process.poll() is None:process.send_signal(signal.SIGINT)
        for process in processes:
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.terminate();process.wait(timeout=3)
        for log in logs:log.close()
        node.destroy_node();rclpy.try_shutdown()
        (output/'report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()

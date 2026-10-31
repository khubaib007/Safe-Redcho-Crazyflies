#!/usr/bin/env python3
"""Read-only ground data qualification. Never arms or sends setpoints/services."""
import argparse
import csv
from datetime import datetime,timezone
import json
import math
from pathlib import Path
import time
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from safe_redcho_flight import health_sync as sync


def evaluate(node):
    report=sync.describe(node);s=node.profile['safety']; faults=[]
    for name,row in report['aircraft'].items():
        for kind,age_limit,minimum_rate in [('mocap',s['mocap_maximum_age_s'],s['minimum_mocap_rate_hz']),
                                          ('onboard',s['maximum_estimator_age_s'],s['minimum_estimator_rate_hz'])]:
            data=row[kind]
            if data['receipt_age_s'] is None or data['receipt_age_s']>age_limit:faults.append(name+':'+kind+'_stale')
            if data['effective_rate_hz']<minimum_rate:faults.append(name+':'+kind+'_rate')
        if row['mocap']['maximum_source_gap_s'] is not None and row['mocap']['maximum_source_gap_s']>s['mocap_maximum_age_s']:
            faults.append(name+':mocap_gap')
        if row['onboard'].get('rate_time_basis') not in (None,'ros_header'):
            faults.append(name+':onboard_header_not_ROS_clock')
        if row.get('alignment_error'):faults.append(name+':alignment_unavailable')
        elif row['aligned']['angle_rad']>s['maximum_estimator_attitude_error_rad']:
            faults.append(name+':orientation_disagreement')
        if row.get('aligned',{}).get('position_difference_m',0)>s['maximum_estimator_position_error_m']:
            faults.append(name+':position_disagreement')
        if row.get('velocity_fit') is None:faults.append(name+':velocity_history')
        elif row['velocity_fit']['speed_mps']>.04:faults.append(name+':not_stationary_on_ground')
    if node.count_publishers('/poses')!=1:faults.append('require_exactly_one_poses_publisher')
    return report,faults


class Probe(Node):
    def __init__(self,cfg,output):
        super().__init__('safe_redcho_ground_health_check')
        self.profile,self.names,self.output=cfg,cfg['names'],Path(output)
        self.output.mkdir(parents=True,exist_ok=True)
        sync.history(self)
        self.create_subscription(NamedPoseArray,'/poses',lambda msg:sync.observe(self,'/poses',msg),qos_profile_sensor_data)
        for n in self.names:
            self.create_subscription(PoseStamped,f'/{n}/pose',lambda msg,n=n:sync.observe(self,f'/{n}/pose',msg),qos_profile_sensor_data)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default=str(Path.home()/'safe_redcho_hardware/config/redcho_hardware.yaml'))
    parser.add_argument('--duration',type=float,default=20.)
    parser.add_argument('--output',default=None)
    args,rosargs=parser.parse_known_args()
    if args.duration<8:parser.error('Use duration >=8 seconds to qualify 5 continuous seconds after warmup')
    cfg=yaml.safe_load(Path(args.config).expanduser().read_text())
    output=args.output or str(Path(cfg['workspace'])/'results'/datetime.now(timezone.utc).strftime('health_check_%Y%m%dT%H%M%S_%fZ'))
    rclpy.init(args=rosargs);node=Probe(cfg,output)
    start,good_since,last_tick,last_notice=time.monotonic(),None,-math.inf,-math.inf
    worst_angles={n:0. for n in node.names};fail_counts={};final=None;continuous=0.
    try:
        with (node.output/'health_samples.csv').open('w',newline='') as f:
            fields=['elapsed_s','agent','mocap_age_s','onboard_age_s','mocap_rate_hz','onboard_rate_hz',
                    'onboard_rate_time_basis','aligned_angle_rad','unaligned_angle_rad','header_pair_skew_s','source_time_speed_mps','issues']
            writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
            while time.monotonic()-start<args.duration:
                rclpy.spin_once(node,timeout_sec=.01)
                now=time.monotonic()
                if now-last_tick<.1:continue
                last_tick=now;report,faults=evaluate(node);final=report
                if not faults:
                    good_since=now if good_since is None else good_since
                    continuous=now-good_since
                else:
                    good_since=None;continuous=0.
                    for item in faults:fail_counts[item]=fail_counts.get(item,0)+1
                for name,row in report['aircraft'].items():
                    aligned=row.get('aligned',{});vv=row.get('velocity_fit') or {}
                    worst_angles[name]=max(worst_angles[name],aligned.get('angle_rad',0.))
                    writer.writerow(dict(elapsed_s=now-start,agent=name,
                        mocap_age_s=row['mocap']['receipt_age_s'],onboard_age_s=row['onboard']['receipt_age_s'],
                        mocap_rate_hz=row['mocap']['effective_rate_hz'],onboard_rate_hz=row['onboard']['effective_rate_hz'],
                        onboard_rate_time_basis=row['onboard']['rate_time_basis'],
                        aligned_angle_rad=aligned.get('angle_rad'),unaligned_angle_rad=aligned.get('latest_unaligned_angle_rad'),
                        header_pair_skew_s=aligned.get('header_pair_skew_s'),source_time_speed_mps=vv.get('speed_mps'),
                        issues=';'.join(x for x in faults if x.startswith(name+':'))))
                if now-last_notice>=1.:
                    last_notice=now
                    message=' / '.join(faults[:8]) if faults else f'all six qualified for {continuous:.1f}s'
                    node.get_logger().info(message)
                    f.flush()
    except KeyboardInterrupt:
        continuous=0.
    finally:
        result={'status':'GROUND_DATA_PASS' if continuous>=5. else 'GROUND_DATA_FAIL',
                'continuous_good_s_at_end':continuous,'worst_aligned_angle_rad':worst_angles,
                'fault_observation_counts':fail_counts,'final_snapshot':final,
                'scope':'Ground pose data only; no flight commands. Not an airborne visibility or radio-load qualification.'}
        (node.output/'health_summary.json').write_text(json.dumps(sync.clean(result),indent=2,allow_nan=False)+'\n')
        print(json.dumps(sync.clean(result),indent=2,allow_nan=False))
        print('Saved:',node.output)
        node.destroy_node();rclpy.try_shutdown()
    if result['status']!='GROUND_DATA_PASS':raise SystemExit(1)

if __name__=='__main__':main()

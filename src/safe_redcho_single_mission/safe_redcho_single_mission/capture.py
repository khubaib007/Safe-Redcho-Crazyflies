"""Capture arbitrary admissible floor XY and yaw; no flight service clients."""
import argparse,json,time
from datetime import datetime,timezone
import numpy as np
import yaml,rclpy
from .ros_io import StateNode
from .health import tilt
from .settings import load,path_of,make_plan,NAMES
from .protocol import fingerprint

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',required=True);a,ros=p.parse_known_args();c=load(a.config);rclpy.init(args=ros);n=StateNode(c,'single_mission_capture',onboard=False);end=time.monotonic()+60;good=None;lastlog=0
    try:
        while time.monotonic()<end:
            n.spin_for(.02);now=time.monotonic()
            try:
                if n.measurement_fault:raise RuntimeError(n.measurement_fault)
                if n.count_publishers('/poses')!=1:raise RuntimeError('Require exactly one /poses publisher')
                positions,velocity,yaws=n.snapshot();plan=make_plan(c,positions,yaws)
                for name in NAMES:
                    h=n.mc[name];rate,coverage=h.stats()
                    if rate<c['safety']['minimum_mocap_rate_hz'] or coverage<.8:raise RuntimeError(name+': waiting for mocap history')
                    recent=[r[2] for r in h.rows if h.rows[-1][0]-r[0]<=.8]
                    if np.max(np.ptp(recent,axis=0))>.03 or tilt(h.rows[-1][3])>c['safety']['measured_maximum_tilt_rad']:raise RuntimeError(name+': keep level and stationary on floor')
                if max(np.linalg.norm(velocity,axis=1))>.05:raise RuntimeError('Keep aircraft stationary')
                good=now if good is None else good
                if now-good>=2:
                    snapshot=dict(captured_utc=datetime.now(timezone.utc).isoformat(),frame_id=c['frame_id'],positions={name:positions[i].tolist() for i,name in enumerate(NAMES)},yaw_rad={name:float(yaws[i]) for i,name in enumerate(NAMES)},capture_config_hash=fingerprint(c),planned_assignment=plan['assignment'])
                    dst=path_of(c,'starts_file');dst.parent.mkdir(parents=True,exist_ok=True);temporary=dst.with_suffix('.tmp');temporary.write_text(yaml.safe_dump(snapshot,sort_keys=False));temporary.replace(dst);n.get_logger().info('Captured starts: '+str(dst));return
            except (RuntimeError,ValueError) as e:
                good=None
                if now-lastlog>1:n.get_logger().info(str(e));lastlog=now
        raise RuntimeError('Capture timed out; no starts written')
    finally:n.destroy_node();rclpy.try_shutdown()

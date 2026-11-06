"""Read-only local software/interface check; no ROS node, services or radio I/O."""
import argparse,json
from pathlib import Path
from .settings import load,path_of
from .backend import radios

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',required=True);args=p.parse_args();c=load(args.config)
 from ament_index_python.packages import get_package_prefix,get_package_share_directory
 from crazyflie_interfaces.msg import Position,Status
 from crazyflie_interfaces.srv import Arm,Takeoff,Land,NotifySetpointsStop
 from motion_capture_tracking_interfaces.msg import NamedPoseArray
 requirements=[(Position(),('header','x','y','z','yaw')),(Status(),('battery_voltage',)),(NamedPoseArray(),('header','poses')),
               (Arm.Request(),('arm',)),(Takeoff.Request(),('height','duration','group_mask')),(Land.Request(),('height','duration','group_mask')),(NotifySetpointsStop.Request(),('remain_valid_millisecs','group_mask'))]
 for obj,fields in requirements:
  if any(not hasattr(obj,k) for k in fields):raise RuntimeError('Unsupported interface '+type(obj).__name__)
 for package,executable in [('crazyflie_server_cpp','crazyflie_server'),('motion_capture_tracking','motion_capture_tracking_node')]:
  target=Path(get_package_prefix(package))/'lib'/package/executable
  if not target.is_file():raise RuntimeError('Missing backend executable '+str(target))
 if not path_of(c,'lab_config_file').is_file():raise RuntimeError('Missing calibrated lab_config.yaml')
 uris=radios(c)
 print(json.dumps(dict(status='PASS_LOCAL_SOFTWARE',profile=c['_path'],radio_uris=uris,scope='No ROS graph, live timing, firmware, RF or hardware verification'),indent=2))
if __name__=='__main__':main()

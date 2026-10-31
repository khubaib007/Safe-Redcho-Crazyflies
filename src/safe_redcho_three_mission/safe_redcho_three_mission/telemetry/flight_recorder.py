"""Node 1: received hardware state and separately labelled mocap data."""
from collections import deque
import math
import time
import numpy as np
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import Status
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from .base import Recorder, run
from .common import pose_data, stamp_ns


def decode_log(message, block):
    values = list(message.values)
    if len(values) != len(block['fields']): raise ValueError('Firmware log field count differs from configured mapping')
    if not all(math.isfinite(float(x)) for x in values): raise ValueError('Nonfinite firmware log')
    data = {}
    for field, value in zip(block['fields'], values):
        target = data
        path = field.split('.')
        for part in path[:-1]: target = target.setdefault(part,{})
        target[path[-1]] = float(value)
    if 'acceleration_raw_g' in data:
        scale = float(block['acceleration_scale_mps2_per_raw'])
        data['acceleration_mps2'] = {a:v*scale for a,v in data['acceleration_raw_g'].items()}
        data['acceleration_conversion_mps2_per_g'] = scale
    return data


class FlightRecorder(Recorder):
    def __init__(self, key, cfg, directory):
        super().__init__(key,cfg,directory)
        self.history = {n:deque(maxlen=100) for n in self.names}
        for name in self.names:
            self.expect(f'{name}:battery',3.)
            self.expect(f'{name}:onboard_pose',.5)
            self.expect(f'{name}:mocap',.2)
            self.create_subscription(Status,f'/{name}/status',lambda m,n=name:self.status(m,n),self.qos)
            self.create_subscription(PoseStamped,f'/{name}/pose',lambda m,n=name:self.onboard_pose(m,n),self.qos)
        self.create_subscription(NamedPoseArray,'/poses',self.mocap,self.qos)
        try:
            from crazyflie_interfaces.msg import LogDataGeneric
        except ImportError:
            self.get_logger().warning('LogDataGeneric unavailable: onboard velocity/acceleration will be MISSING')
            LogDataGeneric = None
        for topic,block in cfg['onboard']['blocks'].items():
            if not block.get('enabled') or block['recorder'] != 'flight': continue
            for name in self.names:
                self.expect(f'{name}:{topic}',max(1.,3./block['frequency']))
                if LogDataGeneric:
                    self.create_subscription(LogDataGeneric,f'/{name}/{topic}',
                        lambda m,n=name,t=topic,b=block:self.onboard_log(m,n,t,b),self.qos)

    def status(self,msg,name):
        self.record('battery',name,{'battery':{'voltage_v':float(msg.battery_voltage)}},
                    f'/{name}/status','onboard_status',frame='',stamp=stamp_ns(msg))

    def onboard_pose(self,msg,name):
        try:
            self.record('onboard_pose',name,{'onboard_pose':pose_data(msg.pose)},f'/{name}/pose',
                        'onboard_estimator',frame=msg.header.frame_id,stamp=stamp_ns(msg))
        except ValueError: self.bad_packets += 1

    def onboard_log(self,msg,name,topic,block):
        try:
            data = decode_log(msg,block)
            self.record(topic,name,{'onboard_'+topic:data},f'/{name}/{topic}',
                        'onboard_firmware_log',frame=block.get('frame','world'),stamp=stamp_ns(msg),
                        firmware_stamp=getattr(msg,'timestamp',''))
        except (ValueError,TypeError) as exc:
            self.bad_packets += 1
            if self.bad_packets <= 3: self.get_logger().warning(str(exc))

    def mocap(self,msg):
        stamp = stamp_ns(msg)
        now = self.get_clock().now().nanoseconds
        age_limit = self.cfg['mocap_maximum_source_age_s']
        if (msg.header.frame_id != 'world' or not stamp or
                not -.05 <= (now-stamp)/1e9 <= age_limit):
            self.bad_packets += 1
            return
        counts = {}
        for entry in msg.poses: counts[entry.name] = counts.get(entry.name,0)+1
        for entry in msg.poses:
            name = entry.name
            if name not in self.history: continue
            if counts[name] != 1:
                self.bad_packets += 1; continue
            history = self.history[name]
            if history and stamp <= history[-1][0]:
                self.bad_packets += 1; continue
            try: data = pose_data(entry.pose)
            except ValueError:
                self.bad_packets += 1; continue
            history.append((stamp,np.array([entry.pose.position.x,entry.pose.position.y,entry.pose.position.z])))
            data['derived_velocity_available'] = False
            if self.options.get('derive_mocap_velocity',False):
                rows = [(t,p) for t,p in history if t >= stamp-160_000_000]
                if (len(rows)>=5 and rows[-1][0]-rows[0][0]>=80_000_000 and
                        max(rows[i][0]-rows[i-1][0] for i in range(1,len(rows)))/1e9 <= self.cfg['mocap_maximum_source_gap_s']):
                    ts = np.array([(t-rows[0][0])/1e9 for t,_ in rows]); ts-=ts.mean()
                    velocity = ts@np.array([p for _,p in rows])/max(float(ts@ts),1e-12)
                    data['derived_velocity_mps'] = dict(zip(('x','y','z'),velocity.tolist()))
                    data['derived_velocity_available'] = True
            self.record('mocap',name,{'mocap':data},'/poses',
                'optitrack_received_pose_and_separately_derived_velocity',frame=msg.header.frame_id,stamp=stamp)


def main(): run('flight',FlightRecorder)

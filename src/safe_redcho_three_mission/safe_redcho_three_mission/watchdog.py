"""Separate-process heartbeat recovery. Never publishes streaming setpoints."""
import argparse,json,math,time
from pathlib import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from std_msgs.msg import String
from crazyflie_interfaces.srv import Arm,Land,NotifySetpointsStop
from .ros_io import StateNode,Services
from .settings import load,NAMES
from .backend import starts
from .protocol import encode,decode,fingerprint

class Watchdog(StateNode):
    def __init__(self,c):
        super().__init__(c,'three_mission_watchdog',onboard=False);self.snapshot_file=starts(c);self.flight_services=Services(self);self.active=False;self.last=None;self.triggered=False;self.stage=None;self.deadline=None;self.pending={};self.report={};self.last_ready=0
        ns=c['namespace'];self.ready=self.create_publisher(String,ns+'/watchdog_ready',4);self.create_subscription(String,ns+'/heartbeat',self.beat,8);self.create_timer(.025,self.tick)
    def beat(self,msg):
        try:
            p=decode(msg.data);stamp=float(p['monotonic_stamp']);age=time.monotonic()-stamp
            if self.triggered or p['config_hash']!=fingerprint(self.c) or not math.isfinite(stamp) or not 0<=age<=self.c['safety']['watchdog_timeout_s'] or (self.last is not None and stamp<=self.last):return
            self.active=bool(p['active']);self.last=stamp;self.session=p['session']
        except (ValueError,KeyError,TypeError):pass
    def send(self,key,requests):
        for n,r in requests.items():
            try:
                client=self.flight_services.clients[n][key]
                if not client.service_is_ready():raise RuntimeError('Service unavailable')
                self.pending[(key,n)]=client.call_async(r)
            except Exception as e:self.report.setdefault('request_errors',[]).append(n+'/'+key+': '+str(e))
    def tick(self):
        now=time.monotonic()
        if now-self.last_ready>.5:
            self.ready.publish(String(data=encode(dict(config_hash=fingerprint(self.c),triggered=self.triggered))));self.last_ready=now
        if not self.triggered:
            if not self.active or self.last is None or now-self.last<=self.c['safety']['watchdog_timeout_s']:return
            self.triggered=True
            self.report={'status':'TRIGGERED','reason':'Mission heartbeat expired','scope':'Service requests; not proof of radio delivery'}
            reqs={}
            for n in NAMES:
                r=Land.Request();r.group_mask=0
                r.height=float(self.snapshot_file['positions'][n][2]+self.c['mission']['landing_offset_m'])
                r.duration=Duration(seconds=self.c['mission']['landing_duration_s']).to_msg()
                reqs[n]=r
            self.handover=self.flight_services.begin_landing(reqs,release=True)
            self.stage='HANDOVER';self.deadline=now
            self.get_logger().error('Heartbeat expired: release -> land per aircraft; no release dwell')
        if self.stage=='HANDOVER':
            self.handover.tick()
            if not self.handover.done:return
            self.report.update(self.handover.report())
            self.stage='LAND';self.deadline=now+self.c['mission']['landing_duration_s']+2
            return
        if self.deadline is None or now<self.deadline:return
        if self.stage=='LAND':
            ros,mono=self.clocks();evidence={n:self.mc[n].touchdown(self.snapshot_file['positions'][n][2],ros,mono,self.c['safety']['mocap_maximum_age_s']) for n in NAMES};verified=[n for n in NAMES if evidence[n]['verified']];reqs={}
            for n in verified:r=Arm.Request();r.arm=False;reqs[n]=r
            self.send('arm',reqs);self.report.update(touchdown_evidence=evidence,touchdown_verified=verified,touchdown_not_verified=[n for n in NAMES if n not in verified]);self.stage='DONE';self.deadline=now+2
        elif self.stage=='DONE':
            for (key,n),f in self.pending.items():
                try:
                    if not f.done():raise RuntimeError('Reply still pending')
                    r=f.result()
                    if r is None or (hasattr(r,'success') and not r.success):raise RuntimeError('Failed reply')
                except Exception as e:self.report.setdefault('request_errors',[]).append(n+'/'+key+': '+str(e))
            out=Path(self.c['workspace']).expanduser()/'results/three_mission_watchdog';out.mkdir(parents=True,exist_ok=True);(out/'watchdog_report.json').write_text(json.dumps(self.report,indent=2));self.deadline=None;self.get_logger().error('Recovery requests finished; inspect '+str(out))

def main():
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);a,ros=p.parse_known_args();rclpy.init(args=ros);n=Watchdog(load(a.config))
    try:rclpy.spin(n)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:n.destroy_node();rclpy.try_shutdown()

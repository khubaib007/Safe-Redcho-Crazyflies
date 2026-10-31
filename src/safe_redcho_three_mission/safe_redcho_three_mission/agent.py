"""One process owns one pair of REDCHO banks and one local HOCBF-QP."""
import argparse,os,time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from .settings import load,NAMES
from .protocol import fingerprint,encode,decode
from .core import LocalController
from .telemetry_trace import DiagnosticEmitter

class Agent(Node):
    def __init__(self,c,index):
        self.agent_name=NAMES[index];super().__init__('mission_agent_'+self.agent_name)
        self.c,self.index,self.session,self.core,self.frame=c,index,None,None,None
        self.sent=False;self.failed=False;self.peers={};ns=c['namespace'];self.emitter=DiagnosticEmitter(self,self.agent_name)
        self.peer=self.create_publisher(String,ns+'/'+self.agent_name+'/neighbor',8);self.result=self.create_publisher(String,ns+'/commands',16);self.ready=self.create_publisher(String,ns+'/agent_ready',8)
        self.create_subscription(String,ns+'/frame',self.on_frame,8)
        for n in NAMES:
            if n!=self.agent_name:self.create_subscription(String,ns+'/'+n+'/neighbor',lambda m,n=n:self.on_peer(m,n),8)
        self.create_timer(.5,self.announce)
    def announce(self):self.ready.publish(String(data=encode(dict(name=self.agent_name,pid=os.getpid(),config_hash=fingerprint(self.c)))))
    def fault(self,e):
        self.failed=True;f=self.frame or {};self.result.publish(String(data=encode(dict(name=self.agent_name,pid=os.getpid(),session=f.get('session'),seq=f.get('seq'),error=f'{type(e).__name__}: {e}'))))
        if self.core is not None and self.frame is not None:
            self.emitter.submit(dict(schema_version=1,name=self.agent_name,session=self.session,seq=f['seq'],stage=f['stage'],config_hash=fingerprint(self.c),flight_active=f['flight_active'],streaming=f['streaming'],producer_ros_ns=self.get_clock().now().nanoseconds,producer_monotonic_ns=time.monotonic_ns(),error=f'{type(e).__name__}: {e}',data=getattr(self.core,'trace',{})))
    def on_frame(self,msg):
        try:
            f=decode(msg.data)
            if f['config_hash']!=fingerprint(self.c):raise ValueError('Configuration mismatch')
            if self.session!=f['session']:
                if f['seq']!=1:return
                self.core=LocalController(self.c,self.index,f['plan'],trace=True);self.session=f['session'];self.failed=False
                self.peers={k:v for k,v in self.peers.items() if k[0]==self.session}
            if self.failed or f['seq']<=self.core.sequence:return
            if self.frame and self.frame.get('session')==self.session and self.frame.get('seq')==f['seq']:
                if self.frame!=f:raise ValueError('Conflicting duplicate frame')
                return
            self.frame,self.sent=f,False;out=self.core.prepare(f);out.update(name=self.agent_name,session=self.session,seq=f['seq']);self.peer.publish(String(data=encode(out)));self.finish()
        except Exception as e:self.fault(e)
    def on_peer(self,msg,name):
        try:
            p=decode(msg.data)
            if p['name']!=name:raise ValueError('Wrong peer identity')
            key=(p['session'],p['seq'],name)
            if key in self.peers and self.peers[key]!=p:raise ValueError('Conflicting peer packet')
            self.peers[key]=p
            while len(self.peers)>24:self.peers.pop(next(iter(self.peers)))
            self.finish()
        except Exception as e:self.fault(e)
    def finish(self):
        if self.failed or self.sent or self.frame is None or self.core is None or not self.core.prepared:return
        keys=[(self.session,self.frame['seq'],n) for n in self.core.neighbors]
        if not all(k in self.peers for k in keys):return
        r=self.core.finish({k[2]:self.peers[k] for k in keys});trace=r.pop('diagnostics');r.update(name=self.agent_name,pid=os.getpid(),session=self.session,seq=self.frame['seq']);self.result.publish(String(data=encode(r)));self.sent=True
        self.emitter.submit(dict(schema_version=1,name=self.agent_name,session=self.session,seq=self.frame['seq'],stage=self.frame['stage'],config_hash=fingerprint(self.c),flight_active=self.frame['flight_active'],streaming=self.frame['streaming'],producer_ros_ns=self.get_clock().now().nanoseconds,producer_monotonic_ns=time.monotonic_ns(),error=None,data=trace))
        for k in keys:self.peers.pop(k,None)
    def destroy_node(self):self.emitter.close();return super().destroy_node()
def main():
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--agent',required=True,choices=NAMES);a,ros=p.parse_known_args();rclpy.init(args=ros);n=Agent(load(a.config),NAMES.index(a.agent))
    try:rclpy.spin(n)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:n.destroy_node();rclpy.try_shutdown()

"""Node 4: exact nominal/QP trace and actual host FullState messages."""
from crazyflie_interfaces.msg import FullState
from rosidl_runtime_py.convert import message_to_ordereddict
from .base import Recorder, run
from .common import stamp_ns
from .flight_recorder import decode_log


class ControlRecorder(Recorder):
    def __init__(self,key,cfg,directory):
        super().__init__(key,cfg,directory)
        self.diagnostic_subscriptions()
        for name in self.names:
            self.expect(f'{name}:published_setpoint',1.)
            self.create_subscription(FullState,f'/{name}/cmd_full_state',lambda m,n=name:self.setpoint(m,n),self.qos)
        try:
            from crazyflie_interfaces.msg import LogDataGeneric
        except ImportError: LogDataGeneric = None
        for topic,block in cfg['onboard']['blocks'].items():
            if not block.get('enabled') or block['recorder'] != 'control': continue
            for name in self.names:
                self.expect(f'{name}:{topic}',max(1.,3./block['frequency']))
                if LogDataGeneric:
                    self.create_subscription(LogDataGeneric,f'/{name}/{topic}',
                        lambda m,n=name,t=topic,b=block:self.echo(m,n,t,b),self.qos)

    def setpoint(self,msg,name):
        # Message introspection retains ALL fields of the installed interface.
        # No reconstruction from the QP output, and no claim of radio receipt.
        self.record('published_setpoint',name,{
            'published_full_state':message_to_ordereddict(msg),
            'delivery':{'onboard_receipt_confirmed':None,'scope':'host_ROS_publication_only'}},
            f'/{name}/cmd_full_state','host_published_FullState',frame=msg.header.frame_id,stamp=stamp_ns(msg))

    def echo(self,msg,name,topic,block):
        try:
            self.record(topic,name,{'onboard_target':decode_log(msg,block)},f'/{name}/{topic}',
                        'sampled_firmware_controller_target_not_per_packet_ack',stamp=stamp_ns(msg),
                        firmware_stamp=getattr(msg,'timestamp',''))
        except (ValueError,TypeError): self.bad_packets += 1

    def on_diagnostic(self,packet):
        qp = packet['data'].get('qp',{})
        if (qp.get('inequality_solver_invoked') or qp.get('hocbf_rows_binding') or
                qp.get('nominal_hocbf_violation_mps2',0)>1e-6):
            self.record('qp_interventions',packet['name'],{'qp':qp},
                        f"/safe_redcho/{packet['name']}/algorithm_telemetry",'offboard_exact_QP_event',
                        stamp=packet['producer_ros_ns'],session=packet['session'],seq=packet['seq'],stage=packet['stage'])

def main(): run('control',ControlRecorder)

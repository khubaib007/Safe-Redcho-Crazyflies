"""Node 4: computed nominal/QP results and actual ROS Position messages."""
import json
from crazyflie_interfaces.msg import Position
from std_msgs.msg import String
from rosidl_runtime_py.convert import message_to_ordereddict
from .base import Recorder,run
from .common import stamp_ns
from .flight_recorder import decode_log


class ControlRecorder(Recorder):
    def __init__(self,key,cfg,directory):
        super().__init__(key,cfg,directory)
        self.diagnostic_subscriptions()
        for name in self.names:
            self.expect(f'{name}:published_setpoint',1.)
            self.create_subscription(Position,f'/{name}/cmd_position',lambda m,n=name:self.setpoint(m,n),self.qos)
        self.create_subscription(String,'/safe_redcho_mission/frame',lambda m:self.receive_cycle_context(m,'frame'),self.qos)
        self.create_subscription(String,'/safe_redcho_mission/commands',lambda m:self.receive_cycle_context(m,'commands'),self.qos)
        self.create_subscription(String,'/safe_redcho_mission/heartbeat',lambda m:self.receive_cycle_context(m,'heartbeat'),self.qos)
        try:
            from crazyflie_interfaces.msg import LogDataGeneric
        except ImportError: LogDataGeneric=None
        for topic,block in cfg['onboard']['blocks'].items():
            if not block.get('enabled') or block['recorder']!='control': continue
            for name in self.names:
                self.expect(f'{name}:{topic}',max(1.,3./block['frequency']))
                if LogDataGeneric:
                    self.create_subscription(LogDataGeneric,f'/{name}/{topic}',
                        lambda m,n=name,t=topic,b=block:self.echo(m,n,t,b),self.qos)

    def setpoint(self,msg,name):
        self.record('published_setpoint',name,{
            'published_position':message_to_ordereddict(msg),
            'delivery':{'onboard_receipt_confirmed':None,'scope':'received_ROS_Position_publication',
                'algorithm_cycle_association':'not_supplied_by_Position_message',
                'velocity_acceleration_angular_rates_transmitted':False}},
            f'/{name}/cmd_position','observed_host_Position_message',frame=msg.header.frame_id,stamp=stamp_ns(msg))

    def receive_cycle_context(self,msg,kind):
        try:
            if len(msg.data)>100000: raise ValueError('Oversized cycle context')
            def reject(x): raise ValueError(x)
            p=json.loads(msg.data,parse_constant=reject)
            if not isinstance(p,dict): raise ValueError('Invalid context')
            if kind=='commands':
                if p.get('name') not in self.names: return
                # The producer's result packet has no profile hash. Preserve it as
                # an unassociated result, not as proof of acceptance/application.
                self.record('controller_result',p['name'],{'controller_result':p},
                    '/safe_redcho_mission/commands','offboard_result_not_proof_of_acceptance',frame='',
                    session=p.get('session',''),seq=p.get('seq',''))
                return
            if p.get('config_hash')!=self.cfg['expected_config_hash']: raise ValueError('Context configuration mismatch')
            if kind=='frame':
                self.record('cycle_frame','fleet',{'cycle_frame':p},'/safe_redcho_mission/frame',
                    'offboard_common_cycle_input',frame='',session=p['session'],seq=p['seq'],stage=p['stage'])
            else:
                self.record('mission_heartbeat','fleet',{'mission_heartbeat':p},'/safe_redcho_mission/heartbeat',
                    'offboard_mission_heartbeat',frame='',session=p.get('session',''),seq=p.get('seq',''))
        except (KeyError,TypeError,ValueError): self.bad_packets+=1

    def echo(self,msg,name,topic,block):
        try:
            self.record(topic,name,{'onboard_target':decode_log(msg,block)},f'/{name}/{topic}',
                'sampled_firmware_target_not_packet_ack',stamp=stamp_ns(msg),firmware_stamp=getattr(msg,'timestamp',''))
        except (ValueError,TypeError): self.bad_packets+=1

    def on_diagnostic(self,packet):
        qp=packet['data'].get('qp',{})
        if (qp.get('inequality_solver_invoked') or qp.get('hocbf_rows_binding') or
            qp.get('nominal_hocbf_violation_mps2',0)>1e-6):
            self.record('qp_interventions',packet['name'],{'qp':qp},
                f"/safe_redcho_mission/{packet['name']}/algorithm_telemetry",'offboard_exact_QP_event',
                stamp=packet['producer_ros_ns'],session=packet['session'],seq=packet['seq'],stage=packet['stage'])


def main(): run('control',ControlRecorder)

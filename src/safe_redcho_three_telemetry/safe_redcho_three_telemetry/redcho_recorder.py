"""Node 3: both banks and exact-cycle three-agent convergence."""
from .base import Recorder, run
from .metrics import convergence

class REDCHORecorder(Recorder):
    def __init__(self,key,cfg,directory):
        super().__init__(key,cfg,directory)
        self.pending, self.incomplete_evicted = {}, 0
        self.expect('fleet:convergence',1.)
        self.diagnostic_subscriptions()

    def on_diagnostic(self,packet):
        if packet.get('error') or not all(k in packet['data'] for k in ('velocity_redcho','position_redcho')): return
        key = (packet['session'],packet['seq'],packet['config_hash'])
        batch = self.pending.setdefault(key,{})
        batch[packet['name']] = packet
        if len(batch) == len(self.names):
            data = convergence(batch,self.names,self.cfg['convergence_thresholds'])
            data['convergence_collection'] = {'incomplete_batches_evicted':self.incomplete_evicted}
            self.record('convergence','fleet',data,'/safe_redcho/*/algorithm_telemetry',
                        'same_cycle_three_agent_metric',stamp=packet['producer_ros_ns'],
                        session=key[0],seq=key[1],stage=packet['stage'])
            del self.pending[key]
        while len(self.pending)>32:
            self.pending.pop(next(iter(self.pending))); self.incomplete_evicted+=1

def main(): run('redcho',REDCHORecorder)

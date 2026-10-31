"""Node 2: exact local governor and SVF instrumentation."""
from .base import Recorder, run

class ReferenceRecorder(Recorder):
    def __init__(self,key,cfg,directory):
        super().__init__(key,cfg,directory)
        self.diagnostic_subscriptions()

def main(): run('reference',ReferenceRecorder)

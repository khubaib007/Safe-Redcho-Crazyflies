"""Receive state on a separate ROS node; only the owner mutates pose histories.

Callback receipt timestamps travel with buffered messages. Draining a buffer
never gives an old packet a new receipt time. No flight publishers/clients.
"""
from collections import deque
import threading


class PacketBuffer:
    def __init__(self, capacities):
        self.lock=threading.Lock()
        self.queues={k:deque(maxlen=n) for k,n in capacities.items()}
        self.stats={k:dict(received=0,dropped=0,last_ros=None,last_mono=None,
                           maximum_callback_gap_s=0.) for k in capacities}

    def put(self,key,msg,ros_now,mono):
        with self.lock:
            q=self.queues[key];s=self.stats[key]
            if len(q)==q.maxlen:s['dropped']+=1
            if s['last_mono'] is not None:
                s['maximum_callback_gap_s']=max(s['maximum_callback_gap_s'],mono-s['last_mono'])
            s.update(received=s['received']+1,last_ros=ros_now,last_mono=mono)
            q.append((msg,ros_now,mono))

    def drain(self):
        with self.lock:
            rows=[(key,*row) for key,q in self.queues.items() for row in q]
            for q in self.queues.values():q.clear()
        # Preserve receipt ordering across pose/status/mocap channels.
        rows.sort(key=lambda row:row[3])
        return rows

    def report(self,mono):
        with self.lock:
            return {key:{**s,'pending':len(self.queues[key]),
                'last_callback_age_s':None if s['last_mono'] is None else mono-s['last_mono']}
                for key,s in self.stats.items()}


class StateReceiver:
    def __init__(self,owner,name,specifications):
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor,ExternalShutdownException
        from rclpy.qos import qos_profile_sensor_data
        self.buffer=PacketBuffer({key:capacity for key,_,_,capacity in specifications})
        self.error=None;self.closed=False;self._external_shutdown=ExternalShutdownException
        self.node=Node(name,context=owner.context)
        self.executor=SingleThreadedExecutor(context=owner.context)
        for key,cls,topic,_ in specifications:
            def receive(msg,key=key):
                ros_now,mono=owner.clocks()
                self.buffer.put(key,msg,ros_now,mono)
            self.node.create_subscription(cls,topic,receive,qos_profile_sensor_data)
        self.executor.add_node(self.node)
        self.worker=threading.Thread(target=self._run,name=name,daemon=True)
        self.worker.start()

    def _run(self):
        try:self.executor.spin()
        except self._external_shutdown:pass
        except Exception as exc:
            if not self.closed:self.error=type(exc).__name__+': '+str(exc)

    def drain(self):return self.buffer.drain()
    def report(self,mono):
        return dict(mode='dedicated_state_receiver',thread_alive=self.worker.is_alive(),
                    executor_error=self.error,channels=self.buffer.report(mono))

    def close(self):
        if self.closed:return
        self.closed=True
        self.executor.shutdown(timeout_sec=1.)
        self.worker.join(timeout=1.)
        if self.worker.is_alive():
            self.error='State receiver did not stop within shutdown deadline'
            return
        self.executor.remove_node(self.node)
        self.node.destroy_node()

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from .pose_metrics import PoseStream


class FleetReceiver(Node):
    def __init__(self, config, name):
        super().__init__(name)
        self.config = config
        self.names = list(config['robots'])
        self.mocap = {n: PoseStream() for n in self.names}
        self.onboard = {n: PoseStream() for n in self.names}
        self.names_seen = []
        self.create_subscription(NamedPoseArray, '/poses', self.receive,
                                 qos_profile_sensor_data)
        for n in self.names:
            self.create_subscription(PoseStamped, f'/{n}/pose',
                                     lambda msg, n=n: self.onboard[n].add(msg.pose, msg.header.frame_id),
                                     qos_profile_sensor_data)

    def receive(self, msg):
        self.names_seen = [p.name for p in msg.poses]
        counts = {name: self.names_seen.count(name) for name in self.names}
        now = time.monotonic()
        for p in msg.poses:
            if p.name in self.mocap and counts[p.name] == 1:
                self.mocap[p.name].add(p.pose, msg.header.frame_id, now)

    def spin_for(self, seconds, guard=None):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=min(.02, max(0., until-time.monotonic())))
            if guard:
                guard()

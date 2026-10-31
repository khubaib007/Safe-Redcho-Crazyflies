import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from .pose_metrics import PoseStream


class Receiver(Node):
    def __init__(self, lab, node_name):
        super().__init__(node_name)
        self.lab = lab
        self.robot = lab['robot']['name']
        self.mocap = PoseStream()
        self.onboard = PoseStream()
        self.names = []
        self.create_subscription(NamedPoseArray, '/poses', self.receive,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, f'/{self.robot}/pose', self.receive_estimator,
                                 qos_profile_sensor_data)

    def receive(self, msg):
        self.names = sorted(p.name for p in msg.poses)
        selected = [p for p in msg.poses if p.name == self.robot]
        if len(selected) == 1:
            self.mocap.add(selected[0].pose, msg.header.frame_id)

    def receive_estimator(self, msg):
        self.onboard.add(msg.pose, msg.header.frame_id)

    def spin_for(self, seconds):
        import time
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.02, max(0.0, end-time.monotonic())))

#!/usr/bin/env python3
"""Read-only display of /poses: XYZ metres, roll/pitch/yaw degrees."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from motion_capture_tracking_interfaces.msg import NamedPoseArray


class MocapPoseDisplay(Node):
    def __init__(self):
        super().__init__('mocap_pose_display')
        self.latest = {}
        self.create_subscription(
            NamedPoseArray, '/poses', self.receive,
            qos_profile_sensor_data)
        self.create_timer(1.0, self.display)
        self.get_logger().info('Listening to /poses')

    def receive(self, msg):
        now = time.monotonic()
        counts = {}
        for body in msg.poses:
            counts[body.name] = counts.get(body.name, 0) + 1

        for body in msg.poses:
            if counts[body.name] != 1:
                self.latest.pop(body.name, None)
                continue

            p, q = body.pose.position, body.pose.orientation
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            if not all(math.isfinite(v) for v in values):
                continue

            norm = math.sqrt(sum(v*v for v in (q.x, q.y, q.z, q.w)))
            if norm < 1e-9:
                continue
            x, y, z, w = (v/norm for v in (q.x, q.y, q.z, q.w))

            # ZYX convention: R = Rz(yaw) Ry(pitch) Rx(roll).
            roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
            pitch = math.asin(max(-1., min(1., 2*(w*y-z*x))))
            yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

            self.latest[body.name] = (
                now, msg.header.frame_id,
                p.x, p.y, p.z,
                math.degrees(roll),
                math.degrees(pitch),
                math.degrees(yaw))

    def display(self):
        if not self.latest:
            print('Waiting for /poses ...', flush=True)
            return

        print('\nPosition [m] | Angles [deg]')
        print(f'{"Name":<10} {"Frame":<12} {"X":>8} {"Y":>8} {"Z":>8}'
              f' {"Roll":>9} {"Pitch":>9} {"Yaw":>9}')
        now = time.monotonic()

        for name, row in sorted(self.latest.items()):
            stamp, frame, x, y, z, roll, pitch, yaw = row
            if now-stamp > 0.5:
                print(f'{name:<10} STALE: no recent received pose')
                continue
            print(f'{name:<10} {frame:<12} {x:8.3f} {y:8.3f} {z:8.3f}'
                  f' {roll:9.2f} {pitch:9.2f} {yaw:9.2f}')
        print(flush=True)


def main():
    rclpy.init()
    node = MocapPoseDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

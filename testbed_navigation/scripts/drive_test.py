#!/usr/bin/env python3
"""
Command fixed velocities and compare them with the robot's actual motion.

The Gazebo diff-drive plugin's default odometry source is the world pose, so
/odom is ground truth here: any gap between commanded and measured motion is
real (wheel slip, wrong wheel_separation, rotation centre off base_footprint).

Run with the simulation up:
  ros2 run testbed_navigation drive_test.py
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class DriveTest(Node):

    def __init__(self):
        super().__init__('drive_test', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.odom = None
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def on_odom(self, msg):
        self.odom = msg

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def spin_for(self, secs, twist=None):
        start = self.now()
        while self.now() - start < secs:
            if twist is not None:
                self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

    def pose(self):
        p = self.odom.pose.pose
        return p.position.x, p.position.y, yaw(p.orientation)

    def run(self, name, vx, wz, secs):
        self.spin_for(1.0, Twist())
        x0, y0, th0 = self.pose()
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = vx, wz
        self.spin_for(secs, cmd)
        self.spin_for(1.0, Twist())
        x1, y1, th1 = self.pose()
        dist = math.hypot(x1 - x0, y1 - y0)
        dth = math.atan2(math.sin(th1 - th0), math.cos(th1 - th0))
        print(f'{name}: commanded {abs(vx) * secs:.2f} m / {math.degrees(wz * secs):.0f} deg, '
              f'measured {dist:.2f} m / {math.degrees(dth):.0f} deg', flush=True)


def main():
    rclpy.init()
    node = DriveTest()
    end = node.now() + 10
    while node.odom is None and node.now() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.odom is None:
        print('no /odom received: is the simulation running?')
        return
    node.run('forward 0.2 m/s x 5 s', 0.2, 0.0, 5.0)
    node.run('turn 0.5 rad/s x 3 s', 0.0, 0.5, 3.0)
    node.run('back 0.2 m/s x 5 s', -0.2, 0.0, 5.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

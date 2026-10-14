#!/usr/bin/env python3
"""
Drive a fixed route and measure AMCL's localization error.

/odom from the Gazebo diff-drive plugin is the true world pose and the map
frame coincides with the world frame, so the map -> odom transform that AMCL
publishes IS its error. Particle spread comes from /amcl_pose covariance.

Run with the simulation and localization.launch.py up:
  ros2 run testbed_navigation localization_test.py
"""
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class AmclTest(Node):

    def __init__(self):
        super().__init__('amcl_test', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.odom = self.amcl = None
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 lambda m: setattr(self, 'amcl', m), qos)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def drive(self, vx, wz, secs):
        t = Twist()
        t.linear.x, t.angular.z = vx, wz
        start = self.now()
        while self.now() - start < secs:
            self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
        start = self.now()
        while self.now() - start < 1.5:
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)

    def report(self, label):
        # odom is the true world pose and map == world, so map->odom IS the error
        t = self.tf.lookup_transform('map', 'odom', rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        err = math.hypot(tr.x, tr.y)
        o = self.odom.pose.pose.position
        cov = self.amcl.pose.covariance if self.amcl else [0.0] * 36
        spread = math.sqrt(max(cov[0], 0) + max(cov[7], 0))
        print(f'{label:<26} truth=({o.x:6.2f},{o.y:6.2f})  localization error={err:.3f} m '
              f'{math.degrees(yaw(q)):+5.1f} deg  particle spread={spread:.3f} m', flush=True)


def main():
    rclpy.init()
    n = AmclTest()
    end = n.now() + 20
    while n.odom is None and n.now() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.drive(0.0, 0.0, 0.5)
    n.report('start (before moving)')
    route = [('spin left 180 deg', 0.0, 0.6, 5.24), ('forward 1.5 m', 0.25, 0.0, 6.0),
             ('turn left 90 deg', 0.0, 0.6, 2.62), ('forward 1.0 m', 0.25, 0.0, 4.0),
             ('spin right 180 deg', 0.0, -0.6, 5.24), ('forward 1.0 m', 0.25, 0.0, 4.0),
             ('spin left 360 deg', 0.0, 0.6, 10.47), ('forward 1.0 m', 0.25, 0.0, 4.0)]
    for label, vx, wz, secs in route:
        n.drive(vx, wz, secs)
        n.report(f'after {label}')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

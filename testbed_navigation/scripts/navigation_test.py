#!/usr/bin/env python3
"""
Send a route of NavigateToPose goals and report how each one went.

For every goal: result status, time taken, distance driven, recoveries used,
the final position error and the worst localization error on the way. /odom is
the true world pose in this simulation and the map frame is the world frame,
so the map -> odom transform from AMCL is its localization error. The driven
path, with the localization error at each point, can be saved to a CSV.

Run with the simulation, localization.launch.py and navigation.launch.py up:
  ros2 run testbed_navigation navigation_test.py            # default route
  ros2 run testbed_navigation navigation_test.py --csv path.csv -- -6,5 2,-3
"""
import argparse
import csv
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

# A tour of the testbed world: the left room through the wide doorway, the
# lower rooms through the 0.9 m gaps, the enclosed corner, and back to spawn.
DEFAULT_ROUTE = [(-6.0, 5.0), (2.0, -3.0), (7.5, -6.5), (-6.0, -5.0), (0.0, 5.0)]
STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
          GoalStatus.STATUS_CANCELED: 'CANCELED'}


class NavigationTest(Node):

    def __init__(self):
        super().__init__('navigation_test', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.odom = None
        self.trail = []
        self.recoveries = 0
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def on_odom(self, msg):
        self.odom = msg
        p = msg.pose.pose.position
        if not self.trail or math.hypot(p.x - self.trail[-1][1], p.y - self.trail[-1][2]) > 0.05:
            self.trail.append((self.now(), p.x, p.y, self.localization_error()))

    def localization_error(self):
        try:
            t = self.tf.lookup_transform('map', 'odom', Time()).transform.translation
            return math.hypot(t.x, t.y)
        except Exception:  # no map -> odom yet
            return float('nan')

    def on_feedback(self, msg):
        self.recoveries = msg.feedback.number_of_recoveries

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def wait(self, future, timeout):
        end = self.now() + timeout
        while not future.done() and self.now() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    def go(self, x, y, timeout=240.0):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.w = 1.0
        self.recoveries = 0
        start_t, start_len, start_i = self.now(), self.path_length(), len(self.trail)
        send = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        if not self.wait(send, 10.0) or not send.result().accepted:
            return 'REJECTED', 0.0, 0.0, 0, float('nan'), float('nan')
        result = send.result().get_result_async()
        if not self.wait(result, timeout):
            send.result().cancel_goal_async()
            status = 'TIMED OUT'
        else:
            status = STATUS.get(result.result().status, str(result.result().status))
        p = self.odom.pose.pose.position
        errs = [pt[3] for pt in self.trail[start_i:] if not math.isnan(pt[3])]
        return (status, self.now() - start_t, self.path_length() - start_len,
                self.recoveries, math.hypot(p.x - x, p.y - y), max(errs, default=float('nan')))

    def path_length(self):
        return sum(math.hypot(b[1] - a[1], b[2] - a[2])
                   for a, b in zip(self.trail, self.trail[1:]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv',
                        help='save the driven path (t, x, y, localization error) to this file')
    parser.add_argument('goals', nargs='*', help='goals as x,y (default: the tour)')
    args = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    route = [tuple(map(float, g.split(','))) for g in args.goals] or DEFAULT_ROUTE

    rclpy.init()
    node = NavigationTest()
    if not node.client.wait_for_server(timeout_sec=20.0):
        print('navigate_to_pose action not available: is navigation.launch.py up?')
        return
    # Wait for /odom and for /clock: until the first /clock message arrives,
    # sim time reads 0 and every timeout would expire at once.
    while node.odom is None or node.now() == 0.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f'{"goal":<14}{"result":<11}{"time":>8}{"driven":>9}{"recoveries":>12}'
          f'{"final error":>13}{"worst AMCL error":>18}')
    ok = 0
    for x, y in route:
        status, secs, dist, rec, err, loc = node.go(x, y)
        ok += status == 'SUCCEEDED'
        print(f'({x:5.1f},{y:5.1f})  {status:<11}{secs:7.1f}s{dist:8.2f}m{rec:12d}{err:12.3f}m'
              f'{loc:17.3f}m', flush=True)
    print(f'{ok} of {len(route)} goals reached; {node.path_length():.1f} m driven in total')

    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            csv.writer(f).writerows([('t', 'x', 'y', 'localization_error')] + node.trail)
        print('path saved to', args.csv)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
Start the Nav2 navigation servers by hand (no nav2_bringup).

Velocity commands go controller_server -> /cmd_vel_nav -> velocity_smoother
-> /cmd_vel, so the robot only ever receives smoothed, limited commands.
Needs a map -> odom transform: run localization.launch.py first.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Every lifecycle node in this file, in start-up order.
LIFECYCLE_NODES = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'velocity_smoother',
]


def generate_launch_description():
    pkg_nav = get_package_share_directory('testbed_navigation')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params = [params_file, {'use_sim_time': use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_nav, 'config', 'nav2_params.yaml'),
            description='Nav2 parameter file'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo /clock'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Configure and activate the nodes on start'),

        # Follows the path; also hosts the local costmap
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=params,
            remappings=[('cmd_vel', 'cmd_vel_nav')]),

        # Computes the global path; also hosts the global costmap
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=params),

        # Recovery actions: spin, back up, wait
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=params),

        # Runs the behaviour tree; serves the navigate_to_pose action
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=params),

        # Limits speed and acceleration before commands reach the robot
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=params,
            remappings=[('cmd_vel', 'cmd_vel_nav'),
                        ('cmd_vel_smoothed', 'cmd_vel')]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': autostart,
                         'node_names': LIFECYCLE_NODES}]),
    ])

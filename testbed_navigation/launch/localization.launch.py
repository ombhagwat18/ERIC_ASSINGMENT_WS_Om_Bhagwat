"""
Localize the robot on the map with nav2_amcl.

AMCL needs /map, so this file starts map_loader.launch.py too unless
load_map:=false (for example when the map server is already running).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('testbed_navigation')
    pkg_bringup = get_package_share_directory('testbed_bringup')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    load_map = LaunchConfiguration('load_map')

    map_loader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'map_loader.launch.py')),
        condition=IfCondition(load_map),
        launch_arguments={'map': map_yaml,
                          'use_sim_time': use_sim_time,
                          'autostart': autostart}.items())

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_bringup, 'maps', 'testbed_world.yaml'),
            description='Full path to the map YAML file'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_nav, 'config', 'amcl_params.yaml'),
            description='AMCL parameter file'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo /clock'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Configure and activate the nodes on start'),
        DeclareLaunchArgument(
            'load_map', default_value='true',
            description='Also start map_loader.launch.py'),

        map_loader,

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': autostart,
                         'node_names': ['amcl']}]),
    ])

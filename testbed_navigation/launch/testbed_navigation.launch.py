"""
One command for the whole stack: simulation, map, localization, navigation.

  ros2 launch testbed_navigation testbed_navigation.launch.py

The three layers are the separate launch files in this package; this file only
includes them. Pass sim:=false to run against a simulation that is already up.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_nav = get_package_share_directory('testbed_navigation')
    pkg_bringup = get_package_share_directory('testbed_bringup')

    def include(pkg_dir, name, args, condition=None):
        # Launch arguments are not scoped per included file: without the
        # scoped group, the params_file that localization.launch.py declares
        # (amcl_params.yaml) would also be used by navigation.launch.py.
        return GroupAction(scoped=True, condition=condition, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', name)),
                launch_arguments=args.items())])

    common = {'use_sim_time': 'true', 'autostart': LaunchConfiguration('autostart')}

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', default_value='true',
            description='Also start Gazebo, the robot and RViz (testbed_full_bringup)'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_bringup, 'maps', 'testbed_world.yaml'),
            description='Full path to the map YAML file'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Configure and activate the Nav2 nodes on start'),

        # Simulation, with RViz showing the navigation view
        include(pkg_bringup, 'testbed_full_bringup.launch.py',
                {'rvizconfig': os.path.join(pkg_nav, 'rviz', 'navigation.rviz')},
                condition=IfCondition(LaunchConfiguration('sim'))),

        # map_server + AMCL (localization.launch.py includes map_loader.launch.py)
        include(pkg_nav, 'localization.launch.py',
                dict(common, map=LaunchConfiguration('map'))),

        # planner, controller, behaviours, BT navigator, velocity smoother
        include(pkg_nav, 'navigation.launch.py', common),
    ])

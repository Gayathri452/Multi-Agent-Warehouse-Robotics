"""Nav2 navigation stack for ONE robot, inside the /<robot> namespace.

  ros2 launch ugp_warehouse nav2.launch.py robot:=robot_2

Uses config/nav2_params_<robot>.yaml (made by make_nav2_params.py).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushROSNamespace


def launch_setup(context):
    robot = LaunchConfiguration('robot').perform(context)
    params = os.path.join(get_package_share_directory('ugp_warehouse'),
                          'config', 'nav2_params_%s.yaml' % robot)
    nav2 = os.path.join(get_package_share_directory('nav2_bringup'),
                        'launch', 'navigation_launch.py')
    return [GroupAction([
        PushROSNamespace(robot),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2),
            launch_arguments={
                'namespace': robot,
                'use_sim_time': 'true',
                'autostart': 'true',
                'params_file': params,
                'use_composition': 'False',
            }.items()),
    ])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='robot_1'),
        OpaqueFunction(function=launch_setup),
    ])

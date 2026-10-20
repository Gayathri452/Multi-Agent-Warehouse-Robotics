import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushROSNamespace

ROBOT = 'robot_1'


def generate_launch_description():
    pkg = get_package_share_directory('ugp_warehouse')
    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch', 'navigation_launch.py')
    params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    navigation = GroupAction([
        PushROSNamespace(ROBOT),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'namespace': ROBOT,
                'use_sim_time': 'true',
                'autostart': 'true',
                'params_file': params,
                'use_composition': 'False',
            }.items(),
        ),
    ])
    return LaunchDescription([navigation])

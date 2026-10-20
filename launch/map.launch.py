import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('ugp_warehouse')
    map_yaml = os.path.join(pkg, 'config', 'warehouse_map.yaml')

    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server', output='screen',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': True}],
    )
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': ['map_server']}],
    )
    return LaunchDescription([map_server, lifecycle])

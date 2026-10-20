import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# Spawn pose of robot_1 (must match warehouse.launch.py)
NAME, X, Y, YAW = 'robot_1', -8.0, 0.0, 0.0

def generate_launch_description():
    pkg = get_package_share_directory('ugp_warehouse')
    params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl',
        namespace=NAME, output='screen',
        parameters=[params, {
            'use_sim_time': True,
            'set_initial_pose': True,
            'odom_frame_id': NAME + '/odom',
            'base_frame_id': NAME + '/base_link',
            'global_frame_id': 'map',
            'initial_pose.x': X, 'initial_pose.y': Y,
            'initial_pose.z': 0.0, 'initial_pose.yaw': YAW,
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'),
                    ('map', '/map')],
    )
    manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', namespace=NAME,
        output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': ['amcl']}],
    )
    return LaunchDescription([amcl, manager])

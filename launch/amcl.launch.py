"""AMCL localization for ONE robot (publishes map -> <robot>/odom).

  ros2 launch ugp_warehouse amcl.launch.py robot:=robot_2 x:=-8.3 y:=-1.0 yaw:=-1.5708

Defaults match robot_1. x, y, yaw are the robot's spawn pose = AMCL's initial pose.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    robot = LaunchConfiguration('robot').perform(context)
    x = float(LaunchConfiguration('x').perform(context))
    y = float(LaunchConfiguration('y').perform(context))
    yaw = float(LaunchConfiguration('yaw').perform(context))
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl',
        namespace=robot, output='screen',
        parameters=[{
            'use_sim_time': True,
            'set_initial_pose': True,
            'initial_pose.x': x, 'initial_pose.y': y,
            'initial_pose.z': 0.0, 'initial_pose.yaw': yaw,
            'odom_frame_id': robot + '/odom',
            'base_frame_id': robot + '/base_link',
            'global_frame_id': 'map',
        }],
        # per-robot TF topics; all robots share the one map server at /map
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'), ('map', '/map')],
    )
    manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', namespace=robot, output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['amcl']}],
    )
    return [amcl, manager]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='robot_1'),
        DeclareLaunchArgument('x', default_value='-8.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=launch_setup),
    ])

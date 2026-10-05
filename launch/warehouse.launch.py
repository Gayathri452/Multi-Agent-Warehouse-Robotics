import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# (name, x, y, yaw) -- add more robots here in Phase 4
ROBOTS = [
    ('robot_1', -8.0, 0.0, 0.0),
]


def static_tf(x, y, z, yaw, parent, child):
    return Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_' + child.replace('/', '_'),
        arguments=['--x', str(x), '--y', str(y), '--z', str(z),
                   '--yaw', str(yaw), '--pitch', '0', '--roll', '0',
                   '--frame-id', parent, '--child-frame-id', child],
    )


def generate_launch_description():
    pkg = get_package_share_directory('ugp_warehouse')
    world = os.path.join(pkg, 'worlds', 'warehouse.sdf')
    template = os.path.join(pkg, 'models', 'robot.sdf.template')
    with open(template) as f:
        template_text = f.read()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world}'}.items(),
    )

    actions = [gz_sim]
    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    remaps = []

    for name, x, y, yaw in ROBOTS:
        sdf = template_text.replace('__NAME__', name)
        spawn = Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=['-world', 'warehouse', '-name', name, '-string', sdf,
                       '-x', str(x), '-y', str(y), '-z', '0.01', '-Y', str(yaw)],
        )
        actions.append(TimerAction(period=3.0, actions=[spawn]))
        bridge_args += [
            f'/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            f'/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/{name}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ]
        remaps.append((f'/{name}/tf', '/tf'))
        actions += [
            static_tf(x, y, 0.0, yaw, 'map', f'{name}/odom'),
            static_tf(0.2, 0.0, 0.12, 0.0, f'{name}/base_link', f'{name}/lidar_link'),
            static_tf(0.0, 0.0, 0.0, 0.0, f'{name}/base_link', f'{name}/imu_link'),
        ]

    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                        arguments=bridge_args, remappings=remaps,
                        output='screen'))
    return LaunchDescription(actions)

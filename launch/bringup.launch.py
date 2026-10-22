"""Start the whole system with one command:

  ros2 launch ugp_warehouse bringup.launch.py

Order matters (each stage needs the one before), so stages start on timers:
   0 s  Gazebo + robots + bridges
  12 s  map server + TF relay (for RViz)
  16 s  AMCL for every robot            (publishes map -> <robot>/odom)
  30 s  Nav2 for every robot, 6 s apart (needs the map frame from AMCL)
Robots come from ugp_warehouse/robots.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ugp_warehouse.robots import ROBOTS


def include(name, **args):
    path = os.path.join(get_package_share_directory('ugp_warehouse'), 'launch', name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments={k: str(v) for k, v in args.items()}.items())


def generate_launch_description():
    sim = include('warehouse.launch.py')
    map_and_tf = [
        include('map.launch.py'),
        Node(package='ugp_warehouse', executable='tf_relay', name='tf_relay',
             output='screen',
             parameters=[{'use_sim_time': True, 'robots': [r[0] for r in ROBOTS]}]),
    ]
    amcl = [include('amcl.launch.py', robot=n, x=x, y=y, yaw=yaw)
            for n, x, y, yaw in ROBOTS]
    nav2 = [TimerAction(period=6.0 * i, actions=[include('nav2.launch.py', robot=r[0])])
            for i, r in enumerate(ROBOTS)]
    return LaunchDescription([
        sim,
        TimerAction(period=12.0, actions=map_and_tf),
        TimerAction(period=24.0, actions=amcl),
        TimerAction(period=38.0, actions=nav2),
    ])

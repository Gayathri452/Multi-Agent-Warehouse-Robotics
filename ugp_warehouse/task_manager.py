"""task_manager: creates pickup/delivery tasks, assigns them to robots, tracks them.

Roles in the project pipeline (kept separate on purpose):
  * Task manager (this node) : owns the task list and each task's state.
  * Assignment (tasks.py)    : decides WHO does a task (A* cost + load balancing).
  * A* (astar_nav.plan_waypoints): decides HOW the robot gets there (waypoints).
  * Nav2                     : drives the robot along those waypoints.

A task has two legs: robot -> pickup, then pickup -> delivery.
States: PENDING -> TO_PICKUP -> TO_DELIVERY -> DONE   (or FAILED).

Run:
  ros2 run ugp_warehouse task_manager --ros-args -p use_sim_time:=true
  ros2 run ugp_warehouse task_manager --ros-args -p use_sim_time:=true \
      -p tasks:="['P1:D1','P2:D2']"

ROS interfaces
  subscribes : /<robot>/amcl_pose   geometry_msgs/PoseWithCovarianceStamped
  action     : /<robot>/navigate_through_poses  nav2_msgs/NavigateThroughPoses
  publishes  : /task_status   std_msgs/String (JSON, 1 Hz): every task and robot state
"""
import json
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from ugp_warehouse.astar_nav import plan_waypoints
from ugp_warehouse.grid_map import GridMap
from ugp_warehouse.locations import LOCATIONS
from ugp_warehouse.robots import ROBOTS
from ugp_warehouse.tasks import (DONE, FAILED, PENDING, TO_DELIVERY, TO_PICKUP,
                                 Task, assign)

SUCCEEDED = 4                                     # action_msgs GoalStatus.STATUS_SUCCEEDED


class Robot:
    def __init__(self, name, xy, client):
        self.name = name
        self.xy = xy                              # updated from AMCL
        self.client = client
        self.task = None                          # the Task it is working on, or None (idle)
        self.completed = 0


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.declare_parameter('tasks', ['P1:D1', 'P2:D2', 'P1:D2', 'P2:D1'])
        self.declare_parameter('map_yaml', os.path.join(
            get_package_share_directory('ugp_warehouse'), 'config', 'warehouse_map.yaml'))
        self.grid = GridMap.from_yaml(self.get_parameter('map_yaml').value)
        self.tasks = []
        for i, spec in enumerate(self.get_parameter('tasks').value, start=1):
            pickup, delivery = spec.split(':')
            self.tasks.append(Task(i, pickup, delivery))

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.robots = {}
        for name, x, y, _ in ROBOTS:
            client = ActionClient(self, NavigateThroughPoses,
                                  '/%s/navigate_through_poses' % name)
            self.robots[name] = Robot(name, (x, y), client)
            self.create_subscription(
                PoseWithCovarianceStamped, '/%s/amcl_pose' % name,
                lambda msg, n=name: self.on_pose(n, msg), qos)
        self.status_pub = self.create_publisher(String, '/task_status', 10)
        self.finished_logged = False
        self.create_timer(1.0, self.tick)
        self.get_logger().info('Task manager: %d tasks, robots %s'
                               % (len(self.tasks), list(self.robots)))

    # ---- inputs -----------------------------------------------------------
    def on_pose(self, name, msg):
        p = msg.pose.pose.position
        self.robots[name].xy = (p.x, p.y)

    # ---- main loop (1 Hz): hand pending tasks to idle robots ----------------
    def tick(self):
        idle = {n: {'xy': r.xy, 'completed': r.completed}
                for n, r in self.robots.items()
                if r.task is None and r.client.server_is_ready()}
        pending = [t for t in self.tasks if t.state == PENDING]
        if idle and pending:
            assigned, infeasible = assign(self.grid, pending, idle, LOCATIONS)
            for t in infeasible:
                t.state = FAILED
                self.get_logger().error('Task %d (%s->%s) FAILED: no robot has a path'
                                        % (t.id, t.pickup, t.delivery))
            for task, name, cost in assigned:
                self.start_task(task, name, cost)
        self.publish_status()
        if not self.finished_logged and all(t.state in (DONE, FAILED) for t in self.tasks):
            self.finished_logged = True
            self.get_logger().info('All tasks finished: %d done, %d failed' % (
                sum(t.state == DONE for t in self.tasks),
                sum(t.state == FAILED for t in self.tasks)))

    def start_task(self, task, name, cost):
        self.get_logger().info('Task %d (%s->%s) assigned to %s (cost %.1f m)'
                               % (task.id, task.pickup, task.delivery, name, cost))
        task.robot = name
        task.state = TO_PICKUP
        self.robots[name].task = task
        self.send_leg(name, LOCATIONS[task.pickup])

    # ---- one leg = A* waypoints -> Nav2 -------------------------------------
    def send_leg(self, name, target_xy):
        robot = self.robots[name]
        waypoints = plan_waypoints(self.grid, robot.xy, target_xy, 0.0)
        if waypoints is None:
            return self.fail(name, 'A* found no path')
        if not waypoints:
            return self.leg_done(name)             # already there
        if not robot.client.server_is_ready():
            return self.fail(name, 'Nav2 action server not available')
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self.make_pose(*w) for w in waypoints]
        future = robot.client.send_goal_async(goal)
        future.add_done_callback(lambda f, n=name: self.on_goal_response(n, f))

    def on_goal_response(self, name, future):
        handle = future.result()
        if not handle.accepted:
            return self.fail(name, 'Nav2 rejected the goal')
        handle.get_result_async().add_done_callback(
            lambda f, n=name: self.on_result(n, f))

    def on_result(self, name, future):
        status = future.result().status
        if status == SUCCEEDED:
            self.leg_done(name)
        else:
            self.fail(name, 'Nav2 ended with status %d' % status)

    def leg_done(self, name):
        robot = self.robots[name]
        task = robot.task
        if task.state == TO_PICKUP:
            self.get_logger().info('Task %d: %s picked up at %s' % (task.id, name, task.pickup))
            task.state = TO_DELIVERY
            self.send_leg(name, LOCATIONS[task.delivery])
        elif task.state == TO_DELIVERY:
            self.get_logger().info('Task %d: DONE, %s delivered at %s'
                                   % (task.id, name, task.delivery))
            task.state = DONE
            robot.task = None
            robot.completed += 1

    def fail(self, name, reason):
        robot = self.robots[name]
        task = robot.task
        self.get_logger().error('Task %d FAILED on %s: %s' % (task.id, name, reason))
        task.state = FAILED            # Phase 7 will release it for reassignment instead
        robot.task = None

    # ---- helpers ------------------------------------------------------------
    def make_pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'tasks': [{'id': t.id, 'pickup': t.pickup, 'delivery': t.delivery,
                       'state': t.state, 'robot': t.robot} for t in self.tasks],
            'robots': {n: {'task': r.task.id if r.task else None, 'completed': r.completed}
                       for n, r in self.robots.items()},
        })
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

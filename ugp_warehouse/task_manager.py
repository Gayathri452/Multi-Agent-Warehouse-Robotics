"""
Create and assign pickup/delivery tasks, then track robot progress.

Roles in the project pipeline (kept separate on purpose):
  * Task manager (this node) : owns the task list and each task's state.
  * Assignment (tasks.py)    : decides WHO does a task (A* cost + load balancing).
  * Coordinator (coordinator.py): schedules pickup and delivery routes.
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
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from ugp_warehouse.astar import simplify
from ugp_warehouse.astar_nav import plan_waypoints
from ugp_warehouse.grid_map import GridMap
from ugp_warehouse.locations import LOCATIONS
from ugp_warehouse.robots import ROBOTS
from ugp_warehouse.tasks import (assign, DONE, FAILED, PENDING, Task,
                                 TO_DELIVERY, TO_PICKUP)

SUCCEEDED = 4                                     # action_msgs GoalStatus.STATUS_SUCCEEDED


class Robot:
    def __init__(self, name, xy, client):
        self.name = name
        self.xy = xy                              # updated from AMCL
        self.client = client
        self.task = None                          # the Task it is working on, or None (idle)
        self.completed = 0
        self.segments = []
        self.segment_index = 0


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.declare_parameter('tasks', ['P1:D1', 'P2:D2', 'P1:D2', 'P2:D1'])
        self.declare_parameter('map_yaml', os.path.join(
            get_package_share_directory('ugp_warehouse'), 'config', 'warehouse_map.yaml'))
        self.grid = GridMap.from_yaml(self.get_parameter('map_yaml').value)
        self.declare_parameter('time_step_seconds', 2.0)
        self.time_step_seconds = self.get_parameter('time_step_seconds').value
        self.declare_parameter('coordination_algorithm', 'cbs')
        self.declare_parameter('coordination_timeout_seconds', 30.0)
        self.coordination_algorithm = self.get_parameter(
            'coordination_algorithm').value
        self.coordination_timeout_seconds = self.get_parameter(
            'coordination_timeout_seconds').value
        self.coordination_pub = self.create_publisher(
            String, '/coordination/request', 10)
        self.create_subscription(
            String, '/coordination/response', self.on_coordination_response, 10)
        self.coordination_requests = {}
        self.request_sequence = 0
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
        self.publish_pending_coordination_requests()
        self.expire_coordination_requests()
        idle = {n: {'xy': r.xy, 'completed': r.completed}
                for n, r in self.robots.items()
                if r.task is None and r.client.server_is_ready()}
        pending = [t for t in self.tasks if t.state == PENDING]
        active = any(robot.task is not None for robot in self.robots.values())
        if idle and pending and not active:
            assigned, infeasible = assign(self.grid, pending, idle, LOCATIONS)
            for t in infeasible:
                t.state = FAILED
                self.get_logger().error('Task %d (%s->%s) FAILED: no robot has a path'
                                        % (t.id, t.pickup, t.delivery))
            started = []
            for task, name, cost in assigned:
                self.start_task(task, name, cost)
                started.append((task, name))
            if started:
                self.request_coordinated_routes(started, TO_PICKUP)
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

    def request_coordinated_routes(self, assignments, state):
        agents = {}
        task_by_robot = {}
        for task, name in assignments:
            start = self.grid.nearest_free(self.grid.world_to_cell(*self.robots[name].xy))
            target = task.pickup if state == TO_PICKUP else task.delivery
            goal = self.grid.world_to_cell(*LOCATIONS[target])
            if start is None:
                self.fail(name, 'no free start cell for coordinated planning')
                continue
            agents[name] = (start, goal)
            task_by_robot[name] = task
        if not agents:
            return
        self.request_sequence += 1
        request_id = '%s-%d' % (self.get_name(), self.request_sequence)
        self.coordination_requests[request_id] = {
            'tasks': task_by_robot,
            'state': state,
            'agents': agents,
            'deadline': time.monotonic() + self.coordination_timeout_seconds,
            'published': False,
        }

    def publish_pending_coordination_requests(self):
        if self.coordination_pub.get_subscription_count() == 0:
            return
        for request_id, request in self.coordination_requests.items():
            if request['published']:
                continue
            message = String()
            message.data = json.dumps({
                'request_id': request_id,
                'algorithm': self.coordination_algorithm,
                'agents': {
                    name: {'start': list(start), 'goal': list(goal)}
                    for name, (start, goal) in request['agents'].items()
                },
            })
            self.coordination_pub.publish(message)
            request['published'] = True

    def expire_coordination_requests(self):
        now = time.monotonic()
        expired = [
            request_id for request_id, request in self.coordination_requests.items()
            if now >= request['deadline']]
        for request_id in expired:
            request = self.coordination_requests.pop(request_id)
            for name in request['tasks']:
                self.fail(name, 'coordinator response timed out')

    def on_coordination_response(self, message):
        request_id = None
        try:
            response = json.loads(message.data)
            request_id = response['request_id']
            request = self.coordination_requests.get(request_id)
            if request is None:
                self.get_logger().warning(
                    'Ignoring unknown coordination response %s' % request_id)
                return
            paths = response.get('paths', {})
            unplanned = set(response.get('unplanned', []))
            if response.get('error'):
                raise ValueError(response['error'])
            if not isinstance(paths, dict):
                raise ValueError('coordinator paths must be an object')
            if not unplanned.issubset(request['tasks']):
                raise ValueError('coordinator returned an unknown robot')
            parsed_paths = {}
            for name, raw_path in paths.items():
                if name not in request['tasks']:
                    raise ValueError('coordinator returned unknown robot %s' % name)
                path = [tuple(cell) for cell in raw_path]
                if not path or any(
                        len(cell) != 2 or
                        any(type(value) is not int for value in cell)
                        for cell in path):
                    raise ValueError('coordinator returned an invalid path for %s' % name)
                parsed_paths[name] = path
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().error('Invalid coordination response: %s' % error)
            request = self.coordination_requests.pop(request_id, None)
            if request is not None:
                for name in request['tasks']:
                    self.fail(name, 'coordinator returned an invalid response')
            return

        self.coordination_requests.pop(request_id)
        for name in unplanned:
            self.fail(name, 'coordinator found no conflict-free route')
        for name, path in parsed_paths.items():
            if name not in unplanned:
                task = request['tasks'][name]
                target_key = task.pickup if request['state'] == TO_PICKUP else task.delivery
                self.send_leg(name, LOCATIONS[target_key], path)
        missing = set(request['tasks']) - unplanned - set(parsed_paths)
        for name in missing:
            self.fail(name, 'coordinator returned no route')

    # ---- one leg = A* waypoints -> Nav2 -------------------------------------
    def send_leg(self, name, target_xy, cell_path=None):
        robot = self.robots[name]
        if cell_path is None:
            waypoints = plan_waypoints(self.grid, robot.xy, target_xy, 0.0)
            if waypoints is None:
                return self.fail(name, 'A* found no path')
            if not waypoints:
                return self.leg_done(name)         # already there
            robot.segments = [(waypoints, 0)]
        else:
            robot.segments = self.make_segments(cell_path, target_xy)
            if not robot.segments:
                return self.leg_done(name)
        robot.segment_index = 0
        self.send_next_segment(name)

    def make_segments(self, path, target_xy):
        """Turn a scheduled cell path into Nav2 route chunks separated by waits."""
        chunks = []
        cells = [path[0]]
        wait_steps = 0
        for cell in path[1:]:
            if cell == cells[-1]:
                if len(cells) > 1:
                    chunks.append((cells, wait_steps))
                    cells = [cells[-1]]
                    wait_steps = 0
                wait_steps += 1
            else:
                cells.append(cell)
        if len(cells) > 1:
            chunks.append((cells, wait_steps))

        segments = []
        for index, (cells, delay) in enumerate(chunks):
            corners = simplify(cells)[1:]
            waypoints = []
            for point_index, cell in enumerate(corners):
                x, y = self.grid.cell_to_world(cell)
                yaw = 0.0
                if point_index < len(corners) - 1:
                    nx, ny = self.grid.cell_to_world(corners[point_index + 1])
                    yaw = math.atan2(ny - y, nx - x)
                if index == len(chunks) - 1 and point_index == len(corners) - 1:
                    x, y = target_xy
                waypoints.append((x, y, yaw))
            if waypoints:
                segments.append((waypoints, delay))
        return segments

    def send_next_segment(self, name):
        robot = self.robots[name]
        if robot.segment_index >= len(robot.segments):
            return self.leg_done(name)
        waypoints, wait_steps = robot.segments[robot.segment_index]
        if wait_steps:
            timer = None

            def resume():
                timer.cancel()
                self.send_action_segment(name, waypoints)

            timer = self.create_timer(
                wait_steps * self.time_step_seconds, resume)
            return
        self.send_action_segment(name, waypoints)

    def send_action_segment(self, name, waypoints):
        robot = self.robots[name]
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
            robot = self.robots[name]
            robot.segment_index += 1
            self.send_next_segment(name)
        else:
            self.fail(name, 'Nav2 ended with status %d' % status)

    def leg_done(self, name):
        robot = self.robots[name]
        task = robot.task
        if task.state == TO_PICKUP:
            self.get_logger().info('Task %d: %s picked up at %s' % (task.id, name, task.pickup))
            task.state = TO_DELIVERY
            active = [
                (robot.task, robot.name)
                for robot in self.robots.values()
                if robot.task is not None
            ]
            if active and all(active_task.state == TO_DELIVERY
                              for active_task, _ in active):
                self.request_coordinated_routes(active, TO_DELIVERY)
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

"""task_manager: creates pickup/delivery tasks, assigns them to robots, tracks them.

Roles in the project pipeline (kept separate on purpose):
  * Task manager (this node) : owns the task list and each task's state; talks to ROS.
  * Assignment (tasks.py)    : decides WHO does a task (A* cost + load balancing).
  * Fleet (fleet.py)         : plans every robot's current leg together (CBS, cbs.py)
                               and decides when each robot may drive on.
  * Nav2                     : drives the robot along those waypoints.

A task has two legs: robot -> pickup, then pickup -> delivery.
States: PENDING -> TO_PICKUP -> TO_DELIVERY -> DONE   (or FAILED).

Coordination (Phase 6). Whenever a leg starts or ends, all robots stop briefly
and are planned together from where they are. Each robot then drives in segments:
up to the last cell it may enter, then it waits until the robots it has to let
pass have moved on. Nothing waits on a timer, so Nav2 driving faster or slower
than planned cannot make robots meet.

Run:
  ros2 run ugp_warehouse task_manager --ros-args -p use_sim_time:=true
  ros2 run ugp_warehouse task_manager --ros-args -p use_sim_time:=true \\
      -p tasks:="['P2:D1','D1:P1']"          # opposing traffic: the deliberate conflict

Parameters
  goal_tolerance (0.25)  keep equal to Nav2's xy_goal_tolerance; sets the separation
  suboptimality  (1.1)   CBS may return a plan up to 10% longer, but finds it much faster
  settle_seconds (1.0)   pause after stopping the robots, before replanning
  stall_seconds  (20.0)  replan if someone waits and no robot got further for this long
  heartbeat_timeout (3.0)  s without odometry / Nav2 before a robot is out of service
  stuck_seconds  (25.0)  s without progress along the route before a robot is stuck
  revive_seconds (10.0)  s healthy again before a failed robot gets tasks again
  block_seconds  (120.0) s an area that blocked a robot is avoided
  max_task_attempts (3)  robots a task may be tried on before it is FAILED

Failures (Phase 7, health.py). Every robot has two heartbeats: its odometry from
Gazebo, and its Nav2 action servers being visible. If either stops for a few
seconds the robot is OUT OF SERVICE: its task goes back to the queue (another robot
takes it; a load already picked up must be picked up again), the robot stays where
it stopped and the others plan around it. If it comes back healthy for
revive_seconds it is BACK IN SERVICE. A robot that makes no progress along its
route (Nav2 aborts after being stuck, or no progress for stuck_seconds) marks the
area ahead of it as blocked for block_seconds and everyone replans around it. A
task is retried on up to max_task_attempts robots before it is marked FAILED.
Fault injection for testing: scripts/fail_robot.sh, revive_robot.sh,
block_route.sh, unblock_route.sh.

Nav2 hiccups are retried, not failed: a goal rejected while Nav2 is still starting
is resent every RETRY_DELAY seconds, and a drive Nav2 aborts (e.g. "Resulting plan
has 0 poses" when the controller loses track of its path) triggers a joint replan
from where the robots are, up to MAX_RETRIES times per leg.

Nav2 quirk handled here: navigate_through_poses drops a pose once the robot is
within 0.7 m of it (RemovePassedGoals). If that was the LAST pose, Nav2 aborts with
"Resulting plan has 0 poses". So one-pose segments use navigate_to_pose, and a
through-poses segment aborted within FINISH_RADIUS of its end is finished with
navigate_to_pose instead of failing the task.

ROS interfaces
  subscribes : /<robot>/amcl_pose   geometry_msgs/PoseWithCovarianceStamped
               /<robot>/odom        nav_msgs/Odometry (heartbeat)
  actions    : /<robot>/navigate_through_poses  nav2_msgs/NavigateThroughPoses
               /<robot>/navigate_to_pose        nav2_msgs/NavigateToPose
  publishes  : /task_status   std_msgs/String (JSON, 1 Hz): every task and robot state
               /<robot>/planned_path  nav_msgs/Path (latched): the robot's current plan
"""
import json
import math
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from std_msgs.msg import String

from ugp_warehouse.fleet import LEG, Fleet, default_min_sep
from ugp_warehouse.grid_map import GridMap
from ugp_warehouse.health import Supervisor
from ugp_warehouse.locations import LOCATIONS
from ugp_warehouse.robots import ROBOTS
from ugp_warehouse.tasks import (DONE, FAILED, PENDING, TO_DELIVERY, TO_PICKUP,
                                 Task, assign)

SUCCEEDED = 4                                     # action_msgs GoalStatus.STATUS_SUCCEEDED
FINISH_RADIUS = 1.0      # m: an abort this close to the segment end is Nav2's dropped last pose
MAX_RETRIES = 3          # Nav2 aborts tolerated per leg before the robot gives the task back
RETRY_DELAY = 3.0        # s to wait before resending a goal Nav2 rejected
MAX_REJECTIONS = 40      # about two minutes of rejections: Nav2 is not coming up
BLOCKED_AFTER = 8.0      # s: an abort after standing still this long means the route is blocked
UNREACHABLE_LIMIT = 60.0  # s a robot may be unable to reach its goal before giving the task back


class Robot:
    def __init__(self, name, xy, through_client, to_client):
        self.name = name
        self.xy = xy                              # updated from AMCL
        self.client = through_client              # navigate_through_poses
        self.to_client = to_client                # navigate_to_pose
        self.task = None                          # the Task it is working on, or None (idle)
        self.completed = 0
        self.gen = 0                              # bumped on every stop; older callbacks ignored
        self.handle = None                        # Nav2 goal handle of the segment in flight
        self.driving = False
        self.segment = None                       # waypoints of the segment in flight
        self.finishing = False                    # True while finishing it with navigate_to_pose
        self.retries = 0                          # Nav2 aborts on the current leg
        self.rejections = 0                       # goals rejected in a row (Nav2 starting up)
        self.hold_until = 0.0                     # do not send goals before this time
        self.unreachable_since = None             # first plan that could not reach its goal

    def ready(self):
        return self.client.server_is_ready() and self.to_client.server_is_ready()


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.declare_parameter('tasks', ['P1:D1', 'P2:D2', 'P1:D2', 'P2:D1'])
        self.declare_parameter('map_yaml', os.path.join(
            get_package_share_directory('ugp_warehouse'), 'config', 'warehouse_map.yaml'))
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('suboptimality', 1.1)
        self.declare_parameter('settle_seconds', 1.0)
        self.declare_parameter('stall_seconds', 20.0)
        self.declare_parameter('heartbeat_timeout', 3.0)
        self.declare_parameter('stuck_seconds', 25.0)
        self.declare_parameter('revive_seconds', 10.0)
        self.declare_parameter('block_seconds', 120.0)
        self.declare_parameter('max_task_attempts', 3)
        p = {n: self.get_parameter(n).value for n in (
            'heartbeat_timeout', 'stuck_seconds', 'revive_seconds', 'block_seconds',
            'max_task_attempts')}
        self.supervisor = Supervisor(
            [r[0] for r in ROBOTS], heartbeat_timeout=p['heartbeat_timeout'],
            nav_timeout=p['heartbeat_timeout'], stuck_timeout=p['stuck_seconds'],
            revive_after=p['revive_seconds'], block_seconds=p['block_seconds'],
            max_attempts=p['max_task_attempts'])
        self.grid = GridMap.from_yaml(self.get_parameter('map_yaml').value)
        self.settle = self.get_parameter('settle_seconds').value
        self.stall = self.get_parameter('stall_seconds').value
        sep = default_min_sep(self.grid.cell,
                              stop_tolerance=self.get_parameter('goal_tolerance').value)
        self.fleet = Fleet(self.grid, [r[0] for r in ROBOTS], min_sep=sep,
                           w=self.get_parameter('suboptimality').value)
        self.tasks = []
        for i, spec in enumerate(self.get_parameter('tasks').value, start=1):
            pickup, delivery = spec.split(':')
            self.tasks.append(Task(i, pickup, delivery))

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.robots = {}
        self.path_pubs = {}
        for name, x, y, _ in ROBOTS:
            through = ActionClient(self, NavigateThroughPoses,
                                   '/%s/navigate_through_poses' % name)
            to_pose = ActionClient(self, NavigateToPose, '/%s/navigate_to_pose' % name)
            self.robots[name] = Robot(name, (x, y), through, to_pose)
            self.fleet.on_pose(name, (x, y))
            self.create_subscription(
                PoseWithCovarianceStamped, '/%s/amcl_pose' % name,
                lambda msg, n=name: self.on_pose(n, msg), qos)
            self.create_subscription(
                Odometry, '/%s/odom' % name,
                lambda msg, n=name: self.supervisor.odom(n, self.now()),
                qos_profile_sensor_data)
            self.path_pubs[name] = self.create_publisher(Path, '/%s/planned_path' % name, qos)
        self.status_pub = self.create_publisher(String, '/task_status', 10)
        self.replan_reason = None                 # set when a joint replan is needed
        self.settle_until = None                  # robots stopped: plan once they settle
        self.signature, self.last_progress = None, 0.0
        self.waiting = {}                         # name -> robots it currently waits for
        self.method = ''
        self.plan_id = 0
        self.last_nobody_warning = -1e9
        self.finished_logged = False
        self.create_timer(1.0, self.tick)
        self.create_timer(0.2, self.control)
        self.get_logger().info('Task manager: %d tasks, robots %s, separation %.2f m'
                               % (len(self.tasks), list(self.robots), sep * self.grid.cell))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- inputs -----------------------------------------------------------
    def on_pose(self, name, msg):
        p = msg.pose.pose.position
        self.robots[name].xy = (p.x, p.y)
        self.fleet.on_pose(name, (p.x, p.y))

    # ---- task loop (1 Hz): hand pending tasks to idle robots ------------------
    def tick(self):
        idle = {n: {'xy': r.xy, 'completed': r.completed}
                for n, r in self.robots.items()
                if r.task is None and self.supervisor.is_up(n) and r.ready()}
        pending = [t for t in self.tasks if t.state == PENDING]
        if idle and pending:
            assigned, infeasible = assign(self.grid, pending, idle, LOCATIONS)
            for t in infeasible:
                t.state = FAILED
                self.get_logger().error('Task %d (%s->%s) FAILED: no robot has a path'
                                        % (t.id, t.pickup, t.delivery))
            for task, name, cost in assigned:
                self.start_task(task, name, cost)
            if assigned:
                self.request_replan('new task')
        self.publish_status()
        now = self.now()
        waiting = [t for t in self.tasks if t.state == PENDING]
        if waiting and any(r.state != 'starting' for r in self.supervisor.robots.values()) \
                and not any(self.supervisor.is_up(n) for n in self.robots) \
                and now - self.last_nobody_warning > 30.0:
            self.last_nobody_warning = now
            self.get_logger().warn('No robot in service: %d task(s) waiting until one is '
                                   'back (scripts/revive_robot.sh)' % len(waiting))
        if not self.finished_logged and all(t.state in (DONE, FAILED) for t in self.tasks):
            self.finished_logged = True
            handed = sum(self.supervisor.attempt(t.id) - 1 for t in self.tasks)
            self.get_logger().info('All tasks finished: %d done, %d failed%s' % (
                sum(t.state == DONE for t in self.tasks),
                sum(t.state == FAILED for t in self.tasks),
                ' (%d handed to another robot after a failure)' % handed if handed else ''))

    def start_task(self, task, name, cost):
        self.get_logger().info('Task %d (%s->%s) assigned to %s (cost %.1f m)'
                               % (task.id, task.pickup, task.delivery, name, cost))
        task.robot = name
        task.state = TO_PICKUP
        self.robots[name].task = task
        self.robots[name].retries = 0

    # ---- control loop (5 Hz): replan when needed, release robots that may drive ----
    def request_replan(self, reason):
        self.replan_reason = self.replan_reason or reason

    def control(self):
        now = self.now()
        self.check_health(now)
        if self.replan_reason is not None:
            reason, self.replan_reason = self.replan_reason, None
            moving = any(r.driving for r in self.robots.values())
            if moving:
                self.stop_all()
            self.settle_until = now + (self.settle if moving else 0.0)
            self.get_logger().info('Replanning (%s)%s'
                                   % (reason, ': stopping all robots' if moving else ''))
        if self.settle_until is not None:
            if now < self.settle_until:
                return
            self.settle_until = None
            self.replan(now)
        someone_waits = False
        for name, robot in self.robots.items():
            if robot.driving or not robot.ready() or now < robot.hold_until or \
                    not self.supervisor.is_up(name):
                continue
            what, data = self.fleet.next_segment(name)
            if what == 'drive':
                self.send_segment(name, data)
            elif what == 'done' and data == LEG:
                self.leg_done(name)
            elif what == 'wait':
                someone_waits = True
                if self.waiting.get(name) != data:
                    self.get_logger().info('%s waits for %s to pass' % (name, ', '.join(data)))
            elif robot.task is not None and name in self.fleet.legs and \
                    self.fleet.legs[name].kind != LEG:
                someone_waits = True              # deferred: its zone is taken for now
            self.waiting[name] = data if what == 'wait' else []
        signature = self.fleet.progress_signature()
        if signature != self.signature or not someone_waits:
            self.signature, self.last_progress = signature, now
        elif now - self.last_progress > self.stall:
            self.get_logger().warn('No robot got further for %.0f s' % self.stall)
            self.request_replan('stalled')

    def replan(self, now):
        goals = {n: self.leg_target(r) for n, r in self.robots.items()}
        urgent = sorted((r.task.id, n) for n, r in self.robots.items() if r.task)
        t0 = time.monotonic()
        blocked = self.supervisor.blocked(now)
        result = self.fleet.plan(goals, [n for _, n in urgent], obstacles=blocked,
                                 down=self.supervisor.down())
        self.plan_id += 1
        self.method = result['method']
        holding = [n for n in result['deferred'] if n not in result['staging']]
        extra = ''
        if any(not self.supervisor.is_up(n) for n in self.robots):
            extra += ', out of service %s' % sorted(self.supervisor.down())
        if blocked:
            extra += ', avoiding %d blocked cells' % len(blocked)
        self.get_logger().info(
            'Plan by %s in %.2f s: moving %s, queuing near a busy zone %s, holding %s, '
            'making way %s%s' % (result['method'], time.monotonic() - t0,
                                 result['planned'] or '-', result['staging'] or '-',
                                 holding or '-', result['make_way'] or '-', extra))
        for name, robot in self.robots.items():  # no way to its goal for too long?
            if robot.task is None or name not in result['unreachable']:
                robot.unreachable_since = None
            elif robot.unreachable_since is None:
                robot.unreachable_since = now
                self.get_logger().warn('%s cannot reach its goal right now (route or zone '
                                       'blocked); waiting' % name)
            elif now - robot.unreachable_since > UNREACHABLE_LIMIT:
                robot.unreachable_since = None
                self.give_back(name, '%s could not reach its goal for %.0f s'
                               % (name, UNREACHABLE_LIMIT))
        for name, pub in self.path_pubs.items():
            path = Path()
            path.header.frame_id = 'map'
            path.header.stamp = self.get_clock().now().to_msg()
            path.poses = [self.make_pose(*self.grid.cell_to_world(c), 0.0)
                          for c in self.fleet.legs[name].cells]
            pub.publish(path)
        self.signature, self.last_progress = None, now

    def leg_target(self, robot):
        task = robot.task
        if task is None:
            return None
        return LOCATIONS[task.pickup if task.state == TO_PICKUP else task.delivery]

    def stop_all(self):
        for robot in self.robots.values():
            robot.gen += 1
            if robot.handle is not None:
                robot.handle.cancel_goal_async()
            robot.handle = None
            robot.driving = False
        self.fleet.stop_all()

    # ---- one segment = waypoints -> Nav2 -------------------------------------
    def send_segment(self, name, waypoints):
        robot = self.robots[name]
        robot.segment = waypoints
        robot.finishing = False
        if len(waypoints) == 1:                   # one pose: navigate_to_pose has no quirk
            return self.send_to_pose(name, waypoints[0])
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self.make_pose(*w) for w in waypoints]
        self.send_goal(name, robot.client, goal)

    def send_to_pose(self, name, waypoint):
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(*waypoint)
        self.send_goal(name, self.robots[name].to_client, goal)

    def send_goal(self, name, client, goal):
        robot = self.robots[name]
        robot.driving = True
        gen = robot.gen
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda f, n=name, g=gen: self.on_goal_response(n, g, f))

    def on_goal_response(self, name, gen, future):
        robot = self.robots[name]
        handle = future.result()
        if gen != robot.gen:                      # sent before the last stop: drop it
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:                   # usually Nav2 still starting up: wait, resend
            robot.driving = False
            robot.rejections += 1
            if robot.rejections > MAX_REJECTIONS:
                robot.rejections = 0
                return self.on_events(self.supervisor.force_down(
                    name, 'Nav2 keeps rejecting goals', self.now()), self.now())
            if robot.rejections == 1:
                self.get_logger().warn('%s: Nav2 is not accepting goals yet (still starting?), '
                                       'retrying every %.0f s' % (name, RETRY_DELAY))
            self.fleet.cancel_segment(name)
            robot.hold_until = self.now() + RETRY_DELAY
            return
        robot.rejections = 0
        robot.handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, n=name, g=gen: self.on_result(n, g, f))

    def on_result(self, name, gen, future):
        robot = self.robots[name]
        if gen != robot.gen:
            return
        robot.handle = None
        robot.driving = False
        status = future.result().status
        if status == SUCCEEDED:
            self.fleet.on_segment_done(name)
            return
        end = robot.segment[-1] if robot.segment else None
        if end and not robot.finishing and \
                math.hypot(robot.xy[0] - end[0], robot.xy[1] - end[1]) <= FINISH_RADIUS:
            robot.finishing = True                # Nav2 dropped the last pose just short of it
            self.get_logger().info('%s: Nav2 stopped %.2f m short of the segment end, '
                                   'finishing with navigate_to_pose'
                                   % (name, math.hypot(robot.xy[0] - end[0],
                                                       robot.xy[1] - end[1])))
            return self.send_to_pose(name, end)
        now = self.now()
        if not self.supervisor.is_up(name):
            return                                # handled as a robot failure
        if self.supervisor.no_progress_for(name, now) >= BLOCKED_AFTER:
            self.mark_blocked(name, now, 'Nav2 gave up after it stopped moving')
        if robot.task is not None and robot.retries < MAX_RETRIES:
            robot.retries += 1
            self.get_logger().warn('%s: Nav2 aborted the drive (status %d), replanning and '
                                   'retrying (%d/%d)' % (name, status, robot.retries, MAX_RETRIES))
            return self.request_replan('%s retry' % name)
        self.give_back(name, 'Nav2 kept aborting (status %d)' % status)

    def leg_done(self, name):
        robot = self.robots[name]
        task = robot.task
        if task is None:
            return
        if task.state == TO_PICKUP:
            self.get_logger().info('Task %d: %s picked up at %s' % (task.id, name, task.pickup))
            task.state = TO_DELIVERY
            robot.retries = 0
            self.request_replan('%s picked up' % name)
        elif task.state == TO_DELIVERY:
            self.get_logger().info('Task %d: DONE, %s delivered at %s'
                                   % (task.id, name, task.delivery))
            task.state = DONE
            robot.task = None
            robot.completed += 1
            self.request_replan('%s delivered' % name)

    # ---- Phase 7: failures -----------------------------------------------------
    def check_health(self, now):
        for name, robot in self.robots.items():
            self.supervisor.nav(name, robot.ready(), now)
            leg = self.fleet.legs.get(name)
            self.supervisor.progress(name, (self.plan_id, leg.progress if leg else -1),
                                     robot.driving, now)
        self.on_events(self.supervisor.check(now), now)

    def on_events(self, events, now):
        log = self.get_logger()
        for event in events:
            kind, name = event[0], event[1]
            robot = self.robots[name]
            if kind == 'ready':
                log.info('%s is ready (odometry and Nav2 heartbeats OK)' % name)
            elif kind == 'nav_only':
                log.warn('%s sends no odometry: watching its Nav2 heartbeat only' % name)
            elif kind == 'back':
                log.info('%s is BACK IN SERVICE' % name)
                self.request_replan('%s back' % name)
            elif kind == 'down':
                log.error('%s is OUT OF SERVICE: %s' % (name, event[2]))
                robot.gen += 1                    # ignore anything its Nav2 still says
                if robot.handle is not None:
                    robot.handle.cancel_goal_async()
                robot.handle, robot.driving = None, False
                self.fleet.cancel_segment(name)
                self.give_back(name, '%s out of service' % name)
                self.request_replan('%s out of service' % name)
            elif kind == 'stuck':
                log.warn('%s has made no progress along its route for %.0f s'
                         % (name, event[2]))
                self.mark_blocked(name, now, 'no progress')
                robot.retries += 1
                if robot.task is not None and robot.retries > MAX_RETRIES:
                    self.give_back(name, '%s stuck too often' % name)
                self.request_replan('%s stuck' % name)

    def mark_blocked(self, name, now, why):
        cells = self.fleet.area_ahead(name)
        if not cells:
            return
        self.supervisor.block(cells, now)
        x, y = self.grid.cell_to_world(sorted(cells)[len(cells) // 2])
        self.get_logger().warn('%s: route blocked near (%.1f, %.1f) (%s): avoiding that area '
                               'for %.0f s' % (name, x, y, why, self.supervisor.block_seconds))

    def give_back(self, name, reason):
        """The robot cannot finish its task: put the task back in the queue (or FAIL it)."""
        robot = self.robots[name]
        task = robot.task
        robot.task, robot.retries = None, 0
        if task is None:
            self.request_replan(reason)
            return
        carried = task.state == TO_DELIVERY
        if self.supervisor.give_back(task.id):
            task.state, task.robot = PENDING, None
            self.get_logger().warn(
                'Task %d (%s->%s) back in the queue (%s), attempt %d of %d%s'
                % (task.id, task.pickup, task.delivery, reason,
                   self.supervisor.attempt(task.id), self.supervisor.max_attempts,
                   ': the load must be picked up again at %s' % task.pickup if carried else ''))
        else:
            task.state = FAILED
            self.get_logger().error('Task %d FAILED after %d attempts (%s)'
                                    % (task.id, self.supervisor.max_attempts, reason))
        self.request_replan(reason)

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
            'robots': {n: {'task': r.task.id if r.task else None, 'completed': r.completed,
                           'driving': r.driving, 'waiting_for': self.waiting.get(n, []),
                           'health': self.supervisor.robots[n].state,
                           'problem': self.supervisor.robots[n].reason}
                       for n, r in self.robots.items()},
            'plan': self.method,
            'blocked_cells': len(self.supervisor.blocked(self.now())),
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

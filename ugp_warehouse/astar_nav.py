"""astar_nav: plan with A*, then let Nav2 drive the waypoints.

Division of labour (this is the project's core idea):
  * A* (our code)  decides WHERE to go: a list of corner waypoints.
  * Nav2           decides HOW to drive between them (local planning, avoidance).

Run (needs sim, map server, AMCL and Nav2 running):
  ros2 run ugp_warehouse astar_nav --ros-args -p use_sim_time:=true \
      -p goal_x:=8.2 -p goal_y:=-2.7

ROS interfaces
  subscribes : /<robot>/amcl_pose   geometry_msgs/PoseWithCovarianceStamped
  publishes  : /<robot>/astar_path  nav_msgs/Path  (for RViz, latched)
  action     : /<robot>/navigate_through_poses  nav2_msgs/NavigateThroughPoses
"""
import math
import os
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ugp_warehouse.astar import astar
from ugp_warehouse.grid_map import GridMap


def line_is_free(grid, a, b):
    """True if the straight segment between cells a and b stays on free cells."""
    steps = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) * 4)))
    for i in range(steps + 1):
        t = i / steps
        cell = (int(round(a[0] + (b[0] - a[0]) * t)), int(round(a[1] + (b[1] - a[1]) * t)))
        if not grid.is_free(cell):
            return False
    return True


def shortcut(grid, path):
    """Line-of-sight smoothing: from each kept cell, jump to the farthest cell
    that can be reached in a straight free line. A* on a 4-connected grid makes
    staircases; this turns them into a few long segments, so Nav2 does not have
    to stop at every tiny corner."""
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_is_free(grid, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def plan_waypoints(grid, start_xy, goal_xy, goal_yaw=0.0):
    """A* from start to goal (world metres). Returns [(x, y, yaw), ...] or None.

    Staircases are smoothed with shortcut(). The first waypoint (the robot's
    own cell) is left out. Intermediate waypoints face the next one; the last
    one uses the exact goal and goal_yaw.
    """
    start = grid.nearest_free(grid.world_to_cell(*start_xy))
    goal = grid.world_to_cell(*goal_xy)
    if start is None:
        return None
    path = astar(grid, start, goal)
    if path is None:
        return None
    corners = shortcut(grid, path)[1:]                # drop the start cell
    points = [grid.cell_to_world(c) for c in corners]
    if not points:
        return []
    points[-1] = tuple(goal_xy)                       # exact goal, not the cell centre
    waypoints = []
    for i, (x, y) in enumerate(points):
        if i < len(points) - 1:
            nx, ny = points[i + 1]
            yaw = math.atan2(ny - y, nx - x)
        else:
            yaw = goal_yaw
        waypoints.append((x, y, yaw))
    return waypoints


class AStarNav(Node):
    def __init__(self):
        super().__init__('astar_nav')
        self.declare_parameter('robot', 'robot_1')
        self.declare_parameter('goal_x', 8.2)
        self.declare_parameter('goal_y', 2.7)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('map_yaml', os.path.join(
            get_package_share_directory('ugp_warehouse'), 'config', 'warehouse_map.yaml'))
        self.robot = self.get_parameter('robot').value
        self.pose = None

        # AMCL publishes amcl_pose with transient-local durability, so we match it.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, '/%s/amcl_pose' % self.robot,
                                 self.on_pose, qos)
        self.path_pub = self.create_publisher(Path, '/%s/astar_path' % self.robot, qos)
        self.client = ActionClient(self, NavigateThroughPoses,
                                   '/%s/navigate_through_poses' % self.robot)

    def on_pose(self, msg):
        self.pose = msg.pose.pose

    def make_pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def run(self):
        log = self.get_logger()

        # 1. where is the robot? (AMCL estimate in the map frame)
        t_end = time.monotonic() + 10.0  # wall time: sim clock may not be running yet
        while self.pose is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.monotonic() > t_end:
                log.error('No /%s/amcl_pose received in 10 s. Is AMCL running?' % self.robot)
                return False
        start = (self.pose.position.x, self.pose.position.y)
        goal = (self.get_parameter('goal_x').value, self.get_parameter('goal_y').value)
        yaw = self.get_parameter('goal_yaw').value
        log.info('Start (%.2f, %.2f) -> goal (%.2f, %.2f)' % (start + goal))

        # 2. A* on the inflated coarse grid
        grid = GridMap.from_yaml(self.get_parameter('map_yaml').value)
        waypoints = plan_waypoints(grid, start, goal, yaw)
        if waypoints is None:
            log.error('A* found NO path (goal blocked or unreachable). Not sending anything.')
            return False
        if not waypoints:
            log.info('Already at the goal.')
            return True
        log.info('A* waypoints: ' + ', '.join('(%.2f, %.2f)' % (w[0], w[1]) for w in waypoints))

        # 3. show it in RViz
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [self.make_pose(start[0], start[1], 0.0)] + \
                     [self.make_pose(*w) for w in waypoints]
        self.path_pub.publish(path)

        # 4. hand the waypoints to Nav2
        if not self.client.wait_for_server(timeout_sec=10.0):
            log.error('Action server /%s/navigate_through_poses not available. Is Nav2 active?'
                      % self.robot)
            return False
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = [self.make_pose(*w) for w in waypoints]
        send = self.client.send_goal_async(goal_msg, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            log.error('Nav2 rejected the goal.')
            return False
        log.info('Nav2 accepted the goal. Driving...')
        try:
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
        except KeyboardInterrupt:
            log.warn('Ctrl+C: cancelling the goal')
            rclpy.spin_until_future_complete(self, handle.cancel_goal_async(), timeout_sec=3.0)
            return False
        res = result_future.result()
        ok = res.status == 4                          # GoalStatus.STATUS_SUCCEEDED
        log.info('Finished: %s (status %d, error_code %d)'
                 % ('SUCCEEDED' if ok else 'NOT succeeded', res.status, res.result.error_code))
        return ok

    def on_feedback(self, msg):
        fb = msg.feedback
        self.get_logger().info(
            'distance_remaining %.1f m, poses_remaining %d, recoveries %d'
            % (fb.distance_remaining, fb.number_of_poses_remaining, fb.number_of_recoveries),
            throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = AStarNav()
    ok = False
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


class DriveTest(Node):
    """Publish a short scripted sequence of velocity commands."""

    def __init__(self):
        super().__init__('drive_test')
        self.declare_parameter('robot', 'robot_1')
        robot = self.get_parameter('robot').value
        self.pub = self.create_publisher(Twist, f'/{robot}/cmd_vel', 10)

        # (linear m/s, angular rad/s, duration s)
        self.steps = [
            (0.3, 0.0, 3.0),                 # forward about 0.9 m
            (0.0, 0.0, 1.0),                 # pause
            (0.0, 0.5, 2 * math.pi / 0.5),   # spin 360 degrees in place
            (0.0, 0.0, 1.0),                 # pause
            (-0.3, 0.0, 3.0),                # back to the start
            (0.0, 0.0, 1.0),                 # pause
        ]
        self.index = 0
        self.done = False
        self.step_start = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(f'Driving {robot}: step 1/{len(self.steps)}')

    def send(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)

    def tick(self):
        now = self.get_clock().now()
        elapsed = (now - self.step_start).nanoseconds / 1e9
        linear, angular, duration = self.steps[self.index]

        if elapsed >= duration:
            self.index += 1
            self.step_start = now
            if self.index >= len(self.steps):
                self.get_logger().info('Sequence finished')
                self.done = True
                return
            self.get_logger().info(f'Step {self.index + 1}/{len(self.steps)}')
            linear, angular, duration = self.steps[self.index]

        self.send(linear, angular)

    def stop(self):
        self.send(0.0, 0.0)
        time.sleep(0.2)


def main():
    # Keep the ROS context alive on Ctrl+C so we can still publish a stop.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DriveTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, stopping robot')
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

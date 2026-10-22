"""tf_relay: copy every robot's private TF topics onto the global /tf and /tf_static.

Why: each robot publishes TF on /<robot>/tf (so Nav2 and AMCL of different robots
never mix). RViz can only listen to ONE tf topic, so it would see just one robot.
This node merges them for RViz. Frame names are prefixed with the robot name
(robot_1/base_link, robot_2/base_link, ...), so merging cannot clash.

  subscribes : /<robot>/tf, /<robot>/tf_static   (tf2_msgs/TFMessage)
  publishes  : /tf, /tf_static
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfRelay(Node):
    def __init__(self):
        super().__init__('tf_relay')
        self.declare_parameter('robots', ['robot_1'])
        dynamic = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        static = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        tf_pub = self.create_publisher(TFMessage, '/tf', dynamic)
        static_pub = self.create_publisher(TFMessage, '/tf_static', static)
        for robot in self.get_parameter('robots').value:
            self.create_subscription(TFMessage, '/%s/tf' % robot, tf_pub.publish, dynamic)
            self.create_subscription(TFMessage, '/%s/tf_static' % robot,
                                     static_pub.publish, static)
            self.get_logger().info('Relaying /%s/tf and /%s/tf_static' % (robot, robot))


def main():
    rclpy.init()
    node = TfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

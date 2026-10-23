"""ROS node that plans prioritized or CBS multi-robot routes."""
import json
import os

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ugp_warehouse.grid_map import GridMap
from ugp_warehouse.space_time import CBSCoordinator, MultiRobotCoordinator


class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('multi_robot_coordinator')
        default_map = os.path.join(
            get_package_share_directory('ugp_warehouse'), 'config',
            'warehouse_map.yaml')
        self.declare_parameter('map_yaml', default_map)
        self.declare_parameter('algorithm', 'cbs')
        self.declare_parameter('max_time_steps', 0)
        self.declare_parameter('max_cbs_nodes', 1000)
        map_yaml = self.get_parameter('map_yaml').value
        max_time = self.get_parameter('max_time_steps').value or None
        self.grid = GridMap.from_yaml(map_yaml)
        self.prioritized = MultiRobotCoordinator(self.grid, max_time=max_time)
        self.cbs = CBSCoordinator(
            self.grid, max_time=max_time,
            max_nodes=self.get_parameter('max_cbs_nodes').value)
        self.response_pub = self.create_publisher(
            String, '/coordination/response', 10)
        self.create_subscription(
            String, '/coordination/request', self.on_request, 10)
        self.get_logger().info(
            'Coordinator ready (default algorithm: %s)' %
            self.get_parameter('algorithm').value)

    def on_request(self, message):
        request_id = ''
        try:
            request = json.loads(message.data)
            if not isinstance(request, dict):
                raise ValueError('request must be a JSON object')
            request_id = request['request_id']
            raw_agents = request['agents']
            if not isinstance(raw_agents, dict) or not raw_agents:
                raise ValueError('agents must be a non-empty object')
            agents = {}
            for name, agent in raw_agents.items():
                start = agent['start']
                goal = agent['goal']
                if (not isinstance(start, list) or not isinstance(goal, list) or
                        len(start) != 2 or len(goal) != 2 or
                        any(type(value) is not int for value in start + goal)):
                    raise ValueError(
                        'each start and goal must be a pair of integer grid coordinates')
                agents[name] = (tuple(start), tuple(goal))
            algorithm = request.get(
                'algorithm', self.get_parameter('algorithm').value)
            if algorithm == 'cbs':
                paths, unplanned = self.cbs.plan(agents)
            elif algorithm == 'prioritized':
                paths, unplanned = self.prioritized.plan(agents)
            else:
                raise ValueError(
                    'algorithm must be "cbs" or "prioritized"')
            response = {
                'request_id': request_id,
                'algorithm': algorithm,
                'paths': dict(paths),
                'unplanned': unplanned,
                'success': not unplanned,
            }
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().error('Invalid coordination request: %s' % error)
            response = {
                'request_id': request_id,
                'algorithm': '',
                'paths': {},
                'unplanned': [],
                'success': False,
                'error': str(error),
            }
        output = String()
        output.data = json.dumps(response)
        self.response_pub.publish(output)


def main():
    rclpy.init()
    node = CoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

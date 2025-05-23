#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class Sonar3D15Node(Node):
    def __init__(self):
        super().__init__('sonar_3d_15_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('sonar_ip', ''),
                ('speed_of_sound', 1480),
                ('acoustics_enabled', False),
                ('multicast_group', '224.0.0.96'),
                ('multicast_port', 4747),
                ('filter_ip', ''),
            ]
        )
        self.get_logger().info('Sonar 3D-15 ROS2 node started.')
        # TODO: Add publishers, subscribers, timers, and main logic

def main(args=None):
    rclpy.init(args=args)
    node = Sonar3D15Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""ROS2 replacement for a dynamic reconfigure server.

This node exposes a parameter named 'trim' and logs changes via the
on_set_parameters callback. It's a lightweight replacement for
dynamic_reconfigure::Server used in ROS1.
"""
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult


class DynServer(Node):
    def __init__(self):
        super().__init__('map_builder_dyn_server_py')
        # Declare parameter with a reasonable default; type can be float
        self.declare_parameter('trim', 0.99)
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        # Log the new parameter values and accept them
        for p in params:
            self.get_logger().info('Reconfigure Request: %s = %s' % (p.name, p.value))
        result = SetParametersResult()
        result.successful = True
        result.reason = ''
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DynServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

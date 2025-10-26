#!/usr/bin/env python3
"""Simple ROS2 parameter client that updates a parameter on the dynamic server node.

This replaces the ROS1 dynamic_reconfigure client by using a ParameterClient
to set parameters on the server node.
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
try:
    # ParameterClient may not be available in some older rclpy versions under this name
    from rclpy.parameter_client import ParameterClient
except Exception:
    # Fallback import path
    from rclpy.parameter_client import ParameterClient


class DynClient(Node):
    def __init__(self):
        super().__init__('surface_recon_dynamic_client')
        # The target node name should match the server node's name
    self.target_node = 'map_builder_dyn_server_py'
        self.param_client = ParameterClient(self, self.target_node)

    def update_trim(self, value: float):
        # Build a Parameter and send it
        param = Parameter('trim', Parameter.Type.DOUBLE, value)
        fut = self.param_client.set_parameters([param])
        # Wait for the result (short blocking wait)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result()


def main(args=None):
    rclpy.init(args=args)
    node = DynClient()

    try:
        # Update trim repeatedly (mirrors original behaviour)
        while rclpy.ok():
            node.get_logger().info('Updating trim -> 0.99')
            try:
                res = node.update_trim(0.99)
                node.get_logger().info('Set parameter result: %s' % str(res))
            except Exception as e:
                node.get_logger().warn('Parameter update failed: %s' % str(e))
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import sensor_msgs_py.point_cloud2 as pc2

# Optional numpy import
try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False


class OAKDPublisher(Node):
    """
    Simple publisher node for testing without physical hardware.
    This can be used for simulation or testing purposes.
    """
    
    def __init__(self):
        super().__init__('oakd_publisher')
        
        if not NUMPY_AVAILABLE:
            self.get_logger().warn("NumPy not available, using simplified test data")
        
        # Publishers
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'oakd/points', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'oakd/pose', 10)
        
        # Timer to publish test data
        self.create_timer(0.1, self.publish_test_data)  # 10 Hz
        
        self.frame_count = 0
        self.get_logger().info('OAK-D publisher node initialized (test mode)')

    def publish_test_data(self):
        """Publish test point cloud data for simulation"""
        try:
            # Generate a simple test point cloud (a grid of points)
            points = []
            
            if NUMPY_AVAILABLE:
                # Create a simple 3D grid pattern with numpy
                for x in np.linspace(-2, 2, 20):
                    for y in np.linspace(-2, 2, 20):
                        z = 2.0 + 0.5 * np.sin(x + self.frame_count * 0.1) * np.cos(y + self.frame_count * 0.1)
                        points.append([float(x), float(y), float(z)])
            else:
                # Simple fallback without numpy
                for i in range(-10, 11):
                    for j in range(-10, 11):
                        x = i * 0.2  # -2 to 2
                        y = j * 0.2  # -2 to 2
                        z = 2.0 + 0.5 * (self.frame_count * 0.01) % 1.0  # Simple animation
                        points.append([float(x), float(y), float(z)])
            
            # Create PointCloud2 message
            header = self.create_header()
            pointcloud_msg = pc2.create_cloud_xyz32(header, points)
            self.pointcloud_pub.publish(pointcloud_msg)
            
            # Publish pose
            pose_msg = PoseStamped()
            pose_msg.header = header
            pose_msg.pose.position.x = 0.0
            pose_msg.pose.position.y = 0.0
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            self.pose_pub.publish(pose_msg)
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Error publishing test data: {str(e)}')

    def create_header(self):
        """Create a standard header with current timestamp"""
        from std_msgs.msg import Header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'oakd_frame'
        return header


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OAKDPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
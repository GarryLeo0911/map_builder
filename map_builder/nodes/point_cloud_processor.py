#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from collections import deque
import threading


class PointCloudProcessor(Node):
    """
    Node for processing point cloud data from OAK-D camera.
    Performs filtering, downsampling, and basic preprocessing.
    """
    
    def __init__(self):
        super().__init__('point_cloud_processor')
        
        # Declare parameters
        self.declare_parameter('voxel_size', 0.05)  # 5cm voxel size
        self.declare_parameter('max_range', 10.0)   # 10m max range
        self.declare_parameter('min_range', 0.3)    # 30cm min range
        self.declare_parameter('statistical_outlier_nb_neighbors', 20)
        self.declare_parameter('statistical_outlier_std_ratio', 2.0)
        self.declare_parameter('buffer_size', 100)  # Number of point clouds to keep
        
        # Get parameters
        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        self.outlier_nb_neighbors = self.get_parameter('statistical_outlier_nb_neighbors').get_parameter_value().integer_value
        self.outlier_std_ratio = self.get_parameter('statistical_outlier_std_ratio').get_parameter_value().double_value
        self.buffer_size = self.get_parameter('buffer_size').get_parameter_value().integer_value
        
        # Subscribers
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            'oak/points',
            self.pointcloud_callback,
            10
        )
        
        # Publishers
        self.filtered_pointcloud_pub = self.create_publisher(
            PointCloud2,
            'map_builder/filtered_points',
            10
        )
        
        self.accumulated_pointcloud_pub = self.create_publisher(
            PointCloud2,
            'map_builder/accumulated_points',
            10
        )
        
        # Point cloud buffer
        self.pointcloud_buffer = deque(maxlen=self.buffer_size)
        self.processing_lock = threading.Lock()
        
        # Timer for publishing accumulated point cloud
        self.create_timer(1.0, self.publish_accumulated_pointcloud)
        
        self.get_logger().info('Point cloud processor node initialized')

    def pointcloud_callback(self, msg):
        """Process incoming point cloud data"""
        try:
            # Convert ROS PointCloud2 to numpy array
            points = self.pointcloud2_to_array(msg)
            
            if points is None or len(points) == 0:
                return
            
            # Filter points
            filtered_points = self.filter_points(points)
            
            # Downsample using voxel grid
            downsampled_points = self.voxel_downsample(filtered_points)
            
            # Store in buffer
            with self.processing_lock:
                self.pointcloud_buffer.append({
                    'points': downsampled_points,
                    'header': msg.header
                })
            
            # Publish filtered point cloud
            if len(downsampled_points) > 0:
                filtered_msg = self.array_to_pointcloud2(downsampled_points, msg.header)
                self.filtered_pointcloud_pub.publish(filtered_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')

    def pointcloud2_to_array(self, pointcloud_msg):
        """Convert PointCloud2 message to numpy array"""
        try:
            points_list = []
            for point in pc2.read_points(pointcloud_msg, skip_nans=True):
                points_list.append([point[0], point[1], point[2]])
            
            if points_list:
                return np.array(points_list, dtype=np.float32)
            else:
                return None
                
        except Exception as e:
            self.get_logger().error(f'Error converting PointCloud2 to array: {str(e)}')
            return None

    def array_to_pointcloud2(self, points, header):
        """Convert numpy array to PointCloud2 message"""
        try:
            return pc2.create_cloud_xyz32(header, points.tolist())
        except Exception as e:
            self.get_logger().error(f'Error converting array to PointCloud2: {str(e)}')
            return None

    def filter_points(self, points):
        """Apply range and outlier filtering to points"""
        try:
            # Range filtering
            distances = np.linalg.norm(points, axis=1)
            range_mask = (distances >= self.min_range) & (distances <= self.max_range)
            filtered_points = points[range_mask]
            
            # Remove NaN and infinite values
            valid_mask = np.isfinite(filtered_points).all(axis=1)
            filtered_points = filtered_points[valid_mask]
            
            return filtered_points
            
        except Exception as e:
            self.get_logger().error(f'Error filtering points: {str(e)}')
            return points

    def voxel_downsample(self, points):
        """Downsample points using voxel grid"""
        try:
            if len(points) == 0:
                return points
            
            # Compute voxel coordinates
            voxel_coords = np.floor(points / self.voxel_size).astype(int)
            
            # Find unique voxels
            _, unique_indices = np.unique(voxel_coords, axis=0, return_index=True)
            
            # Return points at unique voxel centers
            return points[unique_indices]
            
        except Exception as e:
            self.get_logger().error(f'Error in voxel downsampling: {str(e)}')
            return points

    def publish_accumulated_pointcloud(self):
        """Publish accumulated point cloud from buffer"""
        try:
            with self.processing_lock:
                if not self.pointcloud_buffer:
                    return
                
                # Combine all points from buffer
                all_points = []
                latest_header = None
                
                for pc_data in self.pointcloud_buffer:
                    all_points.extend(pc_data['points'])
                    latest_header = pc_data['header']
                
                if all_points and latest_header:
                    combined_points = np.array(all_points, dtype=np.float32)
                    
                    # Apply additional downsampling to combined point cloud
                    downsampled_combined = self.voxel_downsample(combined_points)
                    
                    # Publish combined point cloud
                    combined_msg = self.array_to_pointcloud2(downsampled_combined, latest_header)
                    if combined_msg:
                        self.accumulated_pointcloud_pub.publish(combined_msg)
                        
                        self.get_logger().debug(
                            f'Published accumulated point cloud with {len(downsampled_combined)} points'
                        )
                        
        except Exception as e:
            self.get_logger().error(f'Error publishing accumulated point cloud: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PointCloudProcessor()
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
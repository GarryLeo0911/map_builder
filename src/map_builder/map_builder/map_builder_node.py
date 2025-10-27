#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import threading
from collections import defaultdict


class MapBuilderNode(Node):
    """
    Main mapping node that coordinates point cloud processing and map building.
    Integrates data from multiple sources to build a comprehensive 3D map.
    """
    
    def __init__(self):
        super().__init__('map_builder_node')
        
        # Declare parameters
        self.declare_parameter('map_resolution', 0.05)  # 5cm resolution
        self.declare_parameter('map_width', 400)        # 20m x 20m map
        self.declare_parameter('map_height', 400)
        self.declare_parameter('map_origin_x', -10.0)   # Map centered at robot start
        self.declare_parameter('map_origin_y', -10.0)
        self.declare_parameter('robot_radius', 0.3)     # 30cm robot radius
        self.declare_parameter('min_obstacle_height', 0.1)  # 10cm min obstacle height
        self.declare_parameter('max_obstacle_height', 2.0)   # 2m max obstacle height
        
        # Get parameters
        self.map_resolution = self.get_parameter('map_resolution').get_parameter_value().double_value
        self.map_width = self.get_parameter('map_width').get_parameter_value().integer_value
        self.map_height = self.get_parameter('map_height').get_parameter_value().integer_value
        self.map_origin_x = self.get_parameter('map_origin_x').get_parameter_value().double_value
        self.map_origin_y = self.get_parameter('map_origin_y').get_parameter_value().double_value
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        self.min_obstacle_height = self.get_parameter('min_obstacle_height').get_parameter_value().double_value
        self.max_obstacle_height = self.get_parameter('max_obstacle_height').get_parameter_value().double_value
        
        # Initialize map
        self.occupancy_map = np.full((self.map_height, self.map_width), -1, dtype=np.int8)  # Unknown
        self.height_map = np.full((self.map_height, self.map_width), np.nan, dtype=np.float32)
        self.map_lock = threading.Lock()
        
        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            'map_builder/filtered_points',
            self.pointcloud_callback,
            10
        )
        
        self.robot_pose_sub = self.create_subscription(
            PoseStamped,
            'robot_pose',
            self.robot_pose_callback,
            10
        )
        
        # Publishers
        self.occupancy_grid_pub = self.create_publisher(
            OccupancyGrid,
            'map_builder/occupancy_grid',
            10
        )
        
        # Robot pose
        self.current_robot_pose = None
        
        # Timer for publishing map
        self.create_timer(1.0, self.publish_occupancy_grid)
        
        self.get_logger().info('Map builder node initialized')

    def pointcloud_callback(self, msg):
        """Process point cloud data and update occupancy grid"""
        try:
            # Convert to numpy array
            points = self.pointcloud2_to_array(msg)
            
            if points is None or len(points) == 0:
                return
            
            # Transform points to map frame if necessary
            transformed_points = self.transform_points_to_map_frame(points, msg.header)
            
            if transformed_points is None:
                return
            
            # Update occupancy grid
            self.update_occupancy_grid(transformed_points)
            
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud for mapping: {str(e)}')

    def robot_pose_callback(self, msg):
        """Update current robot pose"""
        self.current_robot_pose = msg

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

    def transform_points_to_map_frame(self, points, header):
        """Transform points to map coordinate frame"""
        try:
            # For simplicity, assume points are already in map frame
            # In a real implementation, you would use TF to transform
            return points
            
        except Exception as e:
            self.get_logger().error(f'Error transforming points: {str(e)}')
            return None

    def update_occupancy_grid(self, points):
        """Update occupancy grid with new point cloud data"""
        try:
            with self.map_lock:
                # Convert world coordinates to grid coordinates
                grid_x = ((points[:, 0] - self.map_origin_x) / self.map_resolution).astype(int)
                grid_y = ((points[:, 1] - self.map_origin_y) / self.map_resolution).astype(int)
                heights = points[:, 2]
                
                # Filter points within map bounds
                valid_mask = (
                    (grid_x >= 0) & (grid_x < self.map_width) &
                    (grid_y >= 0) & (grid_y < self.map_height)
                )
                
                valid_grid_x = grid_x[valid_mask]
                valid_grid_y = grid_y[valid_mask]
                valid_heights = heights[valid_mask]
                
                # Update height map and occupancy grid
                for gx, gy, h in zip(valid_grid_x, valid_grid_y, valid_heights):
                    # Update height map (take maximum height)
                    if np.isnan(self.height_map[gy, gx]) or h > self.height_map[gy, gx]:
                        self.height_map[gy, gx] = h
                    
                    # Update occupancy based on height
                    if self.min_obstacle_height <= h <= self.max_obstacle_height:
                        self.occupancy_map[gy, gx] = 100  # Occupied
                    else:
                        if self.occupancy_map[gy, gx] != 100:  # Don't override obstacles
                            self.occupancy_map[gy, gx] = 0  # Free
                
                # Mark areas between robot and obstacles as free
                if self.current_robot_pose is not None:
                    self.ray_trace_free_space(points)
                    
        except Exception as e:
            self.get_logger().error(f'Error updating occupancy grid: {str(e)}')

    def ray_trace_free_space(self, points):
        """Ray trace from robot position to mark free space"""
        try:
            robot_x = self.current_robot_pose.pose.position.x
            robot_y = self.current_robot_pose.pose.position.y
            
            # Convert robot position to grid coordinates
            robot_gx = int((robot_x - self.map_origin_x) / self.map_resolution)
            robot_gy = int((robot_y - self.map_origin_y) / self.map_resolution)
            
            # Check bounds
            if not (0 <= robot_gx < self.map_width and 0 <= robot_gy < self.map_height):
                return
            
            # Ray trace to each point
            for point in points[::10]:  # Subsample for performance
                point_gx = int((point[0] - self.map_origin_x) / self.map_resolution)
                point_gy = int((point[1] - self.map_origin_y) / self.map_resolution)
                
                # Check bounds
                if not (0 <= point_gx < self.map_width and 0 <= point_gy < self.map_height):
                    continue
                
                # Bresenham's line algorithm for ray tracing
                cells = self.bresenham_line(robot_gx, robot_gy, point_gx, point_gy)
                
                for cell_x, cell_y in cells[:-1]:  # Exclude endpoint
                    if 0 <= cell_x < self.map_width and 0 <= cell_y < self.map_height:
                        if self.occupancy_map[cell_y, cell_x] != 100:  # Don't override obstacles
                            self.occupancy_map[cell_y, cell_x] = 0  # Free
                            
        except Exception as e:
            self.get_logger().error(f'Error in ray tracing: {str(e)}')

    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            points.append((x, y))
            
            if x == x1 and y == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return points

    def publish_occupancy_grid(self):
        """Publish the current occupancy grid"""
        try:
            with self.map_lock:
                msg = OccupancyGrid()
                
                # Header
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'map'
                
                # Map metadata
                msg.info.resolution = self.map_resolution
                msg.info.width = self.map_width
                msg.info.height = self.map_height
                msg.info.origin.position.x = self.map_origin_x
                msg.info.origin.position.y = self.map_origin_y
                msg.info.origin.position.z = 0.0
                msg.info.origin.orientation.w = 1.0
                
                # Map data (flatten the 2D array)
                msg.data = self.occupancy_map.flatten().tolist()
                
                self.occupancy_grid_pub.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing occupancy grid: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MapBuilderNode()
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
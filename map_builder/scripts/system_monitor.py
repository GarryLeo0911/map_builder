#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import time


class SystemMonitor(Node):
    """Simple monitoring node to check if the integration is working"""
    
    def __init__(self):
        super().__init__('system_monitor')
        
        # Counters for received messages
        self.oak_points_count = 0
        self.filtered_points_count = 0
        self.map_count = 0
        
        # Subscribers
        self.oak_sub = self.create_subscription(
            PointCloud2, '/oak/points', self.oak_callback, 10)
        self.filtered_sub = self.create_subscription(
            PointCloud2, '/map_builder/filtered_points', self.filtered_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map_builder/occupancy_grid', self.map_callback, 10)
        
        # Timer for status reporting
        self.create_timer(5.0, self.report_status)
        
        self.get_logger().info('System monitor started - checking integration...')
    
    def oak_callback(self, msg):
        self.oak_points_count += 1
    
    def filtered_callback(self, msg):
        self.filtered_points_count += 1
    
    def map_callback(self, msg):
        self.map_count += 1
    
    def report_status(self):
        self.get_logger().info(
            f'Status - OAK points: {self.oak_points_count}, '
            f'Filtered: {self.filtered_points_count}, '
            f'Maps: {self.map_count}'
        )
        
        if self.oak_points_count > 0:
            self.get_logger().info('✓ OAK-D camera is publishing point clouds')
        else:
            self.get_logger().warn('✗ No point clouds from OAK-D camera')
        
        if self.filtered_points_count > 0:
            self.get_logger().info('✓ Point cloud processor is working')
        else:
            self.get_logger().warn('✗ Point cloud processor not receiving/processing data')
        
        if self.map_count > 0:
            self.get_logger().info('✓ Map builder is generating maps')
        else:
            self.get_logger().warn('✗ Map builder not generating maps')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = SystemMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        if 'monitor' in locals():
            monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
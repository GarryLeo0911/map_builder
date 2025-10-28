#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch OAK-D driver and map_builder with 3D visualization using oakd publisher"""
    
    # Get the package directories
    map_builder_dir = get_package_share_directory('map_builder')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(map_builder_dir, 'config', 'oakd_map_builder_params.yaml'),
        description='Path to the map_builder config file'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='15',
        description='Camera FPS'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 with 3D visualization'
    )
    
    # OAK-D Publisher Node
    oakd_node = Node(
        package='oakd_driver',
        executable='oakd_publisher_node',
        name='oakd_driver',
        parameters=[{
            'fps': LaunchConfiguration('fps'),
        }],
        output='screen'
    )
    
    # Visual Odometry Node - tracks camera motion
    visual_odometry_node = Node(
        package='map_builder',
        executable='visual_odometry',
        name='visual_odometry',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )
    
    # Point cloud processor node
    point_cloud_processor = Node(
        package='map_builder',
        executable='point_cloud_processor',
        name='point_cloud_processor',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )
    
    # Surface reconstructor node (for 3D visualization)
    surface_reconstructor = Node(
        package='map_builder',
        executable='surface_reconstructor',
        name='surface_reconstructor',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )
    
    # Map builder node
    map_builder_node = Node(
        package='map_builder',
        executable='map_builder_node',
        name='map_builder_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )
    
    # Static transform publishers
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'oak_camera_frame'],
        output='screen'
    )

    # Static transform: link oak_camera_frame -> oak_left_camera_optical_frame
    oak_left_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='oak_left_camera_tf',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'oak_camera_frame', 'oak_left_camera_optical_frame'],
        output='screen'
    )
    
    map_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    # RViz2 with fixed configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(map_builder_dir, 'rviz', 'map_builder_fixed.rviz')],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        fps_arg,
        launch_rviz_arg,
        oakd_node,
        visual_odometry_node,
        base_to_camera_tf,
        oak_left_camera_tf,
        map_to_base_tf,
        point_cloud_processor,
        surface_reconstructor,
        map_builder_node,
        rviz_node
    ])
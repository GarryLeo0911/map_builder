#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Simple test launch to verify OAK-D driver connection and point cloud output"""
    
    # Launch arguments
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='15',
        description='Camera FPS'
    )
    
    enable_ir_arg = DeclareLaunchArgument(
        'enable_ir',
        default_value='false',
        description='Enable IR illumination'
    )
    
    # OAK-D Driver Node
    oakd_node = Node(
        package='oakd_driver',
        executable='oakd_node',
        name='oakd_node',
        parameters=[{
            'i_fps': LaunchConfiguration('fps'),
            'i_rgb_resolution': '720p',
            'i_depth_resolution': '720p',
            'i_enable_ir': LaunchConfiguration('enable_ir'),
            'i_tf_camera_name': 'oak',
            'i_depth_confidence_threshold': 200,
            'i_depth_lr_check': True,
            'i_depth_subpixel': False,
            'i_depth_extended_disparity': False,
            'i_depth_preset_mode': 'HIGH_ACCURACY'
        }],
        output='screen'
    )
    
    # Transform publishers for proper frame hierarchy
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'oak_camera_frame'],
        output='screen'
    )
    
    map_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen'
    )
    
    # RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('map_builder'), 'rviz', 'oakd_test.rviz')],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        output='screen'
    )
    
    return LaunchDescription([
        fps_arg,
        enable_ir_arg,
        launch_rviz_arg,
        oakd_node,
        base_to_camera_tf,
        map_to_base_tf,
        rviz_node
    ])
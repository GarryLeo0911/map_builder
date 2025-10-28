#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('map_builder')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_dir, 'config', 'map_builder_params.yaml'),
        description='Path to the config file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 with map_builder configuration'
    )
    
    # Point cloud processor node
    point_cloud_processor = Node(
        package='map_builder',
        executable='point_cloud_processor',
        name='point_cloud_processor',
        parameters=[LaunchConfiguration('config_file'),
                   {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # Surface reconstructor node
    surface_reconstructor = Node(
        package='map_builder',
        executable='surface_reconstructor',
        name='surface_reconstructor',
        parameters=[LaunchConfiguration('config_file'),
                   {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # Map builder node
    map_builder_node = Node(
        package='map_builder',
        executable='map_builder_node',
        name='map_builder_node',
        parameters=[LaunchConfiguration('config_file'),
                   {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # RViz2 node
    rviz_config_file = os.path.join(package_dir, 'rviz', 'map_builder.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        launch_rviz_arg,
        point_cloud_processor,
        surface_reconstructor,
        map_builder_node,
        rviz_node
    ])
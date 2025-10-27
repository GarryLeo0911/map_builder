#!/usr/bin/env python3
"""
Complete system launch file
Launches both OAK-D driver and mapping components
Use this for testing or when running everything on one machine
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    use_test_data_arg = DeclareLaunchArgument(
        'use_test_data',
        default_value='false',
        description='Use test publisher instead of real camera'
    )
    
    # Camera node (real or test)
    camera_node = Node(
        package='oakd_driver',
        executable='oakd_publisher' if LaunchConfiguration('use_test_data') == 'true' else 'oakd_node',
        name='oakd_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # Map builder components
    map_builder_package_dir = get_package_share_directory('map_builder')
    
    point_cloud_processor = Node(
        package='map_builder',
        executable='point_cloud_processor',
        name='point_cloud_processor',
        parameters=[
            os.path.join(map_builder_package_dir, 'config', 'map_builder_params.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )
    
    surface_reconstructor = Node(
        package='map_builder',
        executable='surface_reconstructor',
        name='surface_reconstructor',
        parameters=[
            os.path.join(map_builder_package_dir, 'config', 'map_builder_params.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )
    
    map_builder_node = Node(
        package='map_builder',
        executable='map_builder_node',
        name='map_builder_node',
        parameters=[
            os.path.join(map_builder_package_dir, 'config', 'map_builder_params.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )
    
    # Static transforms
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='oakd_static_tf',
        arguments=[
            '0.1', '0.0', '0.2',  # x, y, z
            '0.0', '0.0', '0.0', '1.0',  # qx, qy, qz, qw
            'base_link', 'oakd_frame'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    map_to_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base_link_tf',
        arguments=[
            '0.0', '0.0', '0.0',  # x, y, z
            '0.0', '0.0', '0.0', '1.0',  # qx, qy, qz, qw
            'map', 'base_link'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # RViz2
    rviz_config_file = os.path.join(
        map_builder_package_dir,
        'rviz',
        'map_builder.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        use_test_data_arg,
        camera_node,
        point_cloud_processor,
        surface_reconstructor,
        map_builder_node,
        static_transform_publisher,
        map_to_base_link_tf,
        rviz_node
    ])
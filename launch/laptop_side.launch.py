#!/usr/bin/env python3
"""
Launch file for laptop-side components (visualization and mapping)
This should be run on the laptop that receives data from the robot
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
    
    # Include map builder launch
    try:
        map_builder_package_dir = get_package_share_directory('map_builder')
        map_builder_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(map_builder_package_dir, 'launch', 'map_builder.launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items()
        )
    except:
        # Fallback if package not found
        map_builder_package_dir = get_package_share_directory('map_builder')
        
        # Point cloud processor
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
        
        # Surface reconstructor
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
        
        # Map builder node
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
        
        map_builder_launch = [point_cloud_processor, surface_reconstructor, map_builder_node]
    
    # RViz2 node
    rviz_config_file = os.path.join(
        get_package_share_directory('map_builder'),
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
        map_builder_launch if isinstance(map_builder_launch, list) else map_builder_launch,
        rviz_node
    ] if isinstance(map_builder_launch, list) else [
        use_sim_time_arg,
        map_builder_launch,
        rviz_node
    ])
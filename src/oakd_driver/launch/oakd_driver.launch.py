#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('oakd_driver')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_dir, 'config', 'oakd_params.yaml'),
        description='Path to the config file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # OAK-D driver node
    oakd_node = Node(
        package='oakd_driver',
        executable='oakd_node',
        name='oakd_node',
        parameters=[LaunchConfiguration('config_file'),
                   {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        oakd_node
    ])
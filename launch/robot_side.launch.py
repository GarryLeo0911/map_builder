#!/usr/bin/env python3
"""
Launch file for robot-side components (Raspberry Pi)
This should be run on the robot with the OAK-D camera
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
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='oakd_robot',
        description='Name of the robot'
    )
    
    # Include OAK-D driver launch
    try:
        oakd_package_dir = get_package_share_directory('oakd_driver')
        oakd_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(oakd_package_dir, 'launch', 'oakd_driver.launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items()
        )
    except:
        # Fallback if package not found
        oakd_launch = Node(
            package='oakd_driver',
            executable='oakd_node',
            name='oakd_node',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        )
    
    # Robot state publisher (for TF tree)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['--ros-args', '--log-level', 'warn']
    )
    
    # Static transform from base_link to oakd_frame
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
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,
        oakd_launch,
        robot_state_publisher,
        static_transform_publisher
    ])
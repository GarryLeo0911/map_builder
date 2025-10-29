#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    """
    Minimal test launch for OAK-D driver to debug connection issues
    """
    
    # Just launch the OAK-D driver with minimal configuration
    oakd_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('oakd_driver'),
                'launch',
                'oakd_driver.launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': 'config/oakd_params.yaml',
        }.items()
    )
    
    return LaunchDescription([
        oakd_driver_launch,
    ])
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)
    
    # RTAB-Map parameters - can subscribe to remote camera topics
    parameters = [
        config_file,  # Load from config file
        {
            "frame_id": "oak_parent_frame",
            "subscribe_rgb": True,
            "subscribe_depth": True,
            "subscribe_odom_info": False,
            "approx_sync": True,
            "use_sim_time": use_sim_time == "true",
            # Add database path to save maps
            "database_path": "~/.ros/rtabmap.db",
        }
    ]

    # Topic remappings - these will subscribe to camera topics from remote robot
    remappings = [
        ("rgb/image", f"/{robot_name}/rgb/image_raw"),
        ("rgb/camera_info", f"/{robot_name}/rgb/camera_info"),
        ("depth/image", f"/{robot_name}/stereo/image_raw"),
        ("odom", "/odom")  # Use the odom topic that RTABmap is already publishing
    ]

    return [
        # RTAB-Map SLAM node
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            parameters=parameters,
            remappings=remappings,
            output="screen",
        ),
        # RTAB-Map visualization
        Node(
            package="rtabmap_viz",
            executable="rtabmap_viz",
            name="rtabmap_viz",
            output="screen",
            parameters=parameters,
            remappings=remappings,
        ),
    ]


def generate_launch_description():
    map_builder_prefix = get_package_share_directory("map_builder")
    
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_name", 
            default_value="oak",
            description="Name of the robot/camera namespace to subscribe to"
        ),
        DeclareLaunchArgument(
            "use_sim_time", 
            default_value="false",
            description="Use simulation time"
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=os.path.join(map_builder_prefix, "config", "rtabmap_config.yaml"),
            description="Path to rtabmap configuration file"
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
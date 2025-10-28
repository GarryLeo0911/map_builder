#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directories
    map_builder_dir = get_package_share_directory('map_builder')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(map_builder_dir, 'config', 'map_builder_params.yaml'),
        description='Path to the map_builder config file'
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
    
    # OAK-D camera arguments
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='oak',
        description='Camera name prefix for topics'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='15',
        description='Camera FPS'
    )
    
    rgb_resolution_arg = DeclareLaunchArgument(
        'rgb_resolution',
        default_value='720p',
        description='RGB camera resolution'
    )
    
    depth_resolution_arg = DeclareLaunchArgument(
        'depth_resolution',
        default_value='720p',
        description='Depth camera resolution'
    )
    
    enable_ir_arg = DeclareLaunchArgument(
        'enable_ir',
        default_value='false',
        description='Enable IR illumination'
    )
    
    # OAK-D Driver Node (assuming it's installed as a separate package)
    # You'll need to adjust this path based on your OAK-D driver installation
    oakd_node = Node(
        package='oakd_driver',
        executable='oakd_node',
        name='oakd_node',
        parameters=[{
            'i_fps': LaunchConfiguration('fps'),
            'i_rgb_resolution': LaunchConfiguration('rgb_resolution'),
            'i_depth_resolution': LaunchConfiguration('depth_resolution'),
            'i_enable_ir': LaunchConfiguration('enable_ir'),
            'i_tf_camera_name': LaunchConfiguration('camera_name'),
            'i_depth_confidence_threshold': 200,
            'i_depth_lr_check': True,
            'i_depth_subpixel': False,
            'i_depth_extended_disparity': False,
            'i_depth_preset_mode': 'HIGH_ACCURACY',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
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
    
    # Transform publisher (static transform from base_link to camera frame)
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='oak_static_transform_publisher',
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
    
    # Map frame transform publisher
    map_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen'
    )
    
    # RViz2 node
    rviz_config_file = os.path.join(map_builder_dir, 'rviz', 'map_builder_minimal.rviz')
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
        # Launch arguments
        config_file_arg,
        use_sim_time_arg,
        launch_rviz_arg,
        camera_name_arg,
        fps_arg,
        rgb_resolution_arg,
        depth_resolution_arg,
        enable_ir_arg,
        
        # Nodes
        oakd_node,
        static_transform_publisher,
        oak_left_camera_tf,
        map_transform_publisher,
        point_cloud_processor,
        surface_reconstructor,
        map_builder_node,
        rviz_node
    ])
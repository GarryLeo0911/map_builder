from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    map_builder_share = get_package_share_directory('map_builder')
    
    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('map_builder'),
            'config',
            'map_builder_params.yaml'
        ]),
        description='Path to the parameters file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Parameters
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Enhanced Visual Odometry Node
    enhanced_visual_odometry_node = Node(
        package='map_builder',
        executable='enhanced_visual_odometry_node',
        name='enhanced_visual_odometry',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/oak/rgb/image_raw', '/oak/rgb/image_raw'),
            ('/oak/stereo/depth', '/oak/stereo/depth'),
            ('/oak/points', '/oak/points'),
            ('/oak/imu', '/oak/imu')
        ]
    )
    
    # Point Cloud Processor Node
    point_cloud_processor_node = Node(
        package='map_builder',
        executable='point_cloud_processor_node',
        name='point_cloud_processor',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/oak/points', '/oak/points')
        ]
    )
    
    # Map Builder Node
    map_builder_node = Node(
        package='map_builder',
        executable='map_builder_node',
        name='map_builder',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/visual_odometry/pose', '/enhanced_visual_odometry/pose')
        ]
    )
    
    # Surface Reconstructor Node (optional, can be resource intensive)
    surface_reconstructor_node = Node(
        package='map_builder',
        executable='surface_reconstructor_node',
        name='surface_reconstructor',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        
        # Core mapping pipeline
        enhanced_visual_odometry_node,
        point_cloud_processor_node,
        map_builder_node,
        
        # Optional surface reconstruction (comment out if too resource intensive)
        # surface_reconstructor_node,
    ])
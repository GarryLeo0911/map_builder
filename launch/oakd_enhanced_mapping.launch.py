from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Enhanced launch file for OAK-D mapping with rtabmap-inspired optimizations
    """
    
    # Package directories
    map_builder_share = get_package_share_directory('map_builder')
    oakd_driver_share = get_package_share_directory('oakd_driver')
    
    # Launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('map_builder'),
            'config',
            'oakd_optimized_params.yaml'
        ]),
        description='Path to the OAK-D optimized parameters file'
    )
    
    oakd_params_file_arg = DeclareLaunchArgument(
        'oakd_params_file',
        default_value='config/oakd_params.yaml',
        description='Path to the OAK-D driver parameters file'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug output for OAK-D driver'
    )
    
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='',  # Empty for auto-detection
        description='OAK-D device ID (MXID) - leave empty for auto-detection'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    # Get launch configurations
    params_file = LaunchConfiguration('params_file')
    oakd_params_file = LaunchConfiguration('oakd_params_file')
    device_id = LaunchConfiguration('device_id')
    use_rviz = LaunchConfiguration('use_rviz')
    debug = LaunchConfiguration('debug')
    
    # OAK-D Driver Launch (delayed to avoid conflicts)
    oakd_driver_launch = TimerAction(
        period=2.0,  # 2 second delay
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('oakd_driver'),
                        'launch',
                        'oakd_driver.launch.py'
                    ])
                ]),
                launch_arguments={
                    'params_file': oakd_params_file,
                    'device_id': device_id,
                    'debug': debug,
                }.items()
            )
        ]
    )
    
    # Enhanced Visual Odometry with IMU fusion
    enhanced_visual_odometry_node = Node(
        package='map_builder',
        executable='enhanced_visual_odometry_node',
        name='enhanced_visual_odometry',
        output='screen',
        parameters=[params_file],
        remappings=[
            # OAK-D topic mappings (correct topic names from driver)
            ('oak/rgb/image_raw', '/oak/rgb/image_raw'),
            ('oak/stereo/depth', '/oak/stereo/depth'),
            ('oak/points', '/oak/points'),
            ('oak/imu', '/oak/imu'),
            # Output mappings
            ('/enhanced_visual_odometry/odometry', '/visual_odometry/odometry'),
            ('/enhanced_visual_odometry/pose', '/visual_odometry/pose'),
        ]
    )
    
    # Enhanced Point Cloud Processor with OAK-D optimizations
    point_cloud_processor_node = Node(
        package='map_builder',
        executable='point_cloud_processor_node',
        name='point_cloud_processor',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('oak/points', '/oak/points'),
            ('/map_builder/filtered_points', '/map_builder/filtered_points'),
        ]
    )
    
    # Map Builder with enhanced loop closure
    map_builder_node = Node(
        package='map_builder',
        executable='map_builder_node',
        name='map_builder',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/visual_odometry/pose', '/visual_odometry/pose'),
            ('/map_builder/filtered_points', '/map_builder/filtered_points'),
            ('/map_builder/occupancy_grid', '/map'),
        ]
    )
    
    # TF2 Static transforms for OAK-D
    tf_oak_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_oak_to_base',
        arguments=[
            '0', '0', '0.1',  # x, y, z
            '0', '0', '0', '1',  # qx, qy, qz, qw
            'base_link',
            'oak_camera_frame'
        ]
    )
    
    # Temporary map frame publisher (until visual odometry starts working)
    tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_odom',
        arguments=[
            '0', '0', '0',  # x, y, z
            '0', '0', '0', '1',  # qx, qy, qz, qw
            'map',
            'odom'
        ]
    )
    
    tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_to_base',
        arguments=[
            '0', '0', '0',  # x, y, z
            '0', '0', '0', '1',  # qx, qy, qz, qw
            'odom',
            'base_link'
        ]
    )
    
    # RViz configuration for enhanced mapping
    rviz_config = PathJoinSubstitution([
        FindPackageShare('map_builder'),
        'rviz',
        'map_builder_fixed.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        params_file_arg,
        oakd_params_file_arg,
        device_id_arg,
        use_rviz_arg,
        debug_arg,
        
        # OAK-D driver
        oakd_driver_launch,
        
        # TF setup
        tf_oak_to_base,
        tf_map_to_odom,
        tf_odom_to_base,
        
        # Enhanced mapping pipeline
        enhanced_visual_odometry_node,
        point_cloud_processor_node,
        map_builder_node,
        
        # Visualization
        rviz_node,
    ])
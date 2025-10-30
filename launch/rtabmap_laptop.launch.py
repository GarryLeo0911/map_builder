#!/usr/bin/env python3
"""
Laptop-side RTABMap launch for ROS2 Jazzy that subscribes to OAK-D topics from robot
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='oak',
        description='Camera name for topic remapping'
    )
    
    delete_db_arg = DeclareLaunchArgument(
        'delete_db_on_start',
        default_value='false',
        description='Delete existing RTABMap database on start'
    )
    
    camera_name = LaunchConfiguration('camera_name')
    delete_db = LaunchConfiguration('delete_db_on_start')
    
    # Static transform for camera (required for RTABMap)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'oak_rgb_camera_optical_frame'],
        name='camera_base_tf'
    )
    
    # RGB-D Sync (RTABMap requirement for ROS2 Jazzy)
    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        parameters=[{
            'approx_sync': True,
            'sync_queue_size': 10,  # Use the new parameter name
            'approx_sync_max_interval': 0.01,
        }],
        remappings=[
            ('/rgb/image', '/oak/rgb/image_raw'),
            ('/depth/image', '/oak/depth/image_raw'),
            ('/rgb/camera_info', '/oak/rgb/camera_info'),
            ('/depth/camera_info', '/oak/rgb/camera_info'),  # Use RGB camera info for depth if depth camera info not available
        ]
    )
    
    # RTABMap Visual Odometry (optimized for ROS2 Jazzy)
    rtabmap_odom = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rtabmap_odom',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'wait_for_transform': 0.2,
            'approx_sync': True,
            # Optimized parameters for OAK-D
            'Odom/Strategy': '1',  # Frame-to-frame
            'Vis/EstimationType': '1',  # 3D-to-2D (PnP)
            'Vis/MaxFeatures': '1000',
            'Vis/MinInliers': '15',
        }],
        remappings=[
            ('/rgb/image', '/rgbd_sync/rgb/image'),
            ('/depth/image', '/rgbd_sync/depth/image'),
            ('/rgb/camera_info', '/rgbd_sync/rgb/camera_info'),
        ]
    )
    
    # RTABMap SLAM (main mapping node for ROS2 Jazzy)
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[{
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_odom_info': True,
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'approx_sync': True,
            'wait_for_transform': 0.2,
            'database_path': '~/.ros/rtabmap.db',
            # Mapping parameters optimized for OAK-D
            'RGBD/CreateOccupancyGrid': 'true',
            'Grid/CellSize': '0.05',
            'Grid/3D': 'true',
            'Rtabmap/DetectionRate': '1.0',
            'Mem/RehearsalSimilarity': '0.30',
            'Vis/MaxFeatures': '400',
            'Vis/MinInliers': '20',
        }],
        remappings=[
            ('/rgb/image', '/rgbd_sync/rgb/image'),
            ('/depth/image', '/rgbd_sync/depth/image'),
            ('/rgb/camera_info', '/rgbd_sync/rgb/camera_info'),
        ],
        arguments=['--delete_db_on_start'] if delete_db else []
    )
    
    return LaunchDescription([
        camera_name_arg,
        delete_db_arg,
        static_tf,
        rgbd_sync,
        rtabmap_odom,
        rtabmap_slam
    ])
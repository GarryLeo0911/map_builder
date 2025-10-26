import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

pkg_share = get_package_share_directory('map_builder')
CONFIG_DIR = os.path.join(pkg_share, 'config')


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='point_cloud_io',
            executable='read',
            name='read_stl_node',
            output='screen',
            parameters=[{
                'file_path': os.path.join(CONFIG_DIR, '../../test/map_medium.ply'),
                'topic': '/test_point_cloud',
                'frame': 'initial_base',
                'rate': 0.2,
            }],
        ),
        Node(
            package='map_builder',
            executable='test_mesh_service.py',
            name='test_mesh_service',
            output='screen',
        ),
    ])

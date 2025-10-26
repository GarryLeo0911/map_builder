from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('map_builder')
    config_file = os.path.join(pkg_share, 'config', 'basic_recon_v3.yaml')

    return LaunchDescription([
        SetEnvironmentVariable('ROSCONSOLE_CONFIG_FILE', os.path.join(pkg_share, 'config', 'general.config')),
        Node(
            package='map_builder',
            executable='map_builder_service',
            name='recon_service_node',
            output='screen',
            parameters=[config_file]
        )
    ])

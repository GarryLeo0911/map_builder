import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

pkg_share = get_package_share_directory('map_builder')
CONFIG_DIR = os.path.join(pkg_share, 'config')


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('ROSCONSOLE_CONFIG_FILE', os.path.join(CONFIG_DIR, 'general.config')),
        Node(
            package='map_builder',
            executable='p_normal_service',
            name='p_normal_node',
            output='screen',
            parameters=[os.path.join(CONFIG_DIR, 'basic_recon_v4_ufmg.yaml')],
        ),
    ])

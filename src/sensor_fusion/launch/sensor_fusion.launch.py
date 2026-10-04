import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sensor_fusion')
    config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')

    return LaunchDescription([
        Node(
            package='sensor_fusion',
            executable='fusion_node',
            name='fusion_node',
            output='screen',
            parameters=[config_file],
        )
    ])

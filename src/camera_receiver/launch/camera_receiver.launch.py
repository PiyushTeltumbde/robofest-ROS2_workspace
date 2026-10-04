import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('camera_receiver')
    config_file = os.path.join(pkg_share, 'config', 'camera.yaml')

    return LaunchDescription([
        Node(
            package='camera_receiver',
            executable='tcp_frame_receiver',
            name='camera_receiver',
            output='screen',
            parameters=[config_file],
        )
    ])

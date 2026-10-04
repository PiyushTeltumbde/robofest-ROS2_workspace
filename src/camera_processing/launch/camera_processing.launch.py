import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('camera_processing')
    config_file = os.path.join(pkg_share, 'config', 'camera.yaml')

    return LaunchDescription([
        Node(
            package='camera_processing',
            executable='image_decoder',
            name='image_decoder',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='camera_processing',
            executable='camera_info_publisher',
            name='camera_info_publisher',
            output='screen',
            parameters=[config_file],
        ),
    ])

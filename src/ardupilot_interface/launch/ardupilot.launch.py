import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ardupilot_interface')
    config_file = os.path.join(pkg_share, 'config', 'ardupilot.yaml')

    return LaunchDescription([
        Node(
            package='ardupilot_interface',
            executable='ardupilot_node',
            name='ardupilot_node',
            output='screen',
            parameters=[config_file],
        )
    ])

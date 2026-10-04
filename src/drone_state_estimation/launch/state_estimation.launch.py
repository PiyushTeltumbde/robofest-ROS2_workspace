import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('drone_state_estimation')
    config_file = os.path.join(pkg_share, 'config', 'state_estimator.yaml')

    return LaunchDescription([
        Node(
            package='drone_state_estimation',
            executable='state_estimator_node',
            name='state_estimator_node',
            output='screen',
            parameters=[config_file],
        )
    ])

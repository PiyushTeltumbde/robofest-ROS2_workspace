import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mission_planner')
    config_file = os.path.join(pkg_share, 'config', 'planner.yaml')

    return LaunchDescription([
        Node(
            package='mission_planner',
            executable='planner_node',
            name='planner_node',
            output='screen',
            parameters=[config_file],
        )
    ])

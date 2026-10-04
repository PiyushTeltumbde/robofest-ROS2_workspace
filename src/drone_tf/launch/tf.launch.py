from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_tf',
            executable='tf_broadcaster',
            name='drone_tf_broadcaster',
            output='screen',
        )
    ])

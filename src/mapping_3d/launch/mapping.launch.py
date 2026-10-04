import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mapping_3d')
    config_file = os.path.join(pkg_share, 'config', 'mapping.yaml')

    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='/drone/odometry',
        description='Pose/Odometry topic source (/drone/odometry or /orbslam/odometry)'
    )

    mapping_node = Node(
        package='mapping_3d',
        executable='mapping_node',
        name='mapping_node',
        output='screen',
        parameters=[
            config_file,
            {
                'pose_topic': LaunchConfiguration('pose_topic')
            }
        ],
    )

    return LaunchDescription([
        pose_topic_arg,
        mapping_node,
    ])

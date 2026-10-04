import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('orbslam3_wrapper')

    default_voc = '/home/harsh-mahamuni/ORB_SLAM3/Vocabulary/ORBvoc.txt'
    default_settings = os.path.join(pkg_share, 'config', 'orbslam3.yaml')

    declare_voc_cmd = DeclareLaunchArgument(
        'vocabulary_file',
        default_value=default_voc,
        description='Path to ORB-SLAM3 vocabulary file'
    )

    declare_settings_cmd = DeclareLaunchArgument(
        'settings_file',
        default_value=default_settings,
        description='Path to ORB-SLAM3 camera calibration YAML file'
    )

    declare_sensor_cmd = DeclareLaunchArgument(
        'sensor_mode',
        default_value='MONOCULAR',
        description='Sensor mode: MONOCULAR, STEREO, RGBD'
    )

    declare_camera_frame_cmd = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_optical_frame',
        description='Camera optical frame ID'
    )

    declare_viewer_cmd = DeclareLaunchArgument(
        'use_viewer',
        default_value='false',
        description='Whether to enable Pangolin graphical viewer'
    )

    orbslam3_node = Node(
        package='orbslam3_wrapper',
        executable='orbslam3_node',
        name='orbslam3_node',
        output='screen',
        parameters=[{
            'vocabulary_file': LaunchConfiguration('vocabulary_file'),
            'settings_file': LaunchConfiguration('settings_file'),
            'sensor_mode': LaunchConfiguration('sensor_mode'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'use_viewer': LaunchConfiguration('use_viewer'),
        }]
    )

    return LaunchDescription([
        declare_voc_cmd,
        declare_settings_cmd,
        declare_sensor_cmd,
        declare_camera_frame_cmd,
        declare_viewer_cmd,
        orbslam3_node
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    receiver_pkg = get_package_share_directory('camera_receiver')
    processing_pkg = get_package_share_directory('camera_processing')

    receiver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(receiver_pkg, 'launch', 'camera_receiver.launch.py')
        )
    )

    processing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(processing_pkg, 'launch', 'camera_processing.launch.py')
        )
    )

    return LaunchDescription([
        receiver_launch,
        processing_launch,
    ])

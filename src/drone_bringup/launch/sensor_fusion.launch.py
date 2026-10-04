import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    fusion_pkg = get_package_share_directory('sensor_fusion')
    estimation_pkg = get_package_share_directory('drone_state_estimation')

    fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fusion_pkg, 'launch', 'sensor_fusion.launch.py')
        )
    )

    estimation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(estimation_pkg, 'launch', 'state_estimation.launch.py')
        )
    )

    return LaunchDescription([
        fusion_launch,
        estimation_launch,
    ])

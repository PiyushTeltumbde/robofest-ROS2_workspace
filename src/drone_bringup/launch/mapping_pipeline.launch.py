import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    mapping_pkg = get_package_share_directory('mapping_3d')

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mapping_pkg, 'launch', 'mapping.launch.py')
        )
    )

    return LaunchDescription([
        mapping_launch,
    ])

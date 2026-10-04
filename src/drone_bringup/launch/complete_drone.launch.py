import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_pkg = get_package_share_directory('drone_bringup')
    tf_pkg = get_package_share_directory('drone_tf')
    ardupilot_pkg = get_package_share_directory('ardupilot_interface')
    planner_pkg = get_package_share_directory('mission_planner')

    # Launch Configurations & Toggles
    enable_tf = LaunchConfiguration('enable_tf')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_slam = LaunchConfiguration('enable_slam')
    enable_fusion = LaunchConfiguration('enable_fusion')
    enable_mapping = LaunchConfiguration('enable_mapping')
    enable_ardupilot = LaunchConfiguration('enable_ardupilot')
    enable_planner = LaunchConfiguration('enable_planner')

    declare_tf = DeclareLaunchArgument('enable_tf', default_value='true',
                                       description='Enable TF coordinate frames broadcaster')
    declare_camera = DeclareLaunchArgument('enable_camera', default_value='true',
                                           description='Enable camera receiver & processing pipeline')
    declare_slam = DeclareLaunchArgument('enable_slam', default_value='true',
                                         description='Enable ORB-SLAM3 visual odometry pipeline')
    declare_fusion = DeclareLaunchArgument('enable_fusion', default_value='true',
                                           description='Enable EKF sensor fusion & state estimation')
    declare_mapping = DeclareLaunchArgument('enable_mapping', default_value='true',
                                            description='Enable 3D pointcloud & occupancy mapping')
    declare_ardupilot = DeclareLaunchArgument('enable_ardupilot', default_value='true',
                                              description='Enable ArduPilot MAVLink interface')
    declare_planner = DeclareLaunchArgument('enable_planner', default_value='true',
                                            description='Enable autonomous SAR mission planner')

    # 1. Drone Coordinate Frames (TF)
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tf_pkg, 'launch', 'tf.launch.py')
        ),
        condition=IfCondition(enable_tf)
    )

    # 2. Camera Ingestion & Processing Pipeline
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'camera_pipeline.launch.py')
        ),
        condition=IfCondition(enable_camera)
    )

    # 3. Visual SLAM Pipeline (ORB-SLAM3)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'slam_pipeline.launch.py')
        ),
        condition=IfCondition(enable_slam)
    )

    # 4. Sensor Fusion & State Estimation Pipeline
    fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'sensor_fusion.launch.py')
        ),
        condition=IfCondition(enable_fusion)
    )

    # 5. 3D Mapping Pipeline
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'mapping_pipeline.launch.py')
        ),
        condition=IfCondition(enable_mapping)
    )

    # 6. ArduPilot Flight Controller Interface
    ardupilot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ardupilot_pkg, 'launch', 'ardupilot.launch.py')
        ),
        condition=IfCondition(enable_ardupilot)
    )

    # 7. SAR Mission & Path Planner
    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planner_pkg, 'launch', 'planner.launch.py')
        ),
        condition=IfCondition(enable_planner)
    )

    return LaunchDescription([
        declare_tf,
        declare_camera,
        declare_slam,
        declare_fusion,
        declare_mapping,
        declare_ardupilot,
        declare_planner,
        tf_launch,
        camera_launch,
        slam_launch,
        fusion_launch,
        mapping_launch,
        ardupilot_launch,
        planner_launch,
    ])

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    gui = Node(
        package='camera_calibration',
        executable='camera_calibration_gui',
        name='camera_calibration_gui',
        output='screen',
    )

    calibration_tf = Node(
        package='camera_calibration',
        executable='calibration_perception',
        name='calibration_perception',
        output='screen',
        parameters=[{
            'marker_prefix': 'aruco_marker',
            'marker_ids': [1, 2, 3, 4],
            'parent_frame': 'camera_link',
            'output_frame': 'tag_frame',
            'publish_rate': 20.0,
            'lookup_timeout': 0.05,
        }],
    )

    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('aruco_perception'),
                         'launch', 'aruco_perception.launch.py')),
        launch_arguments={
            'use_calibration': 'false',
            'parent_frame': 'Link6',
            'child_frame': 'calibrated_camera_link',
        }.items(),
    )

    return LaunchDescription([gui, calibration_tf, aruco_launch])

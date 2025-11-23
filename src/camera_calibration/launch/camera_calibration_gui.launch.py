from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gui = Node(
        package='camera_calibration',
        executable='camera_calibration_gui',
        name='camera_calibration_gui',
        output='screen',
    )
    return LaunchDescription([gui])

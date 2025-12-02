from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  use_calibration = LaunchConfiguration("use_calibration", default="true")
  parent_frame = LaunchConfiguration("parent_frame", default="Link6")
  child_frame = LaunchConfiguration("child_frame", default="calibrated_camera_link")

  return LaunchDescription(
    [
      DeclareLaunchArgument("use_calibration", default_value="true",
                            description="Auto-load calibration YAML; set false to force identity."),
      DeclareLaunchArgument("parent_frame", default_value="Link6",
                            description="Parent frame for calibrated camera."),
      DeclareLaunchArgument("child_frame", default_value="calibrated_camera_link",
                            description="Name of calibrated camera frame."),
      Node(
        package="aruco_perception",
        executable="perception_calibration",
        name="perception_calibration",
        output="screen",
        parameters=[{
          "parent_frame": parent_frame,
          "child_frame": child_frame,
          "auto_discover": use_calibration,
          "identity_on_missing": True,
          # leave calibration_file empty; auto_discover controls whether to search
        }],
      ),
      Node(
        package="aruco_perception",
        executable="aruco_detector_node",
        name="aruco_perception",
        output="screen",
      ),
    ]
  )

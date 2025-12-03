import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  return LaunchDescription(
    [
      DeclareLaunchArgument("use_calibration", default_value="true",
                            description="Auto-load calibration YAML; set false to skip."),
      DeclareLaunchArgument("parent_frame", default_value="Link6",
                            description="Parent frame for calibrated camera."),
      DeclareLaunchArgument("child_frame", default_value="calibrated_camera_link",
                            description="Name of calibrated camera frame."),
      DeclareLaunchArgument("calibration_dir", default_value="~/DOBOT_pickn_place/calibration",
                            description="Directory to search for calibration YAMLs."),
      DeclareLaunchArgument("calibration_file", default_value="",
                            description="Explicit calibration YAML path (overrides discovery)."),
      OpaqueFunction(function=launch_setup),
    ]
  )


def launch_setup(context, *args, **kwargs):
  use_calibration = LaunchConfiguration("use_calibration").perform(context).lower() == "true"
  parent_frame = LaunchConfiguration("parent_frame").perform(context)
  child_frame = LaunchConfiguration("child_frame").perform(context)
  calibration_dir = os.path.expanduser(LaunchConfiguration("calibration_dir").perform(context))
  explicit_file = os.path.expanduser(LaunchConfiguration("calibration_file").perform(context))

  selected_file = ""
  if explicit_file:
    selected_file = explicit_file
  elif use_calibration:
    selected_file = find_latest_calibration(calibration_dir)

  nodes = []
  if selected_file:
    print(f"[aruco_perception.launch] Using calibration file: {selected_file}")
    nodes.append(
      Node(
        package="aruco_perception",
        executable="perception_calibration",
        name="perception_calibration",
        output="screen",
        parameters=[{
          "parent_frame": parent_frame,
          "child_frame": child_frame,
          "auto_discover": False,
          "calibration_file": selected_file,
        }],
      )
    )
    camera_frame = child_frame
  else:
    if use_calibration:
      print(
        "[aruco_perception.launch] No non-empty calibration YAML found in "
        f"{calibration_dir}. Press Enter to continue WITHOUT calibration "
        f"(poses will be in {parent_frame}), or Ctrl+C to abort."
      )
      try:
        input()
      except EOFError:
        pass
    camera_frame = parent_frame

  nodes.append(
    Node(
      package="aruco_perception",
      executable="aruco_detector_node",
      name="aruco_perception",
      output="screen",
      parameters=[{
        "camera_frame": camera_frame,
      }],
    )
  )
  return nodes


def find_latest_calibration(calibration_dir: str) -> str:
  try:
    base = Path(calibration_dir).expanduser()
    if not base.exists() or not base.is_dir():
      return ""
    yaml_files = [
      p for p in base.iterdir()
      if p.is_file() and p.suffix == ".yaml" and p.stat().st_size > 0
    ]
    if not yaml_files:
      return ""
    latest = max(yaml_files, key=lambda p: p.stat().st_mtime)
    return str(latest)
  except Exception as exc:
    print(f"[aruco_perception.launch] Failed to search calibrations in {calibration_dir}: {exc}")
    return ""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _workspace_root() -> Path:
    def looks_like_root(path: Path) -> bool:
        return (
            (path / "src").exists()
            and ((path / "README.md").exists() or (path / "src" / "dobot_msgs_v4").exists())
        )

    for name in ("DOBOT_PICKN_PLACE_ROOT", "DOBOT_WORKSPACE_ROOT"):
        value = os.environ.get(name)
        if value:
            return Path(value).expanduser().resolve()

    for start in (Path.cwd(), Path(__file__).resolve()):
        path = start.expanduser().resolve()
        if path.is_file():
            path = path.parent
        for candidate in (path, *path.parents):
            if looks_like_root(candidate):
                return candidate
    return Path.cwd().resolve()


def _repo_path(*parts: str) -> str:
    return str(_workspace_root().joinpath(*parts))


def _ros_domain_action():
    import importlib.util

    helper_candidates = []
    for parent in Path(__file__).resolve().parents:
        helper_candidates.extend([
            parent / "src" / "dobot_bringup_v4" / "launch" / "ros_domain.py",
            parent / "install" / "cr_robot_ros2" / "share" / "cr_robot_ros2" / "launch" / "ros_domain.py",
            parent / "cr_robot_ros2" / "share" / "cr_robot_ros2" / "launch" / "ros_domain.py",
            parent / "share" / "cr_robot_ros2" / "launch" / "ros_domain.py",
        ])

    for helper_path in helper_candidates:
        if helper_path.exists():
            spec = importlib.util.spec_from_file_location("_dobot_ros_domain", helper_path)
            if spec is None or spec.loader is None:
                continue
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module.ros_domain_action()

    raise RuntimeError("Could not find ros_domain.py helper for ROS_DOMAIN_ID")


def generate_launch_description():
    color_topic = LaunchConfiguration("color_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    use_calibration = LaunchConfiguration("use_calibration")
    calibration_parent_frame = LaunchConfiguration("calibration_parent_frame")
    calibration_child_frame = LaunchConfiguration("calibration_child_frame")
    calibration_dir = LaunchConfiguration("calibration_dir")
    calibration_file = LaunchConfiguration("calibration_file")
    calibration_file_prefix = LaunchConfiguration("calibration_file_prefix")
    robot_ip_address = LaunchConfiguration("robot_ip_address")
    show_aruco_overlay = LaunchConfiguration("show_aruco_overlay")
    publish_aruco_overlay = LaunchConfiguration("publish_aruco_overlay")
    overlay_topic = LaunchConfiguration("overlay_topic")
    use_aruco_overlay = LaunchConfiguration("use_aruco_overlay")
    show_bin_roi_dots_overlay = LaunchConfiguration("show_bin_roi_dots_overlay")
    publish_marker_tfs = LaunchConfiguration("publish_marker_tfs")
    aruco_overlay_rate_hz = LaunchConfiguration("aruco_overlay_rate_hz")
    detections_topic = LaunchConfiguration("detections_topic")
    output_dir = LaunchConfiguration("output_dir")
    bin_name = LaunchConfiguration("bin_name")
    base_frame = LaunchConfiguration("base_frame")
    platform_frame = LaunchConfiguration("platform_frame")
    platform_parent_frame = LaunchConfiguration("platform_parent_frame")
    platform_calibration_dir = LaunchConfiguration("platform_calibration_dir")
    platform_calibration_file = LaunchConfiguration("platform_calibration_file")

    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("aruco_perception"),
                "launch",
                "aruco_perception.launch.py",
            )
        ),
        launch_arguments={
            "use_calibration": use_calibration,
            "parent_frame": calibration_parent_frame,
            "child_frame": calibration_child_frame,
            "calibration_dir": calibration_dir,
            "calibration_file": calibration_file,
            "calibration_file_prefix": calibration_file_prefix,
            "robot_ip_address": robot_ip_address,
            "show_overlay_window": show_aruco_overlay,
            "publish_overlay": publish_aruco_overlay,
            "overlay_topic": overlay_topic,
            "show_bin_roi_dots_overlay": show_bin_roi_dots_overlay,
            "publish_marker_tfs": publish_marker_tfs,
            "overlay_rate_hz": aruco_overlay_rate_hz,
            "detections_topic": detections_topic,
            "color_topic": color_topic,
            "depth_topic": depth_topic,
            "camera_info_topic": camera_info_topic,
        }.items(),
    )

    bin_teach_node = Node(
        package="item_perception_yolo",
        executable="bin_teach_yolo_node.py",
        name="bin_teach_yolo",
        output="screen",
        parameters=[{
            "color_topic": color_topic,
            "depth_topic": depth_topic,
            "camera_info_topic": camera_info_topic,
            "overlay_topic": overlay_topic,
            "use_aruco_overlay": ParameterValue(use_aruco_overlay, value_type=bool),
            "detections_topic": detections_topic,
            "output_dir": output_dir,
            "bin_name": bin_name,
            "marker_prefix": LaunchConfiguration("marker_prefix"),
            "bin_frame_prefix": LaunchConfiguration("bin_frame_prefix"),
            "required_marker_count": ParameterValue(
                LaunchConfiguration("required_marker_count"),
                value_type=int,
            ),
            "max_detection_age_sec": ParameterValue(
                LaunchConfiguration("max_detection_age_sec"),
                value_type=float,
            ),
            "tf_lookup_timeout_sec": ParameterValue(
                LaunchConfiguration("tf_lookup_timeout_sec"),
                value_type=float,
            ),
            "base_frame": base_frame,
            "platform_frame": platform_frame,
            "platform_parent_frame": platform_parent_frame,
            "platform_calibration_dir": platform_calibration_dir,
            "platform_calibration_file": platform_calibration_file,
            "robot_ip_address": robot_ip_address,
            "publish_static_platform_tf": ParameterValue(
                LaunchConfiguration("publish_static_platform_tf"),
                value_type=bool,
            ),
            "headless": ParameterValue(LaunchConfiguration("headless"), value_type=bool),
            "auto_save": ParameterValue(LaunchConfiguration("auto_save"), value_type=bool),
            "save_once": ParameterValue(LaunchConfiguration("save_once"), value_type=bool),
        }],
    )

    return LaunchDescription([
        _ros_domain_action(),
        DeclareLaunchArgument("color_topic", default_value="/bin_camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/bin_camera/depth/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/bin_camera/color/camera_info"),
        DeclareLaunchArgument("use_calibration", default_value="true"),
        DeclareLaunchArgument("calibration_parent_frame", default_value="base_link"),
        DeclareLaunchArgument("calibration_child_frame", default_value="bin_calibrated_camera_link"),
        DeclareLaunchArgument("calibration_dir", default_value=_repo_path("calibration")),
        DeclareLaunchArgument("calibration_file", default_value=""),
        DeclareLaunchArgument("calibration_file_prefix", default_value="axab_calibration_eyetohand_"),
        DeclareLaunchArgument("robot_ip_address", default_value=""),
        DeclareLaunchArgument("show_aruco_overlay", default_value="false"),
        DeclareLaunchArgument("publish_aruco_overlay", default_value="true"),
        DeclareLaunchArgument("overlay_topic", default_value="/aruco_overlay"),
        DeclareLaunchArgument("use_aruco_overlay", default_value="true"),
        DeclareLaunchArgument("show_bin_roi_dots_overlay", default_value="true"),
        DeclareLaunchArgument("publish_marker_tfs", default_value="true"),
        DeclareLaunchArgument("aruco_overlay_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("detections_topic", default_value="/aruco_detections"),
        DeclareLaunchArgument("output_dir", default_value=_repo_path("teach", "bin_teach")),
        DeclareLaunchArgument("bin_name", default_value=""),
        DeclareLaunchArgument("marker_prefix", default_value="aruco_marker"),
        DeclareLaunchArgument("bin_frame_prefix", default_value="bin"),
        DeclareLaunchArgument("required_marker_count", default_value="4"),
        DeclareLaunchArgument("max_detection_age_sec", default_value="0.75"),
        DeclareLaunchArgument("tf_lookup_timeout_sec", default_value="0.2"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("platform_frame", default_value="platform_reference"),
        DeclareLaunchArgument("platform_parent_frame", default_value="base_link"),
        DeclareLaunchArgument("platform_calibration_dir", default_value=_repo_path("calibration")),
        DeclareLaunchArgument("platform_calibration_file", default_value=""),
        DeclareLaunchArgument("publish_static_platform_tf", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("auto_save", default_value="false"),
        DeclareLaunchArgument("save_once", default_value="true"),
        aruco_launch,
        bin_teach_node,
    ])

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ROBOT_CAMERA_COLOR_TOPIC = "/robot_camera/color/image_raw"
ROBOT_CAMERA_DEPTH_TOPIC = "/robot_camera/depth/image_raw"
ROBOT_CAMERA_INFO_TOPIC = "/robot_camera/color/camera_info"
ROBOT_CAMERA_CONTROL_SERVICE_ROOT = "/robot_camera"


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
    params_file = LaunchConfiguration("params_file")
    color_topic = LaunchConfiguration("color_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    motion_service_root = LaunchConfiguration("motion_service_root")
    tray_name = LaunchConfiguration("tray_name")
    bin_teach_dir = LaunchConfiguration("bin_teach_dir")
    runtime_root = LaunchConfiguration("runtime_root")
    saved_sessions_root = LaunchConfiguration("saved_sessions_root")
    runtime_settings_path = LaunchConfiguration("runtime_settings_path")
    profile_dir = LaunchConfiguration("profile_dir")
    model_root = LaunchConfiguration("model_root")
    projection_camera_frame = LaunchConfiguration("projection_camera_frame")
    overlay_topic = LaunchConfiguration("overlay_topic")
    calibration_dir = LaunchConfiguration("calibration_dir")
    calibration_file = LaunchConfiguration("calibration_file")
    robot_ip_address = LaunchConfiguration("robot_ip_address")
    publish_static_calibration_tfs = LaunchConfiguration("publish_static_calibration_tfs")
    camera_control_service_root = LaunchConfiguration("camera_control_service_root")
    color_exposure_min_us = LaunchConfiguration("color_exposure_min_us")
    color_exposure_max_us = LaunchConfiguration("color_exposure_max_us")
    depth_exposure_min_us = LaunchConfiguration("depth_exposure_min_us")
    depth_exposure_max_us = LaunchConfiguration("depth_exposure_max_us")
    sam2_checkpoint = LaunchConfiguration("sam2_checkpoint")
    sam2_config = LaunchConfiguration("sam2_config")
    yolo_base_model = LaunchConfiguration("yolo_base_model")
    yolo_preview_enabled = LaunchConfiguration("yolo_preview_enabled")
    yolo_preview_conf = LaunchConfiguration("yolo_preview_conf")
    yolo_preview_iou = LaunchConfiguration("yolo_preview_iou")
    yolo_preview_imgsz = LaunchConfiguration("yolo_preview_imgsz")
    yolo_preview_max_hz = LaunchConfiguration("yolo_preview_max_hz")
    train_epochs = LaunchConfiguration("train_epochs")
    train_imgsz = LaunchConfiguration("train_imgsz")
    train_device = LaunchConfiguration("train_device")
    train_use_gpu_if_available = LaunchConfiguration("train_use_gpu_if_available")
    record_fps = LaunchConfiguration("record_fps")
    display_scale = LaunchConfiguration("display_scale")
    live_view_enabled = LaunchConfiguration("live_view_enabled")
    overlay_enabled = LaunchConfiguration("overlay_enabled")
    clear_runtime_on_start = LaunchConfiguration("clear_runtime_on_start")
    python_executable = LaunchConfiguration("python_executable")

    return LaunchDescription([
        _ros_domain_action(),
        DeclareLaunchArgument(
            "params_file",
            default_value=_repo_path("src", "tray_perception_yolo", "config", "tray_teach_yolo.yaml"),
        ),
        DeclareLaunchArgument("color_topic", default_value=ROBOT_CAMERA_COLOR_TOPIC),
        DeclareLaunchArgument("depth_topic", default_value=ROBOT_CAMERA_DEPTH_TOPIC),
        DeclareLaunchArgument("camera_info_topic", default_value=ROBOT_CAMERA_INFO_TOPIC),
        DeclareLaunchArgument("motion_service_root", default_value="/dobot_bringup_ros2/srv"),
        DeclareLaunchArgument("tray_name", default_value=""),
        DeclareLaunchArgument("bin_teach_dir", default_value=_repo_path("teach", "bin_teach")),
        DeclareLaunchArgument(
            "runtime_root",
            default_value=_repo_path("config", "tray_perception_yolo", "tray_teach_yolo_runtime"),
        ),
        DeclareLaunchArgument(
            "saved_sessions_root",
            default_value=_repo_path("config", "tray_perception_yolo", "tray_teach_yolo_saved_sessions"),
        ),
        DeclareLaunchArgument(
            "runtime_settings_path",
            default_value=_repo_path("config", "tray_perception_yolo", "tray_teach_yolo_runtime_settings.yaml"),
        ),
        DeclareLaunchArgument("profile_dir", default_value=_repo_path("teach", "tray_teach_yolo")),
        DeclareLaunchArgument("model_root", default_value=_repo_path("teach", "tray_teach_yolo")),
        DeclareLaunchArgument("projection_camera_frame", default_value="arm_calibrated_camera_link"),
        DeclareLaunchArgument("overlay_topic", default_value="tray_overlay"),
        DeclareLaunchArgument("calibration_dir", default_value=_repo_path("calibration")),
        DeclareLaunchArgument("calibration_file", default_value=""),
        DeclareLaunchArgument(
            "robot_ip_address",
            default_value="",
            description="Robot controller IP for eye-on-hand calibration discovery. Empty uses ROBOT_IP_ADDRESS/station_config.",
        ),
        DeclareLaunchArgument("publish_static_calibration_tfs", default_value="true"),
        DeclareLaunchArgument("camera_control_service_root", default_value=ROBOT_CAMERA_CONTROL_SERVICE_ROOT),
        DeclareLaunchArgument("color_exposure_min_us", default_value="1"),
        DeclareLaunchArgument("color_exposure_max_us", default_value="100"),
        DeclareLaunchArgument("depth_exposure_min_us", default_value="1"),
        DeclareLaunchArgument("depth_exposure_max_us", default_value="32000"),
        DeclareLaunchArgument(
            "sam2_checkpoint",
            default_value=_repo_path("third_party", "sam2", "checkpoints", "sam2.1_hiera_tiny.pt"),
        ),
        DeclareLaunchArgument("sam2_config", default_value="configs/sam2.1/sam2.1_hiera_t.yaml"),
        DeclareLaunchArgument(
            "yolo_base_model",
            default_value=_repo_path("third_party", "yolo", "checkpoints", "yolo11n-seg.pt"),
        ),
        DeclareLaunchArgument("yolo_preview_enabled", default_value="true"),
        DeclareLaunchArgument("yolo_preview_conf", default_value="0.35"),
        DeclareLaunchArgument("yolo_preview_iou", default_value="0.45"),
        DeclareLaunchArgument("yolo_preview_imgsz", default_value="640"),
        DeclareLaunchArgument("yolo_preview_max_hz", default_value="3.0"),
        DeclareLaunchArgument("train_epochs", default_value="80"),
        DeclareLaunchArgument("train_imgsz", default_value="640"),
        DeclareLaunchArgument("train_device", default_value="0"),
        DeclareLaunchArgument("train_use_gpu_if_available", default_value="true"),
        DeclareLaunchArgument("record_fps", default_value="5.0"),
        DeclareLaunchArgument("display_scale", default_value="1.0"),
        DeclareLaunchArgument("live_view_enabled", default_value="true"),
        DeclareLaunchArgument("overlay_enabled", default_value="true"),
        DeclareLaunchArgument("clear_runtime_on_start", default_value="false"),
        DeclareLaunchArgument(
            "python_executable",
            default_value=_repo_path(".venv", "bin", "python"),
        ),
        Node(
            package="tray_perception_yolo",
            executable="tray_teach_yolo_node.py",
            name="tray_teach_yolo",
            output="screen",
            prefix=[python_executable, " "],
            parameters=[
                params_file,
                {
                    "color_topic": color_topic,
                    "depth_topic": depth_topic,
                    "camera_info_topic": camera_info_topic,
                    "motion_service_root": motion_service_root,
                    "tray_name": tray_name,
                    "bin_teach_dir": bin_teach_dir,
                    "runtime_root": runtime_root,
                    "saved_sessions_root": saved_sessions_root,
                    "runtime_settings_path": runtime_settings_path,
                    "profile_dir": profile_dir,
                    "model_root": model_root,
                    "projection_camera_frame": projection_camera_frame,
                    "overlay_topic": overlay_topic,
                    "calibration_dir": calibration_dir,
                    "calibration_file": calibration_file,
                    "robot_ip_address": robot_ip_address,
                    "publish_static_calibration_tfs": ParameterValue(publish_static_calibration_tfs, value_type=bool),
                    "camera_control_service_root": camera_control_service_root,
                    "color_exposure_min_us": ParameterValue(color_exposure_min_us, value_type=int),
                    "color_exposure_max_us": ParameterValue(color_exposure_max_us, value_type=int),
                    "depth_exposure_min_us": ParameterValue(depth_exposure_min_us, value_type=int),
                    "depth_exposure_max_us": ParameterValue(depth_exposure_max_us, value_type=int),
                    "sam2_checkpoint": sam2_checkpoint,
                    "sam2_config": sam2_config,
                    "yolo_base_model": yolo_base_model,
                    "yolo_preview_enabled": ParameterValue(yolo_preview_enabled, value_type=bool),
                    "yolo_preview_conf": ParameterValue(yolo_preview_conf, value_type=float),
                    "yolo_preview_iou": ParameterValue(yolo_preview_iou, value_type=float),
                    "yolo_preview_imgsz": ParameterValue(yolo_preview_imgsz, value_type=int),
                    "yolo_preview_max_hz": ParameterValue(yolo_preview_max_hz, value_type=float),
                    "train_epochs": ParameterValue(train_epochs, value_type=int),
                    "train_imgsz": ParameterValue(train_imgsz, value_type=int),
                    "train_device": ParameterValue(train_device, value_type=str),
                    "train_use_gpu_if_available": ParameterValue(train_use_gpu_if_available, value_type=bool),
                    "record_fps": ParameterValue(record_fps, value_type=float),
                    "display_scale": ParameterValue(display_scale, value_type=float),
                    "live_view_enabled": ParameterValue(live_view_enabled, value_type=bool),
                    "overlay_enabled": ParameterValue(overlay_enabled, value_type=bool),
                    "clear_runtime_on_start": ParameterValue(clear_runtime_on_start, value_type=bool),
                },
            ],
        ),
    ])

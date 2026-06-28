import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ROBOT_CAMERA_COLOR_TOPIC = "/robot_camera/color/image_raw"
ROBOT_CAMERA_DEPTH_TOPIC = "/robot_camera/depth/image_raw"
ROBOT_CAMERA_INFO_TOPIC = "/robot_camera/color/camera_info"


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


def _calibration_selection_helper():
    import importlib.util

    helper_candidates = []
    for parent in Path(__file__).resolve().parents:
        helper_candidates.extend([
            parent / "src" / "dobot_bringup_v4" / "launch" / "calibration_selection.py",
            parent / "install" / "cr_robot_ros2" / "share" / "cr_robot_ros2" / "launch" / "calibration_selection.py",
            parent / "cr_robot_ros2" / "share" / "cr_robot_ros2" / "launch" / "calibration_selection.py",
            parent / "share" / "cr_robot_ros2" / "launch" / "calibration_selection.py",
        ])

    for helper_path in helper_candidates:
        if helper_path.exists():
            spec = importlib.util.spec_from_file_location("_dobot_calibration_selection", helper_path)
            if spec is None or spec.loader is None:
                continue
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module

    raise RuntimeError("Could not find calibration_selection.py helper")


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


def _to_bool(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


def _sanitize_filename_token(value: str) -> str:
    token = []
    previous_underscore = False
    for ch in str(value or "").strip():
        if ch.isalnum() or ch in "._-":
            token.append(ch)
            previous_underscore = False
        elif not previous_underscore:
            token.append("_")
            previous_underscore = True
    return "".join(token).strip("_")


def _unquote_config_value(value: str) -> str:
    text = str(value or "").strip()
    if len(text) >= 2 and text[0] == text[-1] and text[0] in ("'", '"'):
        text = text[1:-1]
    return text.strip()


def _station_config_value(*keys: str) -> str:
    try:
        values = {}
        with Path(_repo_path("station_config")).open("r", encoding="utf-8") as stream:
            for raw_line in stream:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                if line.startswith("export "):
                    line = line[len("export "):].strip()
                if "=" not in line:
                    continue
                key, value = line.split("=", 1)
                values[key.strip()] = _unquote_config_value(value)
    except OSError:
        return ""
    for key in keys:
        value = values.get(key)
        if value:
            return value
    return ""


def _resolve_robot_ip_address(value: str = "") -> str:
    requested = str(value or "").strip()
    if requested:
        return requested
    env_ip = os.environ.get("ROBOT_IP_ADDRESS", "").strip()
    if env_ip:
        return env_ip
    return _station_config_value("ROBOT_IP_ADDRESS", "ip_address")


def _calibration_matches_robot_ip(path: Path, robot_ip_address: str) -> bool:
    ip_token = _sanitize_filename_token(robot_ip_address)
    return bool(ip_token) and path.stem.endswith(f"_{ip_token}")


def _find_latest_calibration(calibration_dir: str, robot_ip_address: str = "") -> str:
    try:
        robot_ip_address = str(robot_ip_address or "").strip()
        if not robot_ip_address:
            print("[tray_detect_yolo.launch] Robot IP is not set; cannot auto-select an exact calibration file.")
            return ""
        base = Path(calibration_dir).expanduser()
        if not base.exists() or not base.is_dir():
            return ""
        exact_files = []
        for path in base.iterdir():
            if not path.is_file() or path.suffix != ".yaml" or path.stat().st_size <= 0:
                continue
            if path.name.startswith("axab_calibration_eyeonhand_") and _calibration_matches_robot_ip(path, robot_ip_address):
                exact_files.append(path)
        if not exact_files:
            return ""
        return str(max(exact_files, key=lambda p: p.stat().st_mtime))
    except Exception as exc:
        print(f"[tray_detect_yolo.launch] Failed to search calibrations in {calibration_dir}: {exc}")
        return ""


def _calibration_file_is_usable(path: str) -> bool:
    try:
        p = Path(path).expanduser()
        return p.exists() and p.is_file() and p.stat().st_size > 0
    except Exception:
        return False


def _resolve_runtime_path(path_text: str) -> Path:
    path = Path(os.path.expandvars(str(path_text or "").strip())).expanduser()
    if not path.is_absolute():
        path = _workspace_root() / path
    return path.resolve()


def _read_selected_profile_export_file(path_text: str) -> str:
    if not str(path_text or "").strip():
        return ""
    export_path = _resolve_runtime_path(path_text)
    if not export_path.exists() or not export_path.is_file():
        return ""
    try:
        selected = export_path.read_text(encoding="utf-8").strip().splitlines()[0].strip()
    except IndexError:
        return ""
    except Exception as exc:
        raise RuntimeError(
            f"[tray_detect_yolo.launch] Failed to read selected_profile_export_file "
            f"{export_path}: {exc}"
        ) from exc
    if not selected:
        return ""
    selected_path = _resolve_runtime_path(selected)
    if not selected_path.exists() or not selected_path.is_file() or selected_path.stat().st_size <= 0:
        raise RuntimeError(
            "[tray_detect_yolo.launch] selected_profile_export_file points to a missing/empty "
            f"profile YAML: {selected_path}"
        )
    return str(selected_path)


def _show_missing_calibration_dialog(message: str) -> None:
    try:
        import tkinter as tk
        from tkinter import messagebox

        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        messagebox.showerror("Calibration File Missing", message + "\n\nClick OK to close launch.")
        root.destroy()
    except Exception as exc:
        print(f"[tray_detect_yolo.launch] Could not open GUI dialog: {exc}")
        print(message)


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context).strip()
    profiles_dir = os.path.expanduser(LaunchConfiguration("profiles_dir").perform(context))
    model_root = os.path.expanduser(LaunchConfiguration("model_root").perform(context))
    selected_model_path = os.path.expanduser(LaunchConfiguration("selected_model_path").perform(context).strip())
    selected_profile_path = os.path.expanduser(LaunchConfiguration("selected_profile_path").perform(context).strip())
    runtime_settings_file = os.path.expanduser(LaunchConfiguration("runtime_settings_file").perform(context).strip())
    selected_model_export_file = os.path.expanduser(LaunchConfiguration("selected_model_export_file").perform(context).strip())
    selected_profile_export_file = os.path.expanduser(LaunchConfiguration("selected_profile_export_file").perform(context).strip())
    use_calibration = _to_bool(LaunchConfiguration("use_calibration").perform(context))
    calibration_dir = os.path.expanduser(LaunchConfiguration("calibration_dir").perform(context))
    calibration_file = os.path.expanduser(LaunchConfiguration("calibration_file").perform(context))
    robot_ip_address = _resolve_robot_ip_address(LaunchConfiguration("robot_ip_address").perform(context))
    headless = _to_bool(LaunchConfiguration("headless").perform(context))
    start_visualization = _to_bool(LaunchConfiguration("start_visualization").perform(context))

    if not selected_profile_path:
        selected_profile_path = _read_selected_profile_export_file(selected_profile_export_file)
        if selected_profile_path:
            print(
                "[tray_detect_yolo.launch] Using selected tray profile from "
                f"{selected_profile_export_file}: {selected_profile_path}"
            )
    elif not _calibration_file_is_usable(selected_profile_path):
        msg = f"[tray_detect_yolo.launch] selected_profile_path is set but missing/empty: {selected_profile_path}"
        raise RuntimeError(msg)

    if not selected_profile_path:
        msg = (
            "[tray_detect_yolo.launch] Tray YOLO detect requires an explicit selected_profile_path; "
            "automatic profile fallback is disabled. Pass "
            "selected_profile_path:=/path/to/tray_profile.yaml or write that path into "
            f"{selected_profile_export_file}."
        )
        raise RuntimeError(msg)

    selected_file = ""
    if use_calibration:
        if calibration_file:
            selected_file = calibration_file
            if not _calibration_file_is_usable(selected_file):
                msg = f"[tray_detect_yolo.launch] calibration_file is set but missing/empty: {selected_file}"
                _show_missing_calibration_dialog(msg)
                raise RuntimeError(msg)
        else:
            selection = _calibration_selection_helper()
            if selection.requires_manual_selection(robot_ip_address):
                selected_file = selection.choose_required_calibration(
                    calibration_dir=calibration_dir,
                    filename_pattern="axab_calibration_eyeonhand_*.yaml",
                    calibration_label="eye-on-hand calibration",
                    launch_label="tray_detect_yolo.launch",
                    robot_ip_address=robot_ip_address,
                )
            else:
                selected_file = _find_latest_calibration(calibration_dir, robot_ip_address)
            if not selected_file:
                msg = (
                    "[tray_detect_yolo.launch] No non-empty eye-on-hand calibration YAML "
                    f"tagged for robot IP {robot_ip_address or '<unset>'} found in {calibration_dir}. "
                    "Provide one via calibration_file:=<path>."
                )
                _show_missing_calibration_dialog(msg)
                raise RuntimeError(msg)
        print(f"[tray_detect_yolo.launch] Using calibration file: {selected_file}")

    child_frame = LaunchConfiguration("child_frame").perform(context)
    camera_frame = LaunchConfiguration("camera_frame").perform(context).strip()
    if not camera_frame and use_calibration:
        camera_frame = child_frame

    parameter_sources = [{
        "profiles_dir": profiles_dir,
        "model_root": model_root,
        "selected_model_path": selected_model_path,
        "selected_profile_path": selected_profile_path,
        "runtime_settings_file": runtime_settings_file,
        "selected_model_export_file": selected_model_export_file,
        "selected_profile_export_file": selected_profile_export_file,
        "selected_profile_topic": LaunchConfiguration("selected_profile_topic").perform(context),
        "color_topic": LaunchConfiguration("color_topic").perform(context),
        "depth_topic": LaunchConfiguration("depth_topic").perform(context),
        "camera_info_topic": LaunchConfiguration("camera_info_topic").perform(context),
        "overlay_topic": LaunchConfiguration("overlay_topic").perform(context),
        "publish_overlay": _to_bool(LaunchConfiguration("publish_overlay").perform(context)),
        "use_profile_camera_topics": _to_bool(LaunchConfiguration("use_profile_camera_topics").perform(context)),
        "tray_pose_topic": LaunchConfiguration("tray_pose_topic").perform(context),
        "tray_axis_overlay_topic": LaunchConfiguration("tray_axis_overlay_topic").perform(context),
        "tray_target_pose_topic": LaunchConfiguration("tray_target_pose_topic").perform(context),
        "tray_cube_marker_topic": LaunchConfiguration("tray_cube_marker_topic").perform(context),
        "tray_dimensions_service": LaunchConfiguration("tray_dimensions_service").perform(context),
        "seek_service": LaunchConfiguration("seek_service").perform(context),
        "seek_complete_service": LaunchConfiguration("seek_complete_service").perform(context),
        "seek_status_service": LaunchConfiguration("seek_status_service").perform(context),
        "go_to_teach_service": LaunchConfiguration("go_to_teach_service").perform(context),
        "movj_service": LaunchConfiguration("movj_service").perform(context),
        "start_visualization": start_visualization,
        "headless": headless,
        "use_calibration": use_calibration,
        "publish_static_calibration_tf": use_calibration,
        "calibration_parent_frame": LaunchConfiguration("parent_frame").perform(context),
        "calibration_child_frame": child_frame,
        "calibration_dir": calibration_dir,
        "calibration_file": selected_file,
        "robot_ip_address": robot_ip_address,
        "camera_frame": camera_frame,
        "yolo_imgsz": ParameterValue(LaunchConfiguration("yolo_imgsz"), value_type=int),
        "yolo_conf": ParameterValue(LaunchConfiguration("yolo_conf"), value_type=float),
        "yolo_iou": ParameterValue(LaunchConfiguration("yolo_iou"), value_type=float),
        "max_inference_hz": ParameterValue(LaunchConfiguration("max_inference_hz"), value_type=float),
        "pt_device": ParameterValue(LaunchConfiguration("pt_device"), value_type=str),
        "tray_dimension_tolerance_percent": ParameterValue(
            LaunchConfiguration("tray_dimension_tolerance_percent"),
            value_type=int,
        ),
        "seek_window_sec": ParameterValue(LaunchConfiguration("seek_window_sec"), value_type=float),
    }]
    if params_file:
        parameter_sources.insert(0, params_file)

    return [Node(
        package="tray_perception_yolo",
        executable="tray_detect_yolo_node.py",
        name="tray_detect",
        output="screen",
        prefix=[LaunchConfiguration("python_executable"), " "],
        parameters=parameter_sources,
    )]


def generate_launch_description():
    return LaunchDescription([
        _ros_domain_action(),
        DeclareLaunchArgument("params_file", default_value=""),
        DeclareLaunchArgument("profiles_dir", default_value=_repo_path("teach", "tray_teach_yolo")),
        DeclareLaunchArgument("model_root", default_value=_repo_path("teach", "tray_teach_yolo")),
        DeclareLaunchArgument("selected_model_path", default_value=""),
        DeclareLaunchArgument("selected_profile_path", default_value=""),
        DeclareLaunchArgument(
            "runtime_settings_file",
            default_value=_repo_path("config", "tray_perception_yolo", "tray_detect_yolo_runtime_settings.yaml"),
        ),
        DeclareLaunchArgument(
            "selected_model_export_file",
            default_value=_repo_path("config", "tray_perception_yolo", "tray_detect_yolo_selected_model.txt"),
        ),
        DeclareLaunchArgument(
            "selected_profile_export_file",
            default_value=_repo_path("config", "tray_perception_yolo", "tray_detect_yolo_selected_profile.txt"),
        ),
        DeclareLaunchArgument("selected_profile_topic", default_value="tray_detect/selected_profile"),
        DeclareLaunchArgument("color_topic", default_value=ROBOT_CAMERA_COLOR_TOPIC),
        DeclareLaunchArgument("depth_topic", default_value=ROBOT_CAMERA_DEPTH_TOPIC),
        DeclareLaunchArgument("camera_info_topic", default_value=ROBOT_CAMERA_INFO_TOPIC),
        DeclareLaunchArgument("overlay_topic", default_value="tray_overlay"),
        DeclareLaunchArgument("publish_overlay", default_value="true"),
        DeclareLaunchArgument("use_profile_camera_topics", default_value="true"),
        DeclareLaunchArgument("tray_pose_topic", default_value="tray_pose"),
        DeclareLaunchArgument("tray_axis_overlay_topic", default_value="tray_axis_overlay"),
        DeclareLaunchArgument("tray_target_pose_topic", default_value="tray_target_pose"),
        DeclareLaunchArgument("tray_cube_marker_topic", default_value="tray_cube_marker"),
        DeclareLaunchArgument("tray_dimensions_service", default_value="tray_detect/get_tray_dimensions"),
        DeclareLaunchArgument("seek_service", default_value="tray_detect/seek"),
        DeclareLaunchArgument("seek_complete_service", default_value="tray_detect/seek_complete"),
        DeclareLaunchArgument("seek_status_service", default_value="tray_detect/seek_status"),
        DeclareLaunchArgument("go_to_teach_service", default_value="tray_detect/go_to_teach"),
        DeclareLaunchArgument("movj_service", default_value="/dobot_bringup_ros2/srv/MovJ"),
        DeclareLaunchArgument("use_calibration", default_value="true"),
        DeclareLaunchArgument("parent_frame", default_value="Link6"),
        DeclareLaunchArgument("child_frame", default_value="arm_calibrated_camera_link"),
        DeclareLaunchArgument("calibration_dir", default_value=_repo_path("calibration")),
        DeclareLaunchArgument("calibration_file", default_value=""),
        DeclareLaunchArgument("robot_ip_address", default_value=""),
        DeclareLaunchArgument("camera_frame", default_value=""),
        DeclareLaunchArgument("start_visualization", default_value="true"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("yolo_imgsz", default_value="640"),
        DeclareLaunchArgument("yolo_conf", default_value="0.35"),
        DeclareLaunchArgument("yolo_iou", default_value="0.45"),
        DeclareLaunchArgument("max_inference_hz", default_value="8.0"),
        DeclareLaunchArgument("pt_device", default_value="cpu"),
        DeclareLaunchArgument("tray_dimension_tolerance_percent", default_value="15"),
        DeclareLaunchArgument("seek_window_sec", default_value="60.0"),
        DeclareLaunchArgument("python_executable", default_value=_repo_path(".venv", "bin", "python")),
        OpaqueFunction(function=_launch_setup),
    ])

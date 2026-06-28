#!/usr/bin/env python3
import math
import os
import re
import shutil
import subprocess
import time
import warnings
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

os.environ.setdefault("CUDA_VISIBLE_DEVICES", "-1")
warnings.filterwarnings("ignore", message="CUDA initialization:.*", category=UserWarning)

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from dobot_msgs_v4.srv import GetTrayDimensions, MovJ
from geometry_msgs.msg import Point32, PolygonStamped, Pose, PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker


WINDOW_NAME = "tray_detect_yolo_view"
ROBOT_CAMERA_COLOR_TOPIC = "/robot_camera/color/image_raw"
ROBOT_CAMERA_DEPTH_TOPIC = "/robot_camera/depth/image_raw"
ROBOT_CAMERA_INFO_TOPIC = "/robot_camera/color/camera_info"
SUPPORTED_YOLO_VERSIONS = ("yolo11", "yolo26")
TOP_BAR_HEIGHT = 156
PREVIEW_CANVAS_WIDTH = 1080
PREVIEW_CANVAS_HEIGHT = 680
BUTTON_HEIGHT = 38
METERS_TO_MM = 1000.0
DIMENSION_TOLERANCE_MIN = 1
DIMENSION_TOLERANCE_MAX = 50


def workspace_root() -> Path:
    def looks_like_root(path: Path) -> bool:
        return (
            (path / "src").exists()
            and ((path / "README.md").exists() or (path / "src" / "dobot_msgs_v4").exists())
        )

    def find_from(start: Path) -> Optional[Path]:
        path = start.expanduser().resolve()
        if path.is_file():
            path = path.parent
        for candidate in (path, *path.parents):
            if looks_like_root(candidate):
                return candidate
        return None

    for name in ("DOBOT_PICKN_PLACE_ROOT", "DOBOT_WORKSPACE_ROOT"):
        value = os.environ.get(name)
        if value:
            return find_from(Path(value)) or Path(value).expanduser().resolve()

    for candidate in (Path.cwd(), Path(__file__).resolve()):
        found = find_from(candidate)
        if found is not None:
            return found
    return Path.cwd().resolve()


def workspace_path(*parts: str) -> Path:
    return workspace_root().joinpath(*parts)


def resolve_path(path_text: str) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(path_text))).resolve()


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def normalize_calibration_type(value: object) -> str:
    normalized = []
    for ch in str(value or ""):
        if ch.isalnum():
            normalized.append(ch.lower())
        elif ch in ("_", "-"):
            normalized.append("_")
    return "".join(normalized)


def station_config_value(*keys: str) -> str:
    try:
        values = {}
        with workspace_path("station_config").open("r", encoding="utf-8") as stream:
            for raw_line in stream:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                if line.startswith("export "):
                    line = line[len("export "):].strip()
                if "=" not in line:
                    continue
                key, value = line.split("=", 1)
                values[key.strip()] = value.strip().strip("'\"")
    except OSError:
        return ""
    for key in keys:
        value = values.get(key)
        if value:
            return value
    return ""


def resolve_robot_ip_address(value: object = "") -> str:
    requested = str(value or "").strip()
    if requested:
        return requested
    env_ip = os.environ.get("ROBOT_IP_ADDRESS", "").strip()
    if env_ip:
        return env_ip
    return station_config_value("ROBOT_IP_ADDRESS", "ip_address")


def detect_yolo_version_in_text(value: object) -> Optional[str]:
    text = str(value or "").strip().lower()
    if not text:
        return None
    match = re.search(r"yolo\s*v?[\s_-]*(\d+)", text)
    if match:
        version = f"yolo{match.group(1)}"
        if version in SUPPORTED_YOLO_VERSIONS:
            return version
    for token in re.findall(r"[a-z]+|\d+", text):
        if token in ("11", "26"):
            version = f"yolo{token}"
            if version in SUPPORTED_YOLO_VERSIONS:
                return version
    return None


def detect_yolo_version(*values: object) -> Optional[str]:
    for value in values:
        version = detect_yolo_version_in_text(value)
        if version is not None:
            return version
    return None


def normalize_yolo_version(*values: object) -> str:
    version = detect_yolo_version(*values)
    if version is None:
        raise ValueError("YOLO version marker is required: use yolo11 or yolo26")
    return version


def yolo_version_values_for_model_path(model_path: Path) -> List[str]:
    try:
        resolved = model_path.resolve()
    except Exception:
        resolved = model_path
    values = [resolved.name]
    values.extend(parent.name for parent in resolved.parents if parent.name)
    return values


def yolo_tray_token_for_model_path(model_path: Path) -> str:
    stem = model_path.stem.strip().lower()
    match = re.match(r"^(?P<tray>.+?)[_-]yolo\s*v?(?:11|26)(?:[_-].*)?$", stem)
    if match:
        return match.group("tray").strip("_- ")
    if stem == "best" and model_path.parent.name:
        return model_path.parent.name.lower().removeprefix("tray_")
    return stem


def yolo_detection_backend(version: object) -> str:
    return f"{normalize_yolo_version(version)}_seg_pt"


def yolo_version_label(version: object) -> str:
    return normalize_yolo_version(version).upper()


def supported_yolo_label() -> str:
    return "/".join(yolo_version_label(version) for version in SUPPORTED_YOLO_VERSIONS)


def rotation_to_quaternion(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    trace = float(rotation[0, 0] + rotation[1, 1] + rotation[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rotation[2, 1] - rotation[1, 2]) / s
        qy = (rotation[0, 2] - rotation[2, 0]) / s
        qz = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(max(1e-12, 1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])) * 2.0
        qw = (rotation[2, 1] - rotation[1, 2]) / s
        qx = 0.25 * s
        qy = (rotation[0, 1] + rotation[1, 0]) / s
        qz = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(max(1e-12, 1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])) * 2.0
        qw = (rotation[0, 2] - rotation[2, 0]) / s
        qx = (rotation[0, 1] + rotation[1, 0]) / s
        qy = 0.25 * s
        qz = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(max(1e-12, 1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])) * 2.0
        qw = (rotation[1, 0] - rotation[0, 1]) / s
        qx = (rotation[0, 2] + rotation[2, 0]) / s
        qy = (rotation[1, 2] + rotation[2, 1]) / s
        qz = 0.25 * s
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm > 1e-12:
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return qx, qy, qz, qw


def normalize(vec: np.ndarray) -> Optional[np.ndarray]:
    norm = float(np.linalg.norm(vec))
    if norm < 1e-9 or not np.isfinite(norm):
        return None
    return vec / norm


def pose_to_msg(pose: "Pose3D") -> Pose:
    msg = Pose()
    msg.position.x = float(pose.origin[0])
    msg.position.y = float(pose.origin[1])
    msg.position.z = float(pose.origin[2])
    qx, qy, qz, qw = rotation_to_quaternion(pose.rotation)
    msg.orientation.x = qx
    msg.orientation.y = qy
    msg.orientation.z = qz
    msg.orientation.w = qw
    return msg


def lower_left_corner_index(corners: List[Tuple[float, float]]) -> int:
    if len(corners) != 4:
        return -1
    return max(range(4), key=lambda i: (corners[i][1], -corners[i][0]))


def fit_text(text: str, max_chars: int) -> str:
    return text if len(text) <= max_chars else text[: max(0, max_chars - 3)] + "..."


@dataclass
class Button:
    name: str
    rect: Tuple[int, int, int, int]
    enabled: bool = True


@dataclass
class TrayProfile:
    path: Path
    label: str
    tray_name: str = "tray"
    class_id: int = 0
    class_name: str = "tray"
    teach_date: str = ""
    detection_backend: str = ""
    yolo_version: str = ""
    model_path: str = ""
    model_pt_path: str = ""
    model_dir: str = ""
    color_topic: str = ROBOT_CAMERA_COLOR_TOPIC
    depth_topic: str = ROBOT_CAMERA_DEPTH_TOPIC
    camera_info_topic: str = ROBOT_CAMERA_INFO_TOPIC
    overlay_topic: str = "tray_overlay"
    tray_plane_a: float = 0.0
    tray_plane_b: float = 0.0
    tray_plane_c: float = 0.0
    tray_plane_reference_depth_m: float = 0.0
    tray_plane_roi: List[int] = field(default_factory=list)
    tray_plane_roi_points: List[Tuple[float, float]] = field(default_factory=list)
    tray_width_mm: float = 0.0
    tray_height_mm: float = 0.0
    teach_joints_deg: List[float] = field(default_factory=list)
    has_teach_joints: bool = False


@dataclass
class Detection:
    score: float
    class_id: int
    box: Tuple[int, int, int, int]
    mask: np.ndarray
    center: Tuple[float, float]
    corners: List[Tuple[float, float]]


@dataclass
class Pose3D:
    origin: np.ndarray
    rotation: np.ndarray


@dataclass
class TrayAxes2D:
    origin: Tuple[float, float]
    x_dir: Tuple[float, float]
    y_dir: Tuple[float, float]
    origin_idx: int
    x_idx: int
    y_idx: int


class TrayDetectYoloNode(Node):
    def __init__(self) -> None:
        super().__init__("tray_detect")
        self.bridge = CvBridge()

        self.profiles_dir = resolve_path(str(self.declare_parameter(
            "profiles_dir", str(workspace_path("teach", "tray_teach_yolo"))).value))
        self.model_root = resolve_path(str(self.declare_parameter(
            "model_root", str(workspace_path("teach", "tray_teach_yolo"))).value))
        selected_model_path_text = str(self.declare_parameter("selected_model_path", "").value).strip()
        self.selected_model_path: Optional[Path] = resolve_path(selected_model_path_text) if selected_model_path_text else None
        selected_profile_path_text = str(self.declare_parameter("selected_profile_path", "").value).strip()
        self.selected_profile_path: Optional[Path] = resolve_path(selected_profile_path_text) if selected_profile_path_text else None
        self.runtime_settings_path = resolve_path(str(self.declare_parameter(
            "runtime_settings_file",
            str(workspace_path("config", "tray_perception_yolo", "tray_detect_yolo_runtime_settings.yaml"))).value))
        self.selected_model_export_path = resolve_path(str(self.declare_parameter(
            "selected_model_export_file",
            str(workspace_path("config", "tray_perception_yolo", "tray_detect_yolo_selected_model.txt"))).value))
        self.selected_profile_export_path = resolve_path(str(self.declare_parameter(
            "selected_profile_export_file",
            str(workspace_path("config", "tray_perception_yolo", "tray_detect_yolo_selected_profile.txt"))).value))
        if self.selected_profile_path is None:
            self.selected_profile_path = self.load_selected_profile_path_from_export_file()
        self.selected_profile_topic = str(
            self.declare_parameter("selected_profile_topic", "tray_detect/selected_profile").value
        ).strip() or "tray_detect/selected_profile"

        self.color_topic = str(self.declare_parameter("color_topic", ROBOT_CAMERA_COLOR_TOPIC).value).strip() or ROBOT_CAMERA_COLOR_TOPIC
        self.depth_topic = str(self.declare_parameter("depth_topic", ROBOT_CAMERA_DEPTH_TOPIC).value).strip() or ROBOT_CAMERA_DEPTH_TOPIC
        self.camera_info_topic = str(self.declare_parameter("camera_info_topic", ROBOT_CAMERA_INFO_TOPIC).value).strip() or ROBOT_CAMERA_INFO_TOPIC
        self.overlay_topic = str(self.declare_parameter("overlay_topic", "tray_overlay").value).strip() or "tray_overlay"
        self.use_profile_camera_topics = as_bool(self.declare_parameter("use_profile_camera_topics", True).value)
        self.tray_pose_topic = str(self.declare_parameter("tray_pose_topic", "tray_pose").value).strip() or "tray_pose"
        self.tray_target_pose_topic = str(self.declare_parameter("tray_target_pose_topic", "tray_target_pose").value).strip() or "tray_target_pose"
        self.tray_axis_overlay_topic = str(self.declare_parameter("tray_axis_overlay_topic", "tray_axis_overlay").value).strip() or "tray_axis_overlay"
        self.tray_cube_marker_topic = str(self.declare_parameter("tray_cube_marker_topic", "tray_cube_marker").value).strip() or "tray_cube_marker"
        self.tray_dimensions_service_name = str(self.declare_parameter(
            "tray_dimensions_service", "tray_detect/get_tray_dimensions").value).strip() or "tray_detect/get_tray_dimensions"
        self.seek_service_name = str(self.declare_parameter("seek_service", "tray_detect/seek").value).strip() or "tray_detect/seek"
        self.seek_complete_service_name = str(self.declare_parameter(
            "seek_complete_service", "tray_detect/seek_complete").value).strip() or "tray_detect/seek_complete"
        self.seek_status_service_name = str(self.declare_parameter(
            "seek_status_service", "tray_detect/seek_status").value).strip() or "tray_detect/seek_status"
        self.go_to_teach_service_name = str(self.declare_parameter(
            "go_to_teach_service", "tray_detect/go_to_teach").value).strip() or "tray_detect/go_to_teach"
        self.movj_service_name = str(self.declare_parameter("movj_service", "/dobot_bringup_ros2/srv/MovJ").value).strip()

        self.use_calibration = as_bool(self.declare_parameter("use_calibration", True).value)
        self.publish_static_calibration_tf = as_bool(self.declare_parameter("publish_static_calibration_tf", True).value)
        self.calibration_parent_frame = str(self.declare_parameter("calibration_parent_frame", "Link6").value).strip() or "Link6"
        self.calibration_child_frame = str(self.declare_parameter(
            "calibration_child_frame", "arm_calibrated_camera_link").value).strip() or "arm_calibrated_camera_link"
        self.calibration_file = str(self.declare_parameter("calibration_file", "").value).strip()
        self.camera_frame = str(self.declare_parameter("camera_frame", "").value).strip()
        self.robot_ip_address = resolve_robot_ip_address(self.declare_parameter("robot_ip_address", "").value)
        self.calibration_translation = (0.0, 0.0, 0.0)
        self.calibration_rotation = (0.0, 0.0, 0.0, 1.0)

        self.headless = as_bool(self.declare_parameter("headless", False).value)
        self.start_visualization = as_bool(self.declare_parameter("start_visualization", True).value) and not self.headless
        self.publish_overlay = as_bool(self.declare_parameter("publish_overlay", True).value)
        self.yolo_imgsz = int(self.declare_parameter("yolo_imgsz", 640).value)
        self.yolo_conf = float(np.clip(float(self.declare_parameter("yolo_conf", 0.35).value), 0.0, 1.0))
        self.yolo_iou = float(np.clip(float(self.declare_parameter("yolo_iou", 0.45).value), 0.0, 1.0))
        self.mask_threshold = float(np.clip(float(self.declare_parameter("mask_threshold", 0.5).value), 0.0, 1.0))
        self.max_inference_hz = max(0.1, float(self.declare_parameter("max_inference_hz", 8.0).value))
        self.dimension_tolerance_percent = int(np.clip(
            int(self.declare_parameter("tray_dimension_tolerance_percent", 15).value),
            DIMENSION_TOLERANCE_MIN,
            DIMENSION_TOLERANCE_MAX,
        ))
        self.seek_window_sec = max(0.1, float(self.declare_parameter("seek_window_sec", 60.0).value))
        requested_pt_device = str(self.declare_parameter("pt_device", "cpu").value).strip()
        self.pt_device = "cpu"
        if requested_pt_device and requested_pt_device.lower() != "cpu":
            self.get_logger().warn(f"Ignoring pt_device={requested_pt_device}; .pt tray detection is CPU-only.")

        self.latest_depth_m: Optional[np.ndarray] = None
        self.latest_info: Optional[CameraInfo] = None
        self.latest_detections: List[Detection] = []
        self.selected_detection: Optional[Detection] = None
        self.selected_pose: Optional[Pose3D] = None
        self.selected_axes_2d: Optional[TrayAxes2D] = None
        self.last_inference_time = 0.0
        self.inference_count = 0
        self.inference_duration_ema_ms = 0.0
        self.inference_fps_ema = 0.0
        self.last_inference_duration_ms = 0.0
        self.status = "Loading YOLO tray profiles"
        self.yolo_enabled = True
        self.seek_mode_active = False
        self.seek_result_latched = False
        self.seek_started_time = 0.0
        self.seek_frame_attempted = False
        self.last_seek_pose: Optional[Pose3D] = None
        self.go_to_teach_in_progress = False
        self.delete_confirm_active = False
        self.pending_delete_profile: Optional[Path] = None
        self.buttons: Dict[str, Button] = {}

        self.profiles: List[TrayProfile] = []
        self.selected_profile_index = -1
        self.active_profile: Optional[TrayProfile] = None
        self.pt_model = None
        self.pt_class_names: Dict[int, str] = {}
        self.ignore_yaml_profiles = True

        if self.use_calibration and self.calibration_file:
            reason = self.load_calibration_from_file(resolve_path(self.calibration_file))
            if reason:
                raise RuntimeError(f"Failed to load calibration file '{self.calibration_file}': {reason}")
            if self.camera_frame and self.camera_frame != self.calibration_child_frame:
                self.get_logger().warn(
                    "camera_frame (%s) differs from calibration_child_frame (%s); using calibration_child_frame.",
                    self.camera_frame,
                    self.calibration_child_frame,
                )
            self.camera_frame = self.calibration_child_frame

        self.load_runtime_ui_settings()

        self.overlay_pub = self.create_publisher(Image, self.overlay_topic, 5)
        self.tray_pose_pub = self.create_publisher(PoseStamped, self.tray_pose_topic, 10)
        self.tray_target_pose_pub = self.create_publisher(PoseStamped, self.tray_target_pose_topic, 10)
        self.tray_axis_overlay_pub = self.create_publisher(PolygonStamped, self.tray_axis_overlay_topic, 10)
        marker_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tray_marker_pub = self.create_publisher(Marker, self.tray_cube_marker_topic, marker_qos)
        selected_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.selected_profile_pub = self.create_publisher(String, self.selected_profile_topic, selected_qos)

        self.color_sub = None
        self.depth_sub = None
        self.info_sub = None
        self.configure_camera_subscriptions(self.color_topic, self.depth_topic, self.camera_info_topic, force=True)
        self.movj_client = self.create_client(MovJ, self.movj_service_name)
        self.create_service(GetTrayDimensions, self.tray_dimensions_service_name, self.handle_get_tray_dimensions)
        self.create_service(Trigger, self.seek_service_name, self.handle_seek)
        self.create_service(Trigger, self.seek_complete_service_name, self.handle_seek_complete)
        self.create_service(Trigger, self.seek_status_service_name, self.handle_seek_status)
        self.create_service(Trigger, self.go_to_teach_service_name, self.handle_go_to_teach)
        self.create_timer(0.2, self.check_seek_timeout)

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_calibration_tf()
        self.refresh_profiles()
        self.select_required_runtime_profile()

        if self.start_visualization:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL | getattr(cv2, "WINDOW_GUI_NORMAL", 0))
            cv2.resizeWindow(WINDOW_NAME, PREVIEW_CANVAS_WIDTH, TOP_BAR_HEIGHT + PREVIEW_CANVAS_HEIGHT)
            cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)

        self.get_logger().info(
            f"tray_detect YOLO ready. profiles_dir={self.profiles_dir} model_root={self.model_root} "
            f"pose_topic={self.tray_pose_topic} target_topic={self.tray_target_pose_topic} "
            f"dimensions_service={self.tray_dimensions_service_name} output_frame={self.camera_frame or 'incoming camera frame'}"
        )

    def configure_camera_subscriptions(self, color_topic: str, depth_topic: str, info_topic: str, force: bool = False) -> None:
        next_color = str(color_topic or ROBOT_CAMERA_COLOR_TOPIC).strip() or ROBOT_CAMERA_COLOR_TOPIC
        next_depth = str(depth_topic or ROBOT_CAMERA_DEPTH_TOPIC).strip() or ROBOT_CAMERA_DEPTH_TOPIC
        next_info = str(info_topic or ROBOT_CAMERA_INFO_TOPIC).strip() or ROBOT_CAMERA_INFO_TOPIC
        if not force and next_color == self.color_topic and next_depth == self.depth_topic and next_info == self.camera_info_topic:
            return
        for attr in ("color_sub", "depth_sub", "info_sub"):
            subscription = getattr(self, attr, None)
            if subscription is not None:
                self.destroy_subscription(subscription)
                setattr(self, attr, None)
        self.color_topic = next_color
        self.depth_topic = next_depth
        self.camera_info_topic = next_info
        self.latest_depth_m = None
        self.latest_info = None
        self.color_sub = self.create_subscription(Image, self.color_topic, self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.info_callback, 10)
        self.get_logger().info(f"YOLO tray camera topics: color={self.color_topic} depth={self.depth_topic} info={self.camera_info_topic}")

    def fail_node(self, message: str) -> None:
        self.status = f"FATAL: {message}"
        self.get_logger().fatal(self.status)
        raise RuntimeError(self.status)

    def load_selected_profile_path_from_export_file(self) -> Optional[Path]:
        try:
            if not self.selected_profile_export_path.exists() or not self.selected_profile_export_path.is_file():
                return None
            text = self.selected_profile_export_path.read_text(encoding="utf-8").strip()
            if not text:
                return None
            path = resolve_path(text.splitlines()[0].strip())
            self.get_logger().info(
                f"Using selected tray profile from {self.selected_profile_export_path}: {path}"
            )
            return path
        except Exception as exc:
            raise RuntimeError(
                f"Failed to read selected tray profile file {self.selected_profile_export_path}: {exc}"
            ) from exc

    def select_required_runtime_profile(self) -> None:
        if self.selected_profile_path is None:
            self.fail_node(
                "Tray YOLO detect requires selected_profile_path or a non-empty "
                f"selected_profile_export_file ({self.selected_profile_export_path}); "
                "automatic profile fallback is disabled"
            )
        if not self.selected_profile_path.exists() or not self.selected_profile_path.is_file():
            self.fail_node(f"Selected tray profile is missing: {self.selected_profile_path}")
        if not self.select_model_path(self.selected_profile_path):
            self.fail_node(self.status or f"Failed to load selected tray profile: {self.selected_profile_path}")

    def refresh_profiles(self) -> None:
        self.profiles = []
        self.latest_detections = []
        self.selected_detection = None
        self.selected_pose = None
        self.selected_axes_2d = None
        if self.profiles_dir.exists():
            model_paths_seen = set()
            if not self.ignore_yaml_profiles:
                for path in self.profile_yaml_paths():
                    profile = self.load_profile(path)
                    if profile is not None:
                        self.profiles.append(profile)
                        try:
                            model_paths_seen.add(Path(profile.model_path).resolve())
                        except Exception:
                            pass
            for path in self.profile_pt_paths():
                try:
                    resolved = path.resolve()
                except Exception:
                    resolved = path
                if resolved in model_paths_seen:
                    continue
                profile = self.profile_from_model_path(path)
                if profile is not None:
                    self.profiles.append(profile)
                    model_paths_seen.add(resolved)
        if not self.profiles:
            self.active_profile = None
            self.selected_profile_index = -1
            self.pt_model = None
            self.pt_class_names = {}
            self.selected_model_path = None
            self.status = f"No paired {supported_yolo_label()} .pt + tray YAML profiles in {self.profiles_dir}"
            self.save_selected_model_export_file()
            self.save_selected_profile_export_file()
            self.publish_selected_profile()
            return
        self.status = f"Loaded {len(self.profiles)} YOLO tray profile(s); waiting for explicit selected_profile_path"

    def profile_yaml_paths(self) -> List[Path]:
        patterns = ("*.yaml", "*/*.yaml", "profiles/*.yaml", "models/*/*.yaml")
        paths: List[Path] = []
        seen = set()
        for pattern in patterns:
            for path in self.profiles_dir.glob(pattern):
                if not path.is_file():
                    continue
                resolved = path.resolve()
                if resolved in seen:
                    continue
                seen.add(resolved)
                paths.append(path)
        return sorted(paths, key=lambda p: p.stat().st_mtime, reverse=True)

    def profile_pt_paths(self) -> List[Path]:
        patterns = ("*.pt", "*/*.pt", "models/*/*.pt", "models/*/weights/*.pt")
        paths: List[Path] = []
        seen = set()
        for pattern in patterns:
            for path in self.profiles_dir.glob(pattern):
                if not path.is_file():
                    continue
                resolved = path.resolve()
                if resolved in seen:
                    continue
                seen.add(resolved)
                paths.append(path)
        return sorted(paths, key=lambda p: p.stat().st_mtime, reverse=True)

    def profile_params_from_root(self, root: Dict) -> Optional[Dict]:
        if not isinstance(root, dict):
            return None
        for key in ("tray_detect", "tray_yolo", "teach"):
            node = root.get(key, {})
            if isinstance(node, dict):
                candidate = node.get("ros__parameters")
                if isinstance(candidate, dict):
                    return candidate
        return None

    def resolve_profile_model_path(self, profile_path: Path, configured_path: str) -> Optional[Path]:
        text = str(configured_path or "").strip()
        if not text:
            return self.model_path_for_runtime(profile_path.parent)
        configured = Path(text).expanduser()
        if configured.suffix.lower() != ".pt":
            return None
        candidates: List[Path] = []
        if configured.is_absolute():
            candidates.append(configured)
        else:
            candidates.extend([profile_path.parent / configured, workspace_path(text), configured])
        seen = set()
        first_candidate: Optional[Path] = None
        for candidate in candidates:
            try:
                resolved = candidate.resolve()
            except Exception:
                resolved = candidate
            if first_candidate is None:
                first_candidate = resolved
            if resolved in seen:
                continue
            seen.add(resolved)
            if candidate.exists() and candidate.is_file() and candidate.stat().st_size > 0:
                return resolved
        return first_candidate

    def load_profile(self, path: Path, override_model_path: Optional[Path] = None) -> Optional[TrayProfile]:
        try:
            root = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            params = self.profile_params_from_root(root)
            if not isinstance(params, dict):
                return None
            if override_model_path is not None:
                model_path_resolved = override_model_path.resolve()
            else:
                model_path = params.get("model_pt_path") or params.get("trained_model_path") or params.get("model_path")
                if not model_path:
                    self.status = f"Profile {path.name} missing explicit model_pt_path"
                    self.get_logger().error(self.status)
                    return None
                model_path_resolved = self.resolve_profile_model_path(path, str(model_path))
                if model_path_resolved is None or model_path_resolved.suffix.lower() != ".pt":
                    self.status = f"Profile {path.name} model path is not a .pt file"
                    self.get_logger().error(self.status)
                    return None
            try:
                yolo_version = normalize_yolo_version(*yolo_version_values_for_model_path(model_path_resolved))
            except ValueError:
                self.status = f"Profile {path.name} model path missing YOLO version; the .pt filename/path must contain yolo11 or yolo26"
                self.get_logger().error(self.status)
                return None
            raw_detection_backend = str(params.get("detection_backend", "")).strip()
            if not raw_detection_backend:
                self.status = f"Profile {path.name} missing detection_backend"
                self.get_logger().error(self.status)
                return None
            backend_version = detect_yolo_version(raw_detection_backend)
            metadata_version = detect_yolo_version(params.get("yolo_version", ""), params.get("model_family", ""))
            if metadata_version is None:
                self.status = f"Profile {path.name} missing yolo_version/model_family"
                self.get_logger().error(self.status)
                return None
            if metadata_version != yolo_version:
                self.status = f"Profile {path.name} YOLO metadata mismatch: model path is {yolo_version}, YAML says {metadata_version}"
                self.get_logger().error(self.status)
                return None
            if backend_version != yolo_version:
                self.status = f"Profile {path.name} detection_backend mismatch: model path is {yolo_version}, backend is {raw_detection_backend}"
                self.get_logger().error(self.status)
                return None

            required_keys = (
                "class_id",
                "tray_plane_enabled",
                "tray_plane_a",
                "tray_plane_b",
                "tray_plane_c",
                "tray_plane_reference_depth_m",
                "tray_width_mm",
                "tray_height_mm",
            )
            missing_keys = [key for key in required_keys if key not in params]
            if missing_keys:
                self.status = f"Profile {path.name} missing required tray keys: {', '.join(missing_keys)}"
                self.get_logger().error(self.status)
                return None
            plane_enabled = as_bool(params.get("tray_plane_enabled", False))
            try:
                class_id = int(params.get("class_id"))
                plane_a = float(params.get("tray_plane_a", 0.0))
                plane_b = float(params.get("tray_plane_b", 0.0))
                plane_c = float(params.get("tray_plane_c", 0.0))
                plane_ref = float(params.get("tray_plane_reference_depth_m", 0.0))
                tray_width_mm = float(params.get("tray_width_mm", 0.0))
                tray_height_mm = float(params.get("tray_height_mm", 0.0))
            except (TypeError, ValueError):
                self.status = f"Profile {path.name} has invalid tray class, plane, or dimension values"
                self.get_logger().error(self.status)
                return None
            if (
                not plane_enabled
                or not np.isfinite([plane_a, plane_b, plane_c, plane_ref]).all()
                or plane_ref <= 0.0
            ):
                self.status = f"Profile {path.name} missing valid tray depth plane"
                self.get_logger().error(self.status)
                return None
            if not self.has_valid_tray_dimensions(tray_width_mm, tray_height_mm):
                self.status = f"Profile {path.name} missing valid tray_width_mm/tray_height_mm"
                self.get_logger().error(self.status)
                return None

            tray_node = root.get("tray", {}) if isinstance(root, dict) else {}
            tray_name = str(
                params.get("tray_name", "")
                or params.get("class_name", "")
                or (tray_node.get("display_name", "") if isinstance(tray_node, dict) else "")
                or path.stem
            )
            teach_date = str(params.get("teach_date", ""))
            label = tray_name + (f" | {teach_date}" if teach_date else f" | {path.name}")
            teach_joints = self.parse_teach_joints(params.get("teach_joints_deg", []))
            return TrayProfile(
                path=path,
                label=label,
                tray_name=tray_name,
                class_id=class_id,
                class_name=str(params.get("class_name", tray_name or "tray")),
                teach_date=teach_date,
                detection_backend=raw_detection_backend,
                yolo_version=yolo_version,
                model_path=str(model_path_resolved),
                model_pt_path=str(model_path_resolved),
                model_dir=str(resolve_path(str(params.get("model_dir", "")))) if params.get("model_dir") else "",
                color_topic=str(params.get("color_topic", self.color_topic)),
                depth_topic=str(params.get("depth_topic", self.depth_topic)),
                camera_info_topic=str(params.get("camera_info_topic", self.camera_info_topic)),
                overlay_topic=str(params.get("overlay_topic", self.overlay_topic)),
                tray_plane_a=plane_a,
                tray_plane_b=plane_b,
                tray_plane_c=plane_c,
                tray_plane_reference_depth_m=plane_ref,
                tray_plane_roi=self.parse_int_list(params.get("tray_plane_roi", [])),
                tray_plane_roi_points=self.parse_points(params.get("tray_plane_roi_points", [])),
                tray_width_mm=tray_width_mm,
                tray_height_mm=tray_height_mm,
                teach_joints_deg=teach_joints,
                has_teach_joints=as_bool(params.get("has_teach_joints", False)) or len(teach_joints) >= 6,
            )
        except Exception as exc:
            self.get_logger().warn(f"Skipping YOLO tray profile {path}: {exc}")
            return None

    @staticmethod
    def has_valid_tray_dimensions(width_mm: float, height_mm: float) -> bool:
        return np.isfinite([width_mm, height_mm]).all() and width_mm > 1.0 and height_mm > 1.0

    def parse_int_list(self, value) -> List[int]:
        if not isinstance(value, list):
            return []
        output: List[int] = []
        for item in value:
            try:
                output.append(int(item))
            except (TypeError, ValueError):
                return []
        return output

    def parse_points(self, value) -> List[Tuple[float, float]]:
        if not isinstance(value, list):
            return []
        points: List[Tuple[float, float]] = []
        try:
            if value and not isinstance(value[0], list):
                for i in range(0, len(value) - 1, 2):
                    points.append((float(value[i]), float(value[i + 1])))
            else:
                for point in value:
                    if isinstance(point, list) and len(point) >= 2:
                        points.append((float(point[0]), float(point[1])))
        except (TypeError, ValueError):
            return []
        return points

    def parse_teach_joints(self, value) -> List[float]:
        if not isinstance(value, list) or len(value) < 6:
            return []
        try:
            return [float(v) for v in value[:6]]
        except (TypeError, ValueError):
            return []

    def model_path_for_runtime(self, path: Path) -> Optional[Path]:
        candidate = path.resolve()
        if candidate.is_dir():
            for name in ("best.pt", "model.pt"):
                model_path = candidate / name
                if model_path.exists():
                    return model_path.resolve()
            pt_files = sorted(candidate.glob("*.pt"), key=lambda p: p.stat().st_mtime, reverse=True)
            if pt_files:
                return pt_files[0].resolve()
            weights_pt_files = sorted(candidate.glob("weights/*.pt"), key=lambda p: p.stat().st_mtime, reverse=True)
            return weights_pt_files[0].resolve() if weights_pt_files else None
        if candidate.suffix.lower() == ".pt":
            return candidate
        return None

    def metadata_yaml_for_model(self, model_path: Path) -> Optional[Path]:
        search_dir = model_path if model_path.is_dir() else model_path.parent
        model_token = model_path.stem.lower()
        tray_token = yolo_tray_token_for_model_path(model_path)
        local_candidates = [
            search_dir / f"{model_path.stem}.yaml",
            search_dir / f"{search_dir.name}.yaml",
            search_dir / "profile.yaml",
            search_dir / "tray.yaml",
        ]
        if tray_token:
            local_candidates.extend(sorted(search_dir.glob(f"{tray_token}*.yaml"), key=lambda p: p.stat().st_mtime, reverse=True))
            local_candidates.extend(sorted(search_dir.glob(f"tray_{tray_token}*.yaml"), key=lambda p: p.stat().st_mtime, reverse=True))
        local_candidates.extend(sorted(search_dir.glob("*.yaml"), key=lambda p: p.stat().st_mtime, reverse=True))
        global_candidates: List[Path] = []
        if self.profiles_dir.exists():
            if tray_token:
                global_candidates.extend(sorted(self.profiles_dir.glob(f"{tray_token}*.yaml"), key=lambda p: p.stat().st_mtime, reverse=True))
                global_candidates.extend(sorted(self.profiles_dir.glob(f"tray_{tray_token}*.yaml"), key=lambda p: p.stat().st_mtime, reverse=True))
            global_candidates.extend(sorted(self.profiles_dir.glob("*/*.yaml"), key=lambda p: p.stat().st_mtime, reverse=True))
        seen = set()
        for candidate in local_candidates + global_candidates:
            try:
                resolved = candidate.resolve()
            except Exception:
                resolved = candidate
            if resolved in seen or not candidate.exists() or not candidate.is_file():
                continue
            seen.add(resolved)
            profile = self.load_profile(candidate, override_model_path=model_path)
            if profile is None:
                continue
            profile_tray_token = profile.tray_name.lower().replace(" ", "_")
            candidate_token = candidate.stem.lower()
            if (
                candidate_token == model_token
                or (tray_token and candidate_token.startswith(tray_token))
                or (tray_token and candidate_token.startswith(f"tray_{tray_token}"))
                or (profile_tray_token and tray_token and profile_tray_token == tray_token)
                or (profile_tray_token and profile_tray_token in model_token)
            ):
                return candidate
        return None

    def profile_from_model_path(self, path: Path) -> Optional[TrayProfile]:
        if path.suffix.lower() in (".yaml", ".yml"):
            return self.load_profile(path)
        model_path = self.model_path_for_runtime(path)
        if model_path is None or not model_path.exists():
            self.status = f"Open Model: no .pt model found for {path}"
            return None
        metadata_path = self.metadata_yaml_for_model(model_path)
        if metadata_path is not None:
            profile = self.load_profile(metadata_path, override_model_path=model_path)
            if profile is not None:
                self.get_logger().info(f"Using YOLO tray profile YAML {metadata_path} for model {model_path}")
                profile.model_path = str(model_path)
                try:
                    profile_model_dir = resolve_path(profile.model_dir) if profile.model_dir else None
                except Exception:
                    profile_model_dir = None
                if profile_model_dir != model_path.parent.resolve():
                    profile.model_dir = ""
                profile.label = profile.label or model_path.parent.name
                return profile
        tray_token = yolo_tray_token_for_model_path(model_path)
        self.status = f"Open Model: missing YAML pair for {model_path.name}; expected {tray_token}*.yaml beside the .pt"
        self.get_logger().error(f"{self.status}: {model_path}")
        return None

    def select_model_path(self, path: Path) -> bool:
        profile = self.profile_from_model_path(path)
        if profile is None:
            return False
        for index, existing in enumerate(self.profiles):
            try:
                if Path(existing.model_path).resolve() == Path(profile.model_path).resolve():
                    self.profiles[index] = profile
                    return self.select_profile(index)
            except Exception:
                pass
        self.profiles.insert(0, profile)
        return self.select_profile(0)

    def select_profile(self, index: int) -> bool:
        if index < 0 or index >= len(self.profiles):
            return False
        profile = self.profiles[index]
        model_path = Path(profile.model_path)
        if not model_path.exists():
            self.status = f"Model missing: {model_path}"
            return False
        try:
            model_yolo_version = normalize_yolo_version(*yolo_version_values_for_model_path(model_path))
        except ValueError:
            self.status = f"Model {model_path.name} missing YOLO version; filename/path must contain yolo11 or yolo26"
            self.get_logger().error(self.status)
            return False
        metadata_version = detect_yolo_version(profile.yolo_version)
        if metadata_version and metadata_version != model_yolo_version:
            self.status = f"Model {model_path.name} YOLO version mismatch: filename/path is {model_yolo_version}, profile is {metadata_version}"
            self.get_logger().error(self.status)
            return False
        profile.yolo_version = model_yolo_version
        backend_version = detect_yolo_version(profile.detection_backend)
        if backend_version and backend_version != profile.yolo_version:
            self.status = f"Model {model_path.name} detection_backend mismatch: filename/path is {profile.yolo_version}, backend is {profile.detection_backend}"
            self.get_logger().error(self.status)
            return False
        if not backend_version:
            profile.detection_backend = yolo_detection_backend(profile.yolo_version)
        self.selected_profile_index = index
        self.active_profile = profile
        if self.use_profile_camera_topics:
            self.configure_camera_subscriptions(profile.color_topic, profile.depth_topic, profile.camera_info_topic)
        if not self.load_pt_model(model_path):
            self.selected_model_path = model_path
            self.save_selected_model_export_file()
            self.save_selected_profile_export_file()
            self.publish_selected_profile()
            return False
        self.selected_model_path = model_path
        self.selected_profile_path = profile.path
        self.status = f"Loaded {self.profile_label()}"
        self.save_selected_model_export_file()
        self.save_selected_profile_export_file()
        self.publish_selected_profile()
        return True

    def profile_label(self) -> str:
        return self.active_profile.label if self.active_profile is not None else "No tray profile"

    def load_pt_model(self, model_path: Path) -> bool:
        os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
        self.pt_model = None
        self.pt_class_names = {}
        self.inference_count = 0
        self.inference_duration_ema_ms = 0.0
        self.inference_fps_ema = 0.0
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            self.status = f"{supported_yolo_label()} .pt unavailable: install ultralytics for PT weights"
            self.get_logger().error(f"{self.status}: {exc}")
            return False
        try:
            self.pt_model = YOLO(str(model_path))
        except Exception as exc:
            self.status = f"Failed to load {supported_yolo_label()} .pt model: {model_path.name}"
            self.get_logger().error(f"{self.status}: {exc}")
            return False
        names = getattr(self.pt_model, "names", None)
        if isinstance(names, dict):
            self.pt_class_names = {int(key): str(value) for key, value in names.items()}
        elif isinstance(names, (list, tuple)):
            self.pt_class_names = {index: str(value) for index, value in enumerate(names)}
        if self.active_profile is not None and self.pt_class_names:
            class_name = self.pt_class_names.get(self.active_profile.class_id)
            if class_name:
                self.active_profile.class_name = class_name
        self.get_logger().info(f"Loaded {supported_yolo_label()} PT tray model: {model_path}")
        return True

    def load_calibration_from_file(self, path: Path) -> str:
        try:
            if not path.exists():
                return "File does not exist"
            if path.stat().st_size <= 0:
                return "Calibration file is empty"
            root = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except Exception as exc:
            return f"Could not read YAML: {exc}"
        if not isinstance(root, dict):
            return "Calibration YAML is not a map"
        transform = root.get("transform", {})
        if not isinstance(transform, dict):
            return "Missing 'transform' key"
        translation = transform.get("translation", {})
        rotation = transform.get("rotation", {})
        if not isinstance(translation, dict) or not isinstance(rotation, dict):
            return "Missing rotation/translation keys"
        params = root.get("parameters", {})
        if not isinstance(params, dict):
            params = {}
        calibration_type = str(params.get("calibration_type", "")).strip()
        if normalize_calibration_type(calibration_type) != "eye_in_hand":
            return (
                "Expected eye-on-hand calibration YAML with parameters.calibration_type=eye_in_hand, got "
                f"'{calibration_type or '<missing>'}'"
            )
        parent_frame = str(params.get("transform_parent_frame") or params.get("robot_tool_frame") or "").strip()
        if parent_frame:
            self.calibration_parent_frame = parent_frame
        child_frame = str(params.get("transform_child_frame") or "").strip()
        if child_frame:
            self.calibration_child_frame = child_frame
        try:
            qx = float(rotation.get("x", 0.0))
            qy = float(rotation.get("y", 0.0))
            qz = float(rotation.get("z", 0.0))
            qw = float(rotation.get("w", 1.0))
            tx = float(translation.get("x", 0.0))
            ty = float(translation.get("y", 0.0))
            tz = float(translation.get("z", 0.0))
        except Exception as exc:
            return f"Failed to parse rotation/translation: {exc}"
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-9 or not math.isfinite(norm):
            return "Invalid quaternion (zero norm)"
        inv_norm = 1.0 / norm
        self.calibration_translation = (tx, ty, tz)
        self.calibration_rotation = (qx * inv_norm, qy * inv_norm, qz * inv_norm, qw * inv_norm)
        return ""

    def publish_calibration_tf(self) -> None:
        if not self.use_calibration or not self.publish_static_calibration_tf or not self.calibration_file:
            return
        tx, ty, tz = self.calibration_translation
        qx, qy, qz, qw = self.calibration_rotation
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.calibration_parent_frame
        msg.child_frame_id = self.calibration_child_frame
        msg.transform.translation.x = tx
        msg.transform.translation.y = ty
        msg.transform.translation.z = tz
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        self.static_tf_broadcaster.sendTransform(msg)

    def required_camera_frame_id(self, header, info: Optional[CameraInfo]) -> Optional[str]:
        if self.camera_frame:
            return self.camera_frame
        if getattr(header, "frame_id", ""):
            return str(header.frame_id)
        if info is not None and info.header.frame_id:
            return str(info.header.frame_id)
        return None

    def depth_callback(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warn(f"Depth conversion failed: {exc}")
            return
        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) / 1000.0
        else:
            depth_m = depth.astype(np.float32)
        depth_m[~np.isfinite(depth_m)] = np.nan
        depth_m[depth_m <= 0.0] = np.nan
        self.latest_depth_m = depth_m

    def info_callback(self, msg: CameraInfo) -> None:
        self.latest_info = msg

    def color_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.fail_node(f"Color conversion failed: {exc}")
        info = self.latest_info
        self.process_frame(frame, info)
        frame_id = self.required_camera_frame_id(msg.header, info)
        if self.selected_pose is not None and frame_id is None:
            self.fail_node("Camera frame id is missing; refusing to publish tray pose")
        stamp = msg.header.stamp
        if frame_id is not None:
            self.publish_pose_outputs(stamp, frame_id)
        if self.publish_overlay or self.start_visualization:
            output = self.render_overlay(frame)
            if self.publish_overlay:
                try:
                    overlay_msg = self.bridge.cv2_to_imgmsg(output, encoding="bgr8")
                    overlay_msg.header = msg.header
                    self.overlay_pub.publish(overlay_msg)
                except Exception as exc:
                    self.get_logger().warn(f"Overlay publish failed: {exc}")
            if self.start_visualization:
                cv2.imshow(WINDOW_NAME, output)
                cv2.waitKey(1)

    def process_frame(self, frame: np.ndarray, info: Optional[CameraInfo]) -> None:
        self.selected_detection = None
        self.selected_pose = None
        self.selected_axes_2d = None
        profile = self.active_profile
        if not self.seek_mode_active:
            return
        if self.seek_frame_attempted:
            return
        self.seek_frame_attempted = True
        if profile is None:
            self.fail_node("Tray seek failed: no active tray profile")
        if self.pt_model is None:
            self.fail_node("Tray seek failed: YOLO model is not loaded")
        if info is None:
            self.latest_detections = []
            self.fail_node("Tray seek failed: camera info is unavailable")
        if info.k[0] <= 1e-6 or info.k[4] <= 1e-6:
            self.latest_detections = []
            self.fail_node("Tray seek failed: camera intrinsics are invalid")
        if not self.yolo_enabled:
            self.latest_detections = []
            self.fail_node("Tray seek failed: YOLO inference is disabled")
        try:
            detections = self.run_yolo(frame)
        except Exception as exc:
            self.latest_detections = []
            self.fail_node(f"Tray seek failed: YOLO inference error: {exc}")
        self.latest_detections = detections
        if not detections:
            self.fail_node(f"Tray seek failed: no {profile.class_name} segmentation mask above {self.yolo_conf * 100.0:.0f}%")
        detection = detections[0]
        self.latest_detections = [detection]
        pose = self.estimate_pose_from_tray_plane(detection, frame.shape[:2], info)
        if pose is None:
            self.fail_node("Tray seek failed: could not compute pose from taught tray plane")
        measured = self.estimate_dimensions_from_plane(detection.corners, frame.shape[:2], info)
        if measured is None:
            self.fail_node("Tray seek failed: could not measure tray dimensions from taught plane")
        if not self.dimensions_within_tolerance(measured, profile):
            self.fail_node(
                "Tray seek failed: detected tray size "
                f"{measured[0]:.0f}x{measured[1]:.0f} mm outside taught "
                f"{profile.tray_width_mm:.0f}x{profile.tray_height_mm:.0f} mm "
                f"at {self.dimension_tolerance_percent}% tolerance"
            )
        axes = self.compute_axes_2d(detection.corners)
        if axes is None:
            self.fail_node("Tray seek failed: could not compute tray axis overlay from mask corners")
        self.selected_detection = detection
        self.selected_pose = pose
        self.selected_axes_2d = axes
        self.status = (
            f"Detected one {profile.class_name} frame | conf {detection.score:.2f} | "
            f"size {measured[0]:.0f}x{measured[1]:.0f} mm"
        )

    def run_yolo(self, frame: np.ndarray) -> List[Detection]:
        if self.pt_model is None or self.active_profile is None:
            return []
        classes = [int(self.active_profile.class_id)]
        started = time.perf_counter()
        try:
            results = self.pt_model.predict(
                source=frame,
                imgsz=self.yolo_imgsz,
                conf=self.yolo_conf,
                iou=self.yolo_iou,
                classes=classes,
                device=self.pt_device,
                verbose=False,
            )
        finally:
            self.record_inference_timing(time.perf_counter() - started)
        if not results:
            return []
        return self.pt_result_to_detections(results[0], frame.shape[:2])

    def record_inference_timing(self, elapsed_sec: float) -> None:
        elapsed_sec = max(1e-6, float(elapsed_sec))
        duration_ms = elapsed_sec * 1000.0
        fps = 1.0 / elapsed_sec
        alpha = 0.2
        if self.inference_count <= 0:
            self.inference_duration_ema_ms = duration_ms
            self.inference_fps_ema = fps
        else:
            self.inference_duration_ema_ms = alpha * duration_ms + (1.0 - alpha) * self.inference_duration_ema_ms
            self.inference_fps_ema = alpha * fps + (1.0 - alpha) * self.inference_fps_ema
        self.last_inference_duration_ms = duration_ms
        self.inference_count += 1

    def pt_result_to_detections(self, result, image_shape: Tuple[int, int]) -> List[Detection]:
        boxes_obj = getattr(result, "boxes", None)
        if boxes_obj is None or len(boxes_obj) == 0 or self.active_profile is None:
            return []
        boxes_xyxy = boxes_obj.xyxy.detach().cpu().numpy()
        scores = boxes_obj.conf.detach().cpu().numpy()
        class_ids = boxes_obj.cls.detach().cpu().numpy().astype(np.int32)
        masks_obj = getattr(result, "masks", None)
        mask_polygons = getattr(masks_obj, "xy", None) if masks_obj is not None else None
        mask_data = masks_obj.data.detach().cpu().numpy() if masks_obj is not None and getattr(masks_obj, "data", None) is not None else None
        class_matches = [
            index
            for index, class_id in enumerate(class_ids)
            if int(class_id) == int(self.active_profile.class_id)
        ]
        if not class_matches:
            return []
        index = max(class_matches, key=lambda idx: float(scores[idx]))
        class_id = int(class_ids[index])
        x1, y1, x2, y2 = self.clip_xyxy_box(boxes_xyxy[index], image_shape)
        if x2 <= x1 or y2 <= y1:
            return []
        mask = self.pt_mask_for_detection(index, mask_polygons, mask_data, image_shape)
        if mask is None or cv2.countNonZero(mask) < 16:
            return []
        contour = self.largest_contour(mask)
        if contour is None:
            return []
        corners = self.contour_corners(contour)
        if len(corners) != 4:
            return []
        moments = cv2.moments(contour)
        if abs(moments["m00"]) > 1e-6:
            cx = float(moments["m10"] / moments["m00"])
            cy = float(moments["m01"] / moments["m00"])
        else:
            cx = float(x1 + (x2 - x1) * 0.5)
            cy = float(y1 + (y2 - y1) * 0.5)
        return [Detection(
            score=float(scores[index]),
            class_id=class_id,
            box=(x1, y1, x2 - x1, y2 - y1),
            mask=mask,
            center=(cx, cy),
            corners=corners,
        )]

    def clip_xyxy_box(self, box_xyxy: np.ndarray, image_shape: Tuple[int, int]) -> Tuple[int, int, int, int]:
        height, width = image_shape
        x1, y1, x2, y2 = [float(value) for value in box_xyxy[:4]]
        x1 = int(np.clip(round(x1), 0, width - 1))
        y1 = int(np.clip(round(y1), 0, height - 1))
        x2 = int(np.clip(round(x2), 0, width - 1))
        y2 = int(np.clip(round(y2), 0, height - 1))
        return x1, y1, x2, y2

    def pt_mask_for_detection(self, index: int, mask_polygons, mask_data, image_shape: Tuple[int, int]) -> Optional[np.ndarray]:
        mask = np.zeros(image_shape, dtype=np.uint8)
        if mask_polygons is not None and index < len(mask_polygons):
            polygon = np.asarray(mask_polygons[index], dtype=np.float32)
            if polygon.ndim == 2 and len(polygon) >= 3:
                points = np.round(polygon).astype(np.int32)
                points[:, 0] = np.clip(points[:, 0], 0, image_shape[1] - 1)
                points[:, 1] = np.clip(points[:, 1], 0, image_shape[0] - 1)
                cv2.fillPoly(mask, [points], 255)
        if cv2.countNonZero(mask) < 16 and mask_data is not None and index < len(mask_data):
            raw_mask = mask_data[index].astype(np.float32)
            if raw_mask.shape[:2] != image_shape:
                raw_mask = cv2.resize(raw_mask, (image_shape[1], image_shape[0]), interpolation=cv2.INTER_LINEAR)
            mask = (raw_mask >= self.mask_threshold).astype(np.uint8) * 255
        if cv2.countNonZero(mask) < 16:
            return None
        return mask

    def largest_contour(self, mask: np.ndarray):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [contour for contour in contours if cv2.contourArea(contour) >= 16.0]
        if not contours:
            return None
        return max(contours, key=cv2.contourArea)

    def contour_corners(self, contour) -> List[Tuple[float, float]]:
        hull = cv2.convexHull(contour)
        if hull is None or len(hull) < 3:
            return []
        rect = cv2.minAreaRect(hull)
        if rect[1][0] < 2.0 or rect[1][1] < 2.0:
            return []
        corners = cv2.boxPoints(rect)
        return [(float(x), float(y)) for x, y in corners]

    def plane_depth_at_pixel(self, point: Tuple[float, float], image_shape: Tuple[int, int]) -> Optional[float]:
        profile = self.active_profile
        if profile is None:
            return None
        height, width = image_shape
        if width <= 1 or height <= 1:
            return None
        x = float(np.clip(point[0], 0.0, width - 1))
        y = float(np.clip(point[1], 0.0, height - 1))
        x_norm = x / float(width - 1)
        y_norm = y / float(height - 1)
        depth = profile.tray_plane_a * x_norm + profile.tray_plane_b * y_norm + profile.tray_plane_c
        if not math.isfinite(depth) or depth <= 0.0:
            return None
        return depth

    def project_pixel(self, point: Tuple[float, float], depth: float, info: CameraInfo) -> np.ndarray:
        x = (float(point[0]) - info.k[2]) * depth / info.k[0]
        y = (float(point[1]) - info.k[5]) * depth / info.k[4]
        return np.array([x, y, depth], dtype=np.float64)

    def camera_points_from_plane(self, corners: List[Tuple[float, float]], image_shape: Tuple[int, int], info: CameraInfo) -> Optional[List[np.ndarray]]:
        if len(corners) != 4 or info.k[0] <= 1e-6 or info.k[4] <= 1e-6:
            return None
        points: List[np.ndarray] = []
        for corner in corners:
            depth = self.plane_depth_at_pixel(corner, image_shape)
            if depth is None:
                return None
            points.append(self.project_pixel(corner, depth, info))
        return points

    def compute_axes_2d(self, corners: List[Tuple[float, float]]) -> Optional[TrayAxes2D]:
        origin_idx = lower_left_corner_index(corners)
        if origin_idx < 0:
            return None
        prev_idx = (origin_idx + 3) % 4
        next_idx = (origin_idx + 1) % 4
        origin = np.asarray(corners[origin_idx], dtype=np.float64)
        dir_a = np.asarray(corners[prev_idx], dtype=np.float64) - origin
        dir_b = np.asarray(corners[next_idx], dtype=np.float64) - origin
        len_a = float(np.linalg.norm(dir_a))
        len_b = float(np.linalg.norm(dir_b))
        if len_a < 1e-6 or len_b < 1e-6:
            return None
        prev_is_x = len_a >= len_b
        x_dir = dir_a / len_a if prev_is_x else dir_b / len_b
        y_dir = dir_b / len_b if prev_is_x else dir_a / len_a
        return TrayAxes2D(
            origin=(float(origin[0]), float(origin[1])),
            x_dir=(float(x_dir[0]), float(x_dir[1])),
            y_dir=(float(y_dir[0]), float(y_dir[1])),
            origin_idx=origin_idx,
            x_idx=prev_idx if prev_is_x else next_idx,
            y_idx=next_idx if prev_is_x else prev_idx,
        )

    def estimate_pose_from_tray_plane(self, detection: Detection, image_shape: Tuple[int, int], info: CameraInfo) -> Optional[Pose3D]:
        camera_points = self.camera_points_from_plane(detection.corners, image_shape, info)
        axes = self.compute_axes_2d(detection.corners)
        if camera_points is None or axes is None:
            return None
        origin = camera_points[axes.origin_idx]
        x_edge = camera_points[axes.x_idx] - origin
        y_edge = camera_points[axes.y_idx] - origin
        x_axis = normalize(x_edge)
        if x_axis is None:
            return None
        z_axis = normalize(np.cross(x_axis, y_edge))
        if z_axis is None:
            return None
        y_axis = normalize(np.cross(z_axis, x_axis))
        if y_axis is None:
            return None
        rotation = np.array([
            [x_axis[0], y_axis[0], z_axis[0]],
            [x_axis[1], y_axis[1], z_axis[1]],
            [x_axis[2], y_axis[2], z_axis[2]],
        ], dtype=np.float64)
        return Pose3D(origin=origin, rotation=rotation)

    def estimate_dimensions_from_plane(self, corners: List[Tuple[float, float]], image_shape: Tuple[int, int], info: CameraInfo) -> Optional[Tuple[float, float]]:
        camera_points = self.camera_points_from_plane(corners, image_shape, info)
        if camera_points is None:
            return None
        lengths = []
        for index in range(4):
            length_mm = float(np.linalg.norm(camera_points[(index + 1) % 4] - camera_points[index]) * METERS_TO_MM)
            if not math.isfinite(length_mm) or length_mm <= 0.0:
                return None
            lengths.append(length_mm)
        long_side = (max(lengths[0], lengths[2]) + min(lengths[0], lengths[2])) * 0.5
        short_side = (max(lengths[1], lengths[3]) + min(lengths[1], lengths[3])) * 0.5
        width_mm = max(long_side, short_side)
        height_mm = min(long_side, short_side)
        return width_mm, height_mm

    def dimensions_within_tolerance(self, measured: Tuple[float, float], profile: TrayProfile) -> bool:
        taught = (profile.tray_width_mm, profile.tray_height_mm)
        if not self.has_valid_tray_dimensions(*taught):
            return False
        for actual, expected in zip(measured, taught):
            error_percent = 100.0 * abs(actual - expected) / expected
            if error_percent > float(self.dimension_tolerance_percent):
                return False
        return True

    def publish_pose_outputs(self, stamp, frame_id: str) -> None:
        if not self.seek_mode_active or self.selected_pose is None:
            return
        pose = self.selected_pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = frame_id
        pose_msg.pose = pose_to_msg(pose)
        self.tray_pose_pub.publish(pose_msg)
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose = pose_to_msg(pose)
        self.tray_target_pose_pub.publish(msg)
        self.publish_marker(stamp, frame_id, pose)
        if self.selected_axes_2d is not None:
            self.publish_axis_overlay(stamp, frame_id, self.selected_axes_2d)
        self.last_seek_pose = pose
        self.seek_mode_active = False
        self.seek_result_latched = True
        self.seek_started_time = 0.0
        self.status = "Seek done: published one validated YOLO tray target | waiting for tray intercept release"

    def check_seek_timeout(self) -> None:
        if not self.seek_mode_active or self.seek_started_time <= 0.0:
            return
        if time.monotonic() - self.seek_started_time > self.seek_window_sec:
            self.fail_node(f"Tray seek failed: no validated tray frame within {self.seek_window_sec:.1f}s")

    def publish_axis_overlay(self, stamp, frame_id: str, axes: TrayAxes2D) -> None:
        msg = PolygonStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        origin = Point32(x=float(axes.origin[0]), y=float(axes.origin[1]), z=1.0)
        x_dir = Point32(x=float(axes.x_dir[0]), y=float(axes.x_dir[1]), z=0.0)
        y_dir = Point32(x=float(axes.y_dir[0]), y=float(axes.y_dir[1]), z=0.0)
        msg.polygon.points = [origin, x_dir, y_dir]
        self.tray_axis_overlay_pub.publish(msg)

    def publish_marker(self, stamp, frame_id: str, pose: Pose3D) -> None:
        profile = self.active_profile
        if profile is None or not self.has_valid_tray_dimensions(profile.tray_width_mm, profile.tray_height_mm):
            self.fail_node("Tray seek failed: cannot publish marker without valid active tray dimensions")
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = "tray_detect"
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose_to_msg(pose)
        marker.scale.x = max(0.001, profile.tray_width_mm / METERS_TO_MM)
        marker.scale.y = max(0.001, profile.tray_height_mm / METERS_TO_MM)
        marker.scale.z = 0.015
        marker.color.r = 0.1
        marker.color.g = 0.6
        marker.color.b = 1.0
        marker.color.a = 0.35
        self.tray_marker_pub.publish(marker)

    def handle_get_tray_dimensions(self, request, response):
        del request
        profile = self.active_profile
        if profile is None or not self.has_valid_tray_dimensions(
            profile.tray_width_mm if profile else 0.0,
            profile.tray_height_mm if profile else 0.0,
        ):
            response.success = False
            response.x_size_mm = 0.0
            response.y_size_mm = 0.0
            response.live_detection = False
            response.tray_name = profile.tray_name if profile else ""
            response.message = "Tray dimensions are unavailable: no active YOLO tray profile."
            return response
        response.success = True
        response.x_size_mm = float(profile.tray_width_mm)
        response.y_size_mm = float(profile.tray_height_mm)
        response.live_detection = False
        response.tray_name = profile.tray_name
        response.message = "Using taught YOLO tray profile dimensions."
        return response

    def seek_is_on(self) -> bool:
        return bool(self.seek_mode_active or self.seek_result_latched)

    def reset_seek_state(self) -> None:
        self.seek_started_time = 0.0
        self.seek_frame_attempted = False
        self.last_seek_pose = None

    def validate_ready_for_seek(self) -> None:
        profile = self.active_profile
        if profile is None:
            self.fail_node("Tray seek failed: no active tray profile")
        if self.pt_model is None:
            self.fail_node("Tray seek failed: YOLO model is not loaded")
        if not self.has_valid_tray_dimensions(profile.tray_width_mm, profile.tray_height_mm):
            self.fail_node("Tray seek failed: selected profile has invalid taught tray dimensions")
        if not (
            np.isfinite([
                profile.tray_plane_a,
                profile.tray_plane_b,
                profile.tray_plane_c,
                profile.tray_plane_reference_depth_m,
            ]).all()
            and profile.tray_plane_reference_depth_m > 0.0
        ):
            self.fail_node("Tray seek failed: selected profile has invalid taught tray plane")
        if self.latest_info is None:
            self.fail_node("Tray seek failed: camera info is unavailable")
        if self.latest_info.k[0] <= 1e-6 or self.latest_info.k[4] <= 1e-6:
            self.fail_node("Tray seek failed: camera intrinsics are invalid")

    def handle_seek(self, request, response):
        del request
        if self.seek_is_on():
            self.seek_mode_active = False
            self.seek_result_latched = False
            self.reset_seek_state()
            response.success = True
            response.message = "Seek cancelled"
            self.status = response.message
            return response
        self.validate_ready_for_seek()
        self.yolo_enabled = True
        self.seek_mode_active = True
        self.seek_result_latched = False
        self.reset_seek_state()
        self.seek_started_time = time.monotonic()
        response.success = True
        response.message = "Seek armed"
        self.status = "YOLO ON | Seek armed; next frame must pass tray size check"
        return response

    def handle_seek_complete(self, request, response):
        del request
        self.seek_mode_active = False
        self.seek_result_latched = False
        self.reset_seek_state()
        self.status = "Seek released by tray intercept"
        response.success = True
        response.message = self.status
        return response

    def handle_seek_status(self, request, response):
        del request
        active = self.seek_is_on()
        response.success = active
        response.message = "Seek: ON" if active else "Seek: OFF"
        return response

    def handle_go_to_teach(self, request, response):
        del request
        response.success = self.request_go_to_teach()
        response.message = self.status
        return response

    def can_go_to_teach(self) -> bool:
        profile = self.active_profile
        return profile is not None and profile.has_teach_joints and len(profile.teach_joints_deg) >= 6 and not self.go_to_teach_in_progress

    def request_go_to_teach(self) -> bool:
        profile = self.active_profile
        if profile is None:
            self.status = "Go to Teach: select a tray profile"
            return False
        if not profile.has_teach_joints or len(profile.teach_joints_deg) < 6:
            self.status = "Go to Teach: selected tray profile has no teach joints"
            return False
        if self.go_to_teach_in_progress:
            self.status = "Go to Teach: command in progress"
            return False
        if not self.movj_client.service_is_ready():
            self.status = "Go to Teach: MovJ service not ready"
            return False
        request = MovJ.Request()
        request.mode = True
        request.a, request.b, request.c, request.d, request.e, request.f = [float(v) for v in profile.teach_joints_deg[:6]]
        request.param_value = []
        self.go_to_teach_in_progress = True
        self.status = "Go to Teach: sending MovJ"
        future = self.movj_client.call_async(request)
        future.add_done_callback(self.handle_go_to_teach_done)
        return True

    def handle_go_to_teach_done(self, future) -> None:
        try:
            response = future.result()
            ok = response is not None and response.res != -1
            self.status = "Go to Teach: MovJ accepted" if ok else "Go to Teach: MovJ failed"
        except Exception as exc:
            self.status = "Go to Teach: MovJ error"
            self.get_logger().warn(f"Go to Teach: MovJ call failed: {exc}")
        finally:
            self.go_to_teach_in_progress = False

    def open_model_dialog(self) -> Optional[Path]:
        start_dir = self.model_root if self.model_root.exists() else self.profiles_dir
        filename_arg = str(start_dir)
        if filename_arg and not filename_arg.endswith("/"):
            filename_arg += "/"
        yolo_label = supported_yolo_label()
        command = (
            "if command -v zenity >/dev/null 2>&1; then "
            f"zenity --file-selection --title='Open {yolo_label} Tray PT Model' "
            f"--filename={self.shell_quote(filename_arg)} "
            f"--file-filter='{yolo_label} PT models | *.pt' "
            "--file-filter='All files | *'; "
            "elif command -v kdialog >/dev/null 2>&1; then "
            f"kdialog --title 'Open {yolo_label} Tray PT Model' --getopenfilename "
            f"{self.shell_quote(str(start_dir))} '{yolo_label} PT models (*.pt)'; "
            "fi 2>/dev/null"
        )
        try:
            result = subprocess.run(command, shell=True, check=False, text=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=300)
        except Exception as exc:
            self.status = f"Open Model failed: {exc}"
            return None
        selected = result.stdout.strip()
        return resolve_path(selected) if selected else None

    def request_open_model(self) -> None:
        self.status = f"Open Model: select {supported_yolo_label()} .pt model"
        selected_path = self.open_model_dialog()
        if selected_path is None:
            self.status = "Open Model cancelled"
            return
        if self.select_model_path(selected_path):
            self.save_runtime_ui_settings()
            self.save_selected_model_export_file()
            self.status = f"Loaded {self.profile_label()}"

    @staticmethod
    def shell_quote(text: str) -> str:
        return "'" + text.replace("'", "'\"'\"'") + "'"

    def can_delete_profile(self) -> bool:
        return self.active_profile is not None and not self.go_to_teach_in_progress

    def request_delete_profile(self) -> None:
        profile = self.active_profile
        if profile is None:
            self.status = "Delete Tray: select a YOLO tray model"
            return
        self.delete_confirm_active = True
        self.status = f"Confirm delete {profile.tray_name} model"

    def confirm_delete_profile(self) -> bool:
        profile = self.active_profile
        if profile is None:
            self.delete_confirm_active = False
            return False
        try:
            deleted_name = profile.path.name
            self.pt_model = None
            if profile.path.exists() and profile.path.is_file() and self.is_safe_model_path(profile.path):
                profile.path.unlink()
            deleted_models = self.delete_model_artifacts(profile)
            deleted_message = f"Deleted profile {deleted_name} and model folder" if deleted_models else f"Deleted profile {deleted_name}; model folder missing or outside model root"
        except Exception as exc:
            self.status = f"Delete Tray failed: {exc}"
            self.delete_confirm_active = False
            return False
        self.delete_confirm_active = False
        self.active_profile = None
        self.selected_profile_index = -1
        self.selected_model_path = None
        self.selected_profile_path = None
        self.pt_model = None
        self.latest_detections = []
        self.selected_detection = None
        self.selected_pose = None
        self.refresh_profiles()
        self.save_selected_model_export_file()
        self.save_selected_profile_export_file()
        self.publish_selected_profile()
        self.status = deleted_message if self.profiles else f"{deleted_message}; no profiles remaining"
        return True

    def delete_model_artifacts(self, profile: TrayProfile) -> List[Path]:
        deleted: List[Path] = []
        candidates: List[Path] = []
        if profile.model_dir:
            candidates.append(resolve_path(profile.model_dir))
        for path_text in [profile.model_path, profile.model_pt_path]:
            if not path_text:
                continue
            path = resolve_path(path_text)
            candidates.append(path.parent if path.suffix else path)
        seen = set()
        for candidate in candidates:
            try:
                resolved = candidate.resolve()
            except Exception:
                continue
            if resolved in seen or not self.is_safe_model_path(resolved):
                continue
            seen.add(resolved)
            try:
                if resolved.is_dir():
                    shutil.rmtree(resolved)
                    deleted.append(resolved)
                elif resolved.is_file():
                    resolved.unlink()
                    deleted.append(resolved)
            except Exception as exc:
                self.get_logger().warn(f"Delete Tray: could not delete model path {resolved}: {exc}")
        return deleted

    def is_safe_model_path(self, path: Path) -> bool:
        try:
            resolved_root = self.model_root.resolve()
            resolved_path = path.resolve()
            if resolved_path == resolved_root:
                return False
            resolved_path.relative_to(resolved_root)
            return True
        except Exception:
            return False

    def selected_profile_path_text(self) -> str:
        if self.active_profile is not None:
            return str(self.active_profile.path)
        if self.selected_profile_path is not None:
            return str(self.selected_profile_path)
        return ""

    def publish_selected_profile(self) -> None:
        msg = String()
        msg.data = self.selected_profile_path_text()
        self.selected_profile_pub.publish(msg)

    def save_selected_model_export_file(self) -> None:
        try:
            self.selected_model_export_path.parent.mkdir(parents=True, exist_ok=True)
            model_path = self.active_profile.model_path if self.active_profile is not None else ""
            self.selected_model_export_path.write_text(model_path + "\n", encoding="utf-8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to save selected YOLO tray model file: {exc}")

    def save_selected_profile_export_file(self) -> None:
        try:
            self.selected_profile_export_path.parent.mkdir(parents=True, exist_ok=True)
            self.selected_profile_export_path.write_text(self.selected_profile_path_text() + "\n", encoding="utf-8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to save selected YOLO tray profile file: {exc}")

    def load_runtime_ui_settings(self) -> None:
        try:
            if not self.runtime_settings_path.exists():
                return
            root = yaml.safe_load(self.runtime_settings_path.read_text(encoding="utf-8")) or {}
            if not isinstance(root, dict):
                return
            self.yolo_conf = float(np.clip(float(root.get("yolo_conf", self.yolo_conf)), 0.0, 1.0))
            self.yolo_iou = float(np.clip(float(root.get("yolo_iou", self.yolo_iou)), 0.0, 1.0))
            self.yolo_imgsz = max(32, int(root.get("yolo_imgsz", self.yolo_imgsz)))
            self.max_inference_hz = max(0.1, float(root.get("max_inference_hz", self.max_inference_hz)))
            self.dimension_tolerance_percent = int(np.clip(
                int(root.get("tray_dimension_tolerance_percent", self.dimension_tolerance_percent)),
                DIMENSION_TOLERANCE_MIN,
                DIMENSION_TOLERANCE_MAX,
            ))
            self.seek_window_sec = max(0.1, float(root.get("seek_window_sec", self.seek_window_sec)))
        except Exception as exc:
            self.get_logger().warn(f"Failed to load YOLO tray detect runtime UI settings: {exc}")

    def save_runtime_ui_settings(self) -> None:
        try:
            self.runtime_settings_path.parent.mkdir(parents=True, exist_ok=True)
            payload = {
                "yolo_conf": float(self.yolo_conf),
                "yolo_iou": float(self.yolo_iou),
                "yolo_imgsz": int(self.yolo_imgsz),
                "max_inference_hz": float(self.max_inference_hz),
                "tray_dimension_tolerance_percent": int(self.dimension_tolerance_percent),
                "seek_window_sec": float(self.seek_window_sec),
            }
            self.runtime_settings_path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to save YOLO tray detect runtime UI settings: {exc}")

    def render_overlay(self, frame: np.ndarray) -> np.ndarray:
        canvas = frame.copy()
        for index, detection in enumerate(self.latest_detections[:8]):
            color = (68, 214, 255) if detection is self.selected_detection else (120, 160, 220)
            mask = detection.mask
            tint = np.zeros_like(canvas)
            tint[mask > 0] = color
            canvas = cv2.addWeighted(canvas, 1.0, tint, 0.28, 0.0)
            pts = np.asarray(detection.corners, dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(canvas, [pts], True, color, 2, cv2.LINE_AA)
            x, y, w, h = detection.box
            cv2.rectangle(canvas, (x, y), (x + w, y + h), color, 2)
            cv2.putText(canvas, f"{detection.score:.2f}", (x, max(22, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.58, color, 2, cv2.LINE_AA)
        if self.selected_axes_2d is not None:
            origin = tuple(int(round(v)) for v in self.selected_axes_2d.origin)
            x_tip = (int(round(origin[0] + self.selected_axes_2d.x_dir[0] * 90)), int(round(origin[1] + self.selected_axes_2d.x_dir[1] * 90)))
            y_tip = (int(round(origin[0] + self.selected_axes_2d.y_dir[0] * 70)), int(round(origin[1] + self.selected_axes_2d.y_dir[1] * 70)))
            cv2.arrowedLine(canvas, origin, x_tip, (80, 235, 90), 3, cv2.LINE_AA, tipLength=0.18)
            cv2.arrowedLine(canvas, origin, y_tip, (255, 194, 64), 3, cv2.LINE_AA, tipLength=0.18)
        top = np.zeros((TOP_BAR_HEIGHT, max(canvas.shape[1], PREVIEW_CANVAS_WIDTH), 3), dtype=np.uint8)
        top[:] = (34, 38, 44)
        self.draw_top_bar(top)
        if canvas.shape[1] < top.shape[1]:
            pad = np.zeros((canvas.shape[0], top.shape[1] - canvas.shape[1], 3), dtype=np.uint8)
            canvas = np.hstack([canvas, pad])
        return np.vstack([top, canvas])

    def draw_top_bar(self, top: np.ndarray) -> None:
        self.buttons = {
            "seek": Button("seek", (18, 18, 136, BUTTON_HEIGHT), True),
            "open_model": Button("open_model", (168, 18, 160, BUTTON_HEIGHT), True),
            "go_to_teach": Button("go_to_teach", (342, 18, 156, BUTTON_HEIGHT), self.can_go_to_teach()),
            "delete": Button("delete", (512, 18, 146, BUTTON_HEIGHT), self.can_delete_profile()),
            "toggle_yolo": Button("toggle_yolo", (672, 18, 132, BUTTON_HEIGHT), self.active_profile is not None),
        }
        labels = {
            "seek": "Seek: ON" if self.seek_is_on() else "Seek: OFF",
            "open_model": "Open Model",
            "go_to_teach": "Go Teach..." if self.go_to_teach_in_progress else "Go To Teach",
            "delete": "Confirm" if self.delete_confirm_active else "Delete Tray",
            "toggle_yolo": "YOLO: ON" if self.yolo_enabled else "YOLO: OFF",
        }
        for name, button in self.buttons.items():
            x, y, w, h = button.rect
            active = (name == "seek" and self.seek_is_on()) or (name == "toggle_yolo" and self.yolo_enabled)
            fill = (64, 132, 90) if active else (57, 64, 74)
            if not button.enabled:
                fill = (49, 52, 58)
            cv2.rectangle(top, (x, y), (x + w, y + h), fill, cv2.FILLED)
            cv2.rectangle(top, (x, y), (x + w, y + h), (110, 120, 132), 1)
            cv2.putText(top, labels[name], (x + 10, y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (232, 236, 240), 1, cv2.LINE_AA)
        profile = fit_text(self.profile_label(), 72)
        model_text = Path(self.active_profile.model_path).name if self.active_profile is not None else f"Open {supported_yolo_label()} model"
        cv2.putText(top, profile, (18, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (238, 242, 246), 1, cv2.LINE_AA)
        cv2.putText(top, fit_text(model_text, 82), (18, 116), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (178, 188, 198), 1, cv2.LINE_AA)
        cv2.putText(top, fit_text(self.status, 110), (342, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (210, 226, 246), 1, cv2.LINE_AA)
        speed = "Inference: waiting" if self.inference_count <= 0 else f"Inference: {self.inference_duration_ema_ms:.0f} ms | {self.inference_fps_ema:.1f} FPS"
        cv2.putText(top, speed, (342, 116), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (178, 188, 198), 1, cv2.LINE_AA)

    def mouse_callback(self, event: int, x: int, y: int, flags: int, param) -> None:
        del flags, param
        if event != cv2.EVENT_LBUTTONDOWN or y >= TOP_BAR_HEIGHT:
            return
        for name, button in self.buttons.items():
            bx, by, bw, bh = button.rect
            if not button.enabled or not (bx <= x <= bx + bw and by <= y <= by + bh):
                continue
            if name == "seek":
                self.handle_seek(None, Trigger.Response())
            elif name == "open_model":
                self.request_open_model()
            elif name == "go_to_teach":
                self.request_go_to_teach()
            elif name == "delete":
                if self.delete_confirm_active:
                    self.confirm_delete_profile()
                else:
                    self.request_delete_profile()
            elif name == "toggle_yolo":
                self.yolo_enabled = not self.yolo_enabled
                self.status = "YOLO ON | live inference enabled" if self.yolo_enabled else "YOLO OFF | live inference paused"
            return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrayDetectYoloNode()
    try:
        rclpy.spin(node)
    finally:
        node.save_runtime_ui_settings()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

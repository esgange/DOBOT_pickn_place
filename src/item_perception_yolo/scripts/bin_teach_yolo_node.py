#!/usr/bin/env python3
import base64
import datetime as _dt
import math
import os
import re
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import yaml
from aruco_perception.msg import MarkerDetections
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener


Vec2 = Tuple[float, float]
Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]
Pose7 = Tuple[float, float, float, float, float, float, float]


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

    candidates = [Path.cwd(), Path(__file__).resolve()]
    for name in ("COLCON_PREFIX_PATH", "AMENT_PREFIX_PATH"):
        for token in os.environ.get(name, "").split(os.pathsep):
            if not token:
                continue
            prefix = Path(token)
            candidates.append(prefix)
            if "install" in prefix.parts:
                candidates.append(Path(*prefix.parts[: prefix.parts.index("install")]))

    for candidate in candidates:
        found = find_from(candidate)
        if found is not None:
            return found
    return Path.cwd().resolve()


def workspace_path(*parts: str) -> Path:
    return workspace_root().joinpath(*parts)


def resolve_path(value: object, default: Optional[Path] = None) -> Path:
    text = str(value or "").strip()
    if not text and default is not None:
        return default
    path = Path(text).expanduser()
    if path.is_absolute():
        return path
    return workspace_path(str(path))


def sanitize_name(value: str) -> str:
    token: List[str] = []
    previous_underscore = False
    for ch in str(value or "").strip():
        if ch.isalnum() or ch == "_":
            token.append(ch.lower())
            previous_underscore = False
        elif not previous_underscore:
            token.append("_")
            previous_underscore = True
    return "".join(token).strip("_")


def bin_teach_filename_stem(safe_bin_name: str, compact_date: str) -> str:
    base = safe_bin_name if safe_bin_name == "bin" or safe_bin_name.startswith("bin_") else f"bin_{safe_bin_name}"
    return f"{base}_{compact_date}"


def station_config_value(*keys: str) -> str:
    try:
        values: Dict[str, str] = {}
        with workspace_path("station_config").open("r", encoding="utf-8") as stream:
            for raw_line in stream:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                if line.startswith("export "):
                    line = line[len("export ") :].strip()
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


def resolve_robot_ip_address(value: str = "") -> str:
    requested = str(value or "").strip()
    if requested:
        return requested
    env_ip = os.environ.get("ROBOT_IP_ADDRESS", "").strip()
    if env_ip:
        return env_ip
    return station_config_value("ROBOT_IP_ADDRESS", "ip_address")


def sanitize_filename_token(value: str) -> str:
    token: List[str] = []
    previous_underscore = False
    for ch in str(value or "").strip():
        if ch.isalnum() or ch in "._-":
            token.append(ch)
            previous_underscore = False
        elif not previous_underscore:
            token.append("_")
            previous_underscore = True
    return "".join(token).strip("_")


def find_latest_robot_calibration(calibration_dir: Path, prefix: str, robot_ip_address: str) -> Optional[Path]:
    ip_token = sanitize_filename_token(robot_ip_address)
    if not ip_token or not calibration_dir.exists():
        return None
    candidates = [
        path
        for path in calibration_dir.iterdir()
        if path.is_file()
        and path.suffix.lower() in (".yaml", ".yml")
        and path.name.startswith(prefix)
        and path.stem.endswith(f"_{ip_token}")
        and path.stat().st_size > 0
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


def q_normalize(q: Quat) -> Quat:
    norm = math.sqrt(sum(float(v) * float(v) for v in q))
    if norm <= 1e-12 or not math.isfinite(norm):
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(float(v) / norm for v in q)  # type: ignore[return-value]


def q_conjugate(q: Quat) -> Quat:
    x, y, z, w = q_normalize(q)
    return (-x, -y, -z, w)


def q_multiply(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = q_normalize(a)
    bx, by, bz, bw = q_normalize(b)
    return q_normalize(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def q_rotate(q: Quat, v: Vec3) -> Vec3:
    x, y, z, w = q_normalize(q)
    qv = np.array([x, y, z], dtype=np.float64)
    vec = np.array(v, dtype=np.float64)
    t = 2.0 * np.cross(qv, vec)
    out = vec + (w * t) + np.cross(qv, t)
    return (float(out[0]), float(out[1]), float(out[2]))


def pose_compose(parent_to_child: Pose7, child_to_grandchild: Pose7) -> Pose7:
    parent_t = parent_to_child[:3]
    parent_q = parent_to_child[3:]
    child_t = child_to_grandchild[:3]
    child_q = child_to_grandchild[3:]
    rotated = q_rotate(parent_q, child_t)  # type: ignore[arg-type]
    q = q_multiply(parent_q, child_q)  # type: ignore[arg-type]
    return (
        parent_t[0] + rotated[0],
        parent_t[1] + rotated[1],
        parent_t[2] + rotated[2],
        q[0],
        q[1],
        q[2],
        q[3],
    )


def transform_point(parent_from_child: Pose7, point: Vec3) -> Vec3:
    t = parent_from_child[:3]
    q = parent_from_child[3:]
    rotated = q_rotate(q, point)  # type: ignore[arg-type]
    return (t[0] + rotated[0], t[1] + rotated[1], t[2] + rotated[2])


def average_quaternions(quaternions: List[Quat]) -> Quat:
    clean = [q_normalize(q) for q in quaternions]
    if not clean:
        return (0.0, 0.0, 0.0, 1.0)
    ref = clean[0]
    total = np.zeros(4, dtype=np.float64)
    for quat in clean:
        arr = np.array(quat, dtype=np.float64)
        if float(np.dot(np.array(ref), arr)) < 0.0:
            arr = -arr
        total += arr
    return q_normalize(tuple(float(v) for v in total))  # type: ignore[arg-type]


@dataclass
class MarkerData:
    detection_index: int
    marker_id: int
    same_id_index: int
    position_m: Vec3
    orientation_xyzw: Quat
    pixel_center: Optional[Vec2] = None
    pixel_corners: List[Vec2] = field(default_factory=list)
    camera_corners_m: List[Vec3] = field(default_factory=list)


@dataclass
class DetectionFrame:
    stamp_sec: float
    received_monotonic: float
    frame_id: str
    image_width: int
    image_height: int
    markers: List[MarkerData]


@dataclass
class CornerDot:
    role: str
    x: float
    y: float
    marker_id: int
    detection_index: int
    same_id_index: int
    corner_index: int
    platform_position_m: Vec3


class BinTeachYoloNode(Node):
    def __init__(self) -> None:
        super().__init__("bin_teach_yolo")
        self.detections_topic = str(self.declare_parameter("detections_topic", "/aruco_detections").value)
        self.color_topic = str(self.declare_parameter("color_topic", "/bin_camera/color/image_raw").value).strip()
        self.depth_topic = str(self.declare_parameter("depth_topic", "/bin_camera/depth/image_raw").value).strip()
        self.camera_info_topic = str(
            self.declare_parameter("camera_info_topic", "/bin_camera/color/camera_info").value
        ).strip()
        self.overlay_topic = str(self.declare_parameter("overlay_topic", "/aruco_overlay").value).strip()
        self.use_aruco_overlay = bool(self.declare_parameter("use_aruco_overlay", True).value)
        self.output_dir = resolve_path(
            self.declare_parameter("output_dir", str(workspace_path("teach", "bin_teach"))).value
        )
        self.default_bin_name = str(self.declare_parameter("bin_name", "").value).strip()
        self.marker_prefix = str(self.declare_parameter("marker_prefix", "aruco_marker").value).strip()
        self.bin_frame_prefix = str(self.declare_parameter("bin_frame_prefix", "bin").value).strip() or "bin"
        self.allowed_marker_ids = [
            int(value) for value in self.declare_parameter("allowed_marker_ids", [1, 2, 3, 4]).value
        ]
        self.required_marker_count = int(self.declare_parameter("required_marker_count", 4).value)
        deduped_marker_ids: List[int] = []
        for marker_id in self.allowed_marker_ids:
            if marker_id not in deduped_marker_ids:
                deduped_marker_ids.append(marker_id)
        self.allowed_marker_ids = deduped_marker_ids
        if not self.allowed_marker_ids:
            raise RuntimeError("allowed_marker_ids must contain at least one marker ID")
        if self.required_marker_count != 4:
            raise RuntimeError("required_marker_count must be 4 because bin ROI needs exactly four corner points")
        self.max_detection_age_sec = float(self.declare_parameter("max_detection_age_sec", 0.75).value)
        self.tf_lookup_timeout_sec = float(self.declare_parameter("tf_lookup_timeout_sec", 0.2).value)
        self.base_frame = str(self.declare_parameter("base_frame", "base_link").value).strip().strip("/")
        self.platform_frame = str(self.declare_parameter("platform_frame", "platform_reference").value).strip().strip("/")
        self.platform_parent_frame = str(
            self.declare_parameter("platform_parent_frame", self.base_frame).value
        ).strip().strip("/")
        self.platform_name = self.platform_frame
        self.platform_calibration_dir = resolve_path(
            self.declare_parameter("platform_calibration_dir", str(workspace_path("calibration"))).value
        )
        self.platform_calibration_file = str(self.declare_parameter("platform_calibration_file", "").value).strip()
        self.robot_ip_address = resolve_robot_ip_address(
            str(self.declare_parameter("robot_ip_address", "").value).strip()
        )
        self.publish_static_platform_tf = bool(
            self.declare_parameter("publish_static_platform_tf", True).value
        )
        self.auto_save = bool(self.declare_parameter("auto_save", False).value)
        self.save_once = bool(self.declare_parameter("save_once", True).value)
        self.headless = bool(self.declare_parameter("headless", False).value)

        self._lock = threading.RLock()
        self.latest_frame: Optional[DetectionFrame] = None
        self.latest_saved_path: Optional[Path] = None
        self.last_save_error = ""
        self._auto_saved = False
        self.platform_pose_m: Optional[Pose7] = None
        self.platform_calibration_loaded = False
        self.latest_color_rgb: Optional[np.ndarray] = None
        self.latest_color_received_monotonic = 0.0
        self.latest_overlay_rgb: Optional[np.ndarray] = None
        self.latest_overlay_received_monotonic = 0.0
        self.unsupported_image_encodings: set[str] = set()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.color_sub = self.create_subscription(
            Image,
            self.color_topic,
            self.color_callback,
            qos_profile_sensor_data,
        )
        self.overlay_sub = self.create_subscription(
            Image,
            self.overlay_topic,
            self.overlay_callback,
            qos_profile_sensor_data,
        )
        self.detections_sub = self.create_subscription(
            MarkerDetections,
            self.detections_topic,
            self.detections_callback,
            qos_profile_sensor_data,
        )
        self.status_timer = self.create_timer(0.5, self.timer_callback)
        self.load_platform_calibration_at_start()
        self.get_logger().info(
            f"Bin teach YOLO node watching {self.detections_topic}; output_dir={self.output_dir}"
        )

    def color_callback(self, msg: Image) -> None:
        rgb = self.image_msg_to_rgb_array(msg, self.color_topic)
        if rgb is None:
            return
        with self._lock:
            self.latest_color_rgb = rgb
            self.latest_color_received_monotonic = time.monotonic()

    def overlay_callback(self, msg: Image) -> None:
        rgb = self.image_msg_to_rgb_array(msg, self.overlay_topic)
        if rgb is None:
            return
        with self._lock:
            self.latest_overlay_rgb = rgb
            self.latest_overlay_received_monotonic = time.monotonic()

    def image_msg_to_rgb_array(self, msg: Image, source_name: str) -> Optional[np.ndarray]:
        width = int(msg.width)
        height = int(msg.height)
        step = int(msg.step)
        if width <= 0 or height <= 0 or step <= 0:
            return None
        encoding = str(msg.encoding or "").strip().lower()
        if len(msg.data) < step * height:
            self.get_logger().warn(f"Received invalid {source_name} image: data buffer too small.")
            return None

        try:
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, step))
            if encoding in ("rgb8", "8uc3"):
                row_bytes = width * 3
                if step < row_bytes:
                    return None
                return raw[:, :row_bytes].reshape((height, width, 3)).copy()
            if encoding == "bgr8":
                row_bytes = width * 3
                if step < row_bytes:
                    return None
                bgr = raw[:, :row_bytes].reshape((height, width, 3))
                return bgr[:, :, ::-1].copy()
            if encoding in ("rgba8", "8uc4"):
                row_bytes = width * 4
                if step < row_bytes:
                    return None
                return raw[:, :row_bytes].reshape((height, width, 4))[:, :, :3].copy()
            if encoding == "bgra8":
                row_bytes = width * 4
                if step < row_bytes:
                    return None
                bgra = raw[:, :row_bytes].reshape((height, width, 4))
                return bgra[:, :, 2::-1].copy()
            if encoding in ("mono8", "8uc1"):
                if step < width:
                    return None
                mono = raw[:, :width].copy()
                return np.repeat(mono[:, :, None], 3, axis=2)
        except Exception as exc:
            self.get_logger().warn(f"Could not convert {source_name} image: {exc}")
            return None

        key = f"{source_name}|{encoding}"
        if key not in self.unsupported_image_encodings:
            self.unsupported_image_encodings.add(key)
            self.get_logger().warn(
                f"Unsupported {source_name} encoding '{msg.encoding}'. Expected bgr8/rgb8/mono8."
            )
        return None

    def latest_visualization_rgb(self) -> Tuple[Optional[np.ndarray], str]:
        max_age_sec = 3.0
        now = time.monotonic()
        with self._lock:
            overlay_fresh = (
                self.use_aruco_overlay
                and self.latest_overlay_rgb is not None
                and now - self.latest_overlay_received_monotonic <= max_age_sec
            )
            color_fresh = (
                self.latest_color_rgb is not None
                and now - self.latest_color_received_monotonic <= max_age_sec
            )
            if overlay_fresh:
                return self.latest_overlay_rgb.copy(), self.overlay_topic
            if color_fresh:
                return self.latest_color_rgb.copy(), self.color_topic
        return None, self.overlay_topic if self.use_aruco_overlay else self.color_topic

    def camera_topic_counts(self) -> Dict[str, int]:
        def count(topic: str) -> int:
            return int(self.count_publishers(topic)) if topic else 0

        return {
            "color": count(self.color_topic),
            "depth": count(self.depth_topic),
            "info": count(self.camera_info_topic),
        }

    def camera_topics_have_no_publishers(self) -> bool:
        counts = self.camera_topic_counts()
        return all(value <= 0 for value in counts.values())

    def current_roi_corner_dots(self) -> Tuple[List[CornerDot], str]:
        frame, raw_markers, status = self.current_detection_status()
        if frame is None:
            return [], status[0] if status else "No current ArUco detection frame yet"
        age = time.monotonic() - frame.received_monotonic
        if age > self.max_detection_age_sec:
            return [], f"Latest ArUco detection frame is stale: {age:.2f}s"
        if len(raw_markers) < self.required_marker_count:
            return [], "Need four allowed ArUco marker detections for ROI dots"
        try:
            target_from_source = self.transform_pose_to_teach_frame(frame.frame_id)
            return self.corner_dots_from_markers(raw_markers, target_from_source), ""
        except Exception as exc:
            return [], str(exc)

    @staticmethod
    def yaml_map(value: object) -> Dict:
        return value if isinstance(value, dict) else {}

    @staticmethod
    def parse_transform_values(transform: Dict) -> Tuple[Vec3, Quat]:
        translation_node = BinTeachYoloNode.yaml_map(transform.get("translation", {}))
        rotation_node = BinTeachYoloNode.yaml_map(transform.get("rotation", {}))
        translation = (
            float(translation_node.get("x", 0.0)),
            float(translation_node.get("y", 0.0)),
            float(translation_node.get("z", 0.0)),
        )
        rotation = q_normalize(
            (
                float(rotation_node.get("x", 0.0)),
                float(rotation_node.get("y", 0.0)),
                float(rotation_node.get("z", 0.0)),
                float(rotation_node.get("w", 1.0)),
            )
        )
        if not np.isfinite([*translation, *rotation]).all():
            raise ValueError("non-finite transform value")
        return translation, rotation

    def load_platform_calibration_at_start(self) -> None:
        path: Optional[Path] = None
        if self.platform_calibration_file:
            path = resolve_path(self.platform_calibration_file)
        else:
            path = find_latest_robot_calibration(
                self.platform_calibration_dir,
                "platform_calibration_",
                self.robot_ip_address,
            )
        if path is None:
            raise RuntimeError(
                "No platform calibration selected. Set platform_calibration_file for portable bin teach."
            )
        try:
            root = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            metadata = self.yaml_map(root.get("metadata", {}))
            transform = self.yaml_map(root.get("transform", {}))
            if not transform:
                raise ValueError("missing transform")
            translation, rotation = self.parse_transform_values(transform)
            self.platform_parent_frame = str(
                metadata.get("transform_parent_frame") or self.platform_parent_frame
            ).strip().strip("/")
            self.platform_frame = str(metadata.get("transform_child_frame") or self.platform_frame).strip().strip("/")
            self.platform_name = str(metadata.get("platform_name") or self.platform_frame).strip()
            self.platform_pose_m = (
                translation[0],
                translation[1],
                translation[2],
                rotation[0],
                rotation[1],
                rotation[2],
                rotation[3],
            )
            self.platform_calibration_file = str(path)
            self.platform_calibration_loaded = True
            if self.publish_static_platform_tf:
                self.publish_platform_tf()
            self.get_logger().info(
                f"Loaded platform calibration {self.platform_parent_frame}->{self.platform_frame}: {path}"
            )
        except Exception as exc:
            raise RuntimeError(f"Failed to load platform calibration {path}: {exc}") from exc

    def publish_platform_tf(self) -> None:
        if self.platform_pose_m is None:
            return
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.platform_parent_frame
        msg.child_frame_id = self.platform_frame
        msg.transform.translation.x = self.platform_pose_m[0]
        msg.transform.translation.y = self.platform_pose_m[1]
        msg.transform.translation.z = self.platform_pose_m[2]
        msg.transform.rotation.x = self.platform_pose_m[3]
        msg.transform.rotation.y = self.platform_pose_m[4]
        msg.transform.rotation.z = self.platform_pose_m[5]
        msg.transform.rotation.w = self.platform_pose_m[6]
        self.static_tf_broadcaster.sendTransform(msg)

    def detections_callback(self, msg: MarkerDetections) -> None:
        id_counts: Dict[int, int] = {}
        markers: List[MarkerData] = []
        for index, marker_id_raw in enumerate(msg.ids):
            if index >= len(msg.poses):
                continue
            marker_id = int(marker_id_raw)
            id_counts[marker_id] = id_counts.get(marker_id, 0) + 1
            pose = msg.poses[index]
            marker = MarkerData(
                detection_index=index,
                marker_id=marker_id,
                same_id_index=id_counts[marker_id],
                position_m=(
                    float(pose.position.x),
                    float(pose.position.y),
                    float(pose.position.z),
                ),
                orientation_xyzw=q_normalize(
                    (
                        float(pose.orientation.x),
                        float(pose.orientation.y),
                        float(pose.orientation.z),
                        float(pose.orientation.w),
                    )
                ),
            )
            if index < len(msg.pixel_centers):
                center = msg.pixel_centers[index]
                marker.pixel_center = (float(center.x), float(center.y))
            if index < len(msg.pixel_corners):
                marker.pixel_corners = [
                    (float(point.x), float(point.y))
                    for point in msg.pixel_corners[index].points[:4]
                ]
            if index < len(msg.camera_corners):
                marker.camera_corners_m = [
                    (float(point.x), float(point.y), float(point.z))
                    for point in msg.camera_corners[index].points[:4]
                ]
            markers.append(marker)

        stamp = float(msg.header.stamp.sec) + (float(msg.header.stamp.nanosec) * 1e-9)
        frame = DetectionFrame(
            stamp_sec=stamp,
            received_monotonic=time.monotonic(),
            frame_id=str(msg.header.frame_id).strip().strip("/"),
            image_width=int(msg.image_width),
            image_height=int(msg.image_height),
            markers=markers,
        )
        with self._lock:
            self.latest_frame = frame

    def timer_callback(self) -> None:
        if self.publish_static_platform_tf and self.platform_calibration_loaded:
            self.publish_platform_tf()
        if not self.auto_save:
            return
        if self.save_once and self._auto_saved:
            return
        bin_name = self.default_bin_name
        if not sanitize_name(bin_name):
            return
        try:
            self.save_current_bin_teach(bin_name)
            self._auto_saved = True
        except Exception as exc:
            self.last_save_error = str(exc)

    def selected_markers_with_errors(self, frame: DetectionFrame) -> Tuple[List[MarkerData], List[str]]:
        selected = sorted(
            [marker for marker in frame.markers if marker.marker_id in self.allowed_marker_ids],
            key=lambda marker: marker.detection_index,
        )
        messages: List[str] = []
        by_id: Dict[int, int] = {}
        for marker in selected:
            by_id[marker.marker_id] = by_id.get(marker.marker_id, 0) + 1
        repeated = [
            f"{marker_id} ({count} detections)"
            for marker_id, count in sorted(by_id.items())
            if count > 1
        ]
        if repeated:
            messages.append("Repeated allowed ArUco marker IDs accepted: " + ", ".join(repeated))
        if len(selected) < self.required_marker_count:
            messages.append(
                f"Need {self.required_marker_count} allowed ArUco marker detections; "
                f"found {len(selected)}. Allowed IDs: "
                + ", ".join(str(marker_id) for marker_id in self.allowed_marker_ids)
            )
            return selected, messages
        if len(selected) > self.required_marker_count:
            messages.append(
                f"{len(selected)} allowed ArUco marker detections visible; "
                f"using first {self.required_marker_count} by detection order."
            )
        return selected[: self.required_marker_count], messages

    def selected_markers(self, frame: DetectionFrame) -> List[MarkerData]:
        selected, _ = self.selected_markers_with_errors(frame)
        return selected

    def current_detection_status(self) -> Tuple[Optional[DetectionFrame], List[MarkerData], List[str]]:
        with self._lock:
            frame = self.latest_frame
        lines: List[str] = []
        if frame is None:
            return None, [], ["No ArUco detection frame yet."]
        age = time.monotonic() - frame.received_monotonic
        selected, selection_messages = self.selected_markers_with_errors(frame)
        lines.append(f"Detection frame age {age:.2f}s in {frame.frame_id or '<empty>'}.")
        allowed_ids = ", ".join(str(v) for v in self.allowed_marker_ids)
        lines.append(
            f"Visible allowed marker detections: {len(selected)}/{self.required_marker_count} "
            f"(allowed IDs: {allowed_ids}; repeated IDs allowed)"
        )
        lines.extend(selection_messages)
        if age > self.max_detection_age_sec:
            lines.append(f"Frame is stale; max age is {self.max_detection_age_sec:.2f}s.")
        for marker in selected:
            lines.append(
                f"{self.marker_prefix}_{marker.marker_id}#{marker.same_id_index}: "
                f"x={marker.position_m[0]:.3f} y={marker.position_m[1]:.3f} "
                f"z={marker.position_m[2]:.3f} m"
            )
        return frame, selected, lines

    def transform_pose_to_teach_frame(self, source_frame: str) -> Pose7:
        if not self.platform_calibration_loaded:
            raise RuntimeError("platform_calibration_file is required for portable bin teach")
        if source_frame == self.platform_frame:
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.platform_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=max(0.01, self.tf_lookup_timeout_sec)),
            )
        except TransformException as exc:
            raise RuntimeError(
                f"Cannot transform detections from {source_frame} to {self.platform_frame}: {exc}"
            ) from exc
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        return (
            float(t.x),
            float(t.y),
            float(t.z),
            float(q.x),
            float(q.y),
            float(q.z),
            float(q.w),
        )

    @staticmethod
    def transform_marker(marker: MarkerData, target_from_source: Pose7) -> MarkerData:
        source_to_marker = (
            marker.position_m[0],
            marker.position_m[1],
            marker.position_m[2],
            marker.orientation_xyzw[0],
            marker.orientation_xyzw[1],
            marker.orientation_xyzw[2],
            marker.orientation_xyzw[3],
        )
        target_to_marker = pose_compose(target_from_source, source_to_marker)
        output = MarkerData(
            detection_index=marker.detection_index,
            marker_id=marker.marker_id,
            same_id_index=marker.same_id_index,
            position_m=target_to_marker[:3],  # type: ignore[assignment]
            orientation_xyzw=target_to_marker[3:],  # type: ignore[assignment]
            pixel_center=marker.pixel_center,
            pixel_corners=list(marker.pixel_corners),
            camera_corners_m=list(marker.camera_corners_m),
        )
        return output

    @staticmethod
    def marker_centroid_pose(markers: List[MarkerData]) -> Tuple[Vec3, Quat, Dict[str, Vec3], List[Dict[str, object]]]:
        if not markers:
            raise RuntimeError("No markers available for bin teach")
        positions: Dict[str, Vec3] = {}
        observations: List[Dict[str, object]] = []
        quats: List[Quat] = []
        for order, marker in enumerate(markers):
            key = f"marker_{order + 1}_id_{marker.marker_id}"
            positions[key] = marker.position_m
            observations.append(
                {
                    "key": key,
                    "id": marker.marker_id,
                    "detection_index": marker.detection_index,
                    "same_id_index": marker.same_id_index,
                }
            )
            quats.append(marker.orientation_xyzw)
        xs = [point[0] for point in positions.values()]
        ys = [point[1] for point in positions.values()]
        zs = [point[2] for point in positions.values()]
        center = (
            (min(xs) + max(xs)) * 0.5,
            (min(ys) + max(ys)) * 0.5,
            (min(zs) + max(zs)) * 0.5,
        )
        return center, average_quaternions(quats), positions, observations

    @staticmethod
    def axis_from_quaternion(q: Quat, axis: Vec3) -> Vec3:
        out = q_rotate(q, axis)
        norm = math.sqrt(sum(value * value for value in out))
        if norm <= 1e-12:
            return axis
        return (out[0] / norm, out[1] / norm, out[2] / norm)

    @staticmethod
    def order_corner_dots_for_roi(dots: List[CornerDot]) -> List[CornerDot]:
        if len(dots) != 4:
            raise RuntimeError("Need exactly four outside marker corners for bin ROI")

        center_x = sum(dot.platform_position_m[0] for dot in dots) / 4.0
        center_y = sum(dot.platform_position_m[1] for dot in dots) / 4.0
        ordered = sorted(
            dots,
            key=lambda dot: math.atan2(
                dot.platform_position_m[1] - center_y,
                dot.platform_position_m[0] - center_x,
            ),
        )

        start_index = min(range(4), key=lambda index: ordered[index].x + ordered[index].y)
        ordered = ordered[start_index:] + ordered[:start_index]

        # Keep the saved pixel ROI in the familiar upper-left -> upper-right -> lower-right -> lower-left order
        # when the teach camera is looking down at the bin. The platform-point order remains cyclic either way.
        if ordered[1].x < ordered[-1].x:
            ordered = [ordered[0], *reversed(ordered[1:])]

        roles = ("upper_left", "upper_right", "lower_right", "lower_left")
        for role, dot in zip(roles, ordered):
            dot.role = role
        return ordered

    @staticmethod
    def corner_dots_from_markers(
        markers: List[MarkerData],
        target_from_source: Pose7,
    ) -> List[CornerDot]:
        candidates = [
            marker
            for marker in markers
            if marker.pixel_center is not None
            and len(marker.pixel_corners) >= 4
            and len(marker.camera_corners_m) >= 4
        ]
        if len(candidates) < 4:
            raise RuntimeError("Need four marker pixel/camera corner detections to save bin ROI")

        candidates = candidates[:4]
        marker_centers = [transform_point(target_from_source, marker.position_m) for marker in candidates]
        roi_center = (
            sum(center[0] for center in marker_centers) / 4.0,
            sum(center[1] for center in marker_centers) / 4.0,
            sum(center[2] for center in marker_centers) / 4.0,
        )

        dots: List[CornerDot] = []
        for marker in candidates:
            platform_corners = [
                transform_point(target_from_source, corner)
                for corner in marker.camera_corners_m[:4]
            ]
            corner_index = max(
                range(4),
                key=lambda idx: (
                    (platform_corners[idx][0] - roi_center[0]) ** 2
                    + (platform_corners[idx][1] - roi_center[1]) ** 2
                ),
            )
            pixel = marker.pixel_corners[corner_index]
            dots.append(
                CornerDot(
                    role="",
                    x=float(pixel[0]),
                    y=float(pixel[1]),
                    marker_id=marker.marker_id,
                    detection_index=marker.detection_index,
                    same_id_index=marker.same_id_index,
                    corner_index=corner_index,
                    platform_position_m=platform_corners[corner_index],
                )
            )
        return BinTeachYoloNode.order_corner_dots_for_roi(dots)

    def compute_solution(self, bin_name: str) -> Dict[str, object]:
        safe_bin_name = sanitize_name(bin_name)
        if not safe_bin_name:
            raise RuntimeError("Enter a bin name before saving")
        frame, raw_markers, status = self.current_detection_status()
        if frame is None:
            raise RuntimeError("No current ArUco detection frame yet")
        age = time.monotonic() - frame.received_monotonic
        if age > self.max_detection_age_sec:
            raise RuntimeError(f"Latest ArUco detection frame is stale: {age:.2f}s")
        if len(raw_markers) < self.required_marker_count:
            raise RuntimeError("\n".join(status))

        target_from_source = self.transform_pose_to_teach_frame(frame.frame_id)
        teach_frame = self.platform_frame
        corner_dots = self.corner_dots_from_markers(raw_markers, target_from_source)
        now = _dt.datetime.now()
        teach_date = now.strftime("%Y-%m-%d")
        return {
            "bin_name": safe_bin_name,
            "teach_date": teach_date,
            "platform_roi_corners": {
                "coordinate_frame": teach_frame,
                "points": [
                    {
                        "position": {
                            "x": dot.platform_position_m[0],
                            "y": dot.platform_position_m[1],
                            "z": dot.platform_position_m[2],
                        },
                    }
                    for dot in corner_dots
                ],
            },
        }

    def save_current_bin_teach(self, bin_name: str) -> Path:
        solution = self.compute_solution(bin_name)
        safe_bin_name = str(solution["bin_name"])
        try:
            compact_date = _dt.datetime.strptime(str(solution["teach_date"]), "%Y-%m-%d").strftime("%d%m%Y")
        except ValueError:
            compact_date = _dt.datetime.now().strftime("%d%m%Y")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        path = self.output_dir / f"{bin_teach_filename_stem(safe_bin_name, compact_date)}.yaml"
        payload = {"bin_teach": solution}
        path.write_text(yaml.safe_dump(payload, sort_keys=False, default_flow_style=False), encoding="utf-8")
        with self._lock:
            self.latest_saved_path = path
            self.last_save_error = ""
        self.get_logger().info(f"Saved portable bin teach: {path}")
        return path

    def status_lines(self) -> List[str]:
        _frame, _markers, lines = self.current_detection_status()
        corner_dots, corner_error = self.current_roi_corner_dots()
        if corner_dots:
            lines.append("ROI dots: 4/4 outside ArUco marker corners ready.")
        elif corner_error:
            lines.append(f"ROI dots: {corner_error}")
        lines.append("")
        if self.platform_calibration_loaded:
            lines.append(f"Platform: {self.platform_parent_frame}->{self.platform_frame}")
        else:
            lines.append("Platform: missing platform_calibration_file")
        lines.append(f"Output: {self.output_dir}")
        if self.latest_saved_path is not None:
            lines.append(f"Saved: {self.latest_saved_path}")
        if self.last_save_error:
            lines.append(f"Last save error: {self.last_save_error}")
        return lines


class BinTeachYoloWindow:
    def __init__(self, node: BinTeachYoloNode) -> None:
        import tkinter as tk
        from tkinter import messagebox

        self.tk = tk
        self.messagebox = messagebox
        self.node = node
        self.root = tk.Tk()
        self.root.title("Bin Teach YOLO")
        self.root.geometry("1120x680")
        self.root.minsize(920, 560)
        self.bin_name_var = tk.StringVar(value=node.default_bin_name)
        self.photo = None

        controls = tk.Frame(self.root, padx=10, pady=10, width=360)
        controls.pack(side=tk.LEFT, fill=tk.Y)
        controls.pack_propagate(False)

        top = tk.Frame(controls)
        top.pack(fill=tk.X)
        tk.Label(top, text="Bin name").pack(side=tk.LEFT)
        entry = tk.Entry(top, textvariable=self.bin_name_var, width=28)
        entry.pack(side=tk.LEFT, padx=(8, 8), fill=tk.X, expand=True)
        entry.bind("<Return>", lambda _event: self.save())
        self.save_button = tk.Button(controls, text="Save Bin Teach", command=self.save)
        self.save_button.pack(fill=tk.X, pady=(8, 8))
        self.status = tk.Text(controls, height=18, state=tk.DISABLED, wrap=tk.WORD)
        self.status.pack(fill=tk.BOTH, expand=True)

        image_group = tk.Frame(self.root, padx=10, pady=10)
        image_group.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self.image_title_var = tk.StringVar(value=f"ArUco Overlay ({node.overlay_topic})")
        tk.Label(image_group, textvariable=self.image_title_var, anchor=tk.W).pack(fill=tk.X)
        self.image_label = tk.Label(
            image_group,
            text="Waiting for camera image ...",
            bg="#101010",
            fg="#d0d0d0",
            bd=1,
            relief=tk.SOLID,
            anchor=tk.CENTER,
            justify=tk.CENTER,
        )
        self.image_label.pack(fill=tk.BOTH, expand=True, pady=(6, 0))

        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.refresh()
        self.refresh_image()

    def refresh(self) -> None:
        text = "\n".join(self.node.status_lines())
        self.status.configure(state=self.tk.NORMAL)
        self.status.delete("1.0", self.tk.END)
        self.status.insert(self.tk.END, text)
        self.status.configure(state=self.tk.DISABLED)
        self.root.after(250, self.refresh)

    def fit_rgb_to_label(self, rgb: np.ndarray) -> np.ndarray:
        label_w = max(1, int(self.image_label.winfo_width()))
        label_h = max(1, int(self.image_label.winfo_height()))
        if label_w <= 1 or label_h <= 1:
            label_w, label_h = 720, 520
        height, width = rgb.shape[:2]
        if width <= 0 or height <= 0:
            return rgb
        scale = min(label_w / float(width), label_h / float(height), 1.0)
        new_w = max(1, int(round(width * scale)))
        new_h = max(1, int(round(height * scale)))
        if new_w == width and new_h == height:
            return np.ascontiguousarray(rgb)
        x_idx = np.linspace(0, width - 1, new_w).astype(np.int32)
        y_idx = np.linspace(0, height - 1, new_h).astype(np.int32)
        return np.ascontiguousarray(rgb[y_idx][:, x_idx])

    def rgb_to_photo_image(self, rgb: np.ndarray):
        rgb = self.fit_rgb_to_label(rgb)
        rgb = np.ascontiguousarray(rgb.astype(np.uint8, copy=False))
        try:
            from PIL import Image, ImageTk

            return ImageTk.PhotoImage(Image.fromarray(rgb, mode="RGB"))
        except Exception:
            pass

        try:
            bgr = np.ascontiguousarray(rgb[:, :, ::-1])
            ok, encoded_png = cv2.imencode(".png", bgr)
            if ok:
                payload = base64.b64encode(encoded_png.tobytes()).decode("ascii")
                return self.tk.PhotoImage(data=payload, format="png")
        except Exception:
            pass

        height, width = rgb.shape[:2]
        payload = b"P6\n%d %d\n255\n" % (width, height) + rgb.tobytes()
        encoded = base64.b64encode(payload).decode("ascii")
        return self.tk.PhotoImage(data=encoded, format="PPM")

    def draw_roi_corner_dots(self, rgb: np.ndarray, source: str) -> np.ndarray:
        dots, _error = self.node.current_roi_corner_dots()
        if len(dots) != 4:
            return rgb
        output = rgb.copy()
        height, width = output.shape[:2]
        with self.node._lock:
            detection_frame = self.node.latest_frame
            source_image_height = int(detection_frame.image_height) if detection_frame is not None else 0
        y_offsets = [0]
        if (
            source == self.node.overlay_topic
            and source_image_height > 0
            and height >= source_image_height * 2
        ):
            y_offsets.append(source_image_height)

        base_points: List[Tuple[int, int]] = []
        for dot in dots:
            if not np.isfinite([dot.x, dot.y]).all():
                return rgb
            x = int(round(dot.x))
            y = int(round(dot.y))
            x = min(max(x, 0), max(0, width - 1))
            y = min(max(y, 0), max(0, height - 1))
            base_points.append((x, y))

        for y_offset in y_offsets:
            points = [
                (x, min(max(y + y_offset, 0), max(0, height - 1)))
                for x, y in base_points
            ]
            polygon = np.asarray(points, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(output, [polygon], True, (0, 255, 255), 4, cv2.LINE_AA)
            for index, (x, y) in enumerate(points, start=1):
                cv2.circle(output, (x, y), 24, (0, 0, 0), -1, cv2.LINE_AA)
                cv2.circle(output, (x, y), 18, (255, 0, 255), -1, cv2.LINE_AA)
                cv2.circle(output, (x, y), 27, (255, 255, 255), 3, cv2.LINE_AA)
                cv2.drawMarker(
                    output,
                    (x, y),
                    (255, 255, 255),
                    cv2.MARKER_CROSS,
                    34,
                    3,
                    cv2.LINE_AA,
                )
                label = f"dot {index}"
                label_x = min(max(x + 24, 4), max(4, width - 90))
                label_y = min(max(y - 24, 24), max(24, height - 8))
                cv2.putText(
                    output,
                    label,
                    (label_x, label_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.72,
                    (0, 0, 0),
                    5,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    output,
                    label,
                    (label_x, label_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.72,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
        return output

    def build_no_camera_topics_placeholder(self) -> np.ndarray:
        label_w = max(1, int(self.image_label.winfo_width()))
        label_h = max(1, int(self.image_label.winfo_height()))
        width = max(620, label_w if label_w > 1 else 720)
        height = max(420, label_h if label_h > 1 else 520)
        image = np.zeros((height, width, 3), dtype=np.uint8)
        image[:] = (18, 20, 24)

        cv2.putText(
            image,
            "no camera topics...",
            (44, 96),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.15,
            (255, 210, 0),
            3,
            cv2.LINE_AA,
        )

        counts = self.node.camera_topic_counts()
        status_lines = [
            f"color: {self.node.color_topic}  publishers={counts['color']}",
            f"depth: {self.node.depth_topic}  publishers={counts['depth']}",
            f"info:  {self.node.camera_info_topic}  publishers={counts['info']}",
        ]
        y = 158
        for line in status_lines:
            cv2.putText(
                image,
                line,
                (48, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (225, 230, 235),
                2,
                cv2.LINE_AA,
            )
            y += 40
        return image

    def show_no_camera_topics_placeholder(self) -> bool:
        self.image_title_var.set("Camera Topics")
        try:
            self.photo = self.rgb_to_photo_image(self.build_no_camera_topics_placeholder())
            self.image_label.configure(image=self.photo, text="")
            return True
        except Exception:
            counts = self.node.camera_topic_counts()
            self.photo = None
            self.image_label.configure(
                image="",
                text=(
                    "no camera topics...\n\n"
                    f"color: {self.node.color_topic}  publishers={counts['color']}\n"
                    f"depth: {self.node.depth_topic}  publishers={counts['depth']}\n"
                    f"info:  {self.node.camera_info_topic}  publishers={counts['info']}"
                ),
            )
            return False

    def refresh_image(self) -> None:
        rgb, source = self.node.latest_visualization_rgb()
        if rgb is None:
            self.photo = None
            self.image_title_var.set(
                f"ArUco Overlay ({self.node.overlay_topic})"
                if self.node.use_aruco_overlay
                else f"Camera View ({self.node.color_topic})"
            )
            if self.node.camera_topics_have_no_publishers():
                self.show_no_camera_topics_placeholder()
            else:
                self.image_label.configure(
                    image="",
                    text=f"Waiting for camera image from {source} ...",
                )
        else:
            try:
                rgb = self.draw_roi_corner_dots(rgb, source)
                self.photo = self.rgb_to_photo_image(rgb)
                self.image_title_var.set(
                    f"ArUco Overlay + ROI Dots ({source})"
                    if source == self.node.overlay_topic
                    else f"Camera View + ROI Dots ({source})"
                )
                self.image_label.configure(image=self.photo, text="")
            except Exception as exc:
                if self.node.camera_topics_have_no_publishers():
                    self.show_no_camera_topics_placeholder()
                else:
                    self.photo = None
                    self.image_label.configure(image="", text=f"camera view error:\n{exc}")
        self.root.after(100, self.refresh_image)

    def save(self) -> None:
        try:
            path = self.node.save_current_bin_teach(self.bin_name_var.get())
            self.messagebox.showinfo("Bin Teach", f"Saved:\n{path}")
        except Exception as exc:
            self.node.last_save_error = str(exc)
            self.messagebox.showwarning("Bin Teach", str(exc))

    def close(self) -> None:
        self.root.quit()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    rclpy.init()
    node = BinTeachYoloNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        if node.headless:
            while rclpy.ok():
                time.sleep(0.25)
        else:
            window = BinTeachYoloWindow(node)
            window.run()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import ast
import os
import shutil
import subprocess
import time
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
warnings.filterwarnings("ignore", message="CUDA initialization:.*", category=UserWarning)

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


WINDOW_NAME = "item_detect_yolo_debug"
BIN_CAMERA_COLOR_TOPIC = "/bin_camera/color/image_raw"
TOP_BAR_HEIGHT = 178
PREVIEW_CANVAS_WIDTH = 1180
PREVIEW_CANVAS_HEIGHT = 680
BUTTON_HEIGHT = 38
IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}
MODEL_EXTENSIONS = {".onnx", ".pt"}


@dataclass
class Button:
    name: str
    rect: Tuple[int, int, int, int]
    enabled: bool = True


@dataclass
class Detection:
    score: float
    class_id: int
    box: Tuple[int, int, int, int]
    mask: np.ndarray
    center: Tuple[float, float]


def workspace_root() -> Path:
    def looks_like_root(path: Path) -> bool:
        return (
            (path / "src").exists()
            and ((path / "README.md").exists() or (path / "src" / "dobot_msgs_v4").exists())
        )

    for name in ("DOBOT_PICKN_PLACE_ROOT", "DOBOT_WORKSPACE_ROOT"):
        value = os.environ.get(name)
        if value:
            path = Path(value).expanduser().resolve()
            if looks_like_root(path):
                return path

    for start in (Path.cwd(), Path(__file__).resolve()):
        path = start.expanduser().resolve()
        if path.is_file():
            path = path.parent
        for candidate in (path, *path.parents):
            if looks_like_root(candidate):
                return candidate
    return Path.cwd().resolve()


def workspace_path(*parts: str) -> Path:
    return workspace_root().joinpath(*parts)


def resolve_path(path_text: str) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(path_text))).resolve()


def fit_text(text: str, max_chars: int) -> str:
    return text if len(text) <= max_chars else text[: max(0, max_chars - 3)] + "..."


def sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-np.clip(x, -80.0, 80.0)))


class ItemDetectYoloDebugNode(Node):
    def __init__(self) -> None:
        super().__init__("item_detect_yolo_debug")
        self.bridge = CvBridge()
        self.model_root = resolve_path(
            str(self.declare_parameter(
                "model_root",
                str(workspace_path("teach", "item_teach_yolo")),
            ).value)
        )
        self.samples_root = resolve_path(
            str(self.declare_parameter(
                "samples_root",
                str(workspace_path("config", "item_perception_yolo")),
            ).value)
        )
        model_path_text = str(self.declare_parameter("model_path", "").value).strip()
        samples_path_text = str(self.declare_parameter("samples_path", "").value).strip()
        self.color_topic = str(
            self.declare_parameter("color_topic", BIN_CAMERA_COLOR_TOPIC).value
        ).strip() or BIN_CAMERA_COLOR_TOPIC
        self.capture_dir = resolve_path(
            str(self.declare_parameter("capture_dir", "~/Desktop/images").value)
        )
        self.yolo_imgsz = int(self.declare_parameter("yolo_imgsz", 640).value)
        self.yolo_conf = float(np.clip(
            float(self.declare_parameter("yolo_conf", 0.35).value),
            0.0,
            1.0,
        ))
        self.yolo_iou = float(np.clip(
            float(self.declare_parameter("yolo_iou", 0.45).value),
            0.0,
            1.0,
        ))
        self.mask_threshold = float(np.clip(
            float(self.declare_parameter("mask_threshold", 0.5).value),
            0.0,
            1.0,
        ))
        self.class_id = int(self.declare_parameter("class_id", -1).value)
        self.max_samples = int(self.declare_parameter("max_samples", 2000).value)
        self.ort_threads = int(self.declare_parameter("onnxruntime_threads", 0).value)
        requested_pt_device = str(self.declare_parameter("pt_device", "cpu").value).strip()
        self.pt_device = "cpu"
        if requested_pt_device and requested_pt_device.lower() != "cpu":
            self.get_logger().warn(
                f"Ignoring pt_device={requested_pt_device}; .pt debug inference is CPU-only."
            )

        self.ort_session: Optional[ort.InferenceSession] = None
        self.ort_input_name = ""
        self.ort_output_names: List[str] = []
        self.pt_model = None
        self.model_backend = ""
        self.model_output_summary = "Outputs: none"
        self.onnx_class_names: Dict[int, str] = {}
        self.onnx_num_classes = 0
        self.onnx_task = ""
        self.onnx_has_nms = False
        self.model_path: Optional[Path] = None
        self.sample_paths: List[Path] = []
        self.sample_index = -1
        self.current_image: Optional[np.ndarray] = None
        self.current_image_path: Optional[Path] = None
        self.latest_camera_frame: Optional[np.ndarray] = None
        self.latest_camera_stamp_monotonic = 0.0
        self.detections: List[Detection] = []
        self.last_num_classes = 1
        self.last_inference_ms = 0.0
        self.last_decode_layout = ""
        self.status = "Open model and samples | Space saves camera image"

        self.buttons: Dict[str, Button] = {}
        self.slider_rects: Dict[str, Tuple[int, int, int, int]] = {}
        self.active_slider: Optional[str] = None
        self.preview_scale = 1.0
        self.preview_origin = (0, TOP_BAR_HEIGHT)
        self.window_ready = False

        if model_path_text:
            self.load_model(resolve_path(model_path_text))
        if samples_path_text:
            self.load_samples(resolve_path(samples_path_text), recent_first=False)
        else:
            self.load_default_samples()

        try:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL | getattr(cv2, "WINDOW_GUI_NORMAL", 0))
            cv2.resizeWindow(WINDOW_NAME, PREVIEW_CANVAS_WIDTH, TOP_BAR_HEIGHT + PREVIEW_CANVAS_HEIGHT)
            cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)
            self.window_ready = True
        except cv2.error as exc:
            self.get_logger().error(f"Could not open YOLO debug window: {exc}")

        self.color_sub = self.create_subscription(Image, self.color_topic, self.color_callback, 10)
        self.timer = self.create_timer(0.05, self.render_once)
        self.get_logger().info(
            f"YOLO item detect debug node ready. Space captures {self.color_topic} to {self.capture_dir}")

    def default_sample_roots(self) -> List[Path]:
        return [
            workspace_path("config", "item_perception_yolo", "item_teach_yolo_saved_sessions"),
            workspace_path("config", "item_perception_yolo", "item_teach_yolo_runtime"),
            workspace_path("teach", "item_teach_yolo"),
        ]

    def preferred_default_sample_dirs(self) -> List[Path]:
        dirs: List[Path] = []
        session_roots = [
            workspace_path("config", "item_perception_yolo", "item_teach_yolo_saved_sessions"),
            workspace_path("config", "item_perception_yolo", "item_teach_yolo_runtime"),
            workspace_path("teach", "item_teach_yolo"),
        ]
        for root in session_roots:
            if not root.exists() or not root.is_dir():
                continue
            try:
                sessions = [path for path in root.iterdir() if path.is_dir()]
            except OSError:
                continue
            sessions.sort(key=self.path_mtime, reverse=True)
            for session in sessions:
                for relative in (
                    ("images",),
                    ("dataset", "images", "train"),
                    ("dataset", "images", "val"),
                ):
                    candidate = session.joinpath(*relative)
                    if candidate.exists() and candidate.is_dir():
                        dirs.append(candidate)
        return dirs

    def load_default_samples(self) -> None:
        paths: List[Path] = []
        for root in self.preferred_default_sample_dirs():
            paths.extend(self.collect_images(root, max(0, self.max_samples - len(paths))))
            if len(paths) >= self.max_samples:
                break
        if not paths:
            for root in self.default_sample_roots():
                paths.extend(self.collect_images(root, max(0, self.max_samples - len(paths))))
                if len(paths) >= self.max_samples:
                    break
        paths = self.unique_paths(paths)
        paths.sort(key=self.path_mtime, reverse=True)
        self.set_sample_paths(paths, "Loaded default sample roots")

    def collect_images(self, path: Path, limit: int) -> List[Path]:
        if limit <= 0:
            return []
        if path.is_file():
            return [path] if path.suffix.lower() in IMAGE_EXTENSIONS else []
        if not path.exists() or not path.is_dir():
            return []
        output: List[Path] = []
        try:
            for candidate in path.rglob("*"):
                if candidate.is_file() and candidate.suffix.lower() in IMAGE_EXTENSIONS:
                    output.append(candidate.resolve())
                    if len(output) >= limit:
                        break
        except OSError as exc:
            self.get_logger().warn(f"Could not scan sample folder {path}: {exc}")
        return output

    def unique_paths(self, paths: Sequence[Path]) -> List[Path]:
        output: List[Path] = []
        seen = set()
        for path in paths:
            resolved = path.resolve()
            if resolved in seen:
                continue
            seen.add(resolved)
            output.append(resolved)
        return output

    def path_mtime(self, path: Path) -> float:
        try:
            return path.stat().st_mtime
        except OSError:
            return 0.0

    def color_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Camera image conversion failed: {exc}")
            return
        self.latest_camera_frame = frame.copy()
        self.latest_camera_stamp_monotonic = time.monotonic()

    def next_capture_path(self) -> Path:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        millis = int((time.time() % 1.0) * 1000.0)
        base = self.capture_dir / f"camera_{timestamp}_{millis:03d}.png"
        if not base.exists():
            return base
        for index in range(1, 1000):
            candidate = self.capture_dir / f"camera_{timestamp}_{millis:03d}_{index:03d}.png"
            if not candidate.exists():
                return candidate
        return self.capture_dir / f"camera_{timestamp}_{millis:03d}_{os.getpid()}.png"

    def save_camera_image(self) -> None:
        if self.latest_camera_frame is None:
            self.status = f"No camera image received yet on {self.color_topic}"
            return
        try:
            self.capture_dir.mkdir(parents=True, exist_ok=True)
            output_path = self.next_capture_path()
            if not cv2.imwrite(str(output_path), self.latest_camera_frame):
                self.status = f"Camera image save failed: {self.display_path(output_path)}"
                return
            age_sec = max(0.0, time.monotonic() - self.latest_camera_stamp_monotonic)
            self.status = f"Saved camera image: {output_path} ({age_sec:.1f}s old)"
            self.get_logger().info(self.status)
        except Exception as exc:
            self.status = f"Camera image save failed: {exc}"
            self.get_logger().warn(self.status)

    def set_sample_paths(self, paths: Sequence[Path], source_label: str) -> None:
        self.sample_paths = list(paths)[: max(1, self.max_samples)]
        if not self.sample_paths:
            self.sample_index = -1
            self.current_image = None
            self.current_image_path = None
            self.detections = []
            self.status = f"{source_label}: no images found"
            return
        self.sample_index = 0
        self.load_sample(0)
        self.status = f"{source_label}: {len(self.sample_paths)} image(s)"

    def load_samples(self, path: Path, recent_first: bool = False) -> None:
        paths = self.collect_images(path, self.max_samples)
        paths = self.unique_paths(paths)
        if recent_first:
            paths.sort(key=self.path_mtime, reverse=True)
        else:
            paths.sort(key=lambda p: str(p).lower())
        self.set_sample_paths(paths, f"Loaded samples from {self.display_path(path)}")

    def load_sample(self, index: int) -> None:
        if not self.sample_paths:
            return
        self.sample_index = int(np.clip(index, 0, len(self.sample_paths) - 1))
        path = self.sample_paths[self.sample_index]
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            self.current_image = None
            self.current_image_path = path
            self.detections = []
            self.status = f"Could not read image: {self.display_path(path)}"
            return
        self.current_image = image
        self.current_image_path = path
        self.run_current_sample()

    def next_sample(self, delta: int) -> None:
        if not self.sample_paths:
            self.status = "No sample images loaded"
            return
        self.load_sample((self.sample_index + delta) % len(self.sample_paths))

    def load_model(self, path: Path) -> bool:
        if not path.exists() or not path.is_file():
            self.status = f"Model missing: {self.display_path(path)}"
            return False
        suffix = path.suffix.lower()
        if suffix not in MODEL_EXTENSIONS:
            self.status = "Select a YOLO .onnx or .pt model"
            return False
        if suffix == ".pt":
            return self.load_pt_model(path)
        return self.load_onnx_model(path)

    def clear_loaded_model(self) -> None:
        self.ort_session = None
        self.ort_input_name = ""
        self.ort_output_names = []
        self.pt_model = None
        self.model_backend = ""
        self.model_output_summary = "Outputs: none"
        self.onnx_class_names = {}
        self.onnx_num_classes = 0
        self.onnx_task = ""
        self.onnx_has_nms = False
        self.last_decode_layout = ""
        self.model_path = None

    def model_loaded(self) -> bool:
        if self.model_backend == "onnx":
            return self.ort_session is not None
        if self.model_backend == "pt":
            return self.pt_model is not None
        return False

    def load_onnx_model(self, path: Path) -> bool:
        try:
            self.onnx_class_names = {}
            self.onnx_num_classes = 0
            self.onnx_task = ""
            self.onnx_has_nms = False
            self.last_decode_layout = ""
            self.last_num_classes = 1
            options = ort.SessionOptions()
            options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            options.log_severity_level = 3
            if self.ort_threads > 0:
                options.intra_op_num_threads = self.ort_threads
                options.inter_op_num_threads = 1
            self.ort_session = ort.InferenceSession(
                str(path),
                sess_options=options,
                providers=["CPUExecutionProvider"],
            )
            model_input = self.ort_session.get_inputs()[0]
            self.ort_input_name = model_input.name
            self.ort_output_names = [output.name for output in self.ort_session.get_outputs()]
            self.update_imgsz_from_input(model_input.shape)
            self.update_onnx_metadata()
            self.pt_model = None
            self.model_backend = "onnx"
            self.model_path = path
            self.model_output_summary = self.describe_model_outputs()
            self.status = f"Loaded ONNX: {path.name} | {self.model_output_summary}"
            self.run_current_sample()
            return True
        except Exception as exc:
            self.ort_session = None
            self.ort_input_name = ""
            self.ort_output_names = []
            self.pt_model = None
            self.model_backend = ""
            self.model_output_summary = "Outputs: none"
            self.onnx_class_names = {}
            self.onnx_num_classes = 0
            self.onnx_task = ""
            self.onnx_has_nms = False
            self.last_decode_layout = ""
            self.model_path = None
            self.status = f"ONNX load failed: {exc}"
            self.get_logger().warn(self.status)
            return False

    def load_pt_model(self, path: Path) -> bool:
        try:
            os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
            from ultralytics import YOLO

            model = YOLO(str(path))
            self.clear_loaded_model()
            self.pt_model = model
            self.model_backend = "pt"
            self.model_path = path
            self.model_output_summary = self.describe_pt_model(model)
            self.status = f"Loaded PT: {path.name} | {self.model_output_summary}"
            self.run_current_sample()
            return True
        except Exception as exc:
            self.clear_loaded_model()
            self.status = f"PT load failed: {exc}"
            self.get_logger().warn(self.status)
            return False

    def describe_pt_model(self, model) -> str:
        task = str(getattr(model, "task", "") or "").strip() or "unknown"
        names = getattr(model, "names", None)
        class_count = 0
        if isinstance(names, dict):
            class_count = len(names)
        elif isinstance(names, (list, tuple)):
            class_count = len(names)
        self.last_num_classes = max(1, class_count)
        class_text = f"{class_count} class(es)" if class_count else "classes unknown"
        return f"PT task={task}, {class_text}, device={self.pt_device}"

    def describe_model_outputs(self) -> str:
        if self.ort_session is None:
            return "Outputs: none"
        summaries = []
        for output in self.ort_session.get_outputs():
            summaries.append(f"{output.name}{self.format_shape(output.shape)}")
        extras = []
        if self.onnx_task:
            extras.append(f"task={self.onnx_task}")
        class_count = self.metadata_class_count()
        if class_count > 0:
            extras.append(f"{class_count} class(es)")
        if self.onnx_has_nms:
            extras.append("NMS")
        suffix = " | " + ", ".join(extras) if extras else ""
        return "Outputs: " + ", ".join(summaries[:4]) + suffix

    def format_shape(self, shape) -> str:
        dims = []
        for dim in shape:
            if isinstance(dim, (int, np.integer)) and int(dim) > 0:
                dims.append(str(int(dim)))
            elif dim is None:
                dims.append("?")
            else:
                token = str(dim).strip()
                dims.append(token if token else "?")
        return "[" + "x".join(dims) + "]"

    def update_imgsz_from_input(self, shape) -> None:
        try:
            dims = [int(dim) if isinstance(dim, (int, np.integer)) else -1 for dim in shape]
            if len(dims) >= 4 and dims[2] > 0 and dims[2] == dims[3]:
                self.yolo_imgsz = dims[2]
        except Exception:
            pass

    def update_onnx_metadata(self) -> None:
        self.onnx_class_names = {}
        self.onnx_num_classes = 0
        self.onnx_task = ""
        self.onnx_has_nms = False
        if self.ort_session is None:
            return
        metadata = {}
        try:
            metadata = dict(self.ort_session.get_modelmeta().custom_metadata_map or {})
        except Exception:
            metadata = {}

        self.onnx_task = str(metadata.get("task", "")).strip()
        for key in ("names", "classes", "class_names"):
            parsed_names = self.parse_class_names(metadata.get(key, ""))
            if parsed_names:
                self.onnx_class_names = parsed_names
                break
        if self.onnx_class_names:
            self.onnx_num_classes = max(1, len(self.onnx_class_names))
            self.last_num_classes = max(1, max(self.onnx_class_names.keys()) + 1)
        else:
            try:
                nc = int(metadata.get("nc", metadata.get("num_classes", 0)))
                if nc > 0:
                    self.onnx_num_classes = nc
                    self.last_num_classes = nc
            except Exception:
                pass

        args_value = metadata.get("args", "")
        try:
            args = ast.literal_eval(args_value) if args_value else {}
            self.onnx_has_nms = (
                bool(args.get("nms", False))
                or bool(metadata.get("end2end", "") == "True")
            )
        except Exception:
            lowered = str(args_value).lower()
            self.onnx_has_nms = "nms': true" in lowered or '"nms": true' in lowered

    @staticmethod
    def parse_class_names(value: object) -> Dict[int, str]:
        if value is None:
            return {}
        if isinstance(value, dict):
            output: Dict[int, str] = {}
            for key, name in value.items():
                try:
                    output[int(key)] = str(name)
                except Exception:
                    continue
            return output
        if isinstance(value, (list, tuple)):
            return {index: str(name) for index, name in enumerate(value)}
        text = str(value).strip()
        if not text:
            return {}
        try:
            parsed = ast.literal_eval(text)
            return ItemDetectYoloDebugNode.parse_class_names(parsed)
        except Exception:
            pass
        if "," in text:
            names = [part.strip().strip("'\"") for part in text.split(",") if part.strip()]
            return {index: name for index, name in enumerate(names)}
        return {0: text} if text and ":" not in text and "{" not in text else {}

    def run_current_sample(self) -> None:
        self.detections = []
        self.last_inference_ms = 0.0
        self.last_decode_layout = ""
        if self.current_image is None:
            self.status = "No sample image loaded"
            return
        if not self.model_loaded():
            self.status = "No YOLO model loaded"
            return
        try:
            start = time.monotonic()
            self.detections = self.run_yolo(self.current_image)
            self.last_inference_ms = (time.monotonic() - start) * 1000.0
            if self.detections:
                best = max(self.detections, key=lambda detection: detection.score)
                layout = f" | {self.last_decode_layout}" if self.last_decode_layout else ""
                self.status = (
                    f"{len(self.detections)} detection(s), best {best.score * 100.0:.0f}% "
                    f"in {self.last_inference_ms:.1f} ms{layout}"
                )
            else:
                layout = f" | {self.last_decode_layout}" if self.last_decode_layout else ""
                self.status = f"No detections above {self.yolo_conf * 100.0:.0f}%{layout}"
        except Exception as exc:
            self.detections = []
            self.status = f"Inference failed: {exc}"
            self.get_logger().warn(self.status)

    def letterbox(self, image: np.ndarray) -> Tuple[np.ndarray, float, int, int, int, int]:
        height, width = image.shape[:2]
        scale = min(self.yolo_imgsz / float(width), self.yolo_imgsz / float(height))
        new_w = max(1, int(round(width * scale)))
        new_h = max(1, int(round(height * scale)))
        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((self.yolo_imgsz, self.yolo_imgsz, 3), 114, dtype=np.uint8)
        pad_x = (self.yolo_imgsz - new_w) // 2
        pad_y = (self.yolo_imgsz - new_h) // 2
        canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
        return canvas, scale, pad_x, pad_y, new_w, new_h

    def run_yolo(self, image: np.ndarray) -> List[Detection]:
        if self.model_backend == "pt":
            return self.run_pt_yolo(image)
        return self.run_onnx_yolo(image)

    def run_onnx_yolo(self, image: np.ndarray) -> List[Detection]:
        if self.ort_session is None:
            return []
        letterboxed, scale, pad_x, pad_y, new_w, new_h = self.letterbox(image)
        rgb = cv2.cvtColor(letterboxed, cv2.COLOR_BGR2RGB)
        tensor = rgb.astype(np.float32) / 255.0
        tensor = np.transpose(tensor, (2, 0, 1))[None, :, :, :]
        outputs = self.ort_session.run(self.ort_output_names, {self.ort_input_name: tensor})
        if not outputs:
            raise RuntimeError("Model returned no outputs")

        split_detections = self.postprocess_split_outputs(
            outputs,
            image.shape[:2],
            scale,
            pad_x,
            pad_y,
        )
        if split_detections is not None:
            return split_detections

        pred = np.asarray(outputs[0])
        proto = np.asarray(outputs[1]) if len(outputs) >= 2 else None
        if proto is not None and pred.ndim == 4 and proto.ndim in (2, 3):
            pred, proto = proto, pred
        pred = self.prepare_prediction(pred)
        if pred is None or pred.shape[1] <= 4:
            raise self.unsupported_onnx_layout_error()
        proto = self.prepare_proto(proto) if proto is not None else None
        if proto is not None:
            return self.postprocess_segmentation(
                pred,
                proto,
                image.shape[:2],
                scale,
                pad_x,
                pad_y,
                new_w,
                new_h,
            )
        if self.is_postprocessed_output(pred):
            return self.postprocess_nms_boxes(pred, image.shape[:2], scale, pad_x, pad_y)
        return self.postprocess_boxes(pred, image.shape[:2], scale, pad_x, pad_y)

    def unsupported_onnx_layout_error(self) -> RuntimeError:
        return RuntimeError(
            "Unsupported ONNX output layout for YOLO detection. "
            f"Expected YOLO detect/segment outputs; {self.model_output_summary}"
        )

    def run_pt_yolo(self, image: np.ndarray) -> List[Detection]:
        if self.pt_model is None:
            return []
        os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
        classes = None if self.class_id < 0 else [int(self.class_id)]
        results = self.pt_model.predict(
            source=image,
            imgsz=self.yolo_imgsz,
            conf=self.yolo_conf,
            iou=self.yolo_iou,
            classes=classes,
            device=self.pt_device,
            verbose=False,
        )
        if not results:
            return []
        return self.pt_result_to_detections(results[0], image.shape[:2])

    def pt_result_to_detections(self, result, image_shape: Tuple[int, int]) -> List[Detection]:
        boxes_obj = getattr(result, "boxes", None)
        if boxes_obj is None or len(boxes_obj) == 0:
            return []
        boxes_xyxy = boxes_obj.xyxy.detach().cpu().numpy()
        scores = boxes_obj.conf.detach().cpu().numpy()
        class_ids = boxes_obj.cls.detach().cpu().numpy().astype(np.int32)
        masks_obj = getattr(result, "masks", None)
        mask_polygons = getattr(masks_obj, "xy", None) if masks_obj is not None else None
        mask_data = None
        if masks_obj is not None and getattr(masks_obj, "data", None) is not None:
            mask_data = masks_obj.data.detach().cpu().numpy()

        names = getattr(result, "names", None)
        if isinstance(names, dict):
            self.last_num_classes = max(1, len(names))

        detections: List[Detection] = []
        for index, box_xyxy in enumerate(boxes_xyxy):
            x1, y1, x2, y2 = self.clip_xyxy_box(box_xyxy, image_shape)
            if x2 <= x1 or y2 <= y1:
                continue
            box = (x1, y1, x2 - x1, y2 - y1)
            mask = self.pt_mask_for_detection(index, mask_polygons, mask_data, image_shape, (x1, y1, x2, y2))
            center = self.mask_center(mask, box)
            detections.append(Detection(
                score=float(scores[index]),
                class_id=int(class_ids[index]),
                box=box,
                mask=mask,
                center=center,
            ))
        detections.sort(key=lambda detection: detection.score, reverse=True)
        return detections

    def clip_xyxy_box(self, box_xyxy: np.ndarray, image_shape: Tuple[int, int]) -> Tuple[int, int, int, int]:
        height, width = image_shape
        x1, y1, x2, y2 = [float(value) for value in box_xyxy[:4]]
        x1 = int(np.clip(round(x1), 0, width - 1))
        y1 = int(np.clip(round(y1), 0, height - 1))
        x2 = int(np.clip(round(x2), 0, width - 1))
        y2 = int(np.clip(round(y2), 0, height - 1))
        return x1, y1, x2, y2

    def pt_mask_for_detection(
        self,
        index: int,
        mask_polygons,
        mask_data,
        image_shape: Tuple[int, int],
        box_xyxy: Tuple[int, int, int, int],
    ) -> np.ndarray:
        mask = np.zeros(image_shape, dtype=np.uint8)
        if mask_polygons is not None and index < len(mask_polygons):
            polygon = np.asarray(mask_polygons[index], dtype=np.float32)
            if polygon.ndim == 2 and len(polygon) >= 3:
                pts = np.round(polygon).astype(np.int32)
                pts[:, 0] = np.clip(pts[:, 0], 0, image_shape[1] - 1)
                pts[:, 1] = np.clip(pts[:, 1], 0, image_shape[0] - 1)
                cv2.fillPoly(mask, [pts], 255)
                if cv2.countNonZero(mask) >= 16:
                    return mask

        if mask_data is not None and index < len(mask_data):
            raw_mask = mask_data[index].astype(np.float32)
            if raw_mask.shape[:2] != image_shape:
                raw_mask = cv2.resize(
                    raw_mask,
                    (image_shape[1], image_shape[0]),
                    interpolation=cv2.INTER_LINEAR,
                )
            binary = (raw_mask >= self.mask_threshold).astype(np.uint8) * 255
            if cv2.countNonZero(binary) >= 16:
                return binary

        x1, y1, x2, y2 = box_xyxy
        mask[y1:y2, x1:x2] = 255
        return mask

    def prepare_prediction(self, pred: np.ndarray) -> Optional[np.ndarray]:
        if pred.ndim == 3:
            pred = pred[0]
        if pred.ndim != 2:
            return None
        if pred.shape[0] < pred.shape[1] and pred.shape[0] <= 512:
            pred = pred.T
        return pred.astype(np.float32, copy=False)

    def prepare_proto(self, proto: np.ndarray) -> Optional[np.ndarray]:
        proto = np.asarray(proto)
        while proto.ndim > 3 and proto.shape[0] == 1:
            proto = proto[0]
        if proto.ndim != 3:
            return None
        dims = proto.shape
        first_is_channels = dims[0] <= dims[1] and dims[0] <= dims[2]
        last_is_channels = dims[2] <= dims[0] and dims[2] <= dims[1]
        if last_is_channels and not first_is_channels:
            proto = np.transpose(proto, (2, 0, 1))
        return proto.astype(np.float32, copy=False)

    def metadata_class_count(self) -> int:
        return max(0, int(self.onnx_num_classes))

    def raw_layout_candidates(
        self,
        total_cols: int,
        mask_dim: int,
        row_count: int,
    ) -> List[Tuple[str, bool, int]]:
        candidates: List[Tuple[str, bool, int]] = []
        seen = set()

        def add(label: str, has_objectness: bool, class_count: int) -> None:
            if class_count <= 0:
                return
            expected = 4 + (1 if has_objectness else 0) + class_count + mask_dim
            key = (has_objectness, class_count)
            if expected == total_cols and key not in seen:
                seen.add(key)
                candidates.append((label, has_objectness, class_count))

        metadata_nc = self.metadata_class_count()
        if metadata_nc > 0:
            add("raw-no-objectness", False, metadata_nc)
            add("raw-objectness", True, metadata_nc)
            if candidates:
                return candidates

        no_objectness_nc = total_cols - 4 - mask_dim
        objectness_nc = total_cols - 5 - mask_dim
        common_counts = {1, 2, 3, 4, 5, 10, 20, 80}

        if mask_dim > 0:
            if objectness_nc in common_counts:
                add("raw-objectness", True, objectness_nc)
                add("raw-no-objectness", False, no_objectness_nc)
            else:
                add("raw-no-objectness", False, no_objectness_nc)
                add("raw-objectness", True, objectness_nc)
            return candidates

        if total_cols == 5:
            add("raw-no-objectness", False, 1)
            return candidates
        if total_cols == 6 and row_count > 1000:
            add("raw-objectness", True, 1)
            add("raw-no-objectness", False, 2)
            return candidates
        if objectness_nc in common_counts and total_cols != 84:
            add("raw-objectness", True, objectness_nc)
        if no_objectness_nc in common_counts:
            add("raw-no-objectness", False, no_objectness_nc)
        return candidates

    def probability_values(self, values: np.ndarray) -> np.ndarray:
        values = values.astype(np.float32, copy=False)
        finite = values[np.isfinite(values)]
        if finite.size and (float(np.min(finite)) < -1e-3 or float(np.max(finite)) > 1.001):
            values = sigmoid(values)
        return np.clip(values, 0.0, 1.0)

    def split_raw_scores(
        self,
        pred: np.ndarray,
        class_count: int,
        has_objectness: bool,
    ) -> Tuple[np.ndarray, np.ndarray, int]:
        if has_objectness:
            objectness = self.probability_values(pred[:, 4])
            scores_all = self.probability_values(pred[:, 5:5 + class_count])
            coeff_start = 5 + class_count
        else:
            objectness = None
            scores_all = self.probability_values(pred[:, 4:4 + class_count])
            coeff_start = 4 + class_count
        if scores_all.size == 0:
            return (
                np.zeros((pred.shape[0],), dtype=np.int32),
                np.zeros((pred.shape[0],), dtype=np.float32),
                coeff_start,
            )
        class_ids = np.argmax(scores_all, axis=1).astype(np.int32)
        scores = scores_all[np.arange(scores_all.shape[0]), class_ids]
        if objectness is not None:
            scores = scores * objectness
        return class_ids, scores.astype(np.float32, copy=False), coeff_start

    def choose_layout_result(
        self,
        candidates: List[Tuple[str, int, List[Detection]]],
    ) -> List[Detection]:
        if not candidates:
            self.last_decode_layout = ""
            return []

        def score_candidate(item) -> Tuple[int, int, float]:
            index, (_, _, detections) = item
            best_score = max((detection.score for detection in detections), default=0.0)
            return len(detections), -index, best_score

        _, best = max(enumerate(candidates), key=score_candidate)
        layout_name, class_count, detections = best
        self.last_num_classes = max(1, class_count)
        self.last_decode_layout = f"{layout_name}, {class_count} class(es)"
        return detections

    def postprocess_split_outputs(
        self,
        outputs: Sequence[np.ndarray],
        image_shape: Tuple[int, int],
        scale: float,
        pad_x: int,
        pad_y: int,
    ) -> Optional[List[Detection]]:
        arrays = []
        for output in outputs:
            arr = np.asarray(output)
            while arr.ndim > 2 and arr.shape[0] == 1:
                arr = arr[0]
            arrays.append(arr)

        for boxes in arrays:
            if boxes.ndim != 2 or boxes.shape[1] != 4:
                continue
            row_count = boxes.shape[0]
            score_array = None
            class_array = None
            for arr in arrays:
                if arr is boxes:
                    continue
                squeezed = np.squeeze(arr)
                if squeezed.ndim == 1 and squeezed.shape[0] == row_count:
                    if score_array is None:
                        score_array = squeezed.astype(np.float32)
                    elif class_array is None:
                        class_array = squeezed
                elif squeezed.ndim == 2 and squeezed.shape[0] == row_count:
                    if squeezed.shape[1] == 1 and score_array is None:
                        score_array = squeezed[:, 0].astype(np.float32)
                    elif squeezed.shape[1] > 1 and score_array is None:
                        score_array = np.max(squeezed.astype(np.float32), axis=1)
                        class_array = np.argmax(squeezed.astype(np.float32), axis=1)
            if score_array is None:
                continue
            if class_array is None:
                class_array = np.zeros((row_count,), dtype=np.int32)
            detections = self.detections_from_xyxy(
                boxes.astype(np.float32),
                self.probability_values(score_array),
                class_array.astype(np.int32),
                image_shape,
                scale,
                pad_x,
                pad_y,
                layout_name="split-nms",
            )
            return detections
        return None

    def is_postprocessed_output(self, pred: np.ndarray) -> bool:
        if pred.ndim != 2 or pred.shape[1] not in (6, 7):
            return False
        if self.onnx_has_nms or pred.shape[0] <= 1000:
            return True
        score_col = pred[:, 4] if pred.shape[1] == 6 else pred[:, 5]
        class_col = pred[:, 5] if pred.shape[1] == 6 else pred[:, 6]
        finite_scores = score_col[np.isfinite(score_col)]
        if not finite_scores.size:
            return False
        scores_look_ready = (
            float(np.min(finite_scores)) >= -1e-3
            and float(np.max(finite_scores)) <= 1.001
        )
        classes_look_ready = np.all(np.isclose(class_col, np.round(class_col), atol=1e-3))
        return bool(scores_look_ready and classes_look_ready and pred.shape[0] <= 3000)

    def postprocess_nms_boxes(
        self,
        pred: np.ndarray,
        image_shape: Tuple[int, int],
        scale: float,
        pad_x: int,
        pad_y: int,
    ) -> List[Detection]:
        if pred.shape[1] == 7 and np.all(np.isclose(pred[:, 0], np.round(pred[:, 0]), atol=1e-3)):
            boxes_xyxy = pred[:, 1:5]
            scores = pred[:, 5]
            class_ids = pred[:, 6].astype(np.int32)
        else:
            boxes_xyxy = pred[:, 0:4]
            scores = pred[:, 4]
            class_ids = pred[:, 5].astype(np.int32)
        return self.detections_from_xyxy(
            boxes_xyxy.astype(np.float32),
            self.probability_values(scores.astype(np.float32)),
            class_ids,
            image_shape,
            scale,
            pad_x,
            pad_y,
            layout_name="post-nms",
        )

    def postprocess_segmentation(
        self,
        pred: np.ndarray,
        proto: np.ndarray,
        image_shape: Tuple[int, int],
        scale: float,
        pad_x: int,
        pad_y: int,
        new_w: int,
        new_h: int,
    ) -> List[Detection]:
        mask_dim = int(proto.shape[0])
        candidates: List[Tuple[str, int, List[Detection]]] = []
        layouts = self.raw_layout_candidates(
            pred.shape[1],
            mask_dim,
            pred.shape[0],
        )
        if not layouts:
            raise self.unsupported_onnx_layout_error()
        for layout_name, has_objectness, nc in layouts:
            detections = self.postprocess_segmentation_layout(
                pred,
                proto,
                nc,
                has_objectness,
                image_shape,
                scale,
                pad_x,
                pad_y,
                new_w,
                new_h,
            )
            candidates.append((layout_name, nc, detections))
        return self.choose_layout_result(candidates)

    def postprocess_segmentation_layout(
        self,
        pred: np.ndarray,
        proto: np.ndarray,
        nc: int,
        has_objectness: bool,
        image_shape: Tuple[int, int],
        scale: float,
        pad_x: int,
        pad_y: int,
        new_w: int,
        new_h: int,
    ) -> List[Detection]:
        boxes_xywh = pred[:, :4]
        class_ids, scores, coeff_start = self.split_raw_scores(pred, nc, has_objectness)
        mask_dim = int(proto.shape[0])
        coeffs_all = pred[:, coeff_start:coeff_start + mask_dim]
        if coeffs_all.shape[1] != mask_dim:
            return []
        boxes, rows = self.candidate_boxes(
            boxes_xywh,
            scores,
            class_ids,
            image_shape,
            scale,
            pad_x,
            pad_y,
        )
        if not boxes:
            return []
        keep = cv2.dnn.NMSBoxes(
            boxes,
            [float(scores[row]) for row in rows],
            self.yolo_conf,
            self.yolo_iou,
        )
        if len(keep) == 0:
            return []
        detections: List[Detection] = []
        for keep_idx in np.asarray(keep).reshape(-1):
            row = rows[int(keep_idx)]
            box = boxes[int(keep_idx)]
            mask = self.decode_mask(
                coeffs_all[row],
                proto,
                image_shape,
                box,
                pad_x,
                pad_y,
                new_w,
                new_h,
            )
            if mask is None:
                continue
            center = self.mask_center(mask, box)
            detections.append(Detection(
                score=float(scores[row]),
                class_id=int(class_ids[row]),
                box=tuple(int(v) for v in box),
                mask=mask,
                center=center,
            ))
        detections.sort(key=lambda detection: detection.score, reverse=True)
        return detections

    def postprocess_boxes(
        self,
        pred: np.ndarray,
        image_shape: Tuple[int, int],
        scale: float,
        pad_x: int,
        pad_y: int,
    ) -> List[Detection]:
        candidates: List[Tuple[str, int, List[Detection]]] = []
        layouts = self.raw_layout_candidates(
            pred.shape[1],
            0,
            pred.shape[0],
        )
        if not layouts:
            raise self.unsupported_onnx_layout_error()
        for layout_name, has_objectness, nc in layouts:
            detections = self.postprocess_boxes_layout(
                pred,
                nc,
                has_objectness,
                image_shape,
                scale,
                pad_x,
                pad_y,
            )
            candidates.append((layout_name, nc, detections))
        return self.choose_layout_result(candidates)

    def postprocess_boxes_layout(
        self,
        pred: np.ndarray,
        nc: int,
        has_objectness: bool,
        image_shape: Tuple[int, int],
        scale: float,
        pad_x: int,
        pad_y: int,
    ) -> List[Detection]:
        boxes_xywh = pred[:, :4]
        class_ids, scores, _ = self.split_raw_scores(pred, nc, has_objectness)
        boxes, rows = self.candidate_boxes(
            boxes_xywh,
            scores,
            class_ids,
            image_shape,
            scale,
            pad_x,
            pad_y,
        )
        if not boxes:
            return []
        keep = cv2.dnn.NMSBoxes(
            boxes,
            [float(scores[row]) for row in rows],
            self.yolo_conf,
            self.yolo_iou,
        )
        if len(keep) == 0:
            return []
        detections: List[Detection] = []
        for keep_idx in np.asarray(keep).reshape(-1):
            row = rows[int(keep_idx)]
            x, y, w, h = boxes[int(keep_idx)]
            mask = np.zeros(image_shape, dtype=np.uint8)
            mask[y:y + h, x:x + w] = 255
            detections.append(Detection(
                score=float(scores[row]),
                class_id=int(class_ids[row]),
                box=(x, y, w, h),
                mask=mask,
                center=(float(x + w * 0.5), float(y + h * 0.5)),
            ))
        detections.sort(key=lambda detection: detection.score, reverse=True)
        return detections

    def candidate_boxes(
        self,
        boxes_xywh: np.ndarray,
        scores: np.ndarray,
        class_ids: np.ndarray,
        image_shape: Tuple[int, int],
        scale: float,
        pad_x: int,
        pad_y: int,
    ) -> Tuple[List[List[int]], List[int]]:
        height, width = image_shape
        keep = np.where(scores >= self.yolo_conf)[0]
        normalized = self.boxes_xywh_are_normalized(boxes_xywh[keep]) if keep.size else False
        boxes: List[List[int]] = []
        rows: List[int] = []
        for idx in keep:
            class_id = int(class_ids[idx])
            if self.class_id >= 0 and class_id != self.class_id:
                continue
            cx, cy, bw, bh = boxes_xywh[idx]
            if normalized:
                cx *= float(self.yolo_imgsz)
                bw *= float(self.yolo_imgsz)
                cy *= float(self.yolo_imgsz)
                bh *= float(self.yolo_imgsz)
            x1 = (float(cx - bw / 2.0) - pad_x) / scale
            y1 = (float(cy - bh / 2.0) - pad_y) / scale
            x2 = (float(cx + bw / 2.0) - pad_x) / scale
            y2 = (float(cy + bh / 2.0) - pad_y) / scale
            x1 = int(np.clip(round(x1), 0, width - 1))
            y1 = int(np.clip(round(y1), 0, height - 1))
            x2 = int(np.clip(round(x2), 0, width - 1))
            y2 = int(np.clip(round(y2), 0, height - 1))
            if x2 <= x1 or y2 <= y1:
                continue
            boxes.append([x1, y1, x2 - x1, y2 - y1])
            rows.append(int(idx))
        return boxes, rows

    def boxes_xywh_are_normalized(self, boxes_xywh: np.ndarray) -> bool:
        if boxes_xywh.size == 0:
            return False
        finite = boxes_xywh[np.isfinite(boxes_xywh)]
        if finite.size == 0:
            return False
        return float(np.max(np.abs(finite))) <= 2.0

    def boxes_xyxy_are_normalized(self, boxes_xyxy: np.ndarray) -> bool:
        if boxes_xyxy.size == 0:
            return False
        finite = boxes_xyxy[np.isfinite(boxes_xyxy)]
        if finite.size == 0:
            return False
        return float(np.max(np.abs(finite))) <= 2.0

    def detections_from_xyxy(
        self,
        boxes_xyxy: np.ndarray,
        scores: np.ndarray,
        class_ids: np.ndarray,
        image_shape: Tuple[int, int],
        scale: float,
        pad_x: int,
        pad_y: int,
        layout_name: str,
    ) -> List[Detection]:
        height, width = image_shape
        normalized = self.boxes_xyxy_are_normalized(boxes_xyxy)
        detections: List[Detection] = []
        for index, box_xyxy in enumerate(boxes_xyxy):
            score = float(scores[index]) if index < len(scores) else 0.0
            if score < self.yolo_conf:
                continue
            class_id = int(round(float(class_ids[index]))) if index < len(class_ids) else 0
            if self.class_id >= 0 and class_id != self.class_id:
                continue
            x1, y1, x2, y2 = [float(value) for value in box_xyxy[:4]]
            if normalized:
                x1 *= float(self.yolo_imgsz)
                x2 *= float(self.yolo_imgsz)
                y1 *= float(self.yolo_imgsz)
                y2 *= float(self.yolo_imgsz)
            x1 = (x1 - pad_x) / scale
            y1 = (y1 - pad_y) / scale
            x2 = (x2 - pad_x) / scale
            y2 = (y2 - pad_y) / scale
            x1 = int(np.clip(round(x1), 0, width - 1))
            y1 = int(np.clip(round(y1), 0, height - 1))
            x2 = int(np.clip(round(x2), 0, width - 1))
            y2 = int(np.clip(round(y2), 0, height - 1))
            if x2 <= x1 or y2 <= y1:
                continue
            mask = np.zeros(image_shape, dtype=np.uint8)
            mask[y1:y2, x1:x2] = 255
            detections.append(Detection(
                score=score,
                class_id=class_id,
                box=(x1, y1, x2 - x1, y2 - y1),
                mask=mask,
                center=(float(x1 + (x2 - x1) * 0.5), float(y1 + (y2 - y1) * 0.5)),
            ))
        detections.sort(key=lambda detection: detection.score, reverse=True)
        if detections:
            inferred_classes = max([detection.class_id for detection in detections] + [0]) + 1
            self.last_num_classes = max(self.last_num_classes, inferred_classes)
        if self.onnx_class_names:
            self.last_num_classes = max(
                self.last_num_classes,
                max(self.onnx_class_names.keys()) + 1,
            )
        self.last_decode_layout = f"{layout_name}, {self.last_num_classes} class(es)"
        return detections

    def decode_mask(
        self,
        coeff: np.ndarray,
        proto: np.ndarray,
        image_shape: Tuple[int, int],
        box: Sequence[int],
        pad_x: int,
        pad_y: int,
        new_w: int,
        new_h: int,
    ) -> Optional[np.ndarray]:
        mask = sigmoid(np.matmul(coeff.astype(np.float32), proto.reshape(proto.shape[0], -1)))
        mask = mask.reshape(proto.shape[1], proto.shape[2])
        mask = cv2.resize(mask, (self.yolo_imgsz, self.yolo_imgsz), interpolation=cv2.INTER_LINEAR)
        mask = mask[pad_y:pad_y + new_h, pad_x:pad_x + new_w]
        mask = cv2.resize(mask, (image_shape[1], image_shape[0]), interpolation=cv2.INTER_LINEAR)
        binary = (mask >= self.mask_threshold).astype(np.uint8) * 255
        limited = np.zeros_like(binary)
        x, y, w, h = [int(v) for v in box]
        limited[y:y + h, x:x + w] = binary[y:y + h, x:x + w]
        if cv2.countNonZero(limited) < 16:
            return None
        return limited

    def mask_center(self, mask: np.ndarray, box: Sequence[int]) -> Tuple[float, float]:
        moments = cv2.moments(mask)
        if abs(moments["m00"]) > 1e-6:
            return float(moments["m10"] / moments["m00"]), float(moments["m01"] / moments["m00"])
        x, y, w, h = [float(v) for v in box]
        return x + w * 0.5, y + h * 0.5

    def render_once(self) -> None:
        if not self.window_ready:
            return
        canvas = self.build_ui()
        cv2.imshow(WINDOW_NAME, canvas)
        key = cv2.waitKey(1) & 0xFF
        self.handle_key(key)

    def build_ui(self) -> np.ndarray:
        canvas_h = TOP_BAR_HEIGHT + PREVIEW_CANVAS_HEIGHT
        canvas = np.zeros((canvas_h, PREVIEW_CANVAS_WIDTH, 3), dtype=np.uint8)
        canvas[:] = (28, 30, 34)
        self.buttons.clear()
        self.slider_rects.clear()
        self.draw_top_controls(canvas)
        preview = self.render_preview()
        view, scale, origin = self.fit_preview(preview)
        self.preview_scale = scale
        self.preview_origin = origin
        y0 = TOP_BAR_HEIGHT
        canvas[y0:y0 + PREVIEW_CANVAS_HEIGHT, :] = (8, 10, 12)
        x, y = origin
        canvas[y:y + view.shape[0], x:x + view.shape[1]] = view
        return canvas

    def render_preview(self) -> np.ndarray:
        if self.current_image is None:
            image = np.zeros((PREVIEW_CANVAS_HEIGHT, PREVIEW_CANVAS_WIDTH, 3), dtype=np.uint8)
            image[:] = (14, 15, 17)
            cv2.putText(image, "No sample image", (28, 48),
                        cv2.FONT_HERSHEY_DUPLEX, 0.85, (220, 225, 230), 1, cv2.LINE_AA)
            return image

        output = self.current_image.copy()
        for index, detection in enumerate(self.detections):
            mask = detection.mask
            if mask.shape[:2] != output.shape[:2]:
                mask = cv2.resize(mask, (output.shape[1], output.shape[0]), interpolation=cv2.INTER_NEAREST)
            color = (80, 210, 255) if index == 0 else (90, 165, 245)
            overlay = output.copy()
            overlay[mask > 0] = color
            output = cv2.addWeighted(overlay, 0.32, output, 0.68, 0.0)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(output, contours, -1, color, 2, cv2.LINE_AA)
            x, y, w, h = detection.box
            cv2.rectangle(output, (x, y), (x + w, y + h), color, 2)
            label = f"c{detection.class_id} {detection.score * 100.0:.0f}%"
            cv2.putText(output, label, (x + 4, max(18, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.56, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(output, label, (x + 4, max(18, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.56, (245, 248, 250), 1, cv2.LINE_AA)
            cx, cy = detection.center
            cv2.circle(output, (int(round(cx)), int(round(cy))), 5, (255, 255, 255), -1, cv2.LINE_AA)
            cv2.circle(output, (int(round(cx)), int(round(cy))), 7, color, 2, cv2.LINE_AA)
        return output

    def fit_preview(self, image: np.ndarray) -> Tuple[np.ndarray, float, Tuple[int, int]]:
        h, w = image.shape[:2]
        scale = min(PREVIEW_CANVAS_WIDTH / float(w), PREVIEW_CANVAS_HEIGHT / float(h))
        new_w = max(1, int(round(w * scale)))
        new_h = max(1, int(round(h * scale)))
        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        x = (PREVIEW_CANVAS_WIDTH - new_w) // 2
        y = TOP_BAR_HEIGHT + (PREVIEW_CANVAS_HEIGHT - new_h) // 2
        return resized, scale, (x, y)

    def draw_top_controls(self, canvas: np.ndarray) -> None:
        width = canvas.shape[1]
        cv2.rectangle(canvas, (0, 0), (width, TOP_BAR_HEIGHT), (28, 30, 34), -1)
        cv2.line(canvas, (0, 58), (width, 58), (54, 58, 64), 1)

        margin = 16
        gap = 10
        y = 12
        button_count = 7
        button_w = max(112, min(158, (width - 2 * margin - (button_count - 1) * gap) // button_count))
        x = margin
        buttons = [
            ("open_model", "Open Model", True, False, (58, 78, 96), (130, 166, 198)),
            ("open_image", "Open Image", True, False, (58, 84, 72), (132, 204, 160)),
            ("open_folder", "Open Folder", True, False, (58, 84, 72), (132, 204, 160)),
            ("prev", "Prev", bool(self.sample_paths), False, (58, 64, 72), (116, 124, 134)),
            ("next", "Next", bool(self.sample_paths), False, (58, 64, 72), (116, 124, 134)),
            (
                "run",
                "Run",
                self.model_loaded() and self.current_image is not None,
                False,
                (70, 126, 86),
                (142, 228, 160),
            ),
            (
                "class",
                self.class_button_label(),
                True,
                self.class_id < 0,
                (72, 62, 82),
                (164, 142, 206),
            ),
        ]
        for name, label, enabled, active, fill, border in buttons:
            self.draw_button(
                canvas,
                name,
                (x, y, button_w, BUTTON_HEIGHT),
                label,
                enabled,
                active,
                fill,
                border,
            )
            x += button_w + gap

        panel_y = 68
        line_color = (202, 208, 214)
        model_text = (
            f"Model: {self.display_path(self.model_path)} | {self.model_output_summary}"
            if self.model_path else
            "Model: none"
        )
        image_text = (
            f"Image {self.sample_index + 1}/{len(self.sample_paths)}: "
            f"{self.display_path(self.current_image_path)}"
            if self.current_image_path is not None and self.sample_paths else
            "Image: none"
        )
        result_text = (
            f"Detections: {len(self.detections)} | Conf {self.yolo_conf * 100.0:.0f}% | "
            f"Class {'All' if self.class_id < 0 else self.class_id} | {self.last_inference_ms:.1f} ms"
        )
        cv2.putText(canvas, fit_text(model_text, 150), (margin, panel_y),
                    cv2.FONT_HERSHEY_DUPLEX, 0.48, line_color, 1, cv2.LINE_AA)
        cv2.putText(canvas, fit_text(image_text, 150), (margin, panel_y + 26),
                    cv2.FONT_HERSHEY_DUPLEX, 0.48, line_color, 1, cv2.LINE_AA)
        cv2.putText(canvas, fit_text(result_text, 90), (margin, panel_y + 52),
                    cv2.FONT_HERSHEY_DUPLEX, 0.46, (168, 228, 184), 1, cv2.LINE_AA)

        slider_rect = (margin + 470, panel_y + 35, width - margin * 2 - 470, 32)
        self.draw_slider(
            canvas,
            "confidence",
            slider_rect,
            f"Confidence {self.yolo_conf * 100.0:.0f}%",
            self.yolo_conf,
            (90, 220, 250),
        )

        status_y = TOP_BAR_HEIGHT - 31
        cv2.rectangle(canvas, (margin, status_y), (width - margin, status_y + 23), (36, 39, 44), -1)
        cv2.rectangle(canvas, (margin, status_y), (width - margin, status_y + 23), (72, 78, 86), 1)
        cv2.putText(canvas, "Status", (margin + 10, status_y + 17),
                    cv2.FONT_HERSHEY_DUPLEX, 0.43, (216, 220, 226), 1, cv2.LINE_AA)
        cv2.putText(canvas, fit_text(self.status, 126), (margin + 76, status_y + 17),
                    cv2.FONT_HERSHEY_DUPLEX, 0.43, (196, 202, 210), 1, cv2.LINE_AA)

    def class_button_label(self) -> str:
        return "Class: All" if self.class_id < 0 else f"Class: {self.class_id}"

    def draw_button(
        self,
        canvas: np.ndarray,
        name: str,
        rect: Tuple[int, int, int, int],
        label: str,
        enabled: bool,
        active: bool,
        fill_color: Tuple[int, int, int],
        border_color: Tuple[int, int, int],
    ) -> None:
        x, y, w, h = rect
        fill = fill_color if enabled else (54, 54, 54)
        if active and enabled:
            fill = tuple(min(255, int(channel * 1.18) + 12) for channel in fill)
        border = border_color if enabled else (96, 96, 96)
        text = (238, 242, 245) if enabled else (150, 150, 150)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), fill, -1)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), border, 2)
        cv2.putText(canvas, fit_text(label, max(8, w // 9)), (x + 12, y + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, text, 1, cv2.LINE_AA)
        self.buttons[name] = Button(name, rect, enabled)

    def draw_slider(
        self,
        canvas: np.ndarray,
        name: str,
        rect: Tuple[int, int, int, int],
        label: str,
        value: float,
        color: Tuple[int, int, int],
    ) -> None:
        x, y, w, h = rect
        cv2.putText(canvas, label, (x, y + 11),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (225, 230, 236), 1, cv2.LINE_AA)
        track_y = y + h - 8
        norm = float(np.clip(value, 0.0, 1.0))
        cv2.rectangle(canvas, (x, track_y - 3), (x + w, track_y + 3), (68, 73, 79), -1)
        cv2.rectangle(canvas, (x, track_y - 3), (x + w, track_y + 3), (93, 99, 106), 1)
        cv2.rectangle(canvas, (x, track_y - 2), (x + int(round(w * norm)), track_y + 2), color, -1)
        knob_x = x + int(round(w * norm))
        cv2.circle(canvas, (knob_x, track_y), 8, (235, 235, 235), -1)
        cv2.circle(canvas, (knob_x, track_y), 8, (96, 100, 106), 1)
        self.slider_rects[name] = (x, track_y - 12, w, 24)

    @staticmethod
    def rect_contains(rect: Tuple[int, int, int, int], x: int, y: int) -> bool:
        rx, ry, rw, rh = rect
        return rx <= x <= rx + rw and ry <= y <= ry + rh

    def mouse_callback(self, event, x: int, y: int, flags, param) -> None:
        del flags, param
        if event == cv2.EVENT_LBUTTONUP:
            rerun = self.active_slider is not None
            self.active_slider = None
            if rerun:
                self.run_current_sample()
            return
        if event == cv2.EVENT_MOUSEMOVE and self.active_slider is not None:
            self.update_slider_from_x(self.active_slider, x)
            return
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        for name, rect in list(self.slider_rects.items()):
            if self.rect_contains(rect, x, y):
                self.active_slider = name
                self.update_slider_from_x(name, x)
                return

        for name, button in list(self.buttons.items()):
            if not self.rect_contains(button.rect, x, y):
                continue
            if not button.enabled:
                return
            if name == "open_model":
                self.request_open_model()
            elif name == "open_image":
                self.request_open_image()
            elif name == "open_folder":
                self.request_open_folder()
            elif name == "prev":
                self.next_sample(-1)
            elif name == "next":
                self.next_sample(1)
            elif name == "run":
                self.run_current_sample()
            elif name == "class":
                self.advance_class_filter()
            return

    def update_slider_from_x(self, name: str, x: int) -> None:
        rect = self.slider_rects.get(name)
        if rect is None:
            return
        rx, _, rw, _ = rect
        norm = 0.0 if rw <= 0 else (float(x) - float(rx)) / float(rw)
        norm = float(np.clip(norm, 0.0, 1.0))
        if name == "confidence":
            self.yolo_conf = norm
            self.status = f"Confidence threshold: {self.yolo_conf * 100.0:.0f}%"

    def advance_class_filter(self) -> None:
        if self.last_num_classes <= 1:
            self.class_id = -1 if self.class_id >= 0 else 0
        elif self.class_id < 0:
            self.class_id = 0
        elif self.class_id + 1 >= self.last_num_classes:
            self.class_id = -1
        else:
            self.class_id += 1
        self.run_current_sample()

    def handle_key(self, key: int) -> None:
        if key in (ord("q"), 27):
            rclpy.shutdown()
        elif key == 32:
            self.save_camera_image()
        elif key in (ord("n"), 83):
            self.next_sample(1)
        elif key in (ord("p"), 81):
            self.next_sample(-1)
        elif key == ord("r"):
            self.run_current_sample()

    def request_open_model(self) -> None:
        selected = self.select_file_dialog(
            "Open YOLO Model",
            self.model_root if self.model_root.exists() else workspace_root(),
            [
                ("YOLO models", "*.onnx *.pt"),
                ("ONNX models", "*.onnx"),
                ("PyTorch models", "*.pt"),
                ("All files", "*.*"),
            ],
            "*.onnx *.pt",
        )
        if selected is None:
            self.status = "Open Model cancelled"
            return
        self.load_model(selected)

    def request_open_image(self) -> None:
        initial_dir = self.current_image_path.parent if self.current_image_path else self.samples_root
        selected = self.select_file_dialog(
            "Open Sample Image",
            initial_dir if initial_dir.exists() else workspace_root(),
            [("Image files", "*.png *.jpg *.jpeg *.bmp *.tif *.tiff"), ("All files", "*.*")],
            "*.png *.jpg *.jpeg *.bmp *.tif *.tiff",
        )
        if selected is None:
            self.status = "Open Image cancelled"
            return
        self.set_sample_paths([selected], f"Loaded image {selected.name}")

    def request_open_folder(self) -> None:
        initial_dir = self.current_image_path.parent if self.current_image_path else self.samples_root
        selected = self.select_folder_dialog(
            "Open Sample Folder",
            initial_dir if initial_dir.exists() else workspace_root(),
        )
        if selected is None:
            self.status = "Open Folder cancelled"
            return
        self.load_samples(selected, recent_first=False)

    def select_file_dialog(
        self,
        title: str,
        initial_dir: Path,
        tk_filetypes,
        shell_filter: str,
    ) -> Optional[Path]:
        try:
            import tkinter as tk
            from tkinter import filedialog

            root = tk.Tk()
            root.withdraw()
            try:
                root.attributes("-topmost", True)
            except tk.TclError:
                pass
            selected = filedialog.askopenfilename(
                title=title,
                initialdir=str(initial_dir),
                filetypes=tk_filetypes,
            )
            root.destroy()
            return Path(selected).expanduser().resolve() if selected else None
        except Exception as exc:
            self.get_logger().warn(f"Tk file picker unavailable: {exc}")

        for command in ("zenity", "kdialog"):
            if shutil.which(command) is None:
                continue
            try:
                if command == "zenity":
                    result = subprocess.run(
                        [
                            "zenity",
                            "--file-selection",
                            f"--title={title}",
                            f"--filename={str(initial_dir)}/",
                            f"--file-filter=Matching files | {shell_filter}",
                            "--file-filter=All files | *",
                        ],
                        check=False,
                        capture_output=True,
                        text=True,
                    )
                else:
                    result = subprocess.run(
                        ["kdialog", "--getopenfilename", str(initial_dir), shell_filter],
                        check=False,
                        capture_output=True,
                        text=True,
                    )
                selected = result.stdout.strip()
                if result.returncode == 0 and selected:
                    return Path(selected).expanduser().resolve()
                return None
            except Exception as exc:
                self.get_logger().warn(f"{command} file picker failed: {exc}")
        self.status = "Could not open file picker"
        return None

    def select_folder_dialog(self, title: str, initial_dir: Path) -> Optional[Path]:
        try:
            import tkinter as tk
            from tkinter import filedialog

            root = tk.Tk()
            root.withdraw()
            try:
                root.attributes("-topmost", True)
            except tk.TclError:
                pass
            selected = filedialog.askdirectory(title=title, initialdir=str(initial_dir), mustexist=True)
            root.destroy()
            return Path(selected).expanduser().resolve() if selected else None
        except Exception as exc:
            self.get_logger().warn(f"Tk folder picker unavailable: {exc}")

        for command in ("zenity", "kdialog"):
            if shutil.which(command) is None:
                continue
            try:
                if command == "zenity":
                    result = subprocess.run(
                        [
                            "zenity",
                            "--file-selection",
                            "--directory",
                            f"--title={title}",
                            f"--filename={str(initial_dir)}/",
                        ],
                        check=False,
                        capture_output=True,
                        text=True,
                    )
                else:
                    result = subprocess.run(
                        ["kdialog", "--getexistingdirectory", str(initial_dir)],
                        check=False,
                        capture_output=True,
                        text=True,
                    )
                selected = result.stdout.strip()
                if result.returncode == 0 and selected:
                    return Path(selected).expanduser().resolve()
                return None
            except Exception as exc:
                self.get_logger().warn(f"{command} folder picker failed: {exc}")
        self.status = "Could not open folder picker"
        return None

    def display_path(self, path: Optional[Path]) -> str:
        if path is None:
            return "none"
        try:
            return str(path.resolve().relative_to(workspace_root()))
        except Exception:
            return str(path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ItemDetectYoloDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

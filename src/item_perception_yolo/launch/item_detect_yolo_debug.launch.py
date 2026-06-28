import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    samples_path = LaunchConfiguration("samples_path")
    model_root = LaunchConfiguration("model_root")
    samples_root = LaunchConfiguration("samples_root")
    color_topic = LaunchConfiguration("color_topic")
    capture_dir = LaunchConfiguration("capture_dir")
    yolo_imgsz = LaunchConfiguration("yolo_imgsz")
    yolo_conf = LaunchConfiguration("yolo_conf")
    yolo_iou = LaunchConfiguration("yolo_iou")
    mask_threshold = LaunchConfiguration("mask_threshold")
    class_id = LaunchConfiguration("class_id")
    max_samples = LaunchConfiguration("max_samples")
    onnxruntime_threads = LaunchConfiguration("onnxruntime_threads")
    pt_device = LaunchConfiguration("pt_device")
    python_executable = LaunchConfiguration("python_executable")

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value="",
            description="Optional YOLO .onnx or .pt model to load at startup.",
        ),
        DeclareLaunchArgument(
            "samples_path",
            default_value="",
            description="Optional image file or image folder to load at startup.",
        ),
        DeclareLaunchArgument(
            "model_root",
            default_value=_repo_path("teach", "item_teach_yolo"),
            description="Initial folder for the Open Model dialog.",
        ),
        DeclareLaunchArgument(
            "samples_root",
            default_value=_repo_path("config", "item_perception_yolo"),
            description="Initial folder for sample image/folder dialogs.",
        ),
        DeclareLaunchArgument(
            "color_topic",
            default_value="/bin_camera/color/image_raw",
            description="Camera image topic captured when Space is pressed.",
        ),
        DeclareLaunchArgument(
            "capture_dir",
            default_value="~/Desktop/images",
            description="Folder for Space-key camera captures.",
        ),
        DeclareLaunchArgument(
            "yolo_imgsz",
            default_value="640",
            description="Fallback square input size when the ONNX input shape is dynamic.",
        ),
        DeclareLaunchArgument(
            "yolo_conf",
            default_value="0.35",
            description="Initial confidence threshold.",
        ),
        DeclareLaunchArgument(
            "yolo_iou",
            default_value="0.45",
            description="NMS IoU threshold.",
        ),
        DeclareLaunchArgument(
            "mask_threshold",
            default_value="0.5",
            description="Segmentation mask threshold.",
        ),
        DeclareLaunchArgument(
            "class_id",
            default_value="-1",
            description="Class ID to show, or -1 for all classes.",
        ),
        DeclareLaunchArgument(
            "max_samples",
            default_value="2000",
            description="Maximum images to scan from default or selected sample folders.",
        ),
        DeclareLaunchArgument(
            "onnxruntime_threads",
            default_value="0",
            description="Optional ONNX Runtime thread count; 0 leaves the default.",
        ),
        DeclareLaunchArgument(
            "pt_device",
            default_value="cpu",
            description="Compatibility option; .pt debug inference is always CPU-only.",
        ),
        DeclareLaunchArgument(
            "python_executable",
            default_value=_repo_path(".venv", "bin", "python"),
            description="Python interpreter with ONNX Runtime and Ultralytics installed.",
        ),
        Node(
            package="item_perception_yolo",
            executable="item_detect_yolo_debug_node.py",
            name="item_detect_yolo_debug",
            output="screen",
            prefix=[python_executable, " "],
            parameters=[{
                "model_path": ParameterValue(model_path, value_type=str),
                "samples_path": ParameterValue(samples_path, value_type=str),
                "model_root": ParameterValue(model_root, value_type=str),
                "samples_root": ParameterValue(samples_root, value_type=str),
                "color_topic": ParameterValue(color_topic, value_type=str),
                "capture_dir": ParameterValue(capture_dir, value_type=str),
                "yolo_imgsz": ParameterValue(yolo_imgsz, value_type=int),
                "yolo_conf": ParameterValue(yolo_conf, value_type=float),
                "yolo_iou": ParameterValue(yolo_iou, value_type=float),
                "mask_threshold": ParameterValue(mask_threshold, value_type=float),
                "class_id": ParameterValue(class_id, value_type=int),
                "max_samples": ParameterValue(max_samples, value_type=int),
                "onnxruntime_threads": ParameterValue(onnxruntime_threads, value_type=int),
                "pt_device": ParameterValue(pt_device, value_type=str),
            }],
        ),
    ])

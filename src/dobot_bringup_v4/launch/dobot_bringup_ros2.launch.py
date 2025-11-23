from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from pathlib import Path
import json


def _launch_setup(context, *args, **kwargs):
    # Resolve the config path (defaults to the package's param.json)
    config_path = Path(LaunchConfiguration('config').perform(context))
    if not config_path.exists():
        raise FileNotFoundError(f"[cr_robot_ros2] param.json not found at: {config_path}")

    with open(config_path, 'r') as f:
        cfg = json.load(f)

    # Read high-level config
    robot_number = int(cfg.get('robot_number', 1))
    current_robot = int(cfg.get('current_robot', 1))

    node_info = cfg.get('node_info', [])
    if not isinstance(node_info, list) or len(node_info) == 0:
        raise ValueError("[cr_robot_ros2] `node_info` must be a non-empty list in param.json.")

    # Clamp index (1-indexed in the JSON)
    idx = max(0, min(current_robot - 1, len(node_info) - 1))
    ni = node_info[idx]

    # Required + optional per-robot fields
    try:
        robot_ip = ni['ip_address']
    except KeyError:
        raise KeyError("[cr_robot_ros2] Missing required field `ip_address` in node_info entry.")

    params = {
        'robot_ip_address': robot_ip,
        'robot_type': ni.get('robot_type', 'cr5'),
        'trajectory_duration': float(ni.get('trajectory_duration', 0.3)),
        'robot_node_name': ni.get('robot_node_name', 'dobot_bringup_ros2'),
        'robot_number': robot_number,
    }

    # Bringup node (name set from JSON; parameters only from JSON)
    bringup = Node(
        package='cr_robot_ros2',
        executable='cr_robot_ros2_node',
        name=params['robot_node_name'],
        output='screen',
        parameters=[params],
    )

    return [bringup]


def generate_launch_description():
    # Default to the installed share/config/param.json inside this package
    share_dir = get_package_share_path('cr_robot_ros2')
    default_config = str(share_dir / 'config' / 'param.json')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Path to the param.json containing robot connection info.'
        ),
        OpaqueFunction(function=_launch_setup),
    ])

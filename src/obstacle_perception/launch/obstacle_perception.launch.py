from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    color_topic = LaunchConfiguration("color_topic")
    enable_memory = LaunchConfiguration("enable_memory")
    memory_voxel_size = LaunchConfiguration("memory_voxel_size")
    memory_decay = LaunchConfiguration("memory_decay")
    memory_max_voxels = LaunchConfiguration("memory_max_voxels")
    memory_color_r = LaunchConfiguration("memory_color_r")
    memory_color_g = LaunchConfiguration("memory_color_g")
    memory_color_b = LaunchConfiguration("memory_color_b")
    memory_min_hits = LaunchConfiguration("memory_min_hits")
    voxel_size = LaunchConfiguration("voxel_size")
    pixel_stride = LaunchConfiguration("pixel_stride")
    min_range = LaunchConfiguration("min_range")
    max_range = LaunchConfiguration("max_range")
    marker_lifetime = LaunchConfiguration("marker_lifetime")
    min_points_per_voxel = LaunchConfiguration("min_points_per_voxel")
    publish_pointcloud = LaunchConfiguration("publish_pointcloud")
    publish_markers = LaunchConfiguration("publish_markers")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "color_topic", default_value="/camera/camera_link/color/image_raw"
            ),
            DeclareLaunchArgument(
                "depth_topic", default_value="/camera/camera_link/aligned_depth_to_color/image_raw"
            ),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/camera_link/color/camera_info"
            ),
            DeclareLaunchArgument("enable_memory", default_value="true"),
            DeclareLaunchArgument("memory_voxel_size", default_value="0.03"),
            DeclareLaunchArgument("memory_decay", default_value="0.0"),
            DeclareLaunchArgument("memory_max_voxels", default_value="400000"),
            DeclareLaunchArgument("memory_color_r", default_value="0"),
            DeclareLaunchArgument("memory_color_g", default_value="100"),
            DeclareLaunchArgument("memory_color_b", default_value="255"),
            DeclareLaunchArgument("memory_min_hits", default_value="30"),
            DeclareLaunchArgument("memory_skip_live", default_value="true"),
            DeclareLaunchArgument("memory_blue_tint", default_value="0.3"),
            DeclareLaunchArgument("memory_skip_live_volume", default_value="true"),
            DeclareLaunchArgument("voxel_size", default_value="0.03"),
            DeclareLaunchArgument("pixel_stride", default_value="4"),
            DeclareLaunchArgument("min_range", default_value="0.15"),
            DeclareLaunchArgument("max_range", default_value="2.5"),
            DeclareLaunchArgument("marker_lifetime", default_value="300.0"),
            DeclareLaunchArgument("min_points_per_voxel", default_value="3"),
            DeclareLaunchArgument("publish_pointcloud", default_value="true"),
            DeclareLaunchArgument("publish_markers", default_value="true"),
            Node(
                package="obstacle_perception",
                executable="obstacle_perception_node",
                name="obstacle_perception",
                output="screen",
                parameters=[
                    {
                        "depth_topic": depth_topic,
                        "camera_info_topic": camera_info_topic,
                        "color_topic": color_topic,
                        "voxel_size": voxel_size,
                        "pixel_stride": pixel_stride,
                        "min_range": min_range,
                        "max_range": max_range,
                        "marker_lifetime": marker_lifetime,
                        "min_points_per_voxel": min_points_per_voxel,
                        "publish_pointcloud": publish_pointcloud,
                        "publish_markers": publish_markers,
                    }
                ],
            ),
            Node(
                condition=IfCondition(enable_memory),
                package="obstacle_perception",
                executable="obstacle_memory_node",
                name="obstacle_memory",
                output="screen",
                parameters=[
                    {
                        "input_cloud_topic": "/obstacles/points",
                        "output_cloud_topic": "/obstacles/memory_points",
                        "voxel_size": memory_voxel_size,
                        "decay_seconds": memory_decay,
                        "max_voxels": memory_max_voxels,
                        "color_r": memory_color_r,
                        "color_g": memory_color_g,
                        "color_b": memory_color_b,
                        "min_hits": memory_min_hits,
                        "skip_if_live": LaunchConfiguration("memory_skip_live"),
                        "blue_tint": LaunchConfiguration("memory_blue_tint"),
                        "skip_live_volume": LaunchConfiguration("memory_skip_live_volume"),
                    }
                ],
            ),
        ]
    )

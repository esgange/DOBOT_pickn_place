# obstacle_perception

Depth-to-voxel obstacle visualizer anchored to `calibrated_camera_link` plus an optional memory layer.

## What it does
- Live node: subscribes to depth + color + camera info, projects points in the calibrated camera frame, voxelizes, and publishes:
  - `/obstacles/points` (`sensor_msgs/PointCloud2` with RGB)
  - `/obstacles/markers` (`visualization_msgs/MarkerArray` of cubes)
- Memory node: subscribes to `/obstacles/points`, transforms into a fixed frame (default `base_link`), merges voxels after `memory_min_hits`, and republishes `/obstacles/memory_points` with averaged RGB and a blue tint. Can skip voxels in the current camera frustum to keep line-of-sight clear.

## Parameters (key)
- Live node: `depth_topic` (default aligned depth), `camera_info_topic`, `color_topic`, `voxel_size` (default `0.03`), `pixel_stride`, `min_range`/`max_range`, `marker_lifetime`, `min_points_per_voxel`, `publish_pointcloud`, `publish_markers`, `use_color`. Frame is fixed to `calibrated_camera_link`.
- Memory node: `input_cloud_topic` (`/obstacles/points`), `output_cloud_topic` (`/obstacles/memory_points`), `target_frame` (default `base_link`), `memory_voxel_size`, `memory_min_hits`, `memory_max_voxels`, `memory_decay` (0 = keep), `memory_skip_live`, `frustum_enable`, `frustum_frame` (default `calibrated_camera_link`), `frustum_near/far`, `frustum_hfov_deg`, `frustum_vfov_deg`, `memory_blue_tint`, `color_r/g/b`.

## Run
```bash
ros2 launch obstacle_perception obstacle_perception.launch.py \
  color_topic:=/camera/camera_link/color/image_raw \
  depth_topic:=/camera/camera_link/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera_link/color/camera_info \
  target_frame:=base_link \
  frustum_frame:=calibrated_camera_link \
  memory_min_hits:=30 \
  memory_voxel_size:=0.03
```
In RViz, add `MarkerArray` on `/obstacles/markers`, `PointCloud2` on `/obstacles/points` (live), and `PointCloud2` on `/obstacles/memory_points` (memory). The provided RViz config already includes these displays and uses `RGB8` coloring with box style.

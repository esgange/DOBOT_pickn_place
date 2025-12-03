# aruco_perception

RGB-D ArUco detection pipeline that publishes marker poses/TFs in the calibrated camera frame.

## What it does
- Subscribes to camera color/depth/info topics, detects ArUco markers, estimates 6D poses.
- Publishes `marker_pose` (`geometry_msgs/PoseStamped`) and TFs `aruco_marker_<id>` in `calibrated_camera_link` (or your configured camera frame).
- Optionally loads a static `Link6 -> calibrated_camera_link` transform from a calibration YAML.

## Calibration handling
- Looks for the newest non-empty YAML under `~/DOBOT_pickn_place/calibration` when `use_calibration:=true` (default).
- If none found or load fails, it prompts and will not broadcast `calibrated_camera_link`; set your downstream `camera_frame` accordingly.
- You can force a specific file with `calibration_file:=/path/to/file.yaml`.

## Run
```bash
ros2 launch aruco_perception aruco_perception.launch.py \
  use_calibration:=true \
  parent_frame:=Link6 \
  child_frame:=calibrated_camera_link
```
If you want to skip calibration and keep poses in a raw frame, launch with `use_calibration:=false` and set your detector’s `camera_frame` parameter to `Link6` or your camera frame.

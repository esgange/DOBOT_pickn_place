# DOBOT Pick and Place (ROS 2)

ROS 2 workspace for DOBOT collaborative arms with bringup, custom interfaces, RViz models, ArUco perception, and eye-in-hand camera calibration utilities.

## Packages
- `dobot_bringup_v4`: TCP client wrapper and launch files to connect to a DOBOT CR series arm (configure IP/port in `config/param.json`).
- `dobot_msgs_v4`: Custom messages and service definitions mirroring the DOBOT dashboard/control API.
- `dobot_rviz`: URDFs and meshes for multiple DOBOT models, plus a ready-to-use RViz config.
- `camera_calibration`: Eye-on-hand calibration tools (CLI and GUI) for aligning the robot and camera frames; writes YAML to `~/DOBOT_pickn_place/calibration`.
- `aruco_perception`: RGB-D ArUco detection with TF output; loads the latest calibration YAML (or prompts if missing) and publishes `calibrated_camera_link` for marker TFs.
- `obstacle_perception`: Live depth-to-voxel obstacles and a memory node that accumulates colored obstacles in a fixed frame with configurable frustum filtering to keep line-of-sight clear.

## Prerequisites
- ROS 2 (tested with Foxy/Humble style workspace layout)
- Build tools: `colcon`, `rosdep`
- C++ compiler, Python 3

## Setup
```bash
# From the workspace root
rosdep install --from-paths src -i -r -y
colcon build --symlink-install
source install/setup.bash
```

## Usage
- Connect to the robot: `ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py` (edit `config/param.json` for robot IP and model).
- Calibrate eye-in-hand: `ros2 launch camera_calibration camera_calibration.launch.py` (or `camera_calibration_gui.launch.py`). This also starts:
  - `calibration_perception` to average `aruco_marker_1..4` into `tag_frame`.
  - `aruco_perception` with calibration disabled so markers are effectively parented to `Link6` during calibration (identity `Link6 -> calibrated_camera_link`).
- Run ArUco perception with calibration: `ros2 launch aruco_perception aruco_perception.launch.py` (defaults to auto-discover the latest calibration YAML and broadcast `Link6 -> calibrated_camera_link`; set `use_calibration:=false` to force identity).
- Run obstacles + memory: `ros2 launch obstacle_perception obstacle_perception.launch.py target_frame:=base_link frustum_frame:=calibrated_camera_link` (tune voxel size, frustum FOV/near/far, min hits, and max voxels; RViz config already visualizes live and memory clouds).
- Visualize models: `ros2 launch dobot_rviz dobot_rviz.launch.py` then open the provided RViz config (`rviz/urdf.rviz`).

## Notes
- Only source code is tracked; build/install/log directories are ignored.
- Models and messages cover multiple DOBOT variants (CR, ME, Nova). Adjust launch parameters to match your hardware.

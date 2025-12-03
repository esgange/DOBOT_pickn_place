# dobot_rviz

RViz resources and launch for DOBOT arms.

## What it does
- Loads URDF/meshes for the configured robot model and starts `robot_state_publisher`.
- Provides a ready-to-use RViz config (`rviz/urdf.rviz`) with robot, TF, and obstacle displays.
- Adds an optional static `world -> base_link` transform for quick visualization.

## Run
```bash
ros2 launch dobot_rviz dobot_rviz.launch.py
```
Ensure `dobot_bringup_v4` (or another source of `/joint_states_robot`) is running so the model moves. Obstacle voxels/point cloud will appear automatically if the `obstacle_perception` node is publishing `/obstacles/markers` and `/obstacles/points`.

## Displays
- RobotModel, TF, Grid (default)
- Obstacles: `/obstacles/markers` (cubes), `/obstacles/points` (live point cloud, RGB8 boxes)
- ObstacleMemory: `/obstacles/memory_points` (memory point cloud, RGB8 boxes)

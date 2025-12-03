# dobot_bringup_v4

Launch and TCP bridge for DOBOT CR-series arms.

## What it does
- Reads robot IP/port/model from `config/param.json`.
- Starts the dashboard/control TCP client and publishes ROS 2 topics/services for motion control.
- Remaps `joint_states` from the robot to drive your URDF/TF tree.

## Configure
Edit `config/param.json` to set `current_robot`, robot IP/port, and model. Ensure the network allows TCP access to the arm.

## Run
```bash
ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py
```
Then, load RViz via `dobot_rviz` or control the arm using the exposed topics/services.

## Notes
- Publishes `/joint_states_robot` for visualization.
- Depends on `dobot_msgs_v4` interfaces for dashboard/control commands.

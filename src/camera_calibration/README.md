# camera_calibration

Eye-on-hand AX=XB calibration node for robot-mounted cameras. Inspired by MoveIt's hand-eye calibration plugin but packaged as a standalone ROS 2 node.

## Features
- Collect paired poses from TF frames (base->gripper, camera->target).
- Solve AX=XB using OpenCV's Tsai hand-eye method.
- Write YAML transform for downstream perception stacks.
- Simple UI via ROS services (`/add_sample`, `/compute_calibration`, `/reset_samples`).

## Running
```bash
ros2 launch camera_calibration camera_calibration.launch.py
```
This opens the MoveIt-style GUI; fill in the TF frame names (defaults: base_link, link6, camera_link, tag_frame), the output YAML path (defaults to `~/axab_calibration.yaml`), and minimum samples. Use:
- Start/Stop Calibrator to launch/terminate the node with your inputs.
- Add Sample, Compute, Save YAML, Reset Samples to drive the calibration services and see responses. Compute solves and caches AX=XB; Save YAML writes the last computed result (so you can compute multiple times before saving).

While the node is running:
```bash
# Record a sample (gripper pose + target pose should be updated)
ros2 service call /add_sample std_srvs/srv/Trigger {}

# Compute AX=XB once you have enough poses
ros2 service call /compute_calibration std_srvs/srv/Trigger {}

# Write the most recent computed solution to disk
ros2 service call /save_calibration std_srvs/srv/Trigger {}

# Clear samples if needed
ros2 service call /reset_samples std_srvs/srv/Trigger {}
```

The output YAML (`axab_calibration.yaml` by default) contains quaternion, Euler angles (degrees), translation, metadata, and timestamp suitable for downstream perception packages.

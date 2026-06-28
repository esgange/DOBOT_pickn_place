# tray_intercept

`tray_intercept` is the operator package for moving to a stationary tray target
from `tray_perception` output. It waits for a fresh `tray_target_pose`,
transforms that pose into the robot base frame, and dispatches a staged robot
motion sequence.

## Executable

| Executable | Purpose |
| --- | --- |
| `tray_intercept` | Tkinter operator console and service endpoint for tray intercept motion. |

## Build

```bash
cd WORKSPACE_ROOT
source /opt/ros/humble/setup.bash
colcon build --packages-select tray_intercept
source install/setup.bash
```

## Run

```bash
ros2 launch tray_intercept tray_intercept.launch.py
```

Headless service mode, with no Tkinter window:

```bash
ros2 launch tray_intercept tray_intercept.launch.py headless:=true
```

Important launch arguments are `runtime_settings_file`,
`motion_service_root`, `tray_target_pose_topic`, `tray_axis_overlay_topic`,
`track_service`, `track_status_service`, `tray_dimensions_service`, and
`tray_seek_complete_service`. When `load_runtime_settings:=true`, the JSON
runtime settings are loaded at startup. In headless mode the JSON file must
exist and include the complete runtime key set; launch arguments are treated as
overrides, not the normal place to keep motion settings.

Direct run:

```bash
ros2 run tray_intercept tray_intercept
```

## Inputs

| Input | Type | Source |
| --- | --- | --- |
| `tray_target_pose` | `geometry_msgs/msg/PoseStamped` | `tray_detect` one-frame validated seek target pose. |
| `tray_axis_overlay` | `geometry_msgs/msg/PolygonStamped` | Live 2D tray origin and X/Y axes for the GUI preview. |

The GUI automatically calls `tray_detect/get_tray_dimensions` to keep the tray
preview size synced when the service is available.

GUI runtime settings are saved to:

```text
WORKSPACE_ROOT/config/tray_perception/tray_intercept_runtime_settings.json
```

## Services

Service exposed by this package:

| Service | Type | Purpose |
| --- | --- | --- |
| `tray_intercept/track` | `std_srvs/srv/Trigger` | Arms the same intercept sequence as the GUI track button. |
| `tray_intercept/track_status` | `std_srvs/srv/Trigger` | Returns success while track is armed and waiting for a fresh target pose. |
| `tray_intercept/start_sequence` | `dobot_msgs_v4/srv/TrayInterceptStart` | Arms and starts the intercept sequence. |

Robot services called under `/dobot_bringup_ros2/srv`:

- `GetPose`
- `CP`
- `SpeedFactor`
- `Stop`
- `MovL`
- `MovLIO`
- `DO`

Example:

```bash
ros2 service call /tray_intercept/start_sequence dobot_msgs_v4/srv/TrayInterceptStart \
"{tray_vector_wait_timeout_sec: 60.0, ee_intercept_speed_mm_s: 650.0, tray_intercept_x_offset_mm: 0.0, tray_intercept_y_offset_mm: 0.0, ee_final_pose_angle_deg: 0.0, tray_standoff_z_mm: 100.0, follow_distance_mm: 200.0, post_follow_z_up_mm: 300.0, troubleshoot_tf_only: false}"
```

## Runtime Flow

When armed, the node:

1. Waits for a fresh `tray_target_pose`.
2. Sends `Stop`.
3. Computes a stationary tray target goal in `base_link`.
4. Queues `MovL` to the target pose.
5. Opens the release outputs at the target pose when release grip is enabled.
6. Queues a post-target Z-up move.

Troubleshoot mode publishes goal TFs only and does not send robot motion.
The tray intercept move uses a fixed `650 mm/s` EE speed; the
`ee_intercept_speed_mm_s` service field is kept for compatibility. The GUI
angle control sets a manual `-90..90 deg` tray-Y angle offset: negative rotates
CCW, positive rotates CW, and zero aligns the TCP yaw to the closest parallel
tray Y-axis direction. Trays are assumed stationary, so there is no target
prediction, tray velocity, or tray-direction follow.
The tray standoff Z offset is applied in robot/base +Z, so positive Z remains
an upward standoff even if the detected tray frame has a downward natural Z.
Tray X/Y offsets are projected into the robot base XY plane before motion.

## Release IO

When release grip is enabled, tray intercept opens the gripper at the stationary
target pose and uses the suction exhaust output. `DO4` is pulsed for about
`300 ms`, then the gripper outputs are returned to neutral during cleanup.

Release at the target pose:

| Output | State |
| --- | --- |
| `DO1` gripper close | `OFF` |
| `DO3` suction | `OFF` |
| `DO4` suction exhaust | `ON` |
| `DO2` gripper open | `ON` |

Exhaust pulse:

| Output | State |
| --- | --- |
| `DO4` suction exhaust | `OFF` after the computed 300 ms pulse point |

Post-target cleanup returns the gripper outputs to neutral:

| Output | State |
| --- | --- |
| `DO1` gripper close | `OFF` |
| `DO2` gripper open | `OFF` |
| `DO3` suction | `OFF` |
| `DO4` suction exhaust | `OFF` |

The node waits until the target pose is reached, sends the release outputs with
`DO`, turns `DO4` back off after the 300 ms pulse while keeping `DO2` open,
waits the remaining release settling time, then runs the neutral cleanup.
Suction DI feedback is not used by `tray_intercept`.

The preview origin is fixed at the lower-left tray corner. X/Y preview clicks
are converted from that displayed bottom origin and sent directly in the
canonical tray frame from `tray_detect`. The flat top-down preview uses live
2D axes from `tray_axis_overlay`, so it follows runtime tray orientation without
waiting for a seek `tray_target_pose`.

## Debug TF Frames

Published in the robot goal frame, default `base_link`:

- `tray_movel_goal_tcp`
- `tray_follow_goal_tcp`
- `tray_post_follow_zup_goal_tcp`

## Motion Calibration

The node auto-loads speed mapping from the newest non-empty file matching:

```text
WORKSPACE_ROOT/calibration/relmovl_speed_calibration*.json
```

It also reads startup `CP` and `SpeedFactor` values from the same calibration
file and applies them once before the first real motion command.

## Stationary Tray Handling

- `tray_target_pose` is treated as the final stationary tray pose.
- The service fields `tray_vector_wait_timeout_sec`, `ee_intercept_speed_mm_s`,
  and `follow_distance_mm` remain for compatibility with existing callers.
- TF-only mode publishes the target, hold, and Z-up debug frames without sending
  robot motion.

## Notes

- Run `tray_detect` before arming so `tray_target_pose` is available.
- Use TF-only troubleshoot mode to validate intercept frames before real robot
  motion.
- Keep movement calibration current when robot speed behavior changes.

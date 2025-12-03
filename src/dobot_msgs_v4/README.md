# dobot_msgs_v4

ROS 2 interface definitions mirroring the DOBOT CR dashboard/control API.

## What it provides
- Message, service, and action definitions used by `dobot_bringup_v4` and related nodes.
- Types for robot state, dashboard commands, and motion control.

## Usage
- Build the workspace to generate headers/interfaces: `colcon build --packages-select dobot_msgs_v4`.
- Depend on this package in your own CMake/ROS 2 packages to call DOBOT services or subscribe to status topics.

## Notes
- Used by `dobot_bringup_v4` for dashboard/control and by downstream tooling that speaks DOBOT’s protocol.

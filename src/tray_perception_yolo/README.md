# tray_perception_yolo

`tray_perception_yolo` is the YOLO tray-detection branch. It keeps the runtime
`tray_detect` topics and services used by `tray_intercept`, but replaces classic
RGB edge detection with a paired YOLO `.pt` model and YAML profile.

Profiles live under:

```text
WORKSPACE_ROOT/teach/tray_teach_yolo
```

Each usable profile must include a `.pt` model plus a paired YAML with model
metadata, taught tray dimensions, and a saved 4-point tray depth plane. Runtime
pose and placement height are projected onto that saved plane so items sitting
on the tray do not corrupt Z.

```bash
ros2 launch tray_perception_yolo tray_teach_yolo.launch.py
ros2 launch tray_perception_yolo tray_detect_yolo.launch.py selected_profile_path:=/path/to/tray_profile.yaml
```

Teach flow:

1. Enter a tray name and capture/review tray images like item YOLO teach.
2. Train YOLO11 or open an already-trained `yolo11`/`yolo26` `.pt` model.
3. With the live YOLO preview active, click `Teach Plane` and select four
   exposed ordered tray rectangle points. These points are used both for the
   normal depth plane and for the taught tray width/height.
4. Click `Save Teach` to write the paired `.pt` + YAML profile.

The detector requires an explicit `selected_profile_path`; it does not
auto-select a fallback tray profile. A tray seek consumes one camera frame,
checks the highest-confidence tray mask against the taught width/height
tolerance, then publishes one `tray_target_pose`. Any failed prerequisite,
missing mask, pose failure, or dimension mismatch raises a fatal node error.

The detector publishes `tray_overlay`, `tray_pose`, `tray_axis_overlay`,
`tray_target_pose`, `tray_cube_marker`, and serves
`tray_detect/get_tray_dimensions`, `tray_detect/seek`,
`tray_detect/seek_complete`, `tray_detect/seek_status`, and
`tray_detect/go_to_teach`.

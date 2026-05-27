# Project Teleop — macOS App

SwiftUI macOS control panel for the Project Teleop ROS2 stack. Replaces
the `./startup.sh` tmux flow with a graphical launcher, live robot monitor,
3D MG400 view, and a session log viewer.

## Architecture

```
SwiftUI app  ──spawn──▶  tools/mac_app/launcher.sh   ──docker──▶  project_teleop_ros2_mac
       │                                                                │
       │                                                                ├─ project-teleop-jazzy endpoint  (ROS-TCP)
       │                                                                ├─ project-teleop-jazzy teleop    (vr_teleop_node)
       │                                                                └─ tools/mac_app/ros_bridge.py    (NDJSON on stdout)
       │
       └──reads──▶  logs/teleop_sessions/*.csv   (Session log viewer)
```

The SwiftUI app talks to ROS only indirectly:
- **Process control** through `launcher.sh` (start/stop/status).
- **Live state** through `ros_bridge.py` running inside the container,
  which subscribes to `/joint_states`, `/mg400/robot_mode`,
  `/mg400/error_status`, `/vr/suction_cmd`, `/mg400/light_cmd` and
  prints one JSON object per line on stdout.

This keeps every ROS dependency inside the existing Docker image — the
host only needs Docker Desktop and Xcode.

## Build the .app

Requires Xcode 15+ and `xcodegen`:

```bash
brew install xcodegen
cd project_teleop/tools/mac_app
xcodegen generate
open ProjectTeleopMac.xcodeproj
```

Then in Xcode press ⌘R. The first time, set the *Repo path* in the app's
Settings sheet to your absolute path of `project_teleop/` (default is the
current dev path).

## Features

- **Launcher** — Start / Stop / Status with mode picker
  (`real-sim`, `mock-sim`, `real-unity`, `mock-unity`). Streams stdout
  from `launcher.sh` into a console pane.
- **Monitor** — Joint angles (deg), robot_mode, error_status,
  suction, light. Updates pushed by `ros_bridge.py`.
- **3D View** — SceneKit MG400 skeleton driven by live joint state
  (4-DOF SCARA: J1 yaw, J2/J3 vertical pair, J4 wrist).
- **Logs** — Lists `logs/teleop_sessions/*.csv`, plots robot joint
  angles and end-to-end latency over `elapsed_sec` using Swift Charts.

## Manual smoke test

```bash
# from project_teleop/
./tools/mac_app/launcher.sh status        # → stopped
./tools/mac_app/launcher.sh start mock-sim
./tools/mac_app/launcher.sh status        # → running
./tools/mac_app/launcher.sh bridge        # NDJSON stream (Ctrl+C to quit)
./tools/mac_app/launcher.sh stop
```

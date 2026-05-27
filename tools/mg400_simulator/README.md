# MG400 3-D Simulator

Unity-like 3-D robot simulator for the MG400 arm, built in Python with PyQt5 + OpenGL.

## Features

| Feature | Details |
|---|---|
| 3-D viewport | OpenGL with URDF `.dae` mesh rendering, orbit / pan / zoom |
| 1:1 EE drag | Click the orange sphere → drag to move end-effector in camera plane |
| Axis drag | Click Red/Green/Blue arrows to precisely move EE along X/Y/Z world axes |
| Joint sliders | J1–J4 sliders with live FK preview |
| Teach & Repeat | Record waypoints, preview poses, and play them back sequentially |
| Robot commands | Enable / Disable / Clear Error / E-Stop |
| I/O | Suction & light toggle |
| ROS 2 bridge | Optional – publishes to `/unity/joint_cmd`, subscribes to `/joint_states` feedback |
| Ghost pose | Last-sent pose shown dimmed in viewport |

## Viewport controls

| Input | Action |
|---|---|
| Left-drag | Orbit camera |
| Middle-drag / Alt+Left | Pan camera |
| Scroll wheel | Zoom |
| Drag orange sphere | Move end-effector (1:1 depth-locked drag) |
| Drag Gizmo arrow | Move end-effector along specific X/Y/Z axis |
| `F` | Focus camera on EE |
| `R` | Reset camera |

## Installation

```bash
cd tools/mg400_simulator
pip install -r requirements.txt
```

On Ubuntu you may also need:
```bash
sudo apt install python3-pyqt5 python3-opengl
```

## Running

### Standalone (no ROS required)
```bash
cd tools/mg400_simulator
python3 main.py
# Or use the convenience script: ./run.sh
```

### With ROS 2 bridge (live robot / mock)

If you are running the simulator on the same Linux machine as ROS 2:
```bash
# Terminal 1 – start the mock robot
cd tools/mock_robot && docker compose up -d

# Terminal 2 – start teleop stack
source ~/teleop_mg400_ws/install/setup.bash
export ROBOT_IP=<mock_container_ip>
ros2 run teleop_logic teleop_node

# Terminal 3 – launch simulator
source ~/teleop_mg400_ws/install/setup.bash
python3 tools/mg400_simulator/main.py
# → enable "Connect to ROS 2" in the control panel
```

### Running Simulator on Mac / Windows (Unity Mock Mode)

Since `rclpy` cannot be easily installed on macOS or Windows, the simulator features a built-in **Unity TCP Mock**. This allows the Python simulator to masquerade as the Unity VR Client and communicate directly with the `ros-tcp-endpoint` (Port 10000) on your Ubuntu machine without needing a native ROS 2 distribution installed locally.

**1. On the Ubuntu Machine (Robot Controller):**
Start the complete teleop stack specifying the Unity Frontend:
```bash
cd ~/project_teleop_ws/project_teleop
./scripts/start_teleop.sh
# Select:
#   4 = Mock + Unity / ROS-TCP endpoint
#   3 = Real robot + Unity / ROS-TCP endpoint
```
*(Note down the IP address of this Ubuntu machine, e.g., `192.168.1.6`)*

**2. On your Mac/Windows Machine (Simulator):**
Run the simulator launcher:
```bash
cd /Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop
./tools/mg400_simulator/run.sh
```
In the right control panel under **"ROS 2 Bridge / Unity Mock"**: 
- Enter your Ubuntu machine's IP in the **Host** field.
- Leave the **Port** as `10000`.
- Check **"Connect"**.

You can now control the robot remotely exactly as the Unity VR headset does!

### Sending Teach-And-Repeat Jobs Without Unity

The simulator's **Teach & Repeat** tab can also send the newer teach-job JSON
contract used by the real Unity `TeachJobPublisher`.

1. Connect the simulator to the Ubuntu ROS-TCP endpoint as described above.
2. In the **Teach & Repeat** tab, drag the robot or adjust joints.
3. Press **+ Add Point** for each waypoint.
4. Use **ROS Teach Job**:
   - **Validate Plan** publishes `action=compile` to `/teach/job_request`; ROS
     compiles/checks the plan only and must not move the robot.
   - Set **SpeedJ/AccJ**, **SpeedL/AccL**, and **CP** before Execute. These are
     sent as teach-job options; playback no longer tries to reproduce the
     demonstrator's original timestamps.
   - **Execute** publishes `action=execute`; Mock/robot motion is owned by ROS,
     and the simulator viewport follows `/joint_states` feedback during that job.
     The button is enabled only after validation succeeds.
   - While Execute is running, changing Speed/Acc/CP sends `action=tune`; commands
     that have not yet been sent to the MG400 pick up the new values.

During realtime control, the simulator treats the GUI as the target source and
does not continuously overwrite the sliders/viewport with robot feedback.  It
uses feedback for the initial connection sync and for ROS-owned teach/repeat
execution. The old local "Play Live" path is intentionally hidden from the
Teach tab so it cannot stream `/unity/joint_cmd` on top of a ROS-owned Execute.

The emitted payload is a `std_msgs/String` on:

```text
/teach/job_request
```

Its trajectory body matches the Unity job shape:

```json
{
  "filename": "mg400_simulator_waypoints.json",
  "teach_mode": "waypoint",
  "source": "mg400_simulator",
  "frames": [
    {"timeStamp": 0.0, "j1": 0.0, "j2": 0.0, "j3": -45.0, "j4": 0.0}
  ],
  "events": []
}
```

Runtime tuning uses the same topic with an empty trajectory:

```json
{
  "action": "tune",
  "target": "mg400",
  "trajectory": {"filename": "runtime_tuning", "frames": []},
  "options": {
    "use_recorded_timing": false,
    "speed_j": 40,
    "acc_j": 80,
    "speed_l": 40,
    "acc_l": 80,
    "cp": 30,
    "final_cp": 0,
    "queue_lookahead_commands": 3
  }
}
```

## Architecture

```
main.py
  └── MainWindow (ui/main_window.py)
        ├── RobotViewport (ui/robot_viewport.py)   ← PyOpenGL 3-D scene
        │     • orbit / pan / zoom camera
        │     • accurate URDF mesh loading via trimesh
        │     • Unity-style gizmo drag (axis & sphere) using ray-plane IK
        │     • emits joints_changed signal
        ├── ControlPanel (ui/control_panel.py)      ← right sidebar
        │     • joint sliders ↔ viewport (bidirectional)
        │     • speed / mode / I/O controls
        ├── TeachPanel (ui/teach_panel.py)          ← right sidebar (tab)
        │     • waypoint recording and playback logic
        │     • pub /teach/job_request via ROSBridge / UnityTcpBridge
        └── ROSBridge (core/ros_bridge.py)          ← optional rclpy node
              • pub /unity/joint_cmd
              • pub /teach/job_request
              • sub /joint_states
              • pub dashboard commands
```

## Kinematics

Uses the exact same constants as `teleop_logic/utils/kinematics.py`:

| Link | Vector (mm) |
|---|---|
| LINK1 (shoulder offset) | [43, 0, 0] |
| LINK2 (upper arm) | [0, 0, 175] |
| LINK3 (forearm) | [175, 0, 0] |
| LINK4 (wrist) | [66, 0, -57] |

Analytical IK closed-form solution used for real-time drag.

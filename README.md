# MG400 ROS2 Teleoperation Project

This repository contains a ROS2 workspace for teleoperation and simulation of the Dobot MG400 robot.

The workspace integrates:
- ROS2 control and description packages for MG400
- Unity–ROS communication via ROS-TCP-Endpoint
- A mock MG400 server using Docker for simulation and testing
- Dobot TCP/IP Python examples and reference client

Production runtime:
- Teleoperation logic: `default` queue-aware production mode
- Robot command mode: `JointMovJ`

Some external dependencies are included as Git submodules for reproducibility.

---

## 📥 Clone the Project

This repository uses Git submodules. Please clone using the recursive flag:
```bash
git clone --recurse-submodules https://github.com/th0sun/project_teleop.git
```

If you have already cloned the repository without submodules, run:
```bash
git submodule update --init --recursive
```

---

## 📂 Workspace Structure

The project is organized as follows:
```
project_teleop/
├── Dobot_TCP_IP_Python_V4/    # (Submodule) Official Dobot TCP/IP Python examples
├── MG400_Mock/                # (Submodule) External MG400 mock/simulator from HarvestX
├── src/
    ├── dobot_mg400/           # Custom ROS2 packages for MG400
    │   ├── mg400_bringup/     # Launch files and system integration
    │   ├── mg400_controller/  # Teleop node and control logic (Python)
    │   ├── mg400_description/ # URDF model and meshes
    │   └── mg400_simulator/   # Unity simulation bridge
    └── ROS-TCP-Endpoint/      # (Submodule) ROS–Unity TCP communication package
└── tools/
    ├── mg400_simulator/       # PyQt/OpenGL Unity-like GUI simulator
    └── monitor/               # PyQt monitor GUI and UDP bridge
```

### Detailed Description

- **`Dobot_TCP_IP_Python_V4/`** (submodule)  
  Official Dobot TCP/IP Python examples and API reference client.

- **`MG400_Mock/`** (submodule)  
  Dobot MG400 mock server from HarvestX (Docker-based simulator).

- **`src/dobot_mg400/`**  
  Collection of ROS2 packages handling the robot's description, control logic, and simulation bridging.

- **`src/ROS-TCP-Endpoint/`** (submodule)  
  Handles the TCP communication between ROS2 and Unity.

- **`tools/mg400_simulator/`**  
  Desktop Unity-like GUI simulator built with Python, PyQt, and OpenGL.

- **`tools/monitor/`**
  PyQt monitor GUI and UDP bridge for viewing telemetry and sending dashboard commands.

> **Note:** Submodules are external repositories linked to specific versions.

---

## 🛠️ Build the Workspace

Make sure you have **ROS2 Jazzy** installed.
```bash
cd project_teleop_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

---

## Production Teleop Logic

The runtime teleoperation logic is the queue-aware `default` mode.

The selected production path is:

```text
Unity-like GUI / VR
    -> /unity/joint_cmd
    -> vr_teleop_node.py
    -> TeleopController queue-aware gate
    -> MotionPlanner
    -> TCP 30003 JointMovJ(...)
    -> MG400 motion queue
```

See:

- `docs/teleop_command_logic.md`
- `docs/setup_guide.md`
- `docs/controller_continuation_guide.md`

---

## 🤖 How to Use MG400 Mock Server

The MG400 mock server runs inside Docker and simulates the real robot controller.

### Launch one instance
```bash
cd MG400_Mock
docker compose up
```

### Launch multiple instances (example: 3 robots)
```bash
docker compose up --scale dobot=3
```

### Identify container IP address

Docker containers run in a bridge network. To get the IP address of a specific container:
```bash
docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' docker-dobot-1
```

For multiple instances, container names will be:
- `docker-dobot-1`
- `docker-dobot-2`
- `docker-dobot-3`
- ...

### Shutdown
```bash
docker compose down
```

### Test (debug)
```bash
docker compose -f test-docker-compose.yml run test_dobot python3 -m unittest discover -s tests
```

---

## ⚠️ Notes

- **Do not edit files inside submodules directly.**  
  If you need to modify them, please fork the original repositories and update the submodule URL.

- This project is intended for teleoperation, simulation, and integration testing with ROS2 and Unity.

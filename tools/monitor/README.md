# MG400 Monitor Tools

Utilities for viewing MG400 teleoperation telemetry outside the main ROS2
runtime.

## Files

- `mg400_monitor_pyqt6.py`  
  PyQt6 monitor GUI. It can receive telemetry directly through ROS2 when
  available, or through the UDP bridge when running on macOS without ROS2.

- `monitor_bridge.py`  
  ROS2-to-UDP bridge intended to run on the Ubuntu/ROS2 side. It forwards
  telemetry to the monitor GUI and receives dashboard commands back over UDP.

- `ros_interface.json`  
  Topic/interface reference used by the monitor workflow.

- `requirements.txt`  
  Python dependencies for the monitor GUI.

## Run Monitor GUI

```bash
pip install -r tools/monitor/requirements.txt
python3 tools/monitor/mg400_monitor_pyqt6.py
```

## Run UDP Bridge

From a sourced ROS2 workspace:

```bash
python3 tools/monitor/monitor_bridge.py
```

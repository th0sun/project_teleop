# 🚀 MG400 Teleop System - Complete Setup Guide

## 📋 Overview
ระบบ Teleoperation สำหรับ Dobot MG400 ประกอบด้วย 3 ส่วนหลัก:
1. **macOS Monitor GUI** - แสดงข้อมูลและควบคุมแบบเรียลไทม์
2. **Ubuntu ROS 2 Node** - ประมวลผลหลักและเชื่อมต่อกับหุ่นยนต์
3. **Unity Simulator** - จำลองการควบคุม VR Controller

---

## 🍎 macOS Setup - Monitor GUI

### 1. Environment Setup
```bash
# Clone repository
cd ~
git clone https://github.com/th0sun/project_teleop_ws.git
cd project_teleop_ws

# Create Python virtual environment
python3 -m venv venv_monitor
source venv_monitor/bin/activate

# Install PyQt6 and dependencies
pip install PyQt6 matplotlib numpy rclpy

# Install ROS 2 Humble for macOS (optional, for simulation mode only)
brew install ros-humble
```

### 2. Network Configuration
```bash
# Set ROS domain for cross-VM communication
export ROS_DOMAIN_ID=0

# Add to ~/.zshrc for persistence
echo 'export ROS_DOMAIN_ID=0' >> ~/.zshrc
```

### 3. Run Monitor GUI
```bash
cd ~/project_teleop_ws/project_teleop
source ../venv_monitor/bin/activate
export ROS_DOMAIN_ID=0
python3 mg400_monitor_pyqt6.py
```

### 4. GUI Features
- **Dashboard Tab:** 4 joint graphs, 3D trajectory, control buttons
- **Data Flow Tab:** Live system architecture with animated data flow
- **Real-time Updates:** 125Hz feedback, command frequencies, latency visualization

---

## 🐧 Ubuntu Setup - ROS 2 Node

### 1. Install ROS 2 Humble
```bash
# Ubuntu 22.04
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-humble-desktop python3-argcomplete
```

### 2. Workspace Setup
```bash
cd ~
git clone https://github.com/th0sun/project_teleop_ws.git
cd project_teleop_ws

# Build ROS workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Source workspace
echo 'source ~/project_teleop_ws/install/setup.bash' >> ~/.bashrc
source ~/project_teleop_ws/install/setup.bash
```

### 3. Python Dependencies
```bash
pip install numpy matplotlib PyQt6
```

### 4. Network Configuration
```bash
# Set ROS domain for cross-VM communication
export ROS_DOMAIN_ID=0

# Add to ~/.bashrc
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc

# Optional: Set robot IP
export ROBOT_IP=192.168.1.6
```

### 5. Run ROS Node
```bash
cd ~/project_teleop_ws
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROBOT_IP=192.168.1.6
ros2 run mg400_controller vr_teleop_node
```

---

## 🌐 Cross-VM Communication Setup

### 1. Network Requirements
- macOS และ Ubuntu ต้องอยู่ในเครือข่ายเดียวกัน
- macOS IP: 192.168.1.x (เช่น 192.168.1.10)
- Ubuntu IP: 192.168.1.x (เช่น 192.168.1.20)
- Robot IP: 192.168.1.6

### 2. ROS 2 Domain Configuration
```bash
# บนทั้ง macOS และ Ubuntu
export ROS_DOMAIN_ID=0

# ตรวจสอบว่าเห็นกัน
ros2 topic list
ros2 node list
```

### 3. Firewall Configuration
```bash
# Ubuntu - เปิดพอร์ต ROS 2
sudo ufw allow 10000  # ROS-TCP-Connector
sudo ufw allow 30003  # Robot Command
sudo ufw allow 30004  # Robot Feedback

# macOS - อนุญาต ROS 2 ใน System Preferences
# System Preferences > Security & Privacy > Firewall > Firewall Options
```

---

## 🎮 Unity Simulator Setup

### 1. Unity Project Requirements
- Unity 2022.3 LTS
- ROS-TCP-Connector Asset Store package
- Meta Quest 3 SDK (สำหรับ VR)

### 2. ROS-TCP-Connector Configuration
```csharp
// ใน Unity -> ROS Settings
RosIPAddress: "192.168.1.20"  // Ubuntu VM IP
RosPort: 10000
```

### 3. Topics Configuration
ตาม `ros_interface.json`:
- **Publish:** `/unity/joint_cmd`, `/vr/suction_cmd`
- **Subscribe:** `/joint_states`, `/mg400/tool_vector_actual`

### 4. Run Unity
```bash
# เปิด Unity project
# ตั้งค่า ROS-TCP-Connector
# กด Play ใน Unity Editor
```

---

## 🤖 Robot Connection Setup

### 1. Physical Connection
```bash
# เชื่อมต่อ Ethernet จาก Ubuntu ไปยัง Dobot MG400
# Robot IP: 192.168.1.6 (default)
# ตรวจสอบการเชื่อมต่อ
ping 192.168.1.6
```

### 2. Robot Startup Sequence
```bash
# บน Ubuntu
cd ~/project_teleop_ws
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROBOT_IP=192.168.1.6

# เปิด ROS node
ros2 run mg400_controller vr_teleop_node

# ใน GUI บน macOS กดปุ่ม "Enable" เพื่อเปิดใช้งานหุ่นยนต์
```

### 3. Robot Status Check
```bash
# ตรวจสอบ topics
ros2 topic echo /mg400/robot_mode
ros2 topic echo /mg400/error_status
ros2 topic echo /joint_states
```

---

## 🔄 Complete Startup Sequence

### Step 1: Ubuntu (ROS Node)
```bash
cd ~/project_teleop_ws
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROBOT_IP=192.168.1.6
ros2 run mg400_controller vr_teleop_node
```

### Step 2: macOS (Monitor GUI)
```bash
cd ~/project_teleop_ws/project_teleop
source ../venv_monitor/bin/activate
export ROS_DOMAIN_ID=0
python3 mg400_monitor_pyqt6.py
```

### Step 3: Unity (Simulator)
```bash
# เปิด Unity project
# ตั้งค่า ROS-TCP-Connector -> Ubuntu IP
# กด Play
```

### Step 4: Robot Enable
```bash
# ใน GUI บน macOS:
# 1. ตรวจสอบว่าเห็น ROS topics (ไม่ใช่ SIMULATION)
# 2. กดปุ่ม "Enable" เพื่อเปิดหุ่นยนต์
# 3. ตรวจสอบว่า Robot Mode = 5 (ENABLED)
```

---

## 🛠️ Troubleshooting

### Common Issues

#### 1. ROS 2 Communication Issues
```bash
# ตรวจสอบ domain ID
echo $ROS_DOMAIN_ID

# ตรวจสอบ topics
ros2 topic list
ros2 topic hz /joint_states

# ตรวจสอบ nodes
ros2 node list
```

#### 2. Robot Connection Issues
```bash
# ตรวจสอบ IP connectivity
ping 192.168.1.6

# ตรวจสอบ ports
telnet 192.168.1.6 29999  # Dashboard
telnet 192.168.1.6 30003  # Command
telnet 192.168.1.6 30004  # Feedback
```

#### 3. GUI Issues
```bash
# ตรวจสอบ PyQt6 installation
python3 -c "import PyQt6; print('PyQt6 OK')"

# ตรวจสอบ virtual environment
which python3
python3 --version
```

#### 4. Unity Connection Issues
- ตรวจสอบว่า Ubuntu IP ถูกต้องใน Unity
- ตรวจสอบว่า ROS-TCP-Connector ทำงาน
- ตรวจสอบ firewall settings

### Debug Commands
```bash
# Monitor all topics
ros2 topic list -v

# Check topic data
ros2 topic echo /unity/joint_cmd
ros2 topic echo /joint_states

# Check node graph
ros2 run rqt_graph rqt_graph

# System monitor
htop
```

---

## 📊 Performance Monitoring

### Key Metrics
- **Command Rate:** ~25Hz (Unity → ROS → Robot)
- **Feedback Rate:** 125Hz (Robot → ROS)
- **Latency:** <50ms (Unity → Robot → Unity)
- **Data Flow:** แสดงใน Data Flow Tab

### Optimization Tips
1. **Network:** ใช้ Ethernet แทน WiFi
2. **ROS 2:** ใช้ `MultiThreadedExecutor`
3. **GUI:** ปิดฟีเจอร์ที่ไม่ใช้เพื่อลด load
4. **Unity:** ตั้งค่า target frame rate เป็น 90Hz

---

## 🎯 Success Criteria

ระบบทำงานถูกต้องเมื่อ:
1. ✅ macOS GUI แสดงข้อมูลจริง (ไม่ใช่ SIMULATION)
2. ✅ Unity ส่งคำสั่งไป ROS ได้
3. ✅ Robot เคลื่อนที่ตามคำสั่ง
4. ✅ Feedback loop ทำงาน (125Hz)
5. ✅ Data Flow Tab แสดงการไหลข้อมูลแบบ animated

---

## 📞 Support

ถ้าเจอปัญหา:
1. ตรวจสอบ log files ใน `~/project_teleop_ws/session_logs/`
2. ตรวจสอบ ROS 2 topics ด้วย `ros2 topic list`
3. ตรวจสอบ network connectivity
4. ดูที่ `mg400_monitor_pyqt6.py` สำหรับ GUI issues
5. ดูที่ `vr_teleop_node.py` สำหรับ ROS node issues

**Happy Teleoping! 🚀**

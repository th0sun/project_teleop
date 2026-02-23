#!/bin/bash

# ==========================================
# MG400 Tmux Interactive Launcher
# ==========================================

# 1. แสดงเมนู
clear
echo "=========================================="
echo " 🤖 MG400 Teleop Manager (Tmux) "
echo "=========================================="
echo "1. หุ่นจริง + Simulator (rviz & unity_simulator)"
echo "2. แบบจำลอง (Mock+Docker) + Simulator"
echo "3. หุ่นจริง + Unity (ros_tcp_endpoint)"
echo "4. แบบจำลอง (Mock+Docker) + Unity"
echo "=========================================="
read -p "Select Mode (1-4): " MODE

# ตรวจสอบ Input
if [[ ! "$MODE" =~ ^[1-4]$ ]]; then
    echo "❌ Invalid Selection. Exiting."
    exit 1
fi

# 2. ตั้งค่า Network & Docker
if [ "$MODE" = "2" ] || [ "$MODE" = "4" ]; then
    echo "🐳 Starting Docker Mock Environment..."
    export ROBOT_IP="172.10.0.2"
    docker compose -f MG400_Mock/docker/docker-compose.yml up -d
else
    echo "🌍 Connecting to Real Robot..."
    export ROBOT_IP="192.168.1.6"
fi

# 3. เตรียมคำสั่งสำหรับแต่ละ Pane
CMD_NODE="export ROBOT_IP=$ROBOT_IP && source install/setup.bash && ros2 run mg400_controller vr_teleop_node"
CMD_RVIZ="source install/setup.bash && ros2 launch mg400_bringup main.launch.py"

if [ "$MODE" = "3" ] || [ "$MODE" = "4" ]; then
    CMD_EXTRA="source install/setup.bash && ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0"
else
    CMD_EXTRA="source install/setup.bash && ros2 run mg400_simulator unity_simulator"
fi

# 4. สร้าง Tmux Session
SESSION_NAME="teleop_session"

# ตรวจสอบว่ามี Session อยู่แล้วหรือไม่ ถ้ามีให้ kill ก่อน
tmux has-session -t $SESSION_NAME 2>/dev/null
if [ $? == 0 ]; then
    echo "🗑️  Killing existing tmux session..."
    tmux kill-session -t $SESSION_NAME
fi

echo "🚀 Launching Tmux Session..."

# สร้างหน้าต่างหลัก (Pane 0 - ซ้ายเต็ม) - รัน Node หลัก
tmux new-session -d -s $SESSION_NAME -n "teleop" "$CMD_NODE"

# แบ่งครึ่งแนวนอน (Pane 1 - ขวาบน) - รัน RViz
tmux split-window -h -t $SESSION_NAME "$CMD_RVIZ"

# แบ่งครึ่งล่างของด้านขวา (Pane 2 - ขวาล่าง) - รัน TCP Endpoint หรือ Simulator
tmux split-window -v -t $SESSION_NAME "$CMD_EXTRA"

# ถ้าโหมดเป็น 2 หรือ 4 ให้เพิ่มช่องสำหรับดู Log จาก Docker
if [ "$MODE" = "2" ] || [ "$MODE" = "4" ]; then
    CMD_DOCKER_LOGS="docker compose -f MG400_Mock/docker/docker-compose.yml logs -f"
    # แบ่งครึ่งของ Pane 0 (ซ้าย) ให้เป็นบน-ล่าง สำหรับ Docker
    tmux split-window -v -t $SESSION_NAME:0.0 "$CMD_DOCKER_LOGS"
fi

# เปิดโหมด Mouse เพื่อให้คลิกเลือก Pane สะดวกๆ ได้
tmux set -g mouse on

# ส่งข้อความเตือนความจำไว้ด้านล่างจอ
tmux display-message -t $SESSION_NAME "💡 Tip: Click pane, press Ctrl+C, then Up Arrow(↑) + Enter to restart."

# Attach เข้าไปดู
tmux attach -t $SESSION_NAME

# เมื่อกดออก (detached/killed) ถ้าเป็น Mode 2 หรือ 4 ให้ถามว่าจะปิด Docker ไหม
if [ "$MODE" = "2" ] || [ "$MODE" = "4" ]; then
    echo ""
    read -p "🛑 Do you want to stop Docker Mock? (y/N): " STOP_DOCKER
    if [[ "$STOP_DOCKER" =~ ^[Yy]$ ]]; then
        echo "🐳 Stopping Docker..."
        docker compose -f MG400_Mock/docker/docker-compose.yml down
    fi
fi

echo "✅ Exited MG400 Teleop Manager."

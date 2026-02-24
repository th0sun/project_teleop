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
#    ✅ เพิ่ม "; exec bash" ต่อท้ายทุก command
#    -> pane จะ "ค้าง" หลัง process ตาย, ไม่ปิดทิ้ง
#    -> สามารถกด ↑ Enter เพื่อรันใหม่, หรือ copy error ได้เลย

_src="source install/setup.bash"
_ip="export ROBOT_IP=$ROBOT_IP"
_stay="; exec bash"

CMD_NODE="$_ip && $_src && ros2 run mg400_controller vr_teleop_node$_stay"
CMD_RVIZ="$_src && ros2 launch mg400_bringup main.launch.py$_stay"
CMD_MONITOR="$_src && ros2 run mg400_controller monitor_gui$_stay"

if [ "$MODE" = "3" ] || [ "$MODE" = "4" ]; then
    CMD_EXTRA="$_src && ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0$_stay"
else
    CMD_EXTRA="$_src && ros2 run mg400_simulator unity_simulator$_stay"
fi

# 4. สร้าง Tmux Session
SESSION="mg400"

# ตรวจสอบว่ามี Session อยู่แล้วหรือไม่ ถ้ามีให้ kill ก่อน
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "🗑️  Killing existing session..."
    tmux kill-session -t "$SESSION"
fi

echo "🚀 Launching Tmux Session..."

# --- Layout ---
# Window: teleop (default)
#  ┌──────────────┬──────────────┐
#  │              │   RViz       │
#  │  Teleop Node ├──────────────┤
#  │              │  TCP/Sim     │
#  ├──────────────┴──────────────┤
#  │         Monitor GUI         │
#  └─────────────────────────────┘

# Pane 0: Teleop Node (ซ้าย, ใหญ่)
tmux new-session -d -s "$SESSION" -n "teleop" -x 220 -y 55
tmux send-keys -t "$SESSION":0.0 "echo '[ TELEOP NODE ] Ctrl+C -> stops, stays. Up+Enter to restart.' && $CMD_NODE" Enter

# Pane 1: RViz (ขวาบน) — split right
tmux split-window -h -t "$SESSION":0.0
tmux send-keys -t "$SESSION":0.1 "echo '[ RVIZ ] ' && $CMD_RVIZ" Enter

# Pane 2: TCP Endpoint หรือ Simulator (ขวาล่าง)
tmux split-window -v -t "$SESSION":0.1
tmux send-keys -t "$SESSION":0.2 "echo '[ ENDPOINT/SIM ] ' && $CMD_EXTRA" Enter

# Pane 3: Monitor GUI (ด้านล่างของ Teleop Node)
tmux split-window -v -t "$SESSION":0.0 -p 25
tmux send-keys -t "$SESSION":0.3 "echo '[ MONITOR GUI ] ' && $CMD_MONITOR" Enter

# Docker Logs pane (โหมด Mock เท่านั้น)
if [ "$MODE" = "2" ] || [ "$MODE" = "4" ]; then
    tmux split-window -v -t "$SESSION":0.0 -p 30
    tmux send-keys -t "$SESSION":0.4 "docker compose -f MG400_Mock/docker/docker-compose.yml logs -f; exec bash" Enter
fi

# 5. Global Tmux Settings
tmux set -g mouse on

# ✅ การ Copy ด้วย Mouse: ต้องกด Shift+Click แล้วลากเพื่อ Select text ปกติ
# ✅ ถ้าต้องการ Scroll กอปปี้ normal terminal: กด Shift+PageUp / Shift+PageDown

# Set pane titles (แสดง label ใน status bar)
tmux set -g pane-border-status top
tmux set -g pane-border-format " #{pane_index}: #{pane_title} "
tmux select-pane -t "$SESSION":0.0 -T "🤖 Teleop Node"
tmux select-pane -t "$SESSION":0.1 -T "📐 RViz"
tmux select-pane -t "$SESSION":0.2 -T "🔌 TCP/Simulator"
tmux select-pane -t "$SESSION":0.3 -T "� Monitor GUI"

# 6. Status bar hint
tmux set -g status-right " 💡 Ctrl+C=stop (stays) | ↑+Enter=restart | Shift+drag=copy "
tmux set -g status-right-length 70
tmux set -g status-style "bg=#1a1a2e fg=#aaaaaa"

# โฟกัสไปที่ Pane หลัก (Teleop Node)
tmux select-pane -t "$SESSION":0.0

# Attach
tmux attach -t "$SESSION"

# เมื่อ detach ออกมา ถ้าเป็น Mock mode ถามว่าจะปิด Docker ไหม
if [ "$MODE" = "2" ] || [ "$MODE" = "4" ]; then
    echo ""
    read -p "🛑 Stop Docker Mock? (y/N): " STOP_DOCKER
    if [[ "$STOP_DOCKER" =~ ^[Yy]$ ]]; then
        echo "🐳 Stopping Docker..."
        docker compose -f MG400_Mock/docker/docker-compose.yml down
    fi
fi

echo "✅ Exited MG400 Teleop Manager."

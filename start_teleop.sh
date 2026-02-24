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

if [[ ! "$MODE" =~ ^[1-4]$ ]]; then
    echo "❌ Invalid. Exiting."
    exit 1
fi

# 2. Network & Docker
if [ "$MODE" = "2" ] || [ "$MODE" = "4" ]; then
    echo "🐳 Starting Docker Mock..."
    export ROBOT_IP="172.10.0.2"
    docker compose -f MG400_Mock/docker/docker-compose.yml up -d
else
    echo "🌍 Connecting to Real Robot..."
    export ROBOT_IP="192.168.1.6"
fi

# 3. Commands
WS="$HOME/project_teleop_ws"
_src="cd $WS && source install/setup.bash"
_ip="export ROBOT_IP=$ROBOT_IP"

# note: exec bash ทำให้ pane ค้างไว้หลัง node หยุด, กด ↑ Enter รันใหม่ได้
CMD_NODE="$_ip && $_src && ros2 run mg400_controller vr_teleop_node; exec bash"
CMD_RVIZ="$_src && ros2 launch mg400_bringup main.launch.py; exec bash"
CMD_MONITOR="$_src && ros2 run mg400_controller monitor_gui; exec bash"

if [ "$MODE" = "3" ] || [ "$MODE" = "4" ]; then
    CMD_EXTRA="$_src && ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0; exec bash"
else
    CMD_EXTRA="$_src && ros2 run mg400_simulator unity_simulator; exec bash"
fi

# 4. สร้าง Tmux
SESSION="mg400"
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "🗑️  Killing existing session..."
    tmux kill-session -t "$SESSION"
fi

echo "🚀 Launching..."

# --- สร้างทีละ Pane และจับ ID จริงๆ ---
# Layout:
#  ┌──────────────┬──────────────┐
#  │  Teleop Node │   RViz       │
#  │              ├──────────────┤
#  ├──────────────│  TCP/Sim     │
#  │  Monitor GUI └──────────────┤
#  └─────────────────────────────┘

# Pane A: Teleop Node
PANE_A=$(tmux new-session -d -s "$SESSION" -n "teleop" -x 220 -y 55 -P -F '#{pane_id}')
tmux send-keys -t "$PANE_A" "$CMD_NODE" Enter

# Pane B: RViz (ขวา)
PANE_B=$(tmux split-window -h -t "$PANE_A" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_B" "$CMD_RVIZ" Enter

# Pane C: TCP/Simulator (ขวาล่าง)
PANE_C=$(tmux split-window -v -t "$PANE_B" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_C" "$CMD_EXTRA" Enter

# Pane D: Monitor GUI (ซ้ายล่าง)
PANE_D=$(tmux split-window -v -t "$PANE_A" -l 12 -P -F '#{pane_id}')
tmux send-keys -t "$PANE_D" "$CMD_MONITOR" Enter

# Pane E: Docker Logs (Mock mode เท่านั้น)
if [ "$MODE" = "2" ] || [ "$MODE" = "4" ]; then
    PANE_E=$(tmux split-window -v -t "$PANE_A" -l 8 -P -F '#{pane_id}')
    tmux send-keys -t "$PANE_E" "docker compose -f MG400_Mock/docker/docker-compose.yml logs -f; exec bash" Enter
fi

# 5. Settings
tmux set -g mouse on
tmux set -g pane-border-status top
tmux set -g pane-border-format " #{pane_index}: #{pane_title} "
tmux select-pane -t "$PANE_A" -T "🤖 Teleop"
tmux select-pane -t "$PANE_B" -T "📐 RViz"
tmux select-pane -t "$PANE_C" -T "🔌 TCP/Sim"
tmux select-pane -t "$PANE_D" -T "📊 Monitor GUI"

tmux set -g status-right " 💡 Ctrl+C=stop | ↑Enter=restart | Shift+drag=copy | Ctrl+D=close pane | kill-server=exit all "
tmux set -g status-right-length 90
tmux set -g status-style "bg=#1a1a2e fg=#aaaaaa"

# โฟกัส Teleop Node
tmux select-pane -t "$PANE_A"
tmux attach -t "$SESSION"

# เมื่อ detach ออก
if [ "$MODE" = "2" ] || [ "$MODE" = "4" ]; then
    echo ""
    read -p "🛑 Stop Docker Mock? (y/N): " STOP_DOCKER
    if [[ "$STOP_DOCKER" =~ ^[Yy]$ ]]; then
        docker compose -f MG400_Mock/docker/docker-compose.yml down
        echo "🐳 Docker stopped."
    fi
fi

echo "✅ Done."

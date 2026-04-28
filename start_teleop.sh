#!/bin/bash

# ==========================================
# MG400 Tmux Interactive Launcher
# ==========================================

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$REPO_DIR/install/setup.bash" ]; then
    # Repo checked out directly as the colcon workspace root.
    WS="$REPO_DIR"
else
    # Repo checked out inside a parent colcon workspace.
    WS="$(cd "$REPO_DIR/.." && pwd)"
fi
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/$ROS_DISTRO/setup.bash}"

if ! command -v tmux >/dev/null 2>&1; then
    echo "❌ tmux not found. Install with: sudo apt install tmux"
    exit 1
fi

if [ ! -f "$ROS_SETUP" ]; then
    echo "❌ ROS setup not found: $ROS_SETUP"
    echo "   Set ROS_DISTRO or ROS_SETUP, e.g. ROS_DISTRO=jazzy ./start_teleop.sh"
    exit 1
fi

if [ ! -f "$WS/install/setup.bash" ]; then
    echo "❌ Workspace is not built yet: $WS/install/setup.bash"
    echo "   Run:"
    echo "     cd $WS"
    echo "     source $ROS_SETUP"
    echo "     colcon build --symlink-install"
    exit 1
fi

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
    if ! command -v docker >/dev/null 2>&1; then
        echo "❌ docker not found. Install Docker first."
        exit 1
    fi
    export ROBOT_IP="172.10.0.2"
    MOCK_COMPOSE="$REPO_DIR/MG400_Mock/docker-compose.yml"
    if [ ! -f "$MOCK_COMPOSE" ]; then
        MOCK_COMPOSE="$REPO_DIR/MG400_Mock/docker/docker-compose.yml"
    fi
    if [ ! -f "$MOCK_COMPOSE" ]; then
        echo "❌ MG400 Mock docker compose file not found."
        echo "   Checked:"
        echo "     $REPO_DIR/MG400_Mock/docker-compose.yml"
        echo "     $REPO_DIR/MG400_Mock/docker/docker-compose.yml"
        exit 1
    fi
    docker compose -f "$MOCK_COMPOSE" up -d
else
    echo "🌍 Connecting to Real Robot..."
    export ROBOT_IP="192.168.1.6"
fi

# 3. Commands
_src="source \"$ROS_SETUP\" && cd \"$WS\" && source install/setup.bash"
_ip="export ROBOT_IP=$ROBOT_IP"

# note: exec bash ทำให้ pane ค้างไว้หลัง node หยุด, กด ↑ Enter รันใหม่ได้
CMD_NODE="$_ip && $_src && ros2 run mg400_controller vr_teleop_node; exec bash"
CMD_RVIZ="$_src && ros2 launch mg400_bringup main.launch.py; exec bash"
CMD_BRIDGE="$_src && python3 $REPO_DIR/tools/monitor/monitor_bridge.py; exec bash"

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

# Pane D: Monitor Bridge (ซ้ายล่าง) — UDP telemetry → Mac
PANE_D=$(tmux split-window -v -t "$PANE_A" -l 12 -P -F '#{pane_id}')
tmux send-keys -t "$PANE_D" "$CMD_BRIDGE" Enter

# Pane E: Docker Logs (Mock mode เท่านั้น)
if [ "$MODE" = "2" ] || [ "$MODE" = "4" ]; then
    PANE_E=$(tmux split-window -v -t "$PANE_A" -l 8 -P -F '#{pane_id}')
    tmux send-keys -t "$PANE_E" "docker compose -f \"$MOCK_COMPOSE\" logs -f; exec bash" Enter
fi

# 5. Settings
tmux set -g mouse on
tmux set -g pane-border-status top
tmux set -g pane-border-format " #{pane_index}: #{pane_title} "
tmux select-pane -t "$PANE_A" -T "🤖 Teleop"
tmux select-pane -t "$PANE_B" -T "📐 RViz"
tmux select-pane -t "$PANE_C" -T "🔌 TCP/Sim"
tmux select-pane -t "$PANE_D" -T "🌉 UDP Bridge"

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
        docker compose -f "$MOCK_COMPOSE" down
        echo "🐳 Docker stopped."
    fi
fi

echo "✅ Done."

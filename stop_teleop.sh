#!/bin/bash
# stop_teleop.sh — ปิด MG400 Tmux Session ทั้งหมด

SESSION="mg400"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "🛑 Killing tmux session: $SESSION"
    tmux kill-session -t "$SESSION"
    echo "✅ Done."
else
    echo "ℹ️  No session named '$SESSION' found."
fi

# ถ้ามี Docker Mock รันอยู่ ให้ถามด้วย
if docker compose -f MG400_Mock/docker/docker-compose.yml ps -q 2>/dev/null | grep -q .; then
    read -p "🐳 Stop Docker Mock too? (y/N): " STOP_DOCKER
    if [[ "$STOP_DOCKER" =~ ^[Yy]$ ]]; then
        docker compose -f MG400_Mock/docker/docker-compose.yml down
        echo "🐳 Docker stopped."
    fi
fi

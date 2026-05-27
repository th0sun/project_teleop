#!/usr/bin/env bash
# ==========================================
# Project Teleop — Docker ROS2 Jazzy Launcher (macOS dev)
# ==========================================
# Spins up the project_teleop ROS2 stack inside a docker container plus
# a tmux session of panes. For a native (host-side) tmux launcher on a
# Linux box with ROS2 already sourced, use ./scripts/start_teleop.sh instead.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="$(cd "${REPO_DIR}/.." && pwd)"
IMAGE="${PROJECT_TELEOP_JAZZY_IMAGE:-project-teleop-ros2-jazzy:latest}"
CONTAINER_NAME="${PROJECT_TELEOP_CONTAINER:-project_teleop_ros2_mac}"
ROS_TCP_PORT="${ROS_TCP_PORT:-10000}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
TELEOP_CONTROL_MODE="${TELEOP_CONTROL_MODE:-legacy}"
PROJECT_TELEOP_LOG_TZ="${PROJECT_TELEOP_LOG_TZ:-${TZ:-Asia/Bangkok}}"
RVIZ_DISPLAY="${RVIZ_DISPLAY:-host.docker.internal:0}"
RVIZ_NOVNC_PORT="${RVIZ_NOVNC_PORT:-6080}"
RVIZ_URL="http://127.0.0.1:${RVIZ_NOVNC_PORT}/vnc.html?autoconnect=true&resize=remote"
IMAGE_FINGERPRINT_FILE="${WORKSPACE_ROOT}/.ros_jazzy_docker/image.fingerprint"
REAL_ROBOT_IP="${ROBOT_IP:-192.168.1.6}"
MOCK_ROBOT_IP="172.10.0.2"
TMUX=(tmux -f /dev/null -L mg400_teleop_mac)
RUN_LOG_DIR=""

usage() {
  cat <<EOF
Usage:
  ./startup                 interactive tmux menu
  ./startup real-sim        real MG400 + Mac simulator
  ./startup mock-sim        MG400_Mock + Mac simulator
  ./startup real-unity      real MG400 + Unity
  ./startup mock-unity      MG400_Mock + Unity
  ./startup test            Docker ROS2 Jazzy functional tests

Unity / Mac simulator connection:
  Host: 127.0.0.1
  Port: ${ROS_TCP_PORT}

Control source:
  /unity/joint_cmd drives the robot.
  /unity/teleop_sample is kept for logging and joint_cmd identity sync only.

RViz on Mac:
  Default: RViz runs inside Docker virtual display and opens in browser:
    ${RVIZ_URL}
  Override noVNC port if needed:
    RVIZ_NOVNC_PORT=6081 ./startup mock-sim

Rebuild controls:
  PROJECT_TELEOP_REBUILD_IMAGE=1 ./startup mock-sim
  PROJECT_TELEOP_REBUILD_WS=1 ./startup mock-sim
EOF
}

require_tool() {
  local tool="$1"
  local install_hint="$2"
  if ! command -v "${tool}" >/dev/null 2>&1; then
    echo "${tool} not found. ${install_hint}" >&2
    exit 1
  fi
}

mock_compose_file() {
  local compose="${REPO_DIR}/MG400_Mock/docker-compose.yml"
  if [[ -f "${compose}" ]]; then
    echo "${compose}"
    return
  fi
  echo "MG400 mock docker-compose.yml not found" >&2
  exit 1
}

build_image() {
  DOCKER_BUILDKIT="${DOCKER_BUILDKIT:-1}" docker build \
    -f "${REPO_DIR}/tools/ros_jazzy/Dockerfile" \
    -t "${IMAGE}" \
    "${REPO_DIR}"
  save_image_fingerprint "$(image_fingerprint)"
}

image_fingerprint() {
  {
    printf 'Dockerfile '
    shasum -a 256 "${REPO_DIR}/tools/ros_jazzy/Dockerfile" | awk '{print $1}'
    printf 'entrypoint.sh '
    shasum -a 256 "${REPO_DIR}/tools/ros_jazzy/entrypoint.sh" | awk '{print $1}'
  } | shasum -a 256 | awk '{print $1}'
}

save_image_fingerprint() {
  local fingerprint="$1"
  mkdir -p "$(dirname "${IMAGE_FINGERPRINT_FILE}")"
  printf '%s\n' "${fingerprint}" > "${IMAGE_FINGERPRINT_FILE}"
}

image_has_runtime_requirements() {
  docker run --rm --entrypoint bash "${IMAGE}" -lc \
    'source /opt/ros/jazzy/setup.bash && command -v rviz2 >/dev/null && command -v Xvfb >/dev/null && command -v x11vnc >/dev/null && command -v websockify >/dev/null && test -d /opt/ros/jazzy/share/robot_state_publisher && grep -q "rviz-novnc)" /usr/local/bin/project-teleop-jazzy'
}

ensure_image() {
  local current_fingerprint stored_fingerprint
  current_fingerprint="$(image_fingerprint)"

  if [[ "${PROJECT_TELEOP_REBUILD_IMAGE:-0}" == "1" ]]; then
    echo "Rebuilding ROS2 Jazzy image because PROJECT_TELEOP_REBUILD_IMAGE=1"
    build_image
    return
  fi

  if ! docker inspect --type image "${IMAGE}" >/dev/null 2>&1; then
    echo "ROS2 Jazzy image not found; building ${IMAGE}"
    build_image
    return
  fi

  if [[ -f "${IMAGE_FINGERPRINT_FILE}" ]]; then
    stored_fingerprint="$(<"${IMAGE_FINGERPRINT_FILE}")"
    if [[ "${stored_fingerprint}" == "${current_fingerprint}" ]]; then
      echo "Using cached ROS2 Jazzy image: ${IMAGE}"
      return
    fi

    echo "ROS2 Jazzy image inputs changed; rebuilding ${IMAGE}"
    build_image
    return
  fi

  if image_has_runtime_requirements; then
    echo "Using existing ROS2 Jazzy image: ${IMAGE}"
    save_image_fingerprint "${current_fingerprint}"
    return
  fi

  echo "ROS2 Jazzy image is missing runtime requirements; rebuilding ${IMAGE}"
  build_image
}

start_ros_container() {
  local robot_ip="$1"
  local docker_network="${2:-}"

  docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

  local docker_args=(
    -d --init
    --name "${CONTAINER_NAME}"
  )
  if [[ -n "${docker_network}" ]]; then
    docker_args+=(--network "${docker_network}")
  fi
  docker_args+=(
    -e ROBOT_IP="${robot_ip}"
    -e ROS_TCP_PORT="${ROS_TCP_PORT}"
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
    -e RVIZ_NOVNC_PORT="${RVIZ_NOVNC_PORT}"
    -e TZ="${PROJECT_TELEOP_LOG_TZ}"
    -e PROJECT_TELEOP_LOG_TZ="${PROJECT_TELEOP_LOG_TZ}"
    -p "${ROS_TCP_PORT}:${ROS_TCP_PORT}"
    -p "${RVIZ_NOVNC_PORT}:${RVIZ_NOVNC_PORT}"
    -v "${WORKSPACE_ROOT}:/project_teleop_ws"
    -w /project_teleop_ws/project_teleop
    --entrypoint bash
    "${IMAGE}"
    -lc 'trap "exit 0" TERM INT; while true; do sleep 3600; done'
  )

  docker run "${docker_args[@]}" >/dev/null

  ensure_workspace_built
}

ensure_workspace_built() {
  if [[ "${PROJECT_TELEOP_REBUILD_WS:-0}" == "1" ]]; then
    echo "Rebuilding ROS2 workspace because PROJECT_TELEOP_REBUILD_WS=1"
    docker exec "${CONTAINER_NAME}" project-teleop-jazzy build
    return
  fi

  if docker exec "${CONTAINER_NAME}" bash -lc 'test -f /project_teleop_ws/.ros_jazzy_docker/install/setup.bash'; then
    echo "Using cached ROS2 workspace build"
    return
  fi

  echo "ROS2 workspace install not found; building once"
  docker exec "${CONTAINER_NAME}" project-teleop-jazzy build
}

tmux_cmd() {
  local title="$1"
  local command="$2"
  printf 'printf "\\033]2;%s\\033\\\\"; %s; echo; echo "Pane shell kept open. Ctrl+D closes this pane."; exec "${SHELL:-/bin/zsh}"\n' \
    "${title}" "${command}"
}

pipe_pane_log() {
  local pane="$1"
  local name="$2"
  "${TMUX[@]}" pipe-pane -o -t "${pane}" "cat >> '${RUN_LOG_DIR}/${name}.log'"
}

teleop_command() {
  local cmd="docker exec -it ${CONTAINER_NAME} project-teleop-jazzy teleop"
  case "${TELEOP_CONTROL_MODE}" in
    legacy|joint_cmd|v1) ;;
    *)
      echo "Invalid TELEOP_CONTROL_MODE=${TELEOP_CONTROL_MODE}; use legacy, joint_cmd, or v1" >&2
      exit 2
      ;;
  esac
  echo "${cmd}"
}

rviz_command() {
  cat <<EOF
printf 'RViz noVNC URL: ${RVIZ_URL}\\n';
(sleep 4; open '${RVIZ_URL}' >/dev/null 2>&1) &
docker exec -it \
  -e RVIZ_NOVNC_PORT='${RVIZ_NOVNC_PORT}' \
  ${CONTAINER_NAME} project-teleop-jazzy rviz-novnc
EOF
}

launch_tmux() {
  local mode_label="$1"
  local client_kind="$2"
  local mock_compose="${3:-}"

  local session="mg400"
  RUN_LOG_DIR="${REPO_DIR}/logs/startup_sessions/$(date +%Y%m%d_%H%M%S)"
  mkdir -p "${RUN_LOG_DIR}"
  if "${TMUX[@]}" has-session -t "${session}" 2>/dev/null; then
    "${TMUX[@]}" kill-session -t "${session}"
  fi

  local pane_a pane_b pane_c pane_d pane_e pane_f
  pane_a=$("${TMUX[@]}" new-session -d -s "${session}" -n "teleop" -P -F '#{pane_id}')
  pipe_pane_log "${pane_a}" "teleop"
  "${TMUX[@]}" send-keys -t "${pane_a}" "$(tmux_cmd Teleop "$(teleop_command)")" Enter

  pane_b=$("${TMUX[@]}" split-window -h -t "${pane_a}" -P -F '#{pane_id}')
  pipe_pane_log "${pane_b}" "endpoint"
  "${TMUX[@]}" send-keys -t "${pane_b}" "$(tmux_cmd Endpoint "docker exec -it ${CONTAINER_NAME} project-teleop-jazzy endpoint")" Enter

  pane_c=$("${TMUX[@]}" split-window -v -t "${pane_b}" -P -F '#{pane_id}')
  pipe_pane_log "${pane_c}" "${client_kind}"
  if [[ "${client_kind}" == "sim" ]]; then
    "${TMUX[@]}" send-keys -t "${pane_c}" "$(tmux_cmd "Mac Simulator" "cd '${REPO_DIR}' && ./tools/mg400_simulator/run.sh")" Enter
  else
    "${TMUX[@]}" send-keys -t "${pane_c}" "$(tmux_cmd Unity "printf 'Unity connection\\n  Host: 127.0.0.1\\n  Port: ${ROS_TCP_PORT}\\n\\nKeep this pane for notes/log checks.\\n'")" Enter
  fi

  pane_f=$("${TMUX[@]}" split-window -v -t "${pane_b}" -l 14 -P -F '#{pane_id}')
  pipe_pane_log "${pane_f}" "rviz"
  "${TMUX[@]}" send-keys -t "${pane_f}" "$(tmux_cmd RViz "$(rviz_command)")" Enter

  pane_d=$("${TMUX[@]}" split-window -v -t "${pane_a}" -l 12 -P -F '#{pane_id}')
  pipe_pane_log "${pane_d}" "status"
  "${TMUX[@]}" send-keys -t "${pane_d}" "while true; do clear; echo '${mode_label}'; echo 'control: ${TELEOP_CONTROL_MODE}'; echo; echo 'Unity/Mac simulator -> 127.0.0.1:${ROS_TCP_PORT}'; echo; echo 'ROS nodes:'; docker exec ${CONTAINER_NAME} bash -lc 'set +u; source /opt/ros/jazzy/setup.bash; source /project_teleop_ws/.ros_jazzy_docker/install/setup.bash; ros2 node list' 2>/dev/null || true; echo; echo 'latest logs:'; ls -lt '${REPO_DIR}/logs/teleop_sessions' 2>/dev/null | head; sleep 2; done; exec \"\${SHELL:-/bin/zsh}\"" Enter

  if [[ -n "${mock_compose}" ]]; then
    pane_e=$("${TMUX[@]}" split-window -v -t "${pane_a}" -l 8 -P -F '#{pane_id}')
    pipe_pane_log "${pane_e}" "mock"
    "${TMUX[@]}" send-keys -t "${pane_e}" "docker compose -f '${mock_compose}' logs -f; echo; echo 'Pane shell kept open. Ctrl+D closes this pane.'; exec \"\${SHELL:-/bin/zsh}\"" Enter
  fi

  "${TMUX[@]}" set -g remain-on-exit off
  "${TMUX[@]}" set -g mouse on
  "${TMUX[@]}" set -g mode-keys vi
  "${TMUX[@]}" set -g set-clipboard on
  "${TMUX[@]}" bind-key -T copy-mode-vi y send-keys -X copy-pipe-and-cancel "pbcopy"
  "${TMUX[@]}" bind-key -T copy-mode-vi Enter send-keys -X copy-pipe-and-cancel "pbcopy"
  "${TMUX[@]}" bind-key -T copy-mode-vi MouseDragEnd1Pane send-keys -X copy-pipe-and-cancel "pbcopy"
  "${TMUX[@]}" set -g pane-border-status top
  "${TMUX[@]}" set -g pane-border-format " #{pane_index}: #{pane_title} "
  "${TMUX[@]}" set -g status-right " Ctrl+C=stop | Up+Enter=restart | Ctrl+D=close pane | copy: drag or Ctrl-b [ then y | detach: Ctrl-b d "
  "${TMUX[@]}" set -g status-right-length 120
  "${TMUX[@]}" set -g status-style "bg=#1a1a2e fg=#aaaaaa"
  "${TMUX[@]}" select-pane -t "${pane_a}" -T "Teleop"
  "${TMUX[@]}" select-pane -t "${pane_b}" -T "Endpoint"
  "${TMUX[@]}" select-pane -t "${pane_c}" -T "${client_kind}"
  "${TMUX[@]}" select-pane -t "${pane_d}" -T "Status"
  "${TMUX[@]}" select-pane -t "${pane_f}" -T "RViz"
  "${TMUX[@]}" select-pane -t "${pane_a}"
  echo "Startup pane logs: ${RUN_LOG_DIR}"
  "${TMUX[@]}" attach -t "${session}"
}

cleanup_after_tmux() {
  local mock_compose="${1:-}"
  echo
  read -r -p "Stop ROS2 Docker container? (Y/n): " stop_ros
  if [[ ! "${stop_ros}" =~ ^[Nn]$ ]]; then
    docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${mock_compose}" ]]; then
    read -r -p "Stop MG400_Mock? (y/N): " stop_mock
    if [[ "${stop_mock}" =~ ^[Yy]$ ]]; then
      docker compose -f "${mock_compose}" down
    fi
  fi
}

run_stack() {
  local robot_kind="$1"
  local client_kind="$2"
  require_tool docker "Install/start Docker Desktop first."
  require_tool tmux "Install with: brew install tmux"

  local robot_ip="${REAL_ROBOT_IP}"
  local docker_network=""
  local mock_compose=""
  local mode_label=""

  if [[ "${robot_kind}" == "mock" ]]; then
    mock_compose="$(mock_compose_file)"
    echo "Starting MG400_Mock"
    docker compose -f "${mock_compose}" up -d
    robot_ip="${MOCK_ROBOT_IP}"
    docker_network="dobot_network"
  fi

  if [[ "${robot_kind}" == "real" ]]; then
    mode_label="Real MG400"
  else
    mode_label="MG400_Mock"
  fi
  if [[ "${client_kind}" == "sim" ]]; then
    mode_label="${mode_label} + Mac simulator"
  else
    mode_label="${mode_label} + Unity"
  fi
  mode_label="${mode_label} + JointCmd control + JSON logs"

  ensure_image
  echo "Starting ROS2 container: ${CONTAINER_NAME}"
  echo "ROBOT_IP=${robot_ip}"
  echo "Client connects to 127.0.0.1:${ROS_TCP_PORT}"
  start_ros_container "${robot_ip}" "${docker_network}"
  launch_tmux "${mode_label}" "${client_kind}" "${mock_compose}"
  cleanup_after_tmux "${mock_compose}"
}

run_test() {
  require_tool docker "Install/start Docker Desktop first."
  bash "${REPO_DIR}/tools/ros_jazzy/test_in_docker.sh" function
}

mode="${1:-}"
if [[ -z "${mode}" ]]; then
  echo "=========================================="
  echo " MG400 Teleop Startup (Mac + tmux)"
  echo "=========================================="
  echo "1. Real MG400 + Mac simulator"
  echo "2. MG400_Mock + Mac simulator"
  echo "3. Real MG400 + Unity"
  echo "4. MG400_Mock + Unity"
  echo "5. Functional tests only"
  echo "=========================================="
  read -r -p "Select Mode (1-5): " choice
  case "${choice}" in
    1) mode="real-sim" ;;
    2) mode="mock-sim" ;;
    3) mode="real-unity" ;;
    4) mode="mock-unity" ;;
    5) mode="test" ;;
    *) echo "Invalid mode" >&2; exit 2 ;;
  esac
fi

case "${mode}" in
  real-sim|real-simulator) run_stack real sim ;;
  mock-sim|mock-simulator) run_stack mock sim ;;
  real-unity|real) run_stack real unity ;;
  mock-unity|mock) run_stack mock unity ;;
  test) run_test ;;
  -h|--help|help) usage ;;
  *) usage; exit 2 ;;
esac

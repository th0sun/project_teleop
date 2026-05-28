#!/usr/bin/env bash
# Native SwiftUI launcher backend for the same ROS2 stack as ./scripts/startup.sh.
# This script intentionally does not open Terminal or tmux panes.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORKSPACE_ROOT="$(cd "${REPO_DIR}/.." && pwd)"

IMAGE="${PROJECT_TELEOP_JAZZY_IMAGE:-project-teleop-ros2-jazzy:latest}"
CONTAINER_NAME="${PROJECT_TELEOP_CONTAINER:-project_teleop_ros2_mac}"
ROS_TCP_PORT="${ROS_TCP_PORT:-10000}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
PROJECT_TELEOP_LOG_TZ="${PROJECT_TELEOP_LOG_TZ:-${TZ:-Asia/Bangkok}}"
RVIZ_NOVNC_PORT="${RVIZ_NOVNC_PORT:-6080}"
RVIZ_URL="http://127.0.0.1:${RVIZ_NOVNC_PORT}/vnc.html?autoconnect=true&resize=remote"
PROJECT_TELEOP_START_RVIZ="${PROJECT_TELEOP_START_RVIZ:-0}"
IMAGE_FINGERPRINT_FILE="${WORKSPACE_ROOT}/.ros_jazzy_docker/image.fingerprint"
REAL_ROBOT_IP="${ROBOT_IP:-192.168.1.6}"
MOCK_ROBOT_IP="172.10.0.2"
CONTROL_MODE="${PROJECT_TELEOP_CONTROL_MODE:-${MG400_CONTROL_MODE:-joint_cmd}}"

STATE_DIR="${REPO_DIR}/logs/mac_app"
CURRENT_SESSION_FILE="${STATE_DIR}/current_session"
SIM_PID_FILE="${STATE_DIR}/sim.pid"
SIM_LABEL="${PROJECT_TELEOP_SIM_LABEL:-projectteleop.mg400sim}"
OWNER_FILE="${STATE_DIR}/owner"
TMUX=(tmux -f /dev/null -L mg400_teleop_mac)
TMUX_SESSION="mg400"

cmd="${1:-status}"
shift || true

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

tmux_running() {
  command -v tmux >/dev/null 2>&1 && "${TMUX[@]}" has-session -t "${TMUX_SESSION}" >/dev/null 2>&1
}

clear_stale_tmux_if_safe() {
  if ! tmux_running; then
    return 0
  fi

  if container_running; then
    return 1
  fi

  echo "Closing stale ./scripts/startup.sh tmux session '${TMUX_SESSION}' because no ROS container is running."
  "${TMUX[@]}" kill-session -t "${TMUX_SESSION}" >/dev/null 2>&1 || true
  return 0
}

docker_usable() {
  command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1
}

# Best-effort: ask Docker Desktop to start and block until the daemon
# accepts API calls. macOS only — Linux uses systemd / native daemon.
# Returns 0 if Docker became usable within ``DOCKER_START_TIMEOUT_SEC``
# (default 60), non-zero otherwise so the caller can fall through to
# the existing "not running" error.
ensure_docker_running() {
  if docker_usable; then
    return 0
  fi
  if [[ "$(uname -s)" != "Darwin" ]]; then
    return 1  # macOS-only auto-start; other platforms surface the original error.
  fi
  if ! command -v open >/dev/null 2>&1; then
    return 1
  fi
  if [[ ! -d "/Applications/Docker.app" ]]; then
    echo "Docker Desktop not found at /Applications/Docker.app." >&2
    return 1
  fi

  echo "Starting Docker Desktop (this may take ~30s on a cold launch)..."
  open -ga Docker >/dev/null 2>&1 || true

  local timeout="${DOCKER_START_TIMEOUT_SEC:-60}"
  local waited=0
  while (( waited < timeout )); do
    if docker_usable; then
      echo "Docker is ready."
      return 0
    fi
    sleep 2
    waited=$(( waited + 2 ))
  done
  echo "Timed out waiting ${timeout}s for Docker Desktop to become ready." >&2
  return 1
}

container_running() {
  local running
  running="$(docker inspect --format '{{.State.Running}}' "${CONTAINER_NAME}" 2>/dev/null || true)"
  [[ "${running}" == "true" ]]
}

app_owns_stack() {
  [[ -f "${OWNER_FILE}" ]] && grep -q '^ProjectTeleopMac$' "${OWNER_FILE}" 2>/dev/null
}

current_session_dir() {
  if [[ -f "${CURRENT_SESSION_FILE}" ]]; then
    local session_dir
    session_dir="$(<"${CURRENT_SESSION_FILE}")"
    if [[ -n "${session_dir}" && -d "${session_dir}" && "$(basename "${session_dir}")" == *_mac_app_* ]]; then
      echo "${session_dir}"
      return
    fi
  fi
  true
}

session_name_from_dir() {
  basename "$1"
}

container_log_dir_for_session() {
  local session_dir="$1"
  printf '/project_teleop_ws/project_teleop/logs/startup_sessions/%s' "$(session_name_from_dir "${session_dir}")"
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

build_image() {
  DOCKER_BUILDKIT="${DOCKER_BUILDKIT:-1}" docker build \
    -f "${REPO_DIR}/tools/ros_jazzy/Dockerfile" \
    -t "${IMAGE}" \
    "${REPO_DIR}"
  save_image_fingerprint "$(image_fingerprint)"
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

ensure_workspace_built() {
  local session_dir="$1"
  local build_log="${session_dir}/build.log"

  if [[ "${PROJECT_TELEOP_REBUILD_WS:-0}" == "1" ]]; then
    echo "Rebuilding ROS2 workspace because PROJECT_TELEOP_REBUILD_WS=1"
    docker exec "${CONTAINER_NAME}" project-teleop-jazzy build 2>&1 | tee "${build_log}"
    return
  fi

  if docker exec "${CONTAINER_NAME}" bash -lc 'test -f /project_teleop_ws/.ros_jazzy_docker/install/setup.bash'; then
    echo "Using cached ROS2 workspace build"
    return
  fi

  echo "ROS2 workspace install not found; building once"
  docker exec "${CONTAINER_NAME}" project-teleop-jazzy build 2>&1 | tee "${build_log}"
}

start_ros_container() {
  local robot_ip="$1"
  local docker_network="$2"
  local session_dir="$3"

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
    -e PROJECT_TELEOP_CONTROL_MODE="${CONTROL_MODE}"
    -e PROJECT_TELEOP_RECORD_LOG="${PROJECT_TELEOP_RECORD_LOG:-1}"
    -p "${ROS_TCP_PORT}:${ROS_TCP_PORT}"
    -p "${RVIZ_NOVNC_PORT}:${RVIZ_NOVNC_PORT}"
    -v "${WORKSPACE_ROOT}:/project_teleop_ws"
    -w /project_teleop_ws/project_teleop
    --entrypoint bash
    "${IMAGE}"
    -lc 'trap "exit 0" TERM INT; while true; do sleep 3600; done'
  )

  docker run "${docker_args[@]}" >/dev/null
  ensure_workspace_built "${session_dir}"
}

start_container_service() {
  local name="$1"
  local session_dir="$2"
  local container_log_dir="$3"
  shift 3
  local log_file="${container_log_dir}/${name}.log"
  local command="$*"

  docker exec -d "${CONTAINER_NAME}" bash -lc \
    "mkdir -p '${container_log_dir}'; exec ${command} >> '${log_file}' 2>&1"
  echo "Started ${name}; log: ${session_dir}/${name}.log"
}

verify_required_services() {
  local session_dir="$1"
  local failed=0

  sleep 2
  for name in teleop endpoint; do
    local log_file="${session_dir}/${name}.log"
    if [[ -f "${log_file}" ]] && grep -Eiq 'Traceback|EOFError|Process exited with failure|Exception' "${log_file}"; then
      echo "${name} failed during startup; see ${log_file}" >&2
      tail -n 80 "${log_file}" >&2 || true
      failed=1
    fi
  done

  if [[ "${failed}" -ne 0 ]]; then
    exit 1
  fi
}

start_mac_simulator() {
  local session_dir="$1"
  local log_file="${session_dir}/sim.log"

  stop_mac_simulator
  : > "${log_file}"
  if command -v launchctl >/dev/null 2>&1; then
    local launch_cmd
    launch_cmd="cd '${REPO_DIR}' && MG400_SIM_ROS_HOST=127.0.0.1 MG400_SIM_ROS_PORT='${ROS_TCP_PORT}' MG400_SIM_AUTO_CONNECT=1 MG400_SIM_LIVE_STREAM_MS='${MG400_SIM_LIVE_STREAM_MS:-8}' ./tools/mg400_simulator/run.sh"
    launchctl submit \
      -l "${SIM_LABEL}" \
      -o "${log_file}" \
      -e "${log_file}" \
      -- /bin/zsh -lc "${launch_cmd}"
    printf 'launchctl:%s\n' "${SIM_LABEL}" > "${SIM_PID_FILE}"
  else
    (
      cd "${REPO_DIR}"
      MG400_SIM_ROS_HOST=127.0.0.1 \
        MG400_SIM_ROS_PORT="${ROS_TCP_PORT}" \
        MG400_SIM_AUTO_CONNECT=1 \
        MG400_SIM_LIVE_STREAM_MS="${MG400_SIM_LIVE_STREAM_MS:-8}" \
        nohup ./tools/mg400_simulator/run.sh > "${log_file}" 2>&1 < /dev/null &
      printf '%s\n' "$!" > "${SIM_PID_FILE}"
    )
  fi
  echo "Started Mac simulator; log: ${log_file}"
}

stop_mac_simulator() {
  if command -v launchctl >/dev/null 2>&1; then
    launchctl remove "${SIM_LABEL}" >/dev/null 2>&1 || true
  fi

  if [[ ! -f "${SIM_PID_FILE}" ]]; then
    return
  fi

  local pid
  pid="$(<"${SIM_PID_FILE}")"
  if [[ "${pid}" == launchctl:* ]]; then
    rm -f "${SIM_PID_FILE}"
    return
  fi

  if [[ -n "${pid}" ]] && [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" >/dev/null 2>&1; then
    kill "${pid}" >/dev/null 2>&1 || true
  fi
  rm -f "${SIM_PID_FILE}"
}

write_status_log() {
  local session_dir
  session_dir="$(current_session_dir)"
  if [[ -z "${session_dir}" ]]; then
    return
  fi

  {
    echo "time: $(date '+%Y-%m-%d %H:%M:%S %Z')"
    echo "session: ${session_dir}"
    echo "container: $(container_running && echo running || echo stopped)"
    echo "control_mode: ${CONTROL_MODE}"
    echo "ros_tcp: 127.0.0.1:${ROS_TCP_PORT}"
    echo "rviz: ${RVIZ_URL}"
    echo
    if container_running; then
      echo "ROS nodes:"
      docker exec "${CONTAINER_NAME}" bash -lc 'set +u; source /opt/ros/jazzy/setup.bash; source /project_teleop_ws/.ros_jazzy_docker/install/setup.bash 2>/dev/null || true; ros2 node list' 2>/dev/null || true
      echo
      echo "ROS topics:"
      docker exec "${CONTAINER_NAME}" bash -lc 'set +u; source /opt/ros/jazzy/setup.bash; source /project_teleop_ws/.ros_jazzy_docker/install/setup.bash 2>/dev/null || true; ros2 topic list' 2>/dev/null || true
    fi
  } > "${session_dir}/status.log"
}

start_stack() {
  local mode="${1:-mock-sim}"
  local robot_kind client_kind

  case "${mode}" in
    real-sim|real-simulator) robot_kind="real"; client_kind="sim" ;;
    mock-sim|mock-simulator) robot_kind="mock"; client_kind="sim" ;;
    real-unity|real) robot_kind="real"; client_kind="unity" ;;
    mock-unity|mock) robot_kind="mock"; client_kind="unity" ;;
    *)
      echo "unknown mode: ${mode}" >&2
      echo "modes: real-sim mock-sim real-unity mock-unity" >&2
      exit 2
      ;;
  esac

  require_tool docker "Install/start Docker Desktop first."
  if ! docker_usable; then
    # Try to wake Docker Desktop ourselves before bailing out. The
    # operator clicked Start in ProjectTeleopMac; pop up Docker so the
    # daemon is ready by the time the rest of the launcher pipeline
    # tries to docker-build / docker-run.
    if ! ensure_docker_running; then
      echo "Docker is not running. Start Docker Desktop first." >&2
      exit 1
    fi
  fi

  if ! clear_stale_tmux_if_safe; then
    echo "Refusing to replace active ./scripts/startup.sh tmux session '${TMUX_SESSION}'. Stop ./scripts/startup.sh first, or keep using that terminal session." >&2
    exit 1
  fi

  mkdir -p "${STATE_DIR}" "${REPO_DIR}/logs/startup_sessions"
  local session_dir="${REPO_DIR}/logs/startup_sessions/$(date +%Y%m%d_%H%M%S)_mac_app_${mode}"
  mkdir -p "${session_dir}"
  printf '%s\n' "${session_dir}" > "${CURRENT_SESSION_FILE}"
  printf 'ProjectTeleopMac\n' > "${OWNER_FILE}"

  echo "ProjectTeleopMac start: ${mode}"
  echo "Session logs: ${session_dir}"
  echo "Control mode: ${CONTROL_MODE}"

  stop_mac_simulator

  local mock_compose=""
  local robot_ip="${REAL_ROBOT_IP}"
  local docker_network=""
  if [[ "${robot_kind}" == "mock" ]]; then
    mock_compose="$(mock_compose_file)"
    echo "Starting MG400_Mock"
    docker compose -f "${mock_compose}" up -d
    robot_ip="${MOCK_ROBOT_IP}"
    docker_network="dobot_network"
    docker compose -f "${mock_compose}" logs --tail 80 > "${session_dir}/mock.log" 2>&1 || true
  fi

  ensure_image 2>&1 | tee "${session_dir}/container.log"

  echo "Starting ROS2 container: ${CONTAINER_NAME}"
  echo "ROBOT_IP=${robot_ip}"
  echo "Client connects to 127.0.0.1:${ROS_TCP_PORT}"
  echo "RViz noVNC URL: ${RVIZ_URL}"
  start_ros_container "${robot_ip}" "${docker_network}" "${session_dir}"

  local container_log_dir
  container_log_dir="$(container_log_dir_for_session "${session_dir}")"
  start_container_service "teleop" "${session_dir}" "${container_log_dir}" "project-teleop-jazzy teleop"
  start_container_service "endpoint" "${session_dir}" "${container_log_dir}" "project-teleop-jazzy endpoint"
  if [[ "${PROJECT_TELEOP_START_RVIZ}" == "1" ]]; then
    start_container_service "rviz" "${session_dir}" "${container_log_dir}" "project-teleop-jazzy rviz-novnc"
  else
    {
      echo "RViz noVNC disabled for the Mac app launcher to keep live teleop latency low."
      echo "Run with PROJECT_TELEOP_START_RVIZ=1 to enable it, or use ./scripts/startup.sh for the tmux RViz pane."
      echo "URL when enabled: ${RVIZ_URL}"
    } > "${session_dir}/rviz.log"
    echo "RViz noVNC disabled for low-latency app mode"
  fi

  if [[ "${client_kind}" == "sim" ]]; then
    start_mac_simulator "${session_dir}"
  else
    {
      echo "Unity connection"
      echo "  Host: 127.0.0.1"
      echo "  Port: ${ROS_TCP_PORT}"
      echo
      echo "Waiting for Unity to connect."
    } > "${session_dir}/unity.log"
    cp "${session_dir}/unity.log" "${session_dir}/sim.log"
    echo "Unity mode selected; connect Unity to 127.0.0.1:${ROS_TCP_PORT}"
  fi

  write_status_log
  verify_required_services "${session_dir}"
  echo "Ready"
}

stop_stack() {
  require_tool docker "Install/start Docker Desktop first."

  if ! clear_stale_tmux_if_safe; then
    echo "Active ./scripts/startup.sh tmux session detected; app stop will not kill it because the ROS container is still running."
    echo "Stop that session from Terminal if you are using ./scripts/startup.sh."
    return 0
  fi

  echo "Stopping ProjectTeleopMac stack"
  stop_mac_simulator
  docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
  if [[ -f "${REPO_DIR}/MG400_Mock/docker-compose.yml" ]]; then
    docker compose -f "${REPO_DIR}/MG400_Mock/docker-compose.yml" down >/dev/null 2>&1 || true
  fi
  rm -f "${OWNER_FILE}"
  write_status_log || true
  echo "Stopped"
}

show_status() {
  if container_running; then
    echo "running"
  else
    echo "stopped"
  fi
}

show_logs() {
  local which="${1:-status}"
  local session_dir
  session_dir="$(current_session_dir)"

  case "${which}" in
    status)
      write_status_log || true
      echo "container: $(container_running && echo running || echo stopped)"
      echo "tmux_startup: $(tmux_running && echo running || echo stopped)"
      echo "app_owner: $(app_owns_stack && echo yes || echo no)"
      echo "session: ${session_dir:-none}"
      echo "control_mode: ${CONTROL_MODE}"
      echo "ros_tcp: 127.0.0.1:${ROS_TCP_PORT}"
      echo "rviz: ${RVIZ_URL}"
      if [[ -n "${session_dir}" && -f "${session_dir}/status.log" ]]; then
        echo
        tail -n 160 "${session_dir}/status.log"
      fi
      ;;
    mock)
      # mock.log is captured once at start (docker compose logs --tail 80).
      # That snapshot is empty if the container hadn't logged yet by then,
      # so re-fetch live from docker compose each time the operator
      # opens the Mock tab.
      local mock_compose
      mock_compose="$(mock_compose_file 2>/dev/null || true)"
      if [[ -n "${mock_compose}" ]] && docker compose -f "${mock_compose}" ps -q 2>/dev/null | grep -q .; then
        docker compose -f "${mock_compose}" logs --tail 220 2>&1 || true
      elif [[ -n "${session_dir}" && -f "${session_dir}/mock.log" ]]; then
        tail -n 220 "${session_dir}/mock.log"
      else
        echo "no mock container running and no mock.log snapshot"
      fi
      ;;
    teleop|endpoint|sim|unity|rviz|container|build)
      if [[ -z "${session_dir}" ]]; then
        echo "no startup session logs yet"
        exit 0
      fi
      local log_file="${session_dir}/${which}.log"
      if [[ -f "${log_file}" ]]; then
        tail -n 220 "${log_file}"
      else
        echo "no ${which}.log in ${session_dir}"
      fi
      ;;
    simulator)
      show_logs sim
      ;;
    *)
      echo "unknown log: ${which}" >&2
      echo "logs: status teleop endpoint sim simulator unity rviz mock container build" >&2
      exit 2
      ;;
  esac
}

run_bridge() {
  if ! container_running; then
    echo '{"t":"error","message":"ROS container is not running"}'
    exit 0
  fi
  exec docker exec -i "${CONTAINER_NAME}" project-teleop-jazzy bash -lc \
    "exec python3 -u /project_teleop_ws/project_teleop/tools/mac_app/ros_bridge.py"
}

send_robot_command() {
  local action="${1:-}"
  local dash_cmd
  case "${action}" in
    enable) dash_cmd="EnableRobot()" ;;
    disable) dash_cmd="DisableRobot()" ;;
    clear-error) dash_cmd="ClearError()" ;;
    reset) dash_cmd="ResetRobot()" ;;
    emergency-stop) dash_cmd="EmergencyStop()" ;;
    pause) dash_cmd="Pause()" ;;
    continue) dash_cmd="Continue()" ;;
    *)
      echo "unknown robot command: ${action}" >&2
      echo "commands: enable disable clear-error reset emergency-stop pause continue" >&2
      exit 2
      ;;
  esac
  if ! container_running; then
    echo "ROS container is not running"
    exit 1
  fi
  docker exec "${CONTAINER_NAME}" bash -lc \
    "set +u; source /opt/ros/jazzy/setup.bash; source /project_teleop_ws/.ros_jazzy_docker/install/setup.bash 2>/dev/null || true; ros2 topic pub --once /robot/dashboard_cmd std_msgs/msg/String \"{data: '${dash_cmd}'}\""
  echo "published /robot/dashboard_cmd ${dash_cmd}"
}

case "${cmd}" in
  start)
    start_stack "${1:-mock-sim}"
    ;;
  stop)
    stop_stack
    ;;
  status)
    show_status
    ;;
  bridge)
    run_bridge
    ;;
  logs)
    show_logs "${1:-status}"
    ;;
  robot)
    send_robot_command "${1:-}"
    ;;
  exec)
    exec docker exec "${CONTAINER_NAME}" "$@"
    ;;
  *)
    echo "usage: launcher.sh {start <mode>|stop|status|bridge|logs|robot|exec ...}" >&2
    echo "  modes: real-sim mock-sim real-unity mock-unity" >&2
    exit 2
    ;;
esac

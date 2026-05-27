#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
set -u

REPO_ROOT="${PROJECT_TELEOP_REPO_ROOT:-/project_teleop_ws/project_teleop}"
BUILD_BASE="${COLCON_BUILD_BASE:-/project_teleop_ws/.ros_jazzy_docker/build}"
INSTALL_BASE="${COLCON_INSTALL_BASE:-/project_teleop_ws/.ros_jazzy_docker/install}"
LOG_BASE="${COLCON_LOG_BASE:-/project_teleop_ws/.ros_jazzy_docker/log}"
TEST_RESULT_BASE="${COLCON_TEST_RESULT_BASE:-${BUILD_BASE}}"

mode="${1:-full}"
if [[ $# -gt 0 ]]; then
  shift
fi

cd "${REPO_ROOT}"

common_args=(
  --build-base "${BUILD_BASE}"
  --install-base "${INSTALL_BASE}"
)

run_colcon_test() {
  local status=0
  colcon --log-base "${LOG_BASE}" test "${common_args[@]}" --test-result-base "${TEST_RESULT_BASE}" --event-handlers console_direct+ "$@" || status=$?
  colcon test-result --test-result-base "${TEST_RESULT_BASE}" --verbose || {
    local result_status=$?
    if [[ "${status}" -eq 0 ]]; then
      status="${result_status}"
    fi
  }
  return "${status}"
}

source_install_setup() {
  set +u
  source "${INSTALL_BASE}/setup.bash"
  set -u
}

run_function_tests() {
  local py_path="${REPO_ROOT}/src/dobot_mg400/mg400_controller"
  py_path="${py_path}:${REPO_ROOT}/src/dobot_mg400/mg400_protocol"
  py_path="${py_path}:${REPO_ROOT}/src/robot_teaching_core"
  py_path="${py_path}:${REPO_ROOT}/src/adapters/mg400"
  py_path="${py_path}:${REPO_ROOT}/tools/mg400_simulator"
  export PYTHONPATH="${py_path}${PYTHONPATH:+:${PYTHONPATH}}"

  "${REPO_ROOT}/.claude-handoff/run_tests.sh" "$@"

  (
    cd "${REPO_ROOT}/src/dobot_mg400/mg400_protocol"
    python3 -m unittest discover -s test -p 'test_*.py' -v
  )

  (
    cd "${REPO_ROOT}/src/robot_teaching_core"
    python3 -m unittest discover -s test -p 'test_*.py' -v
  )

  (
    cd "${REPO_ROOT}/src/adapters/mg400"
    python3 -m unittest discover -s test -p 'test_*.py' -v
  )

  (
    cd "${REPO_ROOT}/tools/mg400_simulator"
    python3 -m unittest discover -s test -p 'test_*.py' -v
  )
}

ensure_built() {
  if [[ -f "${INSTALL_BASE}/setup.bash" ]]; then
    return
  fi
  colcon --log-base "${LOG_BASE}" build "${common_args[@]}" --symlink-install
}

run_mac_stack() {
  ensure_built
  source_install_setup

  local ros_tcp_port="${ROS_TCP_PORT:-10000}"
  ros2 run ros_tcp_endpoint default_server_endpoint \
    --ros-args -p ROS_IP:=0.0.0.0 -p "ROS_TCP_PORT:=${ros_tcp_port}" &
  local endpoint_pid=$!

  trap 'kill "${endpoint_pid}" 2>/dev/null || true' EXIT INT TERM
  ros2 run mg400_controller vr_teleop_node "$@"
}

run_rviz_novnc() {
  ensure_built
  source_install_setup

  local display="${RVIZ_VIRTUAL_DISPLAY:-:99}"
  local screen="${RVIZ_SCREEN:-1280x800x24}"
  local vnc_port="${RVIZ_VNC_PORT:-5900}"
  local novnc_port="${RVIZ_NOVNC_PORT:-6080}"
  local novnc_web="${RVIZ_NOVNC_WEB:-/usr/share/novnc}"

  export DISPLAY="${display}"
  export QT_X11_NO_MITSHM=1
  export LIBGL_ALWAYS_SOFTWARE=1
  export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"
  mkdir -p "${XDG_RUNTIME_DIR}"
  chmod 700 "${XDG_RUNTIME_DIR}"

  Xvfb "${display}" -screen 0 "${screen}" +extension GLX +render -noreset &
  local xvfb_pid=$!
  sleep 1

  fluxbox >/tmp/project_teleop_fluxbox.log 2>&1 &
  local wm_pid=$!

  x11vnc -display "${display}" -forever -shared -nopw -rfbport "${vnc_port}" >/tmp/project_teleop_x11vnc.log 2>&1 &
  local x11vnc_pid=$!

  websockify --web="${novnc_web}" "${novnc_port}" "localhost:${vnc_port}" >/tmp/project_teleop_websockify.log 2>&1 &
  local websockify_pid=$!

  trap 'kill "${websockify_pid}" "${x11vnc_pid}" "${wm_pid}" "${xvfb_pid}" 2>/dev/null || true' EXIT INT TERM

  echo "RViz noVNC URL: http://127.0.0.1:${novnc_port}/vnc.html?autoconnect=true&resize=remote"
  ros2 launch mg400_bringup main.launch.py "$@"
}

case "${mode}" in
  build)
    colcon --log-base "${LOG_BASE}" build "${common_args[@]}" --symlink-install "$@"
    ;;
  test)
    run_colcon_test "$@"
    ;;
  full)
    colcon --log-base "${LOG_BASE}" build "${common_args[@]}" --symlink-install "$@"
    source_install_setup
    run_function_tests "$@"
    ;;
  strict)
    colcon --log-base "${LOG_BASE}" build "${common_args[@]}" --symlink-install "$@"
    run_colcon_test "$@"
    ;;
  function)
    source_install_setup
    run_function_tests "$@"
    ;;
  endpoint)
    ensure_built
    source_install_setup
    ros2 run ros_tcp_endpoint default_server_endpoint \
      --ros-args -p ROS_IP:=0.0.0.0 -p "ROS_TCP_PORT:=${ROS_TCP_PORT:-10000}" "$@"
    ;;
  teleop)
    ensure_built
    source_install_setup
    ros2 run mg400_controller vr_teleop_node "$@"
    ;;
  rviz)
    ensure_built
    source_install_setup
    ros2 launch mg400_bringup main.launch.py "$@"
    ;;
  rviz-novnc)
    run_rviz_novnc "$@"
    ;;
  mac-stack)
    run_mac_stack "$@"
    ;;
  unit)
    "${REPO_ROOT}/.claude-handoff/run_tests.sh" "$@"
    ;;
  bash|shell)
    exec bash "$@"
    ;;
  *)
    echo "usage: project-teleop-jazzy [full|strict|build|test|function|endpoint|teleop|rviz|rviz-novnc|mac-stack|unit|bash] [args...]" >&2
    exit 2
    ;;
esac

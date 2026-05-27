#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORKSPACE_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"

IMAGE="${PROJECT_TELEOP_JAZZY_IMAGE:-project-teleop-ros2-jazzy:latest}"
ROBOT_IP="${ROBOT_IP:-192.168.1.6}"
ROS_TCP_PORT="${ROS_TCP_PORT:-10000}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
DOCKER_NETWORK="${PROJECT_TELEOP_DOCKER_NETWORK:-}"

DOCKER_BUILDKIT="${DOCKER_BUILDKIT:-0}" docker build \
  -f "${SCRIPT_DIR}/Dockerfile" \
  -t "${IMAGE}" \
  "${REPO_ROOT}"

tty_args=()
if [[ -t 0 && -t 1 ]]; then
  tty_args=(-it)
fi

network_args=()
if [[ -n "${DOCKER_NETWORK}" ]]; then
  network_args=(--network "${DOCKER_NETWORK}")
fi

docker run --rm --init "${tty_args[@]}" \
  "${network_args[@]}" \
  -e ROBOT_IP="${ROBOT_IP}" \
  -e ROS_TCP_PORT="${ROS_TCP_PORT}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  -p "${ROS_TCP_PORT}:${ROS_TCP_PORT}" \
  -v "${WORKSPACE_ROOT}:/project_teleop_ws" \
  -w /project_teleop_ws/project_teleop \
  "${IMAGE}" \
  mac-stack \
  "$@"

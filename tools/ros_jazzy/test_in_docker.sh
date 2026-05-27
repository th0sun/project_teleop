#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORKSPACE_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"

IMAGE="${PROJECT_TELEOP_JAZZY_IMAGE:-project-teleop-ros2-jazzy:latest}"
IMAGE_FINGERPRINT_FILE="${WORKSPACE_ROOT}/.ros_jazzy_docker/image.fingerprint"
TEACH_REPEAT_SAMPLE_JSON="${TEACH_REPEAT_SAMPLE_JSON:-/project_teleop_ws/_supporting_materials/data/trajectories/json_trajectories/money.json}"
MODE="${1:-full}"
if [[ $# -gt 0 ]]; then
  shift
fi

image_fingerprint() {
  {
    printf 'Dockerfile '
    shasum -a 256 "${SCRIPT_DIR}/Dockerfile" | awk '{print $1}'
    printf 'entrypoint.sh '
    shasum -a 256 "${SCRIPT_DIR}/entrypoint.sh" | awk '{print $1}'
  } | shasum -a 256 | awk '{print $1}'
}

build_image() {
  DOCKER_BUILDKIT="${DOCKER_BUILDKIT:-1}" docker build \
    -f "${SCRIPT_DIR}/Dockerfile" \
    -t "${IMAGE}" \
    "${REPO_ROOT}"
  mkdir -p "$(dirname "${IMAGE_FINGERPRINT_FILE}")"
  image_fingerprint > "${IMAGE_FINGERPRINT_FILE}"
}

current_fingerprint="$(image_fingerprint)"
stored_fingerprint=""
if [[ -f "${IMAGE_FINGERPRINT_FILE}" ]]; then
  stored_fingerprint="$(<"${IMAGE_FINGERPRINT_FILE}")"
fi

if [[ "${PROJECT_TELEOP_REBUILD_IMAGE:-0}" == "1" ]]; then
  echo "Rebuilding ROS2 Jazzy image because PROJECT_TELEOP_REBUILD_IMAGE=1"
  build_image
elif ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "ROS2 Jazzy image not found; building ${IMAGE}"
  build_image
elif [[ "${stored_fingerprint}" != "${current_fingerprint}" ]]; then
  echo "ROS2 Jazzy image inputs changed; rebuilding ${IMAGE}"
  build_image
else
  echo "Using cached ROS2 Jazzy image: ${IMAGE}"
fi

docker run --rm --init \
  -e TEACH_REPEAT_SAMPLE_JSON="${TEACH_REPEAT_SAMPLE_JSON}" \
  -v "${WORKSPACE_ROOT}:/project_teleop_ws" \
  -w /project_teleop_ws/project_teleop \
  "${IMAGE}" \
  "${MODE}" \
  "$@"

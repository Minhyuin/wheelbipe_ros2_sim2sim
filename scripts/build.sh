#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
clean_cache=false

usage() {
  echo "Usage: ./scripts/build.sh [--clean-cache]"
}

while (($#)); do
  case "$1" in
    --clean-cache)
      clean_cache=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

"${repository_root}/scripts/check_env.sh"

ros_setup_file=""
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  ros_setup_file="/opt/ros/${ROS_DISTRO}/setup.bash"
else
  for candidate in jazzy humble; do
    if [[ -f "/opt/ros/${candidate}/setup.bash" ]]; then
      ros_setup_file="/opt/ros/${candidate}/setup.bash"
      break
    fi
  done
fi

if [[ -z "${ros_setup_file}" ]]; then
  echo "No ROS 2 installation found under /opt/ros; install ROS 2 Jazzy or Humble." >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source "${ros_setup_file}"
# shellcheck disable=SC1091
source "${repository_root}/setup_mujoco_env.bash"
set -u

cmake_args=(
  --no-warn-unused-cli
  -DCMAKE_BUILD_TYPE=Release
  -DBUILD_TESTING=OFF
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  -DONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT}"
)

colcon_args=(build --cmake-args "${cmake_args[@]}")
if [[ "${clean_cache}" == true ]]; then
  colcon_args=(build --cmake-clean-cache --cmake-args "${cmake_args[@]}")
fi

cd "${repository_root}"
colcon "${colcon_args[@]}"
echo "Build completed. Source install/setup.bash before direct ros2 commands."

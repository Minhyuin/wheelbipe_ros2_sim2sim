#!/usr/bin/env bash

# Source this file from the workspace root before build or launch.
_wheelbipe_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_wheelbipe_ros_domain_id="${ROS_DOMAIN_ID:-0}"
_wheelbipe_ros_localhost_only="${ROS_LOCALHOST_ONLY:-1}"
_wheelbipe_rmw_implementation="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# Prefer an already selected distro, then the newest supported one.
_wheelbipe_ros_distro="${ROS_DISTRO:-}"
if [ -z "${_wheelbipe_ros_distro}" ]; then
  for _wheelbipe_ros_candidate in jazzy humble; do
    if [ -f "/opt/ros/${_wheelbipe_ros_candidate}/setup.bash" ]; then
      _wheelbipe_ros_distro="${_wheelbipe_ros_candidate}"
      break
    fi
  done
fi

if [ -n "${_wheelbipe_ros_distro}" ] && [ -f "/opt/ros/${_wheelbipe_ros_distro}/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "/opt/ros/${_wheelbipe_ros_distro}/setup.bash"
fi

if [ -f "${_wheelbipe_ws}/setup_mujoco_env.bash" ]; then
  # shellcheck disable=SC1091
  source "${_wheelbipe_ws}/setup_mujoco_env.bash"
fi

if [ -f "${_wheelbipe_ws}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${_wheelbipe_ws}/install/setup.bash"
fi

export WHEELBIPE_WS="${_wheelbipe_ws}"
export ROS_DOMAIN_ID="${_wheelbipe_ros_domain_id}"
# Keep the default demo on one host. Multi-host deployments must opt in after
# choosing an isolated DDS domain and network trust boundary.
export ROS_LOCALHOST_ONLY="${_wheelbipe_ros_localhost_only}"
export RMW_IMPLEMENTATION="${_wheelbipe_rmw_implementation}"

unset _wheelbipe_ws _wheelbipe_ros_domain_id _wheelbipe_ros_localhost_only
unset _wheelbipe_rmw_implementation
unset _wheelbipe_ros_distro _wheelbipe_ros_candidate

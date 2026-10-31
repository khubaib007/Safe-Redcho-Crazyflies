#!/usr/bin/env bash
# Source after environment.bash. No node or radio connection is created here.
set +u
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH VIRTUAL_ENV
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_DISCOVERY_SERVER
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/jazzy/setup.bash
if [[ -n "${REDCHO_VENDOR_PREFIX:-}" ]]; then source "$REDCHO_VENDOR_PREFIX/local_setup.bash"; fi
source "$SAFE_REDCHO_HW_ROOT/install/local_setup.bash"
source "$SAFE_REDCHO_HW_ROOT/scripts/environment.bash"
export ROS_DOMAIN_ID="${REDCHO_HW_DOMAIN_ID:-43}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$SAFE_REDCHO_HW_ROOT/config/fastdds_udp_only.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_LOG_DIR="${ROS_LOG_DIR:-$SAFE_REDCHO_HW_ROOT/log/ros}"

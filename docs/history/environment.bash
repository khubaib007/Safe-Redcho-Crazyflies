# Source in a clean shell. Do not source either simulation workspace.
export SAFE_REDCHO_HW_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=43
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash
if [[ -f "$SAFE_REDCHO_HW_ROOT/.venv/bin/activate" ]]; then
  source "$SAFE_REDCHO_HW_ROOT/.venv/bin/activate"
  # ROS console executables may use /usr/bin/python3 in their shebang.
  # Make workspace-local Python dependencies available to those executables too.
  export PYTHONPATH="$SAFE_REDCHO_HW_ROOT/.venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
fi
if [[ -f "$SAFE_REDCHO_HW_ROOT/install/local_setup.bash" ]]; then
  source "$SAFE_REDCHO_HW_ROOT/install/local_setup.bash"
fi

export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# Safe REDCHO: explicit UDP-only Fast DDS transport.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/safe_redcho_hardware/config/fastdds_udp_only.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# Safe REDCHO: explicit UDP-only Fast DDS transport.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/safe_redcho_hardware/config/fastdds_udp_only.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# XML controls DDS discovery; avoid LOCALHOST adding SHM.
unset ROS_LOCALHOST_ONLY
unset ROS_STATIC_PEERS
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/safe_redcho_hardware/config/fastdds_udp_only.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

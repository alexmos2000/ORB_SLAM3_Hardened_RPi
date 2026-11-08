#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
bridge="${script_dir}/rviz2_tcp_bridge.py"
rviz_config="${script_dir}/rviz2_icm20948.rviz"
ros_setup="/opt/ros/humble/setup.bash"
port="${ORB_SLAM_VIZ_PORT:-8765}"

if [[ ! -f "$ros_setup" ]]; then
    printf 'ROS 2 Humble setup not found: %s\n' "$ros_setup" >&2
    exit 1
fi

if [[ ! -f "$bridge" || ! -f "$rviz_config" ]]; then
    printf 'RViz bridge or configuration file is missing.\n' >&2
    exit 1
fi

# ROS setup scripts may read environment variables that are unset under nounset.
set +u
source "$ros_setup"
set -u

if ! command -v rviz2 >/dev/null 2>&1; then
    printf 'rviz2 executable not found after sourcing %s.\n' "$ros_setup" >&2
    exit 1
fi

if ! python3 -c 'import rclpy; import geometry_msgs.msg; import nav_msgs.msg; import sensor_msgs.msg; import sensor_msgs_py.point_cloud2; import std_msgs.msg; import visualization_msgs.msg' 2>/dev/null; then
    printf 'The ROS 2 Python message packages required by the bridge are missing.\n' >&2
    printf 'Install ROS 2 Humble rclpy, geometry_msgs, nav_msgs, sensor_msgs, sensor_msgs_py, and std_msgs.\n' >&2
    exit 1
fi

python3 "$bridge" --bind 0.0.0.0 --port "$port" &
bridge_pid=$!
cleanup() {
    kill "$bridge_pid" 2>/dev/null || true
    wait "$bridge_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if rviz2 -d "$rviz_config"; then
    exit 0
else
    rviz_status=$?
fi

if [[ "$rviz_status" -eq 139 ]]; then
    printf 'rviz2 crashed with SIGSEGV; retrying with software OpenGL rendering.\n' >&2
    LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d "$rviz_config"
    exit $?
fi

exit "$rviz_status"

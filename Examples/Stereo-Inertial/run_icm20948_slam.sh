#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/../.." && pwd)"
slam="${script_dir}/stereo_inertial_icm20948"
settings="${script_dir}/ICM20948_Kalibr.yaml"

if [[ -f "${repo_root}/build/Vocabulary/ORBvoc.txt" ]]; then
    vocabulary="${repo_root}/build/Vocabulary/ORBvoc.txt"
else
    vocabulary="${repo_root}/Vocabulary/ORBvoc.txt"
fi

usage() {
    printf 'Usage: %s [--viz-host HOST [--viz-port PORT]] [--] \\\n' "$0" >&2
    printf '       [i2c_device [left_camera_index right_camera_index [trajectory.txt]]]\n' >&2
    printf 'Camera rates and exposure are configured in %s.\n' "$settings" >&2
    printf 'Build with: cmake --build "%s/build" --target stereo_inertial_icm20948 -j1\n' \
        "$repo_root" >&2
}

visualization_host="${ORB_SLAM_VIZ_HOST:-}"
visualization_port="${ORB_SLAM_VIZ_PORT:-8765}"
visualization_port_requested=false
if [[ -n "${ORB_SLAM_VIZ_PORT:-}" ]]; then
    visualization_port_requested=true
fi
while (( $# > 0 )); do
    case "$1" in
    --viz-host)
        if (( $# < 2 )); then
            usage
            exit 2
        fi
        visualization_host="$2"
        shift 2
        ;;
    --viz-port)
        if (( $# < 2 )); then
            usage
            exit 2
        fi
        visualization_port="$2"
        visualization_port_requested=true
        shift 2
        ;;
    --help|-h)
        usage
        exit 0
        ;;
    --)
        shift
        break
        ;;
    --*)
        printf 'Unknown option: %s\n' "$1" >&2
        usage
        exit 2
        ;;
    *)
        break
        ;;
    esac
done

if [[ ! -x "$slam" ]]; then
    printf 'SLAM executable not found: %s\n' "$slam" >&2
    usage
    exit 1
fi

if [[ -z "$visualization_host" && "$visualization_port_requested" == true ]]; then
    printf 'A visualization host is required when setting a visualization port.\n' >&2
    usage
    exit 2
fi

if (( $# > 4 || $# == 2 )); then
    usage
    exit 2
fi

if [[ ! -f "$vocabulary" ]]; then
    printf 'ORB vocabulary not found: %s\n' "$vocabulary" >&2
    exit 1
fi

if [[ ! -f "$settings" ]]; then
    printf 'Calibration settings not found: %s\n' "$settings" >&2
    exit 1
fi

if [[ -n "$visualization_host" ]]; then
    if [[ ! "$visualization_port" =~ ^[0-9]+$ ]] ||
        (( 10#$visualization_port < 1 || 10#$visualization_port > 65535 )); then
        printf 'Visualization port must be an integer from 1 to 65535: %s\n' \
            "$visualization_port" >&2
        exit 2
    fi
    export ORB_SLAM_VIZ_HOST="$visualization_host"
    export ORB_SLAM_VIZ_PORT="$visualization_port"
    printf 'Visualization relay: %s:%s\n' "$visualization_host" "$visualization_port"
else
    unset ORB_SLAM_VIZ_HOST ORB_SLAM_VIZ_PORT
fi

printf 'Starting stereo-inertial SLAM with:\n  Calibration: %s\n  Vocabulary:  %s\n' \
    "$settings" "$vocabulary"
if [[ -z "${ORB_SLAM_IMU_CALIBRATION:-}" && -f "${script_dir}/icm20948_calibration.yaml" ]]; then
    export ORB_SLAM_IMU_CALIBRATION="${script_dir}/icm20948_calibration.yaml"
fi

exec "$slam" "$vocabulary" "$settings" -- "$@"

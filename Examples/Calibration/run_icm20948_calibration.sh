#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
recorder="${script_dir}/recorder_icm20948"

usage() {
    printf 'Usage: %s [output_directory [duration_seconds]]\n' "$0" >&2
    printf 'Omit duration or use 0 to record until Ctrl+C.\n' >&2
}

if (( $# > 2 )); then
    usage
    exit 2
fi

if [[ ! -x "$recorder" ]]; then
    printf 'Recorder executable not found: %s\n' "$recorder" >&2
    printf 'Build it with: cmake --build "%s/build" --target recorder_icm20948 -j1\n' \
        "$(cd -- "${script_dir}/../.." && pwd)" >&2
    exit 1
fi

if (( $# >= 2 )); then
    duration="$2"
    if [[ ! "$duration" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        printf 'Duration must be a non-negative number of seconds: %s\n' "$duration" >&2
        exit 2
    fi
else
    duration=0
fi

if (( $# >= 1 )); then
    output_dir="$1"
else
    output_dir="${script_dir}/recordings/sequence_$(date +%Y%m%d_%H%M%S)"
fi

default_calibration="${script_dir}/../Stereo-Inertial/icm20948_calibration.yaml"
if [[ -z "${ORB_SLAM_IMU_CALIBRATION:-}" && -f "$default_calibration" ]]; then
    export ORB_SLAM_IMU_CALIBRATION="$default_calibration"
fi

printf 'Recording stereo + ICM-20948 data to:\n  %s\n' "$output_dir"
if [[ "$duration" == "0" || "$duration" == "0.0" ]]; then
    printf 'Recording until stopped. Press Ctrl+C once to finish cleanly.\n'
else
    printf 'Recording for %s seconds. Press Ctrl+C to stop earlier.\n' "$duration"
fi

if (( $# >= 2 )); then
    exec "$recorder" "$output_dir" "$duration"
else
    exec "$recorder" "$output_dir"
fi

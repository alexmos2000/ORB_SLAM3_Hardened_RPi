#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cross_root="$project_root/.cross/rpi5"
sysroot="$cross_root/root"
compiler="$sysroot/usr/bin/aarch64-linux-gnu-g++-11"
build_dir="$cross_root/build"
object_dir="$build_dir/core-objects"
log_dir="$build_dir/core-logs"
core_library="$project_root/lib/libORB_SLAM3.so"
temporary_library="$build_dir/libORB_SLAM3.so"
example_object="$build_dir/stereo_inertial_icm20948.o"
compile_log="$build_dir/compile-example.log"
link_log="$build_dir/link-example.log"
output="$project_root/Examples/Stereo-Inertial/stereo_inertial_icm20948"
temporary_output="$build_dir/stereo_inertial_icm20948"
build_jobs="${ORB_SLAM_BUILD_JOBS:-2}"
pi_host="${ORB_SLAM_PI_HOST:-pi@rpi5.local}"
pi_project_root="${ORB_SLAM_PI_ROOT:-/home/pi/projects/ORB_SLAM3_Hardened_RPi}"
pi_project_root="${pi_project_root%/}"
pi_target_directory="$pi_project_root/Examples/Stereo-Inertial"
pi_target="$pi_target_directory/stereo_inertial_icm20948"
pi_temporary_target="$pi_target.tmp.$$"
pi_library="$pi_project_root/lib/libORB_SLAM3.so"
pi_temporary_library="$pi_library.tmp.$$"

if [[ ! "$build_jobs" =~ ^[1-9][0-9]*$ ]]; then
    printf 'ORB_SLAM_BUILD_JOBS must be a positive integer.\n' >&2
    exit 1
fi

if [[ ! "$pi_project_root" =~ ^/[A-Za-z0-9_./-]+$ || "$pi_project_root" == *"/../"* ]]; then
    printf 'ORB_SLAM_PI_ROOT must be an absolute path without shell metacharacters or parent segments.\n' >&2
    exit 1
fi

required_paths=(
    "$compiler"
    "$sysroot/usr/include/opencv4/opencv2/core.hpp"
    "$sysroot/usr/include/libcamera/libcamera/camera.h"
    "$sysroot/usr/include/boost/serialization/serialization.hpp"
    "$sysroot/usr/local/include/pangolin/pangolin.h"
    "$sysroot/lib/aarch64-linux-gnu/libopencv_core.so.410"
    "$sysroot/lib/aarch64-linux-gnu/libopencv_imgproc.so.410"
    "$sysroot/lib/aarch64-linux-gnu/libopencv_highgui.so.410"
    "$sysroot/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.410"
    "$sysroot/lib/aarch64-linux-gnu/libopencv_calib3d.so.410"
    "$sysroot/lib/aarch64-linux-gnu/libopencv_features2d.so.410"
    "$sysroot/lib/aarch64-linux-gnu/libcamera.so.0.7"
    "$sysroot/lib/aarch64-linux-gnu/libboost_serialization.so.1.83.0"
    "$sysroot/lib/aarch64-linux-gnu/libcrypto.so.3"
    "$sysroot/usr/lib/gcc/aarch64-linux-gnu/14/libstdc++.so"
    "$sysroot/usr/lib/aarch64-linux-gnu/libc.so"
    "$sysroot/lib/ld-linux-aarch64.so.1"
    "$sysroot/usr/local/lib/libpango_display.so.0"
    "$sysroot/usr/local/lib/libpango_vars.so.0"
    "$sysroot/usr/local/lib/libpango_opengl.so.0"
    "$sysroot/usr/local/lib/libpango_core.so.0"
)

for path in "${required_paths[@]}"; do
    if [[ ! -e "$path" ]]; then
        printf 'Missing cross-build input: %s\n' "$path" >&2
        printf 'Restore the ARM64 sysroot under %s before building.\n' "$cross_root" >&2
        exit 1
    fi
done

mkdir -p "$build_dir" "$object_dir" "$log_dir" "$project_root/lib"
export LD_LIBRARY_PATH="$sysroot/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

thirdparty_objects="$build_dir/thirdparty-objects"
mkdir -p "$thirdparty_objects" "$project_root/Thirdparty/DBoW2/lib" "$project_root/Thirdparty/g2o/lib"
sed 's/^#cmakedefine \(G2O_[A-Z_]*\).*/\/\* #undef \1 \*\//' \
    "$project_root/Thirdparty/g2o/config.h.in" > "$project_root/Thirdparty/g2o/config.h"

compile_dependency() {
    set -euo pipefail
    local source="$1"
    local object="$thirdparty_objects/${source//\//_}.o"
    local log="$log_dir/${source//\//_}.log"
    if ! "$compiler" --sysroot="$sysroot" -std=c++17 -mcpu=cortex-a76 -O3 -DNDEBUG -fPIC \
        -I"$project_root/Thirdparty/g2o" \
        -I"$project_root/Thirdparty/g2o/g2o/core" \
        -I"$project_root/Thirdparty/g2o/g2o/types" \
        -I"$project_root/Thirdparty/g2o/g2o/stuff" \
        -I"$project_root/Thirdparty/DBoW2" \
        -isystem "$sysroot/usr/include/eigen3" -isystem "$sysroot/usr/include/opencv4" \
        -c "$project_root/$source" -o "$object" >"$log" 2>&1; then
        printf 'Failed to compile %s:\n' "$source" >&2
        tail -n 80 "$log" >&2
        return 1
    fi
}

mapfile -t g2o_sources < <(awk '
    /^ADD_LIBRARY\(g2o / { inside=1; next }
    inside && /^\)/ { exit }
    inside && $1 ~ /^g2o\/.*\.(cpp|c)$/ { print "Thirdparty/g2o/" $1 }
' "$project_root/Thirdparty/g2o/CMakeLists.txt")
dbow_sources=(
    Thirdparty/DBoW2/DBoW2/BowVector.cpp Thirdparty/DBoW2/DBoW2/FORB.cpp
    Thirdparty/DBoW2/DBoW2/FeatureVector.cpp Thirdparty/DBoW2/DBoW2/ScoringObject.cpp
    Thirdparty/DBoW2/DUtils/Random.cpp Thirdparty/DBoW2/DUtils/Timestamp.cpp
)
if (( ${#g2o_sources[@]} == 0 )); then
    printf 'No g2o sources found in CMakeLists.txt.\n' >&2
    exit 1
fi
export -f compile_dependency
export compiler sysroot project_root thirdparty_objects log_dir
printf '%s\n' "${g2o_sources[@]}" "${dbow_sources[@]}" |
    xargs -r -n 1 -P "$build_jobs" bash -c 'compile_dependency "$1"' _
g2o_objects=()
for source in "${g2o_sources[@]}"; do g2o_objects+=("$thirdparty_objects/${source//\//_}.o"); done
dbow_objects=()
for source in "${dbow_sources[@]}"; do dbow_objects+=("$thirdparty_objects/${source//\//_}.o"); done
"$compiler" --sysroot="$sysroot" -shared -o "$project_root/Thirdparty/g2o/lib/libg2o.so" "${g2o_objects[@]}"
"$compiler" --sysroot="$sysroot" -shared -o "$project_root/Thirdparty/DBoW2/lib/libDBoW2.so" "${dbow_objects[@]}" \
    -L"$sysroot/lib/aarch64-linux-gnu" -l:libopencv_core.so.410

core_sources=(
    src/System.cc
    src/Tracking.cc
    src/LocalMapping.cc
    src/LoopClosing.cc
    src/ORBextractor.cc
    src/ORBmatcher.cc
    src/FrameDrawer.cc
    src/Converter.cc
    src/MapPoint.cc
    src/KeyFrame.cc
    src/Atlas.cc
    src/Map.cc
    src/MapDrawer.cc
    src/Optimizer.cc
    src/Frame.cc
    src/KeyFrameDatabase.cc
    src/Sim3Solver.cc
    src/Viewer.cc
    src/ImuTypes.cc
    src/G2oTypes.cc
    src/CameraModels/Pinhole.cpp
    src/CameraModels/KannalaBrandt8.cpp
    src/OptimizableTypes.cpp
    src/MLPnPsolver.cpp
    src/GeometricTools.cc
    src/TwoViewReconstruction.cc
    src/Config.cc
    src/Settings.cc
    src/VisualizationSource.cc
    src/WebViewerProtocol.cc
    src/WebViewerBackend.cc
    src/CrashMonitor.cc
)

trace_flag=-DORB_SLAM_MI_DISABLED
if [[ "${ORB_SLAM_MI_TRACE:-0}" == 1 ]]; then
    trace_flag=-DORB_SLAM_MI_TRACE
fi

compile_core_source() {
    set -euo pipefail
    local source="$1"
    local object="$object_dir/${source//\//_}.o"
    local log="$log_dir/${source//\//_}.log"

    if ! "$compiler" \
        --sysroot="$sysroot" \
        -DCOMPILEDWITHC11 \
        "$trace_flag" \
        -DHAVE_EIGEN \
        -DHAVE_EPOXY \
        -DHAVE_PANGOLIN \
        '-DPANGO_DEFAULT_WIN_URI="x11"' \
        -D_LINUX_ \
        -I"$project_root" \
        -I"$project_root/include" \
        -I"$project_root/include/CameraModels" \
        -I"$project_root/Thirdparty/DBoW2" \
        -I"$project_root/Thirdparty/g2o" \
        -I"$project_root/Thirdparty/Sophus" \
        -isystem "$sysroot/usr/include/eigen3" \
        -isystem "$sysroot/usr/include/opencv4" \
        -isystem "$sysroot/usr/include/libcamera" \
        -isystem "$sysroot/usr/local/include" \
        -std=c++23 \
        -mcpu=cortex-a76 \
        -O3 \
        -DNDEBUG \
        -Wall \
        -fPIC \
        -c "$project_root/$source" \
        -o "$object" >"$log" 2>&1; then
        printf 'Failed to compile %s:\n' "$source" >&2
        tail -n 80 "$log" >&2
        return 1
    fi
}

link_paths=(
    "$project_root/Thirdparty/DBoW2/lib"
    "$project_root/Thirdparty/g2o/lib"
    "$sysroot/usr/lib/aarch64-linux-gnu"
    "$sysroot/lib/aarch64-linux-gnu"
    "$sysroot/usr/local/lib"
    "$sysroot/usr/lib/gcc/aarch64-linux-gnu/14"
)
link_path_list="$(IFS=:; printf '%s' "${link_paths[*]}")"
target_runpath="$pi_project_root/lib:/usr/local/lib:$pi_project_root/Thirdparty/DBoW2/lib:$pi_project_root/Thirdparty/g2o/lib"

if [[ "${ORB_SLAM_SKIP_CORE:-0}" != 1 ]]; then
export -f compile_core_source
export trace_flag project_root cross_root sysroot compiler build_dir object_dir log_dir
printf '%s\n' "${core_sources[@]}" |
    xargs -r -n 1 -P "$build_jobs" bash -c 'compile_core_source "$1"' _

objects=()
for source in "${core_sources[@]}"; do
    objects+=("$object_dir/${source//\//_}.o")
done


if ! "$compiler" \
    --sysroot="$sysroot" \
    -mcpu=cortex-a76 \
    -O3 \
    -DNDEBUG \
    -shared \
    "-Wl,-soname,libORB_SLAM3.so" \
    "-Wl,-rpath-link,$link_path_list" \
    "-Wl,-rpath,$target_runpath" \
    -o "$temporary_library" \
    "${objects[@]}" \
    -L"$project_root/Thirdparty/DBoW2/lib" \
    -L"$project_root/Thirdparty/g2o/lib" \
    -L"$sysroot/usr/lib/aarch64-linux-gnu" \
    -L"$sysroot/lib/aarch64-linux-gnu" \
    -L"$sysroot/usr/local/lib" \
    -L"$sysroot/usr/lib/gcc/aarch64-linux-gnu/14" \
    -Wl,--no-as-needed \
    -l:libDBoW2.so \
    -l:libg2o.so \
    -l:libboost_serialization.so.1.83.0 \
    -l:libboost_thread.so.1.83.0 \
    -l:libboost_filesystem.so.1.83.0 \
    -l:libcrypto.so.3 \
    -l:libopencv_highgui.so.410 \
    -l:libopencv_imgcodecs.so.410 \
    -l:libopencv_calib3d.so.410 \
    -l:libopencv_features2d.so.410 \
    -l:libopencv_imgproc.so.410 \
    -l:libopencv_core.so.410 \
    -l:libpango_display.so.0 \
    -l:libpango_vars.so.0 \
    -l:libpango_opengl.so.0 \
    -l:libepoxy.so.0 \
    -l:libpango_core.so.0 >"$build_dir/link-core.log" 2>&1; then
    cat "$build_dir/link-core.log" >&2
    exit 1
fi

if grep -q 'warning:' "$build_dir/link-core.log"; then
    warning_count="$(grep -c 'warning:' "$build_dir/link-core.log")"
    printf 'Core link warnings: %s (details: %s)\n' "$warning_count" "$build_dir/link-core.log"
fi
else
    cp "$core_library" "$temporary_library"
fi

if ! "$compiler" \
    --sysroot="$sysroot" \
    -DCOMPILEDWITHC11 \
    -DHAVE_EIGEN \
    -DHAVE_EPOXY \
    -DHAVE_PANGOLIN \
    '-DPANGO_DEFAULT_WIN_URI="x11"' \
    -D_LINUX_ \
    -I"$project_root" \
    -I"$project_root/include" \
    -I"$project_root/include/CameraModels" \
    -I"$project_root/Thirdparty/DBoW2" \
    -I"$project_root/Thirdparty/g2o" \
    -I"$project_root/Thirdparty/Sophus" \
    -isystem "$sysroot/usr/include/eigen3" \
    -isystem "$sysroot/usr/include/opencv4" \
    -isystem "$sysroot/usr/include/libcamera" \
    -isystem "$sysroot/usr/local/include" \
    -std=c++23 \
    -mcpu=cortex-a76 \
    -O3 \
    -DNDEBUG \
    -Wall \
    -c "$project_root/Examples/Stereo-Inertial/stereo_inertial_icm20948.cc" \
    -o "$example_object" >"$compile_log" 2>&1; then
    cat "$compile_log" >&2
    exit 1
fi

if grep -q 'warning:' "$compile_log"; then
    warning_count="$(grep -c 'warning:' "$compile_log")"
    printf 'Example compile warnings: %s (details: %s)\n' "$warning_count" "$compile_log"
fi

if ! "$compiler" \
    --sysroot="$sysroot" \
    -mcpu=cortex-a76 \
    -O3 \
    -DNDEBUG \
    "$example_object" \
    "$temporary_library" \
    -L"$project_root/Thirdparty/DBoW2/lib" \
    -L"$project_root/Thirdparty/g2o/lib" \
    -L"$sysroot/usr/lib/aarch64-linux-gnu" \
    -L"$sysroot/lib/aarch64-linux-gnu" \
    -L"$sysroot/usr/local/lib" \
    -L"$sysroot/usr/lib/gcc/aarch64-linux-gnu/14" \
    "-Wl,-rpath-link,$link_path_list" \
    "-Wl,-rpath,$target_runpath" \
    -o "$temporary_output" \
    -l:libopencv_core.so.410 \
    -l:libopencv_imgproc.so.410 \
    -l:libcamera.so.0.7 \
    -l:libcamera-base.so.0.7 \
    -l:libpango_display.so.0 \
    -l:libpango_opengl.so.0 \
    -l:libepoxy.so.0 \
    -l:libm.so.6 \
    -l:libcrypto.so.3 \
    -l:libboost_serialization.so.1.83.0 >"$link_log" 2>&1; then
    cat "$link_log" >&2
    exit 1
fi

if grep -q 'warning:' "$link_log"; then
    warning_count="$(grep -c 'warning:' "$link_log")"
    printf 'Example link warnings: %s (details: %s)\n' "$warning_count" "$link_log"
fi

mv "$temporary_library" "$core_library"
mv "$temporary_output" "$output"
printf 'Built optimized AArch64 core library and target with -O3 (%s parallel jobs).\n' "$build_jobs"
printf 'Core library: %s\nTarget: %s\n' "$core_library" "$output"

if [[ "${ORB_SLAM_DEPLOY:-0}" != 1 ]]; then
    printf 'Deployment skipped. Set ORB_SLAM_DEPLOY=1 to copy the binaries to the Pi.\n'
    exit 0
fi
ssh -o BatchMode=yes "$pi_host" "mkdir -p '$pi_project_root/lib' '$pi_target_directory' '$pi_project_root/Thirdparty/DBoW2/lib' '$pi_project_root/Thirdparty/g2o/lib'"
scp -o BatchMode=yes "$project_root/Thirdparty/DBoW2/lib/libDBoW2.so" "$pi_host:$pi_project_root/Thirdparty/DBoW2/lib/libDBoW2.so"
scp -o BatchMode=yes "$project_root/Thirdparty/g2o/lib/libg2o.so" "$pi_host:$pi_project_root/Thirdparty/g2o/lib/libg2o.so"
scp -o BatchMode=yes "$core_library" "$pi_host:$pi_temporary_library"
scp -o BatchMode=yes "$output" "$pi_host:$pi_temporary_target"
ssh -o BatchMode=yes "$pi_host" \
    "chmod 644 '$pi_temporary_library' && chmod 755 '$pi_temporary_target' && mv -f '$pi_temporary_library' '$pi_library' && mv -f '$pi_temporary_target' '$pi_target'"
printf 'Copied optimized core library and target to %s:%s\n' "$pi_host" "$pi_project_root"

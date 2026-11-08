# ORB-SLAM3 (Hardened & Extended Fork)

A production-grade, hardened fork of **ORB-SLAM3 V1.0** featuring multi-session map persistence, pure relocalization, robust cross-map place recognition, memory and concurrency hardening, Three.js web visualization, and hermetic Nix flake integration.

---

## Current Status

- **Branch**: `master` (synchronized with PR #7 core stability patches and PR #8 map relocalization & extension).
- **Test Suite**: 100% pass rate on `orb_slam3_tests` (21/21 GoogleTest cases passing via CTest).
- **Build Toolchain**: Fully reproducible via Nix Flake (`flake.nix`), supporting modern GCC, CMake 4.x policy shims, OpenCV 4 with GTK3, and Pangolin 0.9.1.
- **Tested Hardware & Sensors**: Stereo (RealSense D435i), Monocular, Monocular-Inertial, Stereo-Inertial, and RGB-D.

---

## Raspberry Pi stereo-inertial input

`Examples/Stereo-Inertial/stereo_inertial_icm20948.cc` captures a stereo pair
with libcamera and ICM-20948 measurements over I2C. Configure the cameras and
IMU for your rig in `Examples/Stereo-Inertial/ICM20948_Kalibr.yaml`. Run
`calibrate_icm20948.py gyro` and `calibrate_icm20948.py accel` on the Pi with
the IMU available and SLAM stopped. The tracked camera and IMU calibrations are
specific to the current rig and must be replaced for other hardware.

When libcamera is available, CMake also provides the `recorder_icm20948`
target and `Examples/Calibration/run_icm20948_calibration.sh` launcher. The
recorder uses the same IMU ranges and calibration as the live driver.

On WSL, `./build_rpi5_target.sh` cross-builds the fork and its own DBoW2 and
g2o libraries using the sysroot in `.cross/rpi5/root`. The sysroot must match
the target Pi and is not tracked in Git. Set `ORB_SLAM_BUILD_JOBS` to change
the default of two parallel jobs. The build does not deploy by default;
`ORB_SLAM_DEPLOY=1 ./build_rpi5_target.sh` copies the binaries and libraries
to the separate `/home/pi/projects/ORB_SLAM3_Hardened_RPi` directory.
`ORB_SLAM_PI_HOST` and `ORB_SLAM_PI_ROOT` override that destination.
Provide the vocabulary, configuration, launcher and optional IMU calibration
at the destination before starting the application.

The TCP/RViz bridge and configuration are in `Examples/Stereo-Inertial`.
The bridge publishes tracking state and pose using this fork's frame snapshot.
The image feeds show tracked map points on the stereo frames.
On the WSL machine run `Examples/Stereo-Inertial/run_rviz2_icm20948.sh`;
on the Pi run `Examples/Stereo-Inertial/run_icm20948_slam.sh --viz-host WSL_IP`.
In `ICM20948_Kalibr.yaml`, set `Camera.captureFps` (camera capture),
`Camera.fps` (SLAM processing) and `Camera.exposureTimeUs` (maximum automatic
shutter in microseconds; previously a fixed shutter). The supplied starting
values are 30 FPS, 15 FPS and 15000 us.
The rates must be positive integers, with processing no higher than capture;
the shutter must be positive and shorter than the capture frame period. The
driver checks each camera's supported shutter range, but the sensor may still
clamp the actual shutter due to timing constraints. The application creates a
temporary Raspberry Pi 5 IMX219 tuning file from
`/usr/share/libcamera/ipa/rpi/pisp/imx219.json` (or the file specified by
`LIBCAMERA_RPI_TUNING_FILE`) before starting libcamera. Its normal AGC profile
prefers short exposures and increases analogue gain up to approximately 10x
before extending shutter up to the configured maximum. Both cameras use this
profile independently; their actual exposure and gain can differ. The log
periodically reports actual exposure, analogue gain and digital gain for each
camera. Building the libcamera stereo example requires Boost.JSON headers
(Boost 1.75 or newer). The calibration recorder retains its separate
fixed-shutter setting.
Longer exposures reduce noise in dark scenes
but increase motion blur. Non-integer capture/processing ratios select frames
at alternating intervals; processing load can reduce the actual input rate.
If the Pi cannot reach the WSL address, an SSH reverse tunnel can be used
instead: from WSL run `ssh -N -R 127.0.0.1:8765:127.0.0.1:8765 pi@rpi5.local`
and pass `--viz-host 127.0.0.1` on the Pi.

Stereo-inertial visual initialization does not require the initial acceleration
change. It waits for at least 50 usable stereo points (finite positive depths,
and for two-camera models valid matches and finite triangulated points), in
addition to the existing feature-count requirement. The threshold reflects
the tracker's 50-inlier requirement after relocalization; passing it cannot
guarantee that a subsequent frame will track successfully.

For stereo-inertial maps, place recognition starts after initial IMU
initialization rather than waiting for BA2; other inertial sensors still wait
for BA2. Recognition still requires at least 12 keyframes and the existing
geometric and scale checks. As in the previous project, an inertial merge
before BA2 runs bias optimization and marks BA1 and BA2 complete. Merging
also transforms the active map into the matched map's coordinate frame; the
`Merge alignment` log reports the applied rotation and scale, and an
`Early inertial merge` message reports the early BA state change.
For stereo-inertial merges, `Merge IMU shadow` compares the raw visual
alignment with a gyro-only prediction anchored at the last trusted pose of
the matched map. This is diagnostic only and never gates a merge. The model
starts with zero orientation and bias covariance because their actual initial
uncertainties are not known; `model_sigma_max_rad` therefore represents only
the modeled growth from configured gyro noise and bias random walk, not a
calibrated confidence bound. Gaps, missing snapshots, a different matched map,
or an unavailable anchor reference make the comparison unavailable. The
provisional shadow policy reports `accept` if the angular disagreement is at
most 10 degrees plus three times the modeled maximum rotation sigma; otherwise
it reports `defer`. It reports `disabled` if the anchor is older than 30 seconds,
the modeled sigma is at least 10 degrees, or required data are unavailable.
These deliberately provisional values are not a calibrated confidence interval
or a merge gate; the existing merge decision is unchanged in every case.
`IMU frame diagnostic` logs body roll, pitch and yaw (degrees, ZYX convention)
when the rotation discrepancy first reaches 3 degrees, alongside the gyro
prediction from the last trusted pose; another event is logged after the
discrepancy drops below the threshold and rises again. During a loss it also reports the separate merge accumulator's
relative rotation and anchor. The gyro prediction is reanchored on each trusted
frame, so it detects frame-to-frame discontinuities rather than measuring
long-term drift. Check map and anchor IDs before comparing world-frame angles;
the frames can differ during a map transition. Euler yaw is ambiguous near
pitch ±90 degrees; use the quaternion/rotation difference there.
This does not make accelerometer bias and velocity observable while stationary:
the later inertial optimization still needs suitable motion, and the BA1/BA2
motion-based timing remains unchanged.

The ICM-20948 example records a separate five-second gyro history, sized from
the configured IMU frequency, independently of the SLAM delivery queue. Once
an inertial map has a successfully tracked pose, that pose and its estimated
gyro bias anchor the history. Older samples are folded into a cumulative
rotation, so a reset lasting longer than five seconds can still recover the
orientation if the measurements are continuous. The new stereo map starts
with its camera position at the origin and uses the recovered orientation
instead of performing another gravity alignment. Subsequent visual-inertial
optimization can still adjust poses; gyro bias error causes drift while visual
tracking is unavailable. Before any trustworthy inertial pose exists, normal
gravity initialization is used. The history, anchor and inherited-orientation
flag are runtime-only; saving and reloading an atlas does not preserve them.

The history reports an error rather than inventing rotation when timestamps
are invalid, a sample gap exceeds ten nominal IMU periods, the frame is not
covered, or the measured sample rate exceeds its allocated capacity. The
separate SLAM delivery queue can also overflow while tracking is blocked; its
first overflow is reported and counted as `capacity_drops`. Rotation recovery
does not reconstruct the accelerometer samples dropped from that queue.

The example logs `[slam-orientation]` approximately every two seconds with
tracking state, map ID and camera-to-world quaternion `camera_Rwc_q_wxyz`.
Only compare poses with a valid pose and a trustworthy tracking state. On
recovery the `IMU orientation recovered` line reports the anchor time and
age, folded interval and segment count, whether any folded delta was used,
and quaternions in `w,x,y,z` order: anchor body-to-world, folded body-relative
delta, total body-relative delta, and recovered body-to-world orientation
`Rwb`. The folded delta can be identity even when segments were integrated.
Comparing `Rwb` with `Rwc` requires the calibrated camera-to-IMU rotation.

---

## Key Fork Improvements

This fork retains upstream ORB-SLAM3 algorithmic accuracy while significantly expanding system robustness, embedding safety, and deployment workflows:

### 1. Multi-Session Mapping & Map Extension
- **Bidirectional Atlas Persistence**: Save (`--save-map <path>`) and load (`--load-map <path>`) multi-map Atlas files (`.osa`).
- **PostLoad Active Map Resolution**: Automatically binds and activates the most relevant map upon loading, allowing continuous mapping and keyframe expansion across sessions.
- **Seamless Extension**: Start a session with a pre-existing map, accumulate new keyframes and map points in newly explored areas, and serialize the extended Atlas back to disk.
- **Path Normalization & Overrides**: Automatic file extension normalization (`.osa` appended if omitted) and flexible environment variable overrides (`ORB_SLAM3_SAVE_ATLAS`, `ORB_SLAM3_LOAD_ATLAS`).

### 2. Pure Relocalization Mode
- **Zero Map Mutation**: Run tracking in localization-only mode (`--localize-only` or `ORB_SLAM3_LOCALIZE_ONLY=1`), disabling `LocalMapping` and freezing map points/keyframes for zero memory drift during repetitive routes.
- **Cross-Map Relocalization Fallback**: If the active map yields no relocalization candidates in a multi-map Atlas, the query automatically falls back across all inactive maps in the Atlas database.
- **Safe Dynamic Map Adoption**: When a match is found in an inactive map, the tracker synchronizes map pointers (`pCurrentMap`), safely transfers map update locks (`mMutexMapUpdate`), and adopts the target map seamlessly without race conditions or dropped frames.
- **Continuity Preservation**: Tracking retains state, velocity, and motion models upon successful relocalization in mapping mode instead of aborting the frame pipeline.

### 3. Concurrency, Memory & Lifecycle Hardening
- **Safe Thread Shutdown**: Background threads (`LocalMapping`, `LoopClosing`) employ timed joins with leak-on-deadlock safety guards, ensuring `System::Shutdown()` never indefinitely blocks host processes.
- **Thread-Safe KeyFrameDatabase**: Added mutex protection and caching for `queryScores` during candidate detection, eliminating data races and uninitialized score reads.
- **Bounded Tombstones**: Culled `KeyFrame` snapshot tombstones in `Map` are constrained to a strict FIFO bounded queue (max 1,000) to prevent unbounded memory growth during long-running sessions.
- **Atlas & Memory Cleanup**: Destructor `Atlas::~Atlas()` systematically releases bad maps, `Atlas::PreSave()` purges stale backup map lists, and raw pointers in `KeyFrame` constructors are strictly zero-initialized.
- **Algorithmic Bounds & Null Safety**:
  - Null pointer and culled keyframe guards added in `OptimizeEssentialGraph` and `CorrectLoop`.
  - Observation descriptor bounds checking in `ComputeDistinctiveDescriptors` and stereo `rightIndex`.
  - Removed unsynchronized `ChangeMap` invocations during localization mode switches.

### 4. Safe Value-Based Snapshots
- **No Leaked Pointers**: `System::GetLastFrameSnapshot()` and `System::GetGraphSnapshot()` expose frame tracking state and keyframe-graph topology without leaking raw internal ORB-SLAM3 object pointers.
- **Consistent Graph View**: Snapshots provide map identity, monotonic revision counters, keyframe poses, parent/loop edges, and culled tombstones so external ROS/web bridges retain a consistent graph view.
- **Multi-Lap Revisits**: Covisibility-connected loop candidates separated by at least 20 keyframe IDs remain eligible for geometric verification, enabling revisit loop closures on subsequent laps while filtering immediate neighbors.

### 5. Dual Visualization: Modern Web UI & Pangolin
- **Three.js Web Visualizer**: Integrated WebSocket backend streaming camera trajectory, keyframe frustums, and point clouds directly to a browser UI (`web_viewer/`).
- **Native Pangolin**: Maintained and isolated (Pangolin 0.9.1 via `nixpkgs-pango`) with real-time pacing, window lifecycle cleanup, and headless execution support.

---

## Quickstart with Nix (Recommended)

The easiest and most reproducible way to build and run the project is using the included Nix Flake:

### 1. Enter the Nix Development Shell
```bash
nix develop
```

This sets up a complete environment with GCC, CMake, OpenCV 4 (GTK3 enabled), Eigen3, Boost, OpenSSL, GTest, Node.js, and Pangolin.

### 2. Prepare Vocabulary and Build Thirdparty
```bash
# Extract pre-trained ORB vocabulary (gitignored)
tar -xf Vocabulary/ORBvoc.txt.tar.gz -C Vocabulary/

# Build Thirdparty dependencies (DBoW2, g2o, Sophus) and ORB-SLAM3
./build.sh
```

Alternatively, if Thirdparty libraries are already compiled:
```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) ORB_SLAM3 stereo_benchmark orb_slam3_tests
cd ..
```

### 3. Run the Automated Test Suite
```bash
ctest --test-dir build --output-on-failure
# Or execute GoogleTest suite directly (21 test cases across 5 suites):
./build/tests/orb_slam3_tests
```

---

## CLI Usage & Examples

The unified `stereo_benchmark` binary supports standard dataset execution, live visualization, map saving/loading, and relocalization.

> **Note**: Compiled runner executables reside in their respective `Examples/<Sensor>/` directories (e.g., `Examples/Stereo/stereo_benchmark`).

### Positional Syntax
```bash
./Examples/Stereo/stereo_benchmark \
  <path_to_vocabulary> \
  <path_to_settings> \
  <path_to_dataset> \
  [output_json] \
  [crash_report_dir] \
  [run_label] \
  [OPTIONS]
```

### Common Workflows

#### 1. Mapping and Saving Atlas
Build a new map from a dataset sequence and save the resulting Atlas:
```bash
./Examples/Stereo/stereo_benchmark \
  Vocabulary/ORBvoc.txt \
  Examples/Stereo/RealSense_D435i.yaml \
  dataset/session_1 \
  benchmark_run1.json /tmp/crashes run1 \
  --save-map /path/to/maps/area_map.osa \
  --viewer
```

#### 2. Extending an Existing Map
Load a previously saved map, continue mapping a new trajectory, and save the expanded map:
```bash
./Examples/Stereo/stereo_benchmark \
  Vocabulary/ORBvoc.txt \
  Examples/Stereo/RealSense_D435i.yaml \
  dataset/session_2 \
  benchmark_run2.json /tmp/crashes run2 \
  --load-map /path/to/maps/area_map.osa \
  --save-map /path/to/maps/extended_area_map.osa \
  --viewer
```

#### 3. Pure Relocalization Mode
Localize against a pre-built map without modifying or adding new map points:
```bash
./Examples/Stereo/stereo_benchmark \
  Vocabulary/ORBvoc.txt \
  Examples/Stereo/RealSense_D435i.yaml \
  dataset/reloc_test \
  benchmark_reloc.json /tmp/crashes reloc \
  --load-map /path/to/maps/extended_area_map.osa \
  --localize-only \
  --viewer
```

---

## CLI Flags & Environment Variables

| CLI Flag | Environment Variable | Description |
| :--- | :--- | :--- |
| `--save-map <path>` or `--save-map=<path>` | `ORB_SLAM3_SAVE_ATLAS` | Path to save the serialized Atlas (`.osa`) upon completion. |
| `--load-map <path>` or `--load-map=<path>` | `ORB_SLAM3_LOAD_ATLAS` | Path to an existing Atlas file (`.osa`) to load before processing. |
| `--localize-only` or `--loc-only` | `ORB_SLAM3_LOCALIZE_ONLY=1` or `ORB_SLAM3_LOCALIZATION_MODE=1` | Disables map building (`LocalMapping`); performs pure relocalization and tracking. |
| `--viewer` or `--pangolin` | `VIEWER=1` or `USE_PANGOLIN=1` | Enables the native Pangolin 3D viewer window. |

---

## Web Visualizer

A lightweight, modern web visualizer built on Vite and Three.js is provided in `web_viewer/`.

### 1. Launch C++ Backend Runner
```bash
./Examples/Stereo/stereo_web_runner \
  Vocabulary/ORBvoc.txt \
  Examples/Stereo/RealSense_D435i.yaml \
  path/to/sequence
```
*(The backend defaults to WebSocket port `8080`, or port `8085` if configured in the YAML settings via `WebViewer.Port: 8085`).*

### 2. Launch Web Frontend
```bash
cd web_viewer
npm install
npm run dev
```
Open `http://localhost:3000` in any modern browser to view the 3D trajectory and map point cloud in real time.

---

## Repository Structure

```text
ORB_SLAM3/
├── include/              # Public headers (System.h, Atlas.h, Tracking.h, etc.)
├── src/                  # Core SLAM implementation & stability hardening
├── Examples/             # Runners for Monocular, Stereo, RGB-D, and Inertial
│   └── Stereo/           # stereo_benchmark, stereo_web_runner, settings
├── web_viewer/           # Vite + Three.js browser visualizer
├── tests/                # GoogleTest suite (Atlas lifecycle, crash monitor, web mirror)
├── flake.nix             # Hermetic Nix development environment
└── CMakeLists.txt        # Top-level CMake build configuration
```

---

## License

This fork retains the original GNU General Public License v3.0 (GPLv3) from ORB-SLAM3. See [LICENSE](LICENSE) for details.

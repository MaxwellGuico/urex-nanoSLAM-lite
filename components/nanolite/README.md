# NanoSLAM-Lite

NanoSLAM-Lite is a fixed-memory 2D SLAM package for the UREX ESP32-S3 platform. It follows the
NanoSLAM information flow while using smaller, deterministic data sets that can coexist with PX4
telemetry, eight VL53L5CX sensors, and camera work.

The package is feature-complete for bench and replay testing. It is **not flight-validated** and its
default sensor, ICP, and loop-acceptance thresholds are starting values that must be calibrated on
the assembled vehicle.

## Included pipeline

```text
PX4 optical-flow/IMU pose + reduced ToF ring
                    |
                    v
       key poses + live occupancy map
                    |
                    v
       160-point bounded scan builder
                    |
                    v
       conservative revisit candidate
                    |
                    v
          10-iteration bounded ICP
                    |
                    v
            loop-closure edge
                    |
                    v
       3-pass bounded graph correction
                    |
                    v
        corrected occupancy-map replay
```

The scan-matching sequence is based on the NanoSLAM paper's ICP flow
([PDF pp. 6-7](../../papers/2309.12008.pdf)), with smaller fixed limits for the ESP32-S3. The graph
correction follows the paper's loop-edge then pose-correction flow ([PDF pp. 7, 9, and
11](../../papers/2309.12008.pdf)). This code is an independent bounded implementation; the pinned
GPL-3.0 upstream checkout is used as a behavioral reference and is not copied into this package.

## Modules

| File | Job |
|---|---|
| `nanolite_tof.*` | Reduces an 8x8 ToF frame to one valid range and bearing column. |
| `nanolite_core.*` | Stores 64 key poses, odometry links, and up to 8 loop links. |
| `nanolite_map.*` | Maintains the packed 40x40, 0.5 m/cell occupancy grid. |
| `nanolite_scan.*` | Aggregates up to 160 points and retains 16 scans in a fixed ring. |
| `nanolite_icp.*` | Matches two scans with bounded 2D point-to-point ICP and acceptance gates. |
| `nanolite_optimize.*` | Corrects an anchored SE(2) pose graph using bounded constraint relaxation. |
| `nanolite_slam.*` | Runs the complete pipeline and keeps raw odometry aligned after corrections. |

No hot-path function allocates memory. `nanolite_slam_t` owns all working buffers and occupies about
29 KiB with the current limits (the exact `sizeof` is compiler-dependent). Capacity is explicit:
poses stop at 64, loop links stop at 8, scan points stop at 160, and the oldest of 16 retained scans
is replaced deterministically.

## Run NanoSLAM-Lite on the ESP32-S3

This is the shortest repeatable path from a checkout to a rendered props-off map. It uses the
integration in `firmware/nus_flight_app`; the standalone package does not communicate with the
hardware by itself.

### 1. Prepare the bench

Required hardware and software:

- Seeed Studio XIAO ESP32S3 Sense running the NUS flight application;
- PX4 telemetry connected to the configured ESP32-S3 UART, including a common ground;
- eight VL53L5CX sensors connected through the TCA9548A mux;
- a USB data cable for flashing and monitoring;
- ESP-IDF 6.0.2 and Python 3; and
- a terminal opened at the repository root.

Before powering the system:

1. Remove every propeller and keep the vehicle disarmed.
2. Check that the ToF ring, mux, PX4 UART, and USB connections are secure.
3. Put the drone level above a matte, textured floor so optical-flow odometry can work.
4. Keep people and moving objects outside the ToF test area while recording a baseline.

NanoSLAM-Lite is not flight-validated. Bench mode intentionally does not create the navigation or
mission tasks, and the NanoSLAM-Lite task never sends a setpoint to PX4.

### 2. Run the host tests

The host tests do not require the drone. Run them before flashing whenever the SLAM package changes:

```bash
cmake -S components/nanolite -B /tmp/nanolite-build
cmake --build /tmp/nanolite-build
ctest --test-dir /tmp/nanolite-build --output-on-failure
```

They cover pose selection and capacity, ToF filtering, map projection, scan aggregation, synthetic
ICP recovery, graph correction, map rebuilding, and an end-to-end synthetic revisit.

### 3. Export ESP-IDF and configure the application

Export an ESP-IDF 6.0.2 installation in the current shell. Replace the example path with the path on
your computer:

```bash
source /path/to/esp-idf/export.sh
idf.py --version
cd /path/to/nus_flight_app
```

For a new build directory, select the ESP32-S3 target once:

```bash
idf.py set-target esp32s3
```

Review the hardware and bench settings:

```bash
idf.py menuconfig
```

They are under **Drone Configuration**. Confirm at least the following:

| Setting | Bench value or action |
|---|---|
| NanoSLAM-Lite props-off bench mode | Enabled |
| Pose observation rate | 20 Hz |
| Maximum pose age | 250 ms |
| Translation/yaw key-pose threshold | 25 cm / 15 degrees |
| Mapping range | 2000 mm |
| Maximum ToF age | 250 ms |
| Reduced ToF calibration records | Enabled |
| Packed map snapshots | Enabled, every 10 seconds |
| Front-facing ToF sensor index | Set to the physically forward sensor |
| TCA9548A SDA/SCL pins | Match the wired board |
| PX4 TX/RX pins | Match the wired UART; TX and RX must be crossed |
| Wi-Fi and host IPv4 settings | Match the test network, if used |

Wi-Fi is not required for serial map capture. Its connection process is non-blocking, so an absent
access point must not prevent PX4, ToF, camera, or NanoSLAM-Lite startup.

### 4. Build, flash, and capture a complete log

Build the complete application:

```bash
idf.py build
```

Set the serial device and a unique log name. `/dev/ttyACM0` is an example; use the device reported by
your operating system:

```bash
PORT=/dev/ttyACM0
RUN_LOG=/tmp/nanolite-small-area.log
idf.py -p "$PORT" flash
idf.py -p "$PORT" monitor 2>&1 | tee "$RUN_LOG"
```

Press `Ctrl+]` to exit the ESP-IDF monitor. If the correct image is already installed, reboot the
ESP32-S3 and run only the monitor command to start a fresh experiment.

### 5. Check startup before moving the drone

Wait for all of these conditions:

| Log text | Meaning |
|---|---|
| `NANOLITE BENCH MODE` | Navigation and mission flight tasks are disabled. |
| `read-only PX4 pose bridge ready` | NanoSLAM-Lite is sampling the PX4 pose. |
| `8 / 8 sensors ready` | Every ToF sensor initialized and started ranging. |
| `RX/5s` near 100 position and 100 attitude messages | PX4 is publishing both streams near 20 Hz. |
| `graph/map reset` | A fresh pose became the map anchor. |
| `NANOTOF` with non-zero `valid` and `fresh` masks | Reduced ToF returns are usable and timely. |
| `map: rays=...` increasing | ToF rays are reaching the occupancy map. |

Both ages in `RX/5s` should remain below 250 ms. Do not continue when bench mode is absent, fewer
than eight sensors are ready, pose data is stale, or the ESP32-S3 reports a panic, watchdog reset,
or allocation failure.

### 6. Record a small-area SLAM run

Start with a measured, static wall or corner within the 2 m mapping gate. Mark the initial drone
centre and heading on the floor.

1. Hold the drone stationary for at least 20 seconds to establish a baseline map.
2. Keep it level and move it slowly along a measured route. Cross the 25 cm marks deliberately.
3. Include a corner or another distinctive shape; a single long wall is ambiguous for ICP.
4. Complete a loop and revisit an earlier location with a similar heading.
5. Continue until at least one post-loop `NANOMAP` record has appeared.
6. Stop the monitor and save the complete log.

The task stores a key pose after either 25 cm translation or 15 degrees of yaw. It can store 64
poses, including the anchor. At 25 cm translation-only spacing that is a nominal 15.75 m trajectory;
turning consumes additional pose slots. The current graph does not delete old poses: at capacity it
prints `pose graph full at 64 poses` and stops advancing the graph. Reboot between independent runs.

A scan accumulates de-duplicated reduced returns until the next key pose, requires at least 12
points, and is capped at 160 points. Loop matching is not expected in a stationary run. A revisit
candidate must be separated by at least six key poses and pass the distance, heading, overlap, RMSE,
improvement, and maximum-correction gates.

### 7. Render and inspect the map

Return to the repository root and render the latest complete `NANOMAP` snapshot:

```bash
cd ../..
./tools/nanolite-map-view.py /tmp/nanolite-small-area.log
```

The ASCII legend is:

| Symbol | Meaning |
|---|---|
| `?` | Unknown cell |
| `.` | Observed free cell |
| `+` | Tentative occupied endpoint |
| `#` | Confirmed occupied cell |
| `A` | Initial map anchor |
| `R` | Latest corrected robot cell |

The current map is 40 x 40 cells at 0.5 m/cell, covering a fixed 20 x 20 m area. It is deliberately
coarse, so a wall may appear thick, stepped, or discontinuous rather than as a clean line.

Extract a compact health summary from the same log:

```bash
rg "NANOLITE BENCH MODE|read-only PX4 pose bridge|graph/map reset|sensors ready|RX/5s|NANOTOF|nanolite: pose|nanolite: map:|loop accepted|loop rejected|pose graph full|panic|watchdog" "$RUN_LOG"
```

For each accepted loop, check that overlap is adequate, RMSE decreases, the correction is physically
plausible, and graph cost decreases. A logged `loop accepted` is evidence that the software gates
passed; it is not by itself proof that the match is geometrically correct.

### 8. Decide whether the run passed

A useful props-off run has:

- eight healthy sensors and fresh PX4 position and attitude throughout;
- increasing ray, observed-cell, scan, and active-point counts;
- zero or explainable dropped rays while the drone remains inside the map;
- key-pose increments near the measured 25 cm and 15 degree marks;
- stationary walls that remain fixed while the reported robot pose moves;
- no panic, watchdog, repeated camera overflow, or persistent sensor reinitialization; and
- for a loop test, a recorded accepted or rejected ICP decision that agrees with the measured
  geometry.

Save the firmware revision, `sdkconfig`, complete monitor log, rendered map, physical dimensions,
initial heading, sensor facing the reference wall, motion performed, test duration, floor, lighting,
and approximate vehicle height with every result.

## Troubleshooting

| Symptom | Checks |
|---|---|
| Serial port cannot be opened | Confirm the USB data cable, device name, permissions, and that no other monitor owns the port. |
| `waiting for fresh PX4 position and attitude` | Check crossed UART TX/RX, common ground, configured pins, baud rate, and PX4 message publication. |
| Fewer than `8 / 8 sensors ready` | Check sensor power, TCA9548A address/channel wiring, SDA/SCL pins, and I2C connections. |
| `NANOTOF` is present but `valid=0x00` | Put a matte obstacle 0.05-2.0 m from the sensors and check target-status/range filtering. |
| `valid` is non-zero but `fresh` is zero | Check ToF frame timestamps, sensor recovery messages, task delays, and the 250 ms age gate. |
| Rays increase but no cell becomes `#` | Keep the wall still; the same endpoint cell needs repeated observations before confirmation. |
| No new key poses | Move more than 25 cm or rotate more than 15 degrees while PX4 data remains fresh. |
| No completed scans | Ensure valid ToF points accumulate before crossing the next key-pose threshold. |
| No loop attempt | Revisit a retained scan after at least six key poses, within the configured position/yaw gates. |
| Every loop is rejected | Inspect point count, overlap, RMSE, correction size, sensor yaw, column sign, and pose/ToF alignment. |
| No `NANOMAP` line | Confirm bench mode and packed map snapshots are enabled, then wait for the configured interval. |
| Graph reaches 64 poses | End/reboot the run or deliberately change the fixed-capacity design; poses are not deleted automatically. |

The longer staged experiment procedure is in
[`../nus_flight_app/experiments/README.md`](../nus_flight_app/experiments/README.md), and integration
details are in [`../nus_flight_app/UREX_INTEGRATION.md`](../nus_flight_app/UREX_INTEGRATION.md).

## Minimal API use

Applications other than `nus_flight_app` can drive the package through its high-level API:

```c
#include "nanolite_slam.h"

static nanolite_slam_t slam;

void start(const nanolite_pose_t *anchor)
{
    const float sensor_yaw_rad[NANOLITE_SENSOR_COUNT] = {
        0.0f, 0.7853982f, 1.5707963f, 2.3561945f,
        3.1415927f, 3.9269908f, 4.7123890f, 5.4977871f,
    };
    nanolite_slam_config_t config = nanolite_default_slam_config();
    (void)nanolite_slam_init(&slam, &config, anchor, sensor_yaw_rad);
}

void update(const nanolite_pose_t *pose,
            const nanolite_tof_return_t returns[NANOLITE_SENSOR_COUNT])
{
    (void)nanolite_slam_observe(&slam, pose, returns,
                                NANOLITE_SENSOR_COUNT);
}
```

`nanolite_slam_observe()` ignores duplicate ToF timestamps. When a new key pose is created, it
closes the preceding scan, checks one conservative revisit candidate, accepts only a gated ICP
match, corrects the graph, updates the odometry-to-map transform, and rebuilds the map from retained
scan points. Call `nanolite_slam_finalize_active_scan()` to close the last scan in a replay. Use
`nanolite_slam_correct_odometry_pose()` when a consumer needs the latest live pose in the corrected
map frame.

## Important limitations

- The current firmware reduces 512 raw zones from the eight 8x8 ToF sensors to at most eight points
  per fresh ring update, then accumulates those points into a scan.
- ICP uses nearest-neighbor point-to-point matching; repeated or feature-poor walls can be
  ambiguous, so acceptance thresholds must be learned from recorded local data.
- The optimizer is a bounded constraint-relaxation solver, not NanoSLAM's GAP9 sparse Cholesky
  solver.
- The 64-pose graph saturates; it does not yet roll over into hierarchical submaps or offload pose
  graph optimization.
- The map covers a fixed local 20 x 20 m window and does not slide when the vehicle leaves it.
- Sensor yaw, column direction, field of view, effective range, and timestamp alignment require
  physical calibration.
- Passing host tests or compiling the ESP-IDF image does not establish localization accuracy or
  flight safety.

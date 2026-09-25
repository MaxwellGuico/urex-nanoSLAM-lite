# UREX NanoSLAM-Lite integration

This directory is based on the public NUS SAFMC flight application at
<https://github.com/nus-safmc/esp-everything>, pinned at commit
`99cde05b1acf568a86a642451af3e7f19a6af048` when imported.

The upstream repository did not contain a top-level licence file when it was
imported. Keep this working copy internal unless the upstream licensing terms
are clarified.

## Current NanoSLAM-Lite milestone

`main/nanolite_task.c` is a read-only bridge from the fused PX4 state and ToF
ring to the fixed-capacity NanoSLAM-Lite pose graph and occupancy map. It:

- samples `LOCAL_POSITION_NED` and `ATTITUDE` at 20 Hz;
- rejects position or attitude older than 250 ms;
- stores a pose after 0.25 m translation or 15 degrees yaw;
- selects one return per sensor using the valid minimum from zero-based rows 3
  and 4, limited to 50-2000 mm;
- integrates each fresh return into a 40x40, 0.5 m/cell packed map;
- requires two endpoint observations in the same cell before marking it
  occupied;
- emits a versioned, 1 Hz `NANOTOF` calibration record in bench mode with all
  eight reduced distances, winning columns, validity/freshness masks, source
  timestamps, and ages;
- keeps independent VL53L5CX driver state and work buffers for all eight mux
  channels, transfers only the distance/target outputs consumed by SLAM, and
  bounds I2C transactions to 100 ms. Both mux-control and downstream sensor
  transactions use the TCA9548A's rated 400 kHz limit. The ring runs at 10 Hz
  to leave bus-bandwidth margin and rapidly sweeps the short readiness headers before
  transferring complete frames. A sensor is invalidated after three seconds
  without a fresh frame and queued to a lower-priority recovery worker. Mux
  selection and each sensor transfer are atomic, while large recovery writes
  are divided into 256-byte transactions so healthy channels continue being
  polled between recovery chunks. Stale mapping data is never retained;
- aggregates up to 160 de-duplicated points in each key-pose scan and retains
  16 scans in a deterministic ring;
- considers only spatially and directionally plausible revisits separated by
  at least six key poses;
- runs at most 10 ICP iterations, with overlap, residual, improvement,
  translation, and yaw acceptance gates;
- stores accepted matches as loop edges, performs three bounded graph
  correction passes, updates the odometry-to-map transform, and regenerates
  the map from retained scan points;
- resets the graph on the transition to armed, giving each flight a clean
  anchor; and
- runs at priority 1 on core 0, below MAVLink, ToF, Wi-Fi, and navigation.

It does **not** send MAVLink messages, alter setpoints, change the vehicle's
flight estimator, or feed its corrected map into navigation. Those boundaries
keep bench evaluation observational until the acceptance tests pass.

`CONFIG_NANOLITE_BENCH_MODE` defaults to enabled. In that configuration, the
navigation and mission tasks are not spawned, so the development image cannot
arm or fly the vehicle. MAVLink telemetry, ToF, Wi-Fi, AprilTag/camera
detection, and NanoSLAM-Lite remain active for concurrent-load testing.

Wi-Fi initialization is non-blocking. If the configured access point is not
available, its event handler continues reconnecting while MAVLink, ToF, camera,
and NanoSLAM-Lite start normally. The original blocking initialization prevented
all sensing and localization tasks from starting during the first bench test.

The connected XIAO ESP32S3 Sense reports 8 MB of physical flash. The imported
configuration declared 2 MB, so the UREX configuration now declares 8 MB and
removes the boot-time size mismatch warning.

The first complete bench boot received an initial PX4 pose but the position and
attitude streams then became stale. The MAVLink task now requests
`LOCAL_POSITION_NED` and `ATTITUDE` at 20 Hz using
`MAV_CMD_SET_MESSAGE_INTERVAL`, retries the request only while either stream is
stale, and reports five-second receive counts and message ages. This command
changes telemetry publication only; it does not arm the vehicle or enable
offboard mode.

The canonical NanoSLAM-Lite core is self-contained in `components/nanolite` so a fresh clone has
the source, host tests, and ESP-IDF component together. Its `CMakeLists.txt` supports both the
standalone host-test build and ESP-IDF component registration.

ESP-IDF 6 splits the I2C and GPIO APIs into `esp_driver_i2c` and
`esp_driver_gpio`; the imported VL53L5CX component's CMake requirements include
those components explicitly for IDF 6 compatibility.

The main component likewise declares the split GPIO, I2C, and UART driver
components required by its public headers when building with ESP-IDF 6.

The pinned `esp32-camera` 2.1.5 component also uses `hal/clk_gate_ll.h` on
ESP32-S3 with IDF 6.0, so its private requirements include `esp_hal_clock` for
IDF 6.0 and newer. This preserves the pinned camera code rather than silently
upgrading it during the SLAM integration.

The review configuration leaves `CONFIG_APRILTAG_CAMERA_ENABLED` disabled. MAVLink, ToF, Wi-Fi,
and NanoSLAM-Lite remain pinned to Core 0; bench mode does not start navigation or mission tasks,
so Core 1 has no project task. The AprilTag task and the camera driver's internal worker are both
assigned to Core 1 when camera support is enabled later.

The graph sequencing follows NanoSLAM: odometry edges connect consecutive
poses, and graph correction begins only after a scan match provides a valid
loop-closure edge (`papers/2309.12008.pdf`, pp. 7 and 9).

ToF endpoints use the scan projection described by NanoSLAM
(`papers/2309.12008.pdf`, p. 6), adapted to the UREX eight-sensor ring and its
one-minimum-per-sensor row-3/row-4 policy. No-target or rejected returns leave
space unknown rather than clearing it.

Bench builds emit a packed `NANOMAP` snapshot every ten seconds. Capture the
monitor output and render its latest snapshot with
`tools/nanolite-map-view.py`. The dump is disabled automatically when bench
mode is disabled, keeping it out of the flight-time logging path.

## Props-off acceptance test

The repeatable setup, log-capture commands, pass/fail checks, and staged mapping experiments are
documented in [`experiments/README.md`](experiments/README.md).

1. Connect PX4 telemetry and start the ESP32-S3 with propellers removed.
2. Confirm `read-only PX4 pose bridge ready` appears.
3. Confirm the MAVLink `RX/5s` line reports roughly 100 position and 100
   attitude messages, with both ages below 250 ms.
4. Confirm `graph reset` appears after fresh position and attitude arrive.
5. Move the vehicle more than 0.25 m or rotate it more than 15 degrees.
6. Confirm `pose 2/64` appears, followed by another increment for each threshold
   crossing.
7. Confirm the five-second `map:` summary shows increasing `rays` and
   `observed`; repeated wall returns should eventually increase `occupied`.
8. Confirm key-pose lines contain a nonzero `scan=0x..` mask once the ToF ring
   is ready.
9. Carry the props-off vehicle around a measured loop and confirm the map
   summary reports growing `scans` and `active_pts`. Treat every `loop
   accepted` record as a proposal to inspect against the recorded ground truth;
   its pair count, overlap, RMSE, correction, and graph cost are logged.
10. Keep bench mode enabled; navigation and mission must not appear in the task
   startup sequence.

Do not let NanoSLAM-Lite influence flight navigation yet. The end-to-end
software path now exists and the ESP-IDF image builds, but the sensor geometry,
loop thresholds, false-match rate, execution time, stack margin, correction
quality, and map accuracy still require props-off replay and bench testing.

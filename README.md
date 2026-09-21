# UREX ESP32-S3 flight firmware

This repository contains the current ESP-IDF firmware for the UREX/NUS SAFMC drone. The review
snapshot integrates PX4 MAVLink telemetry, an eight-sensor VL53L5CX ToF ring, AprilTag camera
processing, Wi-Fi telemetry, and the fixed-memory NanoSLAM-Lite bench pipeline.

> **Safety:** NanoSLAM-Lite is not flight-validated. Keep the propellers removed and
> `CONFIG_NANOLITE_BENCH_MODE=y` while reproducing these experiments.

## Repository layout

| Path | Purpose |
| --- | --- |
| `main/` | ESP32-S3 application tasks and hardware integration |
| `components/nanolite/` | Self-contained NanoSLAM-Lite source, host tests, and design notes |
| `components/rjrp44__vl53l5cx/` | VL53L5CX driver with the project's ESP-IDF integration changes |
| `components/espressif__esp32-camera/` | Camera driver used by the AprilTag task |
| `components/esp-apriltag/` | AprilTag detector |
| `components/mavlink/` | MAVLink component and generated C headers |
| `laptop/` | Host-side mission, relay, visualization, and diagnostic tools |
| `experiments/` | Bench procedure and the latest serial logs, grouped by ToF power source |
| `UREX_INTEGRATION.md` | Integration status and known limitations |

Generated build output, editor databases, Python caches, and `sdkconfig.old` are intentionally
excluded from version control.

## Build

The latest hardware run used ESP-IDF 6.0.2 and a Seeed Studio XIAO ESP32S3 Sense.

```bash
git clone --recurse-submodules <repository-url>
cd nus_flight_app

idf.py set-target esp32s3
idf.py menuconfig
idf.py build
```

Under **Drone Configuration**, replace `YOUR_WIFI_SSID`, `YOUR_WIFI_PASSWORD`, and the documentation
address `192.0.2.1` with local values if Wi-Fi telemetry is required. Serial-only bench logging does
not require Wi-Fi.

To run the portable NanoSLAM-Lite tests without hardware:

```bash
cmake -S components/nanolite -B /tmp/nanolite-build
cmake --build /tmp/nanolite-build
ctest --test-dir /tmp/nanolite-build --output-on-failure
```

## Latest bench logs

The preserved logs are indexed in [`experiments/README.md`](experiments/README.md#preserved-log-index).
Power conditions are explicit in the directory names:

- `experiments/with-external-tof-power/`
- `experiments/without-external-tof-power/`

The logs are evidence from props-off development runs, including failures. They are not proof that
the firmware is ready for flight.

## Review focus

The most relevant project changes are the NanoSLAM-Lite integration in `main/nanolite_task.c`, ToF
acquisition and recovery in `main/tof_task.c`, PX4 pose handling in `main/mavlink_task.c`, and the
bounded SLAM implementation in `components/nanolite/`. See `UREX_INTEGRATION.md` for the current
status and `experiments/README.md` for reproduction steps and interpretation guidance.

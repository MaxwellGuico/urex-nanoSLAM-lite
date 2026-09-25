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
./scripts/idf_logged.sh build
```

Use `scripts/idf_logged.sh` instead of calling `idf.py` directly whenever terminal output must be
kept. Each invocation prints and saves the exact command, timestamp, Git revision, power label,
output, and exit status. A sibling `.evidence` directory preserves the full commit ID, tracked
binary patch, untracked files, `sdkconfig`, defaults, partition/dependency inputs, and hashes of the
resulting ELF and binaries. This makes dirty development images reproducible without silently
committing unrelated work. Routine logs and evidence go into the ignored `run-logs/` directory so
they do not accidentally bloat Git history. Evidence can contain Wi-Fi credentials from
`sdkconfig`; keep it private or scrub secrets before sharing. Copy only review-worthy hardware logs
into the matching `experiments/with-external-tof-power/` or
`experiments/without-external-tof-power/` directory.

For a flashed hardware run, explicitly record the ToF power arrangement:

```bash
./scripts/idf_logged.sh --power external -p /dev/ttyACM0 flash monitor
# Or, when the ToF ring has no external supply:
./scripts/idf_logged.sh --power no-external -p /dev/ttyACM0 flash monitor
```

Under **Drone Configuration**, replace `YOUR_WIFI_SSID`, `YOUR_WIFI_PASSWORD`, and the documentation
address `192.0.2.1` with local values if Wi-Fi telemetry is required. Serial-only bench logging does
not require Wi-Fi.

The review configuration keeps **Enable AprilTag camera task** disabled. MAVLink, ToF, Wi-Fi, and
NanoSLAM-Lite run on Core 0; no project task is started on Core 1 in bench mode. The camera code is
retained and both its detector and driver are assigned to Core 1 when the option is enabled later.

To run the portable NanoSLAM-Lite tests without hardware:

```bash
cmake -S components/nanolite -B /tmp/nanolite-build
cmake --build /tmp/nanolite-build
ctest --test-dir /tmp/nanolite-build --output-on-failure
python3 -m unittest -v laptop/test_nanolite_run_view.py
```

Bench firmware emits matching `NANOMAP` and `NANOGRAPH` snapshots. Render the
latest complete pair from a recorded monitor log as a PNG:

```bash
./laptop/nanolite_run_view.py run-logs/with-external-tof-power/<run>.log
```

The figure overlays the corrected key-pose trajectory on the occupancy map and
shows the raw odometry reconstruction, optimized graph, scan poses, heading
samples, and accepted loop-closure edges in a second panel. The output defaults
to `<run>-nanolite.png` beside the input log. Matplotlib is required on the
laptop; it is not part of the embedded firmware. Onboard projection aligns
each muxed sensor return with an interpolated PX4 pose at that return's own
timestamp instead of assigning the complete ring to the newest pose.

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

# Testing

| Folder | Subsystem | What is tested |
|---|---|---|
| `line_barcode/` | S1 | Line following, crossroads, barcodes |
| `motion/` | S2 | Motors and wheel encoders |
| `obstacle/` | S3 | Ultrasonic, servo scan, obstacle bypass |
| `comms/` | S4 | Wi-Fi, MQTT, web dashboard, telemetry |
| `imu_hump/` | S5 | IMU, hump detection and height, pick-up |
| `integration/` | — | Two or more subsystems together (interfaces in `TEAM.md`) |
| `system/` | — | Full demo runs on the course |
| `host/` | — | Host-side unit tests of the C modules (Python + desktop `cc`) |
| `review/` | — | Recorded build, static-analysis and BARR-C review outputs |

## Test IDs

`T-S<n>-<nn>` for an on-car test of subsystem *n*, `T-S<n>-H<nn>` for a host
test, `T-INT-<nn>` for integration and `T-SYS-<nn>` for system tests. Keep IDs
stable: add rows, never renumber or delete old results.

## Test logs

Each test folder has a `TEST_LOG.md` (one row per run) and a `logs/` folder
for the raw evidence: the MQTT log (`.\mqtt` writes
`%USERPROFILE%\mqtt_log.txt`), USB serial output, or photos. Record the
firmware commit so a result can be traced to the code that produced it.
Videos are not committed: list their links in `videos/README.md`.

## Host tests

The tests in `host/` compile firmware modules with a desktop C compiler
invoked as `cc` and run them on the PC. They need a `cc` command on the
`PATH` (on most systems this is gcc or clang); the Pico toolchain
(`arm-none-eabi-gcc`) alone is not enough. From the repository root:

```
python testing/host/test_barcode_host.py
python testing/host/test_obstacle_host.py
python testing/host/test_ultrasonic_host.py
python testing/host/test_stationary_scan_host.py
python testing/host/test_web_host.py
python testing/host/test_bypass_host.py
python testing/host/test_bypass_geometry_host.py
```

The last recorded run (all passed) is in `review/final-tests.txt`, from
commit `59e95d3`, before the files moved to this layout.

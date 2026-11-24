# Test log: S3 ultrasonic, servo scan and obstacle bypass

Code under test: `app_program/obstacle/`.

Keep test IDs stable: add new rows, never renumber or delete old ones. Put raw evidence (MQTT log, serial output, photos) in `logs/` and name it in the Evidence column.

| ID | Date | Test | Setup / firmware | Expected | Result | Evidence |
|---|---|---|---|---|---|---|
| T-S3-H01 | 2026-11-24 | Obstacle scan host test (`test_obstacle_host.py`) | commit `59e95d3` (old `src/` layout), host PC | Scan geometry, reading freshness, cancellation, timeout | Pass | `testing/review/final-tests.txt`, `host-tests.txt` |
| T-S3-H02 | 2026-11-24 | Ultrasonic host test (`test_ultrasonic_host.py`) | commit `59e95d3` (old `src/` layout), host PC | "Ultrasonic host checks passed" | Pass | `testing/review/final-tests.txt`, `host-tests.txt` |
| T-S3-H03 | 2026-11-24 | Stationary scan host test (`test_stationary_scan_host.py`) | commit `59e95d3` (old `src/` layout), host PC | Shared dispatch, rejection, STOP, completion, braking | Pass | `testing/review/final-tests.txt`, `host-tests.txt` |
| T-S3-H04 | 2026-11-24 | Bypass host test (`test_bypass_host.py`) | commit `59e95d3` (old `src/` layout), host PC | Bypass, MQTT compatibility and calibration helper checks | Pass | `testing/review/final-tests.txt`, `host-tests.txt` |
| T-S3-H05 | 2026-11-24 | Bypass geometry host test (`test_bypass_geometry_host.py`) | commit `59e95d3` (old `src/` layout), host PC | "Independent swept-footprint geometry checks passed" | Pass | `testing/review/final-tests.txt`, `host-tests.txt` |

**To fill in:**

- On-car tests: stop distance, stationary scan of the test obstacle, full bypass and return to the line (target ≥ 9/10 runs). Record the `BYPASS …` log lines for each run in `logs/`.

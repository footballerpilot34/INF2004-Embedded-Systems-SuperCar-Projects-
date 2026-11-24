# Test log: S3 ultrasonic, servo scan and obstacle bypass

Code under test: `app_program/obstacle/`.

Keep test IDs stable: add new rows, never renumber or delete old ones. Raw evidence is in `logs/`: MQTT log events per day, copied from the `.\mqtt` console log (`%USERPROFILE%\mqtt_log.txt`) with its line numbers. Counts below were taken from those logs.

| ID | Date | Test | Setup / firmware | Expected | Result | Evidence |
|---|---|---|---|---|---|---|
| T-S3-H01 | 2026-11-24 | Obstacle scan host test (`test_obstacle_host.py`) | commit `59e95d3` (old `src/` layout), host PC | "Obstacle scan host checks passed (geometry, freshness, cancellation, timeout)" | Pass | `testing/review/final-tests.txt`, `host-tests.txt` |
| T-S3-H02 | 2026-11-24 | Ultrasonic host test (`test_ultrasonic_host.py`) | commit `59e95d3` (old `src/` layout), host PC | "Ultrasonic host checks passed" | Pass | `testing/review/final-tests.txt`, `host-tests.txt` |
| T-S3-H03 | 2026-11-24 | Stationary scan host test (`test_stationary_scan_host.py`) | commit `59e95d3` (old `src/` layout), host PC | "Stationary scan checks passed (shared dispatch, rejection, STOP, completion, braking)" | Pass | `testing/review/final-tests.txt`, `host-tests.txt` |
| T-S3-H04 | 2026-11-24 | Bypass host test (`test_bypass_host.py`) | commit `59e95d3` (old `src/` layout), host PC | "Bypass, MQTT compatibility and calibration helper host checks passed" | Pass | `testing/review/final-tests.txt`, `host-tests.txt` |
| T-S3-H05 | 2026-11-24 | Bypass geometry host test (`test_bypass_geometry_host.py`) | commit `59e95d3` (old `src/` layout), host PC | "Independent swept-footprint geometry checks passed" | Pass | `testing/review/final-tests.txt`, `host-tests.txt` |
| T-S3-01 | 2026-11-20 to 24 | Obstacle stop | On the car, course mat; development firmware of that day (commit not recorded in the log) | The car brakes when the ultrasonic reading is within the stop distance | Pass. 74 stops (9, 36, 29 per day) at readings of 53 to 300 mm (the stop distance changed between days) | `logs/2026-11-20_mqtt_events.txt`, `logs/2026-11-23_mqtt_events.txt`, `logs/2026-11-24_mqtt_events.txt` |
| T-S3-02 | 2026-11-23 to 24 | Obstacle bypass and return to the line | On the car, course mat; development firmware of that day (commit not recorded in the log) | Drive round the obstacle and resume line following (target ≥ 9 of 10) | Fail. 27 bypasses started; 8 returned to FOLLOW_LINE (1 on 2026-11-23, 7 on 2026-11-24); 19 ended STOPPED ("BYPASS recovery failed") | `logs/2026-11-23_mqtt_events.txt`, `logs/2026-11-24_mqtt_events.txt` |
| T-S3-03 | 2026-11-23 | Bypass refused when unsafe | On the car, course mat; development firmware of that day (commit not recorded in the log) | No bypass is started from an incomplete scan or too close to the obstacle | Pass. 3 refused: 1 "incomplete/implausible forward scan", 2 "bumper gap too small to turn"; the car stayed BLOCKED | `logs/2026-11-23_mqtt_events.txt` |

# Test log: S1 line following, crossroads and barcodes

Code under test: `app_program/line_barcode/` (and the line steering in `app_program/vehicle.c`).

Keep test IDs stable: add new rows, never renumber or delete old ones. Raw evidence is in `logs/`: MQTT log events per day, copied from the `.\mqtt` console log (`%USERPROFILE%\mqtt_log.txt`) with its line numbers. Counts below were taken from those logs.

| ID | Date | Tester | Test | Setup / firmware | Expected | Result | Evidence |
|---|---|---|---|---|---|---|---|
| T-S1-H01 | 2026-11-24 | Ryan | Barcode decoder host test (`testing/host/test_barcode_host.py`) | commit `59e95d3` (old `src/` layout), host PC | "Barcode host checks passed (GPIO streams, distance/time, reverse, malformed, re-enable)" | Pass | `testing/review/final-tests.txt`, `host-tests.txt` |
| T-S1-01 | 2026-11-23 to 24 | Ryan | START calibration (drive forward onto the line) | On the car, course mat; development firmware of that day (commit not recorded in the log) | Both on black: follow at once. One on black: arc onto the line, then follow. Both on white: stay READY | Pass. Skipped (both on black) 74 + 38 times; arced on and confirmed (`LINE_CAL OK`) 16 + 26 times; both on white stayed READY 3 + 1 times; no calibration ended in a stop | `logs/2026-11-23_mqtt_events.txt`, `logs/2026-11-24_mqtt_events.txt` |
| T-S1-02 | 2026-11-20 to 24 | Ryan | Barcode reading while following the line | On the car, course mat; development firmware of that day (commit not recorded in the log) | Valid single-letter Code 39 frames decoded and their command reported; partial reads ignored | Pass. 55 barcodes decoded (letters A, F, M, Z); 108 partial scans ignored ("only N bars+spaces seen"). Read accuracy against the printed letters was not recorded | `logs/2026-11-20_mqtt_events.txt`, `logs/2026-11-23_mqtt_events.txt`, `logs/2026-11-24_mqtt_events.txt` |
| T-S1-03 | 2026-11-20 to 24 | Ryan | Crossroad detection and barcode turn | On the car, course mat; development firmware of that day (commit not recorded in the log) | Crossroad found after a barcode; the turn is made and line following resumes | Pass. 20 crossroads detected; 17 turns started; 15 ended back in FOLLOW_LINE, 2 were ended by the operator's STOP (2026-11-23 16:35, 17:13) | `logs/2026-11-20_mqtt_events.txt`, `logs/2026-11-23_mqtt_events.txt`, `logs/2026-11-24_mqtt_events.txt` |
| T-S1-04 | 2026-11-20 to 24 | Ryan | Line lost: reverse along the recorded path | On the car, course mat; development firmware of that day (commit not recorded in the log) | Car reverses to where both sensors were last on the line | Observed 70 times (14, 24, 32 per day). Whether each one found the line again was not assessed | `logs/2026-11-20_mqtt_events.txt`, `logs/2026-11-23_mqtt_events.txt`, `logs/2026-11-24_mqtt_events.txt` |

**Notes:**

- 2026-11-20 runs used the older pivoting START calibration (38 skipped, 7 aligned); the forward version is from 2026-11-23.

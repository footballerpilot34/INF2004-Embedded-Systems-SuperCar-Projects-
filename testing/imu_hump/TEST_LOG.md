# Test log: S5 IMU, hump detection and height, pick-up

Code under test: `app_program/imu_hump/` (and the hump and pick-up logic in `app_program/vehicle.c`).

Keep test IDs stable: add new rows, never renumber or delete old ones. Raw evidence is in `logs/`: MQTT log events per day, copied from the `.\mqtt` console log (`%USERPROFILE%\mqtt_log.txt`) with its line numbers. Counts below were taken from those logs.

| ID | Date | Tester | Test | Setup / firmware | Expected | Result | Evidence |
|---|---|---|---|---|---|---|---|
| T-S5-01 | 2026-11-19 to 24 | Rui Feng, Saketh | Hump detected and its height reported (3° tilt entry) | On the car, course mat; development firmware of that day (commit not recorded in the log) | Each crossing enters HUMP, reports a peak height and returns to following | Pass for detection: 45 humps with a reported peak (15, 17, 5 and 8 per day), from 3.1 to 26.6 mm, mostly 9 to 13 mm and 24 to 27 mm. Height accuracy not assessed: the true height of each hump was not recorded | `logs/2026-11-19_mqtt_events.txt`, `logs/2026-11-20_mqtt_events.txt`, `logs/2026-11-23_mqtt_events.txt`, `logs/2026-11-24_mqtt_events.txt` |
| T-S5-02 | 2026-11-24 | Rui Feng, Saketh | 1 mm hump entry height (from 15:41) | On the car, course mat; development firmware of that day (commit not recorded in the log) | Real humps detected; no humps on flat ground | Fail. 94 humps reported between 15:44 and 15:56, almost all 1.0 to 3.1 mm (largest 7.2 mm): flat-ground noise is above 1 mm. The entry height needs to be raised | `logs/2026-11-24_mqtt_events.txt` |
| T-S5-03 | 2026-11-20 to 24 | Rui Feng, Saketh | Pick-up stops the car | On the car, course mat; development firmware of that day (commit not recorded in the log) | Lifting the car while it drives stops it; it never stops while on the floor | Fail. 54 pick-up stops (6, 16, 32 per day), by state: 25 in HUMP, 15 in FOLLOW_LINE, 14 in OBSTACLE_SCAN. At least 3 were false (2026-11-23 16:34, 17:17, 17:19, while following the line). The rest were not classified as real or false lifts | `logs/2026-11-20_mqtt_events.txt`, `logs/2026-11-23_mqtt_events.txt`, `logs/2026-11-24_mqtt_events.txt` |
| T-S5-04 | 2026-11-24 | Rui Feng, Saketh | IMU lift-kick measurement (for an IMU pick-up check) | On the car, course mat; development firmware of that day (commit not recorded in the log) | Each kick's upward and downward speed is logged | Data recorded: 197 "IMU kick" lines. Not yet used to set a threshold | `logs/2026-11-24_mqtt_events.txt` |

**Notes:**

- 25 pick-up stops happened during HUMP; tipping on the hump may lift the IR sensors off the mat. Worth checking before the demo.

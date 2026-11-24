# Test log: S2 motors and wheel encoders

Code under test: `app_program/motion/`. Motors and encoders were used in every run below; these rows cover the power boost on a stall.

Keep test IDs stable: add new rows, never renumber or delete old ones. Raw evidence is in `logs/`: MQTT log events per day, copied from the `.\mqtt` console log (`%USERPROFILE%\mqtt_log.txt`) with its line numbers. Counts below were taken from those logs.

| ID | Date | Test | Setup / firmware | Expected | Result | Evidence |
|---|---|---|---|---|---|---|
| T-S2-01 | 2026-11-20 to 24 | Stall detection (encoders) and power boost (motors) on the hump | On the car, course mat; development firmware of that day (commit not recorded in the log) | Wheels stalled on the slope are detected and given more power until they turn | Partial. 41 stalls detected (19, 10, 12 per day) and boosted; 2 runs on 2026-11-24 stayed stuck at full power and ended BLOCKED | `logs/2026-11-20_mqtt_events.txt`, `logs/2026-11-23_mqtt_events.txt`, `logs/2026-11-24_mqtt_events.txt` |

**Notes:**

- No separate on-car record of wheel direction or encoder distance calibration (`w`, `e`) is in the logs.

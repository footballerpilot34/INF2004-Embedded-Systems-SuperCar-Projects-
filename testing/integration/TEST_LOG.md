# Test log: Integration tests

Two or more subsystems together. Interfaces are listed in `TEAM.md` section 5.

Keep test IDs stable: add new rows, never renumber or delete old ones. Raw evidence is in `logs/`: MQTT log events per day, copied from the `.\mqtt` console log (`%USERPROFILE%\mqtt_log.txt`) with its line numbers. Counts below were taken from those logs.

| ID | Date | Test | Setup / firmware | Expected | Result | Evidence |
|---|---|---|---|---|---|---|
| T-INT-01 | 2026-11-20 to 24 | Barcode command → crossroad turn (IF-01, IF-02, IF-03, IF-04) | On the car, course mat; development firmware of that day (commit not recorded in the log) | A decoded letter's command is carried out at the next crossroad | Pass. See T-S1-02, T-S1-03: 17 turns started, 15 back to FOLLOW_LINE, 2 stopped by the operator | `logs/2026-11-20_mqtt_events.txt`, `logs/2026-11-23_mqtt_events.txt`, `logs/2026-11-24_mqtt_events.txt` |
| T-INT-02 | 2026-11-23 to 24 | Obstacle stop → scan → bypass → line following (IF-01, IF-04 to IF-08) | On the car, course mat; development firmware of that day (commit not recorded in the log) | Line following resumes after the bypass (target ≥ 9 of 10) | Fail. 8 of 27 (see T-S3-02) | `logs/2026-11-23_mqtt_events.txt`, `logs/2026-11-24_mqtt_events.txt` |
| T-INT-03 | 2026-11-20 to 24 | Hump → line following (IF-04, IF-05, IF-09, IF-10) | On the car, course mat; development firmware of that day (commit not recorded in the log) | After a hump the car goes back to following the line | Partial. 123 HUMP → FOLLOW_LINE (16, 5, 102 per day); also 3 HUMP → BLOCKED and 25 pick-up stops during HUMP (see T-S5-03). The 2026-11-24 count includes the 1 mm false humps (T-S5-02) | `logs/2026-11-20_mqtt_events.txt`, `logs/2026-11-23_mqtt_events.txt`, `logs/2026-11-24_mqtt_events.txt` |
| T-INT-04 | 2026-11-19 to 24 | MQTT start / stop / feature tests (IF-11, IF-12) | On the car, course mat; development firmware of that day (commit not recorded in the log) | Commands change the car's state; state is reported in telemetry | Pass. 185 `start`, 195 `stop`, 84 `ft…` feature-test commands sent; state changes followed in the log (see T-S4-01 for acknowledgements) | `logs/2026-11-20_mqtt_events.txt`, `logs/2026-11-23_mqtt_events.txt`, `logs/2026-11-24_mqtt_events.txt` |

**Notes:**

- Not yet covered by a recorded test: IF-13 (log), IF-14 (GPIO interrupts), IF-15 (battery voltage), beyond being in use in every run.

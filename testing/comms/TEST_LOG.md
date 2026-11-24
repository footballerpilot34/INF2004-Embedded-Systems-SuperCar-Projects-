# Test log: S4 Wi-Fi, MQTT, web dashboard and telemetry

Code under test: `app_program/comms/`.

Keep test IDs stable: add new rows, never renumber or delete old ones. Raw evidence is in `logs/`: MQTT log events per day, copied from the `.\mqtt` console log (`%USERPROFILE%\mqtt_log.txt`) with its line numbers. Counts below were taken from those logs.

| ID | Date | Test | Setup / firmware | Expected | Result | Evidence |
|---|---|---|---|---|---|---|
| T-S4-H01 | 2026-11-24 | Web host test (`test_web_host.py`) | commit `59e95d3` (old `src/` layout), host PC | "Web host checks passed (hotspot recovery, DHCP, reconnect, JSON bounds, dashboard syntax)" | Pass | `testing/review/final-tests.txt`, `host-tests.txt` |
| T-S4-01 | 2026-11-19 to 24 | MQTT commands acknowledged | On the car, course mat; development firmware of that day (commit not recorded in the log) | Every command on `…/cmd` is answered on `…/cmd/ack` | Partial. 518 commands, 495 acknowledged (4/4, 212/216, 175/184, 104/114 per day). The 23 without an acknowledgement were not investigated (the car may have been restarting or offline) | `logs/2026-11-19_mqtt_events.txt`, `logs/2026-11-20_mqtt_events.txt`, `logs/2026-11-23_mqtt_events.txt`, `logs/2026-11-24_mqtt_events.txt` |
| T-S4-02 | 2026-11-20 to 24 | Malformed commands rejected | On the car, course mat; development firmware of that day (commit not recorded in the log) | An unknown word (e.g. a typo) is ignored and does not move the car | Pass. 9 unknown commands (e.g. `sotp`, `stp[`) logged as "unknown command … ignored" | `logs/2026-11-20_mqtt_events.txt`, `logs/2026-11-23_mqtt_events.txt`, `logs/2026-11-24_mqtt_events.txt` |
| T-S4-03 | 2026-11-20, 24 | Barcode letter table set over MQTT | On the car, course mat; development firmware of that day (commit not recorded in the log) | `bc<letter><cmd>` changes what a letter does; `bclist` lists the table | Pass. Letter commands sent on both days; letter F was later reported with four different commands (U-TURN, GO STRAIGHT, TURN LEFT, TURN RIGHT) as its setting changed | `logs/2026-11-20_mqtt_events.txt`, `logs/2026-11-24_mqtt_events.txt` |

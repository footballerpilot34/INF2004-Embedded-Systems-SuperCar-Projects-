# Test log: S4 Wi-Fi, MQTT, web dashboard and telemetry

Code under test: `app_program/comms/`.

Keep test IDs stable: add new rows, never renumber or delete old ones. Put raw evidence (MQTT log, serial output, photos) in `logs/` and name it in the Evidence column.

| ID | Date | Test | Setup / firmware | Expected | Result | Evidence |
|---|---|---|---|---|---|---|
| T-S4-H01 | 2026-11-24 | Web host test (`test_web_host.py`) | commit `59e95d3` (old `src/` layout), host PC | Hotspot recovery, DHCP, reconnect, JSON bounds, dashboard syntax | Pass | `testing/review/final-tests.txt`, `host-tests.txt` |

**To fill in:**

- On-car tests: hotspot join and recovery AP, MQTT connect/reconnect, telemetry rate, each MQTT command (start, stop, `set <letter> <cmd>`, …). The `.\mqtt` console log (`mqtt_log.txt`) is the evidence: copy the relevant part into `logs/`.

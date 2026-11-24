# Test log: System tests

Full demo runs on the course: START to pick-up, controlled over MQTT.

Keep test IDs stable: add new rows, never renumber or delete old ones. Raw evidence is in `logs/`: MQTT log events per day, copied from the `.\mqtt` console log (`%USERPROFILE%\mqtt_log.txt`) with its line numbers. Counts below were taken from those logs.

| ID | Date | Test | Setup / firmware | Expected | Result | Evidence |
|---|---|---|---|---|---|---|

**Notes:**

- No complete course run (line, barcode turn, hump, obstacle bypass, pick-up in one run) can be identified in the MQTT logs, so no system test result is recorded. Record the demo run here.

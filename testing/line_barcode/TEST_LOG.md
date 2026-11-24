# Test log: S1 line following, crossroads and barcodes

Code under test: `app_program/line_barcode/`.

Keep test IDs stable: add new rows, never renumber or delete old ones. Put raw evidence (MQTT log, serial output, photos) in `logs/` and name it in the Evidence column.

| ID | Date | Test | Setup / firmware | Expected | Result | Evidence |
|---|---|---|---|---|---|---|
| T-S1-H01 | 2026-11-24 | Barcode decoder host test (`testing/host/test_barcode_host.py`) | commit `59e95d3` (old `src/` layout), host PC | GPIO edge streams decode; distance/time widths; reverse reads; malformed frames rejected; re-enable | Pass | `testing/review/final-tests.txt`, `host-tests.txt` |

**To fill in:**

- On-car tests: line following on the course, START calibration (forward arc / skip / stay READY), each barcode letter A–Z read at cruise speed, crossroad detection, each turn command (left, right, straight, U-turn). Give each an ID (T-S1-01, T-S1-02, …).

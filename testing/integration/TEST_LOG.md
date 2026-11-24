# Test log: Integration tests

Two or more subsystems together. Each interface in `TEAM.md` section 5 (IF-01 … IF-15) should be covered by at least one test here.

Keep test IDs stable: add new rows, never renumber or delete old ones. Put raw evidence (MQTT log, serial output, photos) in `logs/` and name it in the Evidence column.

| ID | Date | Test | Setup / firmware | Expected | Result | Evidence |
|---|---|---|---|---|---|---|

**To fill in:**

- No recorded integration tests yet. Suggested: barcode turn at a crossroad (IF-01, IF-02, IF-04), obstacle stop → scan → bypass → line rejoin (IF-06 … IF-08), hump then line following (IF-09, IF-10), MQTT start/stop and letter commands (IF-12), telemetry contents (IF-11).

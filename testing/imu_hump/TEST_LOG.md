# Test log: S5 IMU, hump detection and height, pick-up

Code under test: `app_program/imu_hump/`.

Keep test IDs stable: add new rows, never renumber or delete old ones. Put raw evidence (MQTT log, serial output, photos) in `logs/` and name it in the Evidence column.

| ID | Date | Test | Setup / firmware | Expected | Result | Evidence |
|---|---|---|---|---|---|---|

**To fill in:**

- No recorded tests in the repository yet. Hump runs and pick-up runs were logged over MQTT during development (local `mqtt_log.txt`); copy the relevant runs into `logs/` and record them here: hump detected and height vs measured, no false hump on flat ground, pick-up stops the car, no false pick-up while following the line.

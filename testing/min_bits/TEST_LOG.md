# Test log: change requirement 1 (minimum-bit data representation)

Code under test: the whole of `app_program/`; the change touches every
subsystem. Keep test IDs stable: add new rows, never renumber or delete
old ones. Subsystem-specific checks are logged in that subsystem's
`TEST_LOG.md` (MQTT frames: `testing/comms/TEST_LOG.md`, T-S4-H02 to H05).

The car was dismantled before this change, so no test here ran on the car.

| ID | Date | Test | Setup / firmware | Expected | Result | Evidence |
|---|---|---|---|---|---|---|
| T-CR1-01 | 2026-11-25 | Firmware build | commit `7750e18`, Debug, SDK 2.3.1, GNU Arm 10.3-2021.10 | Builds and links | Pass | `after/checks/build.txt`, `after/size.txt` |
| T-CR1-02 | 2026-11-25 | BARR-C style check (`tools/check_barr.py`, run by the build) | same | 0 findings | Pass (52 files, 0 findings) | `after/checks/style_check.txt` |
| T-CR1-03 | 2026-11-25 | All 23 `app_program` modules with `-Wall -Wextra`, on top of the build's flags | same | 0 warnings | Pass (0) | `after/checks/strict_compile.txt` |
| T-CR1-04 | 2026-11-25 | Host test C sources type-checked (Arm GCC, `-std=c11 -Wall -Wextra -Werror -fsyntax-only`) | same; all 8 host tests | 0 errors | Pass (0) | `after/checks/host_tests_typecheck.txt` |
| T-CR1-05 | — | Host tests run (`testing/host/*.py`) | needs a desktop `cc` | All pass | Not run: no desktop `cc` on the machine used | `README.md` §4 |
| T-CR1-06 | 2026-11-25 | RAM before/after (`tools/ram_report.py`) | `94ebfcc` vs `7750e18` | Less RAM | Static RAM 73,988 -> 72,392 B; application modules 19,186 -> 17,558 B | `before/`, `after/ram_report.txt` |
| T-CR1-07 | 2026-11-25 | MQTT message size before/after (`tools/mqtt_size_report.py --convert`) | the team's MQTT log, 131,259 messages | Smaller messages | Payload -81.2 %, wire -65.0 % | `before/`, `after/mqtt_sizes.txt` |
| T-CR1-08 | 2026-11-25 | Stack per function (`-fstack-usage`) | `94ebfcc` vs `7750e18` | No function needs more stack than before, apart from small changes | Largest MQTT frame 696 -> 288 B; other modules within ±8 B | `after/stack_usage.txt` |
| T-CR1-09 | 2026-11-25 | Packing is mechanical: token diff against `94ebfcc` with the renames undone | same | Only the intended changes remain | Pass: `vehicle.c`, `line_sensor.c`, `line_diag.c`, `power.c` identical; the other modules differ only by the listed type and feature changes | `after/checks/token_diff.txt` |
| T-CR1-10 | — | On-car behaviour and loop timing (heartbeat `loop max`, overruns) | needs the car | No loss of function; loop pass well under 10 ms | Not run: car dismantled | — |

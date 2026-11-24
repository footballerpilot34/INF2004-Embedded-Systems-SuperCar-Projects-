# Change requirement 1: minimum-bit data representation

This folder holds the Week 13 evidence for the change requirement:

1. the data representation table, in `DATA_TABLE.md`;
2. the RAM usage comparison;
3. the message size comparison;
4. correctness and performance.

"Before" is commit `94ebfcc`, the last commit without the change;
"after" is commit `7750e18`, the change itself. Both versions were built
the same way (Debug, Pico SDK 2.3.1, GNU Arm
10.3-2021.10).

**Limitation: the car could not be tested.** It had already been
dismantled when this change was made. Nothing in this folder comes from
running the new firmware on the car. Section 4 lists what was checked
instead, and what still needs the car.

## What changed

- **Flags and small state values.** Each module keeps them in one
  bit-field struct, so a flag takes 1 bit and an n-value state takes
  ⌈log2 n⌉ bits. Small ranges use the smallest type that holds them
  (`int8_t` motor power, `int16_t` IMU angles, …).
- **Large arrays.** Obstacle scan points went from 12 to 4 bytes, the
  explore-mode bypass moves from 16 to 8 bytes, and the barcode edge
  colours from 1 byte to 1 bit each.
- **Faults.** One shared fault byte, with one bit each for motor stall,
  encoder fault, IMU fault, no echo, Wi-Fi loss, MQTT down, battery low
  and loop overrun (`app_program/fault.c`). Before, there were no fault
  variables.
- **Interrupt safety.** Packed data shared with an interrupt is written
  with interrupts off where needed (`DATA_TABLE.md` §6).
- **MQTT.** Telemetry, heartbeat, events, log and commands are compact
  binary frames, not JSON text. The laptop tool decodes and sends them
  (`tools/mqtt_car.ps1`, `tools/mqtt_frames.ps1`), and the Python
  reference is `tools/mqtt_frame.py`.

## 2. RAM usage comparison

Measured from the linked firmware with `tools/ram_report.py`. The full
per-variable lists are in `before/ram_report.txt` and
`after/ram_report.txt`.

| | Before | After | Change |
|---|---|---|---|
| Whole image: static RAM (`.data` + `.bss`) | 73,988 B | 72,392 B | **-1,596 B (-2.2 %)** |
| Application modules (`app_program/`) | 19,186 B | 17,558 B | **-1,628 B (-8.5 %)** |
| Code (`.text` + `.rodata` of `app_program/`) | 83,886 B | 87,495 B | +3,609 B (flash) |

Most of the RAM belongs to the Wi-Fi driver, lwIP and the SDK (about
55 KB). The 64-line log (6 KB of text) and the web page's text buffers are
text, not packed values, so the reduction in total RAM is modest.

Per module (RAM and code, bytes):

| Module | RAM before | RAM after | Change | Code before | Code after | Change |
|---|---|---|---|---|---|---|
| `obstacle` | 1218 | 505 | -713 | 4260 | 4418 | +158 |
| `bypass` | 1281 | 748 | -533 | 15533 | 15777 | +244 |
| `barcode` | 4818 | 4587 | -231 | 5654 | 5890 | +236 |
| `hump` | 121 | 78 | -43 | 536 | 620 | +84 |
| `mqtt_comm` | 906 | 873 | -33 | 4837 | 4009 | -828 |
| `imu` | 156 | 127 | -29 | 2584 | 2662 | +78 |
| `vehicle` | 1173 | 1144 | -29 | 9714 | 10042 | +328 |
| `web` | 2970 | 2960 | -10 | 27824 | 28340 | +516 |
| `motor` | 26 | 20 | -6 | 856 | 904 | +48 |
| `line_diag` | 21 | 16 | -5 | 1725 | 1765 | +40 |
| `app_main` | 122 | 118 | -4 | 6147 | 6361 | +214 |
| `line_sensor` | 13 | 9 | -4 | 484 | 488 | +4 |
| `servo` | 13 | 11 | -2 | 466 | 482 | +16 |
| `ultrasonic` | 43 | 41 | -2 | 564 | 596 | +32 |
| `power` | 16 | 14 | -2 | 759 | 771 | +12 |
| `encoder` | 64 | 64 | 0 | 442 | 408 | -34 |
| `fault` (new) | — | 18 | +18 | — | 592 | +592 |
| `mqtt_frame` (new) | — | 0 | 0 | — | 1553 | +1553 |
| `bitpack` (new) | — | 0 | 0 | — | 296 | +296 |
| others (`log`, `telemetry`, `buzzer`, `gpio_isr`) | 6225 | 6225 | 0 | 1501 | 1521 | +20 |
| **total** | **19186** | **17558** | **-1628** | **83886** | **87495** | **+3609** |

Code grew by 3.6 KB. Reading or writing a bit-field compiles to
shift-and-mask instructions on the Cortex-M0+, and the frame packer and
fault module are new. Flash has room for it: 2 MB, about 455 KB used.

**Stack.** The JSON payloads were built in 512-byte buffers on the stack.
`after/stack_usage.txt` has the per-function frames (`-fstack-usage`):

| Function | Before | After |
|---|---|---|
| `publish_telemetry` | 696 B | 112 B |
| `publish_events` | 672 B | 88 B |
| `publish_heartbeat` | 552 B | 120 B |
| `mqtt_comm_barcode` | 544 B | 56 B |
| `publish_log` | 544 B | 288 B |

## 3. Message size comparison

**Inter-task messages.** None: the firmware has no RTOS and no queues
(`DATA_TABLE.md` §7).

**MQTT.** "Before" is every message in the team's MQTT log from
19–24 November 2026 (131,259 messages; `before/mqtt_sizes.txt`). "After"
takes the same messages, rebuilds each as the binary frame the new
firmware sends, and measures it (`tools/mqtt_size_report.py --convert`;
`after/mqtt_sizes.txt`).

"Wire" is the whole MQTT PUBLISH packet: header, topic, packet id for QoS
1, and payload. The topic name (about 30 bytes) is the same in both, so
the wire saving is smaller than the payload saving.

| Topic | Messages | JSON payload (mean) | Binary payload | Payload saved | Wire saved |
|---|---|---|---|---|---|
| telemetry | 103,709 | 142.8 B | 24 B | 83.2 % | 66.6 % |
| heartbeat | 13,075 | 204.4 B | 28 B | 86.3 % | 73.5 % |
| log | 13,155 | 63.7 B | 44.9 B | 29.5 % | 20.1 % |
| obstacle | 40 | 206.6 B | 14 B | 93.2 % | 79.2 % |
| hump | 145 | 97.1 B | 12 B | 87.6 % | 65.6 % |
| barcode | 55 | 39.4 B | 7.0 B | 82.2 % | 43.0 % |
| cmd | 509 | 4.9 B | 2.2 B | 54.5 % | 7.2 % |
| cmd/ack | 486 | 4.9 B | 2.2 B | 54.4 % | 6.5 % |
| online | 85 | 1 B | 1 B | 0 % | 0 % |
| **all** | **131,259** | | | **81.2 %** | **65.0 %** |

In total, payloads go from 18.3 MB to 3.5 MB, and wire bytes from 23.1 MB
to 8.1 MB.

- **Log lines.** These stay text: the line is the message. Only the JSON
  wrapper and category name were removed.
- **Commands.** Nine old commands have no binary equivalent, so they are
  not counted. They were mainly the `mb` broker change, which only the
  web page can send.
- **Fixed frame sizes.** telemetry 24 B, heartbeat 28 B, obstacle 14 B,
  hump 12 B, barcode 6 B + 6 bits per character, command 2–3 B.

## 4. Correctness and performance

### Checked here

The raw output of each check is in `after/checks/` (test IDs in
`TEST_LOG.md` and `testing/comms/TEST_LOG.md`).

| Check | Result |
|---|---|
| Firmware build (`ninja -C build`) | Builds, links, `supercar.uf2` produced |
| BARR-C style check (`tools/check_barr.py`, part of the build) | 52 files, 0 findings |
| All 23 `app_program` modules compiled with `-Wall -Wextra` (the build's flags plus these) | 0 warnings |
| `bitpack.c`, `mqtt_frame.c` with `-Wconversion -Wsign-conversion -pedantic` | 0 warnings |
| Python frame reference: `python tools/mqtt_frame.py --selftest` | Passed. Round trip of every frame type, held values, every command word, and text rejected as not a frame |
| PowerShell decoder (`tools/mqtt_frames.ps1`) against the Python reference | Every field of every frame type and every command identical; text rejected |
| Real broker round trip (local Mosquitto): `mosquitto_pub -f` a command frame, `mosquitto_sub -F '%t %x'` | Bytes arrive unchanged, including `0x00`; decoded back to the same command |
| Host test sources (`testing/host/*.py`, generated C) type-checked with Arm GCC, `-std=c11 -Wall -Wextra -Werror` | 0 errors. Includes the new `test_mqtt_frame_host.py` and the updated tests below |
| Bit-field structs touched by an interrupt handler (checked by listing every global each handler uses) | None |

### Not checked yet

- **Host tests not run.** They need a desktop C compiler (`cc`), which the
  machine used does not have. They were updated for the change (new
  names, binary MQTT, 13-bit scan ranges) and type-check, but **none of
  them has been run since the change**. The new
  `test_mqtt_frame_host.py` compares the C packer with the Python
  reference byte for byte. Run them on a machine with `cc`:

  ```
  python testing/host/test_mqtt_frame_host.py
  python testing/host/test_ultrasonic_host.py
  python testing/host/test_obstacle_host.py
  python testing/host/test_bypass_host.py
  python testing/host/test_stationary_scan_host.py
  python testing/host/test_web_host.py
  python testing/host/test_barcode_host.py
  python testing/host/test_bypass_geometry_host.py
  ```

- **On-car behaviour.** Not tested: the car is dismantled.
- **Timing.** Not measured, which also needs hardware. The firmware now
  measures it. Every loop pass is timed, and the heartbeat reports the
  longest pass since the last beat (`loop max … us`), the number of passes
  over 10 ms, and the `loop overrun` fault bit. Running the new firmware
  shows these in the OUTPUT and PINGS windows. The old firmware has no
  such measurement, so there is no recorded "before" timing.

### Intended behaviour changes

Apart from the MQTT format, the car should behave as before. These are
the places where a value's resolution or limit changed:

| Where | Change | Effect |
|---|---|---|
| Hump steady height | Kept in 0.1 mm steps, not as a float | The height was already reported in 0.1 mm; the 1 mm entry threshold is unaffected |
| Explore-mode bypass moves | 1 mm and 0.1° resolution | The demo uses the fixed route (`BYPASS_FIXED_WIDTH_TEST`), which does not use these moves |
| Motor power read-back | Held at ±100 % when stored | The motors were already limited to ±100 %; the stored value now matches what is driven |
| Scan point range | Held at 8190 mm | Beyond the sensor's reach (about 4 m) |
| MQTT commands | Must be binary frames; text is ignored | The web page is unchanged and still controls the car |
| MQTT heartbeat | Wi-Fi details as numbers, not text | The web page still shows the text |

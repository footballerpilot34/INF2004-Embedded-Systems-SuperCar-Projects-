# SuperCar (team AAI14): autonomous line-following car on Raspberry Pi Pico W

An INF2004 robot car on a **Cytron Robo Pico** carrier with a **Raspberry Pi
Pico W** (RP2040). It follows a black line, reads Code 39 barcodes beside
the line and turns at the next crossroad as each letter commands, stops for
obstacles and drives round them, climbs a hump and reports its height, and
stops when it is picked up. It is monitored and controlled over Wi-Fi: a web
dashboard and MQTT.

The project uses the course template's folder layout (`app_program/`,
`config/`, `testing/`, `videos/`, `TEAM.md`), but it is built with the
**Pico SDK and CMake**, not micro T-Kernel: one bare-metal control loop on
core 0 every 10 ms. The template's kernel folders (`build_make/`, `kernel/`,
`include/`, `etc/`, `device/`, `lib/libtk` …) are therefore not part of this
repository. See `TEAM.md` for owners, pins, peripherals and interfaces.

You need a Raspberry Pi Pico W on the car, a USB cable, and a computer
running Windows, macOS or Linux.

---

## 1. Set up the Pico SDK and the compiler

**Easiest (all systems): VS Code with the Raspberry Pi Pico extension.**

1. Install VS Code, then the **Raspberry Pi Pico** extension (publisher:
   Raspberry Pi).
2. Open this folder. The top of `CMakeLists.txt` asks the extension for
   **Pico SDK 2.3.1**, **Arm toolchain 15_2_Rel1** and picotool 2.3.1, which
   it downloads into `~/.pico-sdk/` the first time (a few minutes).
3. Install **Python 3** if you do not have it: the build runs a coding
   standard check (`tools/check_barr.py`) before it compiles.

**Which compiler is actually used.** If another `arm-none-eabi-gcc` is
earlier on your `PATH` (for example from an older "Pico SDK v1.5.1"
Windows installer), CMake may pick that one instead of 15_2_Rel1: check
`CMAKE_C_COMPILER` in `build/CMakeCache.txt`. The firmware in this
repository has been built with GNU Arm 10.3-2021.10 found that way, against
Pico SDK 2.3.1.

**Without VS Code:** install `cmake` (3.13 or later), `ninja`, Python 3 and
the Arm GNU Toolchain (`arm-none-eabi-gcc` on your `PATH`), and get Pico SDK
2.3.1 with the submodules this project uses (USB serial, Wi-Fi, lwIP):

```sh
git clone --branch 2.3.1 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
```

Then set `PICO_SDK_PATH` to that folder. **Note:** if
`~/.pico-sdk/cmake/pico-vscode.cmake` exists (the VS Code extension has
been installed), `CMakeLists.txt` uses `~/.pico-sdk/sdk/2.3.1` and
**ignores** your `PICO_SDK_PATH`.

---

## 2. Build the `.uf2` file

**2.1 Fill in your Wi-Fi hotspot and MQTT broker (required, once)**

> ⚠️ **The repository contains no Wi-Fi credentials.** Before you build,
> tell the car which hotspot to join, or it cannot reach your laptop.

1. Copy `config/wifi_secrets.example.h` to **`config/wifi_secrets.h`**.
2. In `config/wifi_secrets.h`, replace the example entries in
   `HOTSPOT_LIST` with your own hotspot's name and password:

   ```c
   #define HOTSPOT_LIST                                                  \
       {"Your Hotspot Name", "your-password"}
   ```

   For more than one hotspot, separate the entries with commas:
   `{"Phone A", "passwordA"}, {"Phone B", "passwordB"}`. The car tries them
   in that order. The name must match exactly; an iPhone's name uses a
   curly apostrophe (`Name’s iPhone`). Use a 2.4 GHz hotspot; on an iPhone
   turn on **Maximise Compatibility**.
3. In `config/config.h`, set **`MQTT_BROKER_HOST`** to the IP address of
   the laptop running the MQTT broker on that hotspot (see section 6). It
   can also be changed later from the dashboard.
4. Build (2.2). If you add or change `wifi_secrets.h` after a build, save
   `config/config.h` too or do a clean build, so the change is picked up.

`config/wifi_secrets.h` is git-ignored: **never commit it** or put real
passwords in any other file. Without it, the car makes its own network
instead (section 4).

**2.2 Build**

In VS Code, click **Compile** in the status bar. From a terminal, in the
repository folder:

```sh
cmake -S . -B build -G Ninja      # first time only
cmake --build build --target supercar
```

Your firmware is **`build/supercar.uf2`**. Build output is git-ignored: build
it from source rather than committing it.

The build first runs `tools/check_barr.py` and stops if it finds blocking
BARR-C layout problems in project code (for example CR line endings or a
missing include guard). C files must use LF line endings; `.gitattributes`
keeps them LF on every checkout, including Windows.

**2.3 Flash it onto the Pico W**

1. Hold **BOOTSEL** on the Pico W, plug it into USB, then release. A drive
   called **RPI-RP2** appears.
2. Copy `build/supercar.uf2` onto that drive. The Pico restarts by itself.

**2.4 See the output**

Open the Pico's USB serial port (any baud rate) with the VS Code Serial
Monitor, PuTTY, or `screen /dev/ttyACM0` (Linux) /
`screen /dev/tty.usbmodem*` (macOS). The car logs its start-up, Wi-Fi
address and every state change, and accepts the single-key commands in
section 5.

**2.5 Write your own code**

| Where | What goes there |
|---|---|
| `app_program/app_main.c` | `main()`: initialises every module, then runs the 10 ms control loop that calls each module's `*_task()` step (order in `TEAM.md` section 4) |
| `app_program/vehicle.c` | The vehicle state machine that ties the subsystems together |
| `app_program/<subsystem>/` | One folder per subsystem: `line_barcode/` (S1), `motion/` (S2), `obstacle/` (S3), `comms/` (S4), `imu_hump/` (S5). Headers used by other folders are that subsystem's public interface; `obstacle/bypass_geometry.h` is private to `bypass.c`, and `obstacle/scan_plot.h` (the dashboard's scan plot) is used only by `comms/web.c` |
| `app_program/*.c` (others) | Shared helpers: GPIO interrupt dispatch, battery voltage, buzzer, the fault bitmask and loop timing (`fault.c`), bit packing for MQTT frames (`bitpack.c`) |
| `config/config.h` | Every pin, threshold, speed and network setting, shared by the team |
| `config/lwipopts.h` | Network stack settings |
| `lib/libwifi/dhcpserver/` | DHCP server for the recovery network (Raspberry Pi pico-examples, MIT licence) |
| `testing/` | Test records and host tests (`testing/README.md`) |
| `tools/` | MQTT console and its frame decoder, broker setup, BARR-C checker, frame reference (`mqtt_frame.py`), RAM and message-size reports, analysis scripts |

- **Drivers live in the subsystem that owns the hardware** (for example the
  ultrasonic driver in `app_program/obstacle/`), not in a separate driver
  folder.
- **CMake lists every source file.** A new `.c` file must be added to
  `add_executable(supercar …)` in `CMakeLists.txt`, and a new folder to
  `target_include_directories`. Keep code in `app_program/` itself or one
  subsystem folder below it. This is a team convention from the course
  template; CMake itself compiles whatever is listed, at any depth.
- Each peripheral is driven by one module; record pins, owners and
  interfaces in **`TEAM.md`** before wiring or coding them, and record every
  test in **`testing/`**. Videos are shared as unlisted YouTube links listed
  in **`videos/README.md`**, never committed.
- **Pins:** never use GP23, GP24, GP25: they are wired to the Pico W radio.
  GP29 is shared with the radio and is used only to read the battery
  voltage after Wi-Fi has started.

After editing, build again and copy the new `.uf2`.

---

## 3. Build options and feature switches

| Option | Where | Values | What it does |
|---|---|---|---|
| `PICO_BOARD` | `CMakeLists.txt` / `-DPICO_BOARD=` | **`pico_w`**, `pico2_w` | Which Pico is on the Robo Pico |
| `CMAKE_BUILD_TYPE` | `-DCMAKE_BUILD_TYPE=` | `Debug`, `Release` | Debug is what the VS Code extension uses. With no build type given, the Pico SDK defaults to Release, which is optimised for speed and was about 30 KB *larger* here (478,452 vs 447,408 bytes for `supercar.bin`, measured once with GNU Arm 10.3-2021.10; other toolchains may differ) |
| `MQTT_ENABLED` | `config/config.h` | **`true`**, `false` | MQTT telemetry and commands |
| `MQTT_BROKER_HOST` | `config/config.h` | `"172.20.10.3"` | Default broker (the laptop on the phone hotspot); can be changed at run time |
| `BYPASS_ENABLED` | `config/config.h` | **`true`**, `false` | Drive round obstacles; `false` stays BLOCKED |
| `BYPASS_FIXED_WIDTH_TEST` | `config/config.h` | **`true`**, `false` | Fixed-size left bypass of the test obstacle; `false` uses the older scan-based planner |
| `OBSTACLE_STOP_AT_ONCE` | `config/config.h` | **`true`**, `false` | Brake as soon as something is within `OBSTACLE_STOP_MM` (200 mm) |
| `LINE_BACKTRACK_ENABLE` | `config/config.h` | **`true`**, `false` | Reverse along the recorded path when both line sensors lose the line |
| `BARCODE_USE_ANALOG` | `config/config.h` | `true`, **`false`** | Read the barcode sensor's analog output (GP27, ADC) instead of its digital output (GP26) |
| `BARCODE_DEBUG_LOG` | `config/config.h` | `true`, **`false`** | Log every black/white change of the barcode sensor |
| `IDLE_POWER_SAVING` | `config/config.h` | `true`, **`false`** | Release the servo while parked |

Defaults are in bold. Rebuild after changing any of them.

---

## 4. Connect to the car

The car tries the hotspots in `config/wifi_secrets.h` in order. For an
iPhone hotspot, turn on **Allow Others to Join** and **Maximise
Compatibility**, and keep the Personal Hotspot screen open while it joins.
Connect your laptop to the same hotspot, then open
**http://supercar-aai14.local** or the address printed on the serial port
(`UI: open ...`).

If it cannot join a hotspot within 60 seconds (or no list is configured),
the car starts its own recovery network:

- Wi-Fi **SuperCar-AAI14**, password **supercar14**
- Dashboard **http://192.168.4.1**

Stay connected if the computer reports no Internet. Choose a hotspot on the
dashboard and press **USE** to switch.

---

## 5. Run the car

- **START** (GP20, dashboard, MQTT `start`, serial `s`): calibrate onto the
  line and follow it. Both line sensors on black: follow at once. One on
  black: drive forward, curving towards it. Both on white: do not move.
- **STOP** (GP21, dashboard, MQTT `stop`, serial `x`): brake from any state.
  STOP always takes priority.
- **Pick-up:** lifting the car while it drives stops it (end of the demo).
- **Feature test:** turn line following, obstacle avoidance, barcodes and
  hump/IMU on or off independently (MQTT `ft1111`).
- **IMU calibration:** automatic at power-up and on joining a hotspot (soft
  beep), or hold START 2 s / serial `c` / MQTT `calibrate`, with the car
  still and level.

| Serial key | Action |
|---|---|
| `s` / `x` | Start / stop |
| `c` | Calibrate IMU (car still) |
| `w` | Wheel test (lift the car first) |
| `d` | IR diagnostics on/off |
| `b` / `n` | Learn black / learn white |
| `t` | Barcode test |
| `l` / `r` / `u` | Test left / right / U-turn |
| `e` | Encoder distance mark (roll the car 300 mm between two marks) |
| `g` | Stationary ultrasonic scan |
| `v` | Empty-floor bypass rehearsal (moves the car; ends STOPPED) |

---

## 6. MQTT

Topics start with `inf2004/aai14/supercar` (`MQTT_TOPIC_PREFIX`):
`online`, `heartbeat` (every 2 s), `telemetry` (4 per second), `barcode`,
`obstacle`, `hump`, `log`, `cmd` and `cmd/ack`.

**Payloads are compact binary frames, not JSON** (change requirement 1):
each value takes only the bits its range needs (telemetry is 24 bytes,
the heartbeat 28). `online` is still the single character `1` or `0`.
Commands must be binary command frames too; plain text commands are
ignored. The layouts are in `app_program/comms/mqtt_frame.h` and
`testing/min_bits/DATA_TABLE.md`.

On a Windows laptop, run Mosquitto as the broker
(`tools/mqtt_broker_setup.ps1` opens the listener and firewall once), then
type **`.\mqtt`** in PowerShell in this folder. It opens three windows:
INPUT (type commands), OUTPUT (the car's log and events, also saved to
`%USERPROFILE%\mqtt_log.txt`) and PINGS (heartbeat and telemetry). It
decodes the frames and sends commands as frames (`tools/mqtt_frames.ps1`).
Commands include `start`, `stop`, `ft1111`, `bt`, `calibrate`, `wheel`,
`scan`, `v`, and `set <letters> <left|right|straight|uturn|none>` to change
what each barcode letter does.

With the Mosquitto tools directly, show payloads as hex and decode them
with the Python reference, and send a command frame from a file:

```sh
mosquitto_sub -h BROKER_IP -t 'inf2004/aai14/supercar/#' -F '%t %x'
python tools/mqtt_frame.py --decode telemetry <hex>
python tools/mqtt_frame.py --cmd start          # prints a701
mosquitto_pub -h BROKER_IP -t 'inf2004/aai14/supercar/cmd' -f start.bin
```

(`start.bin` holds the bytes `a7 01`.)

Use a broker on your own network only: anyone who can reach it can move
the car.

---

## 7. Obstacle avoidance notes

The car brakes when the ultrasonic sensor sees something within 200 mm
(`OBSTACLE_STOP_MM`), scans it with the pan servo (60–120°), then drives
round it on the left using wheel-encoder distances and returns to the
line. With `BYPASS_FIXED_WIDTH_TEST` the route assumes the controlled test
obstacle (150 × 150 mm, centred on the line): 210 mm sideways offset and a
+65° return. Normal success needs both line sensors on the line and the
heading aligned before line following resumes; any failed check stops the
car (STOPPED). The bypass rehearsal `v` always ends STOPPED. Settings are in
the obstacle section of `config/config.h`; the route is in
`app_program/obstacle/bypass.c`.

Latest notes from the obstacle work (24 November 2026):

- Continuous scan geometry replaces width bins and scanner fallbacks;
  missing centre/edge/width data stays UNKNOWN/null. Nominal dimensions are
  used only in bypass planning.
- The bypass uses the stationary measured range, with the +65° return,
  140 mm post-contact advance, 160 mm lateral cap and bounded IR recovery.
- Genuine full-angle scans keep the raw and calibrated width and measured
  range; a separate corrected width uses a labelled 150 mm fallback when
  needed. Planning width is at least 150 mm, depth 150 mm.
- If the line is not confirmed after alignment, one low-speed,
  forward-only correction is allowed, up to 240 mm / 8 s with ±25° heading
  targets (`BYPASS_NUDGE_MAX_MM`, `BYPASS_NUDGE_MAX_MSEC`,
  `BYPASS_NUDGE_HEADING_DEG`).

Before relying on the bypass, check forward clearance, encoder distance,
pivot angles and final line alignment on the real surface. Full BARR-C
compliance and physical validation of these settings are not claimed; the
review records are in `testing/review/`.

---

## 8. Testing and safe operation

1. Check wheel direction and encoder counts with the car lifted before any
   floor test.
2. Calibrate the line sensors and IMU where they will be used.
3. Run stationary scans first, then test obstacle avoidance in a clear,
   supervised area. Use `v` only with enough empty floor.
4. Keep STOP within reach; the software cannot prevent every collision.
5. Record each test in `testing/` (IDs and evidence: `testing/README.md`).

---

*`lib/libwifi/dhcpserver/` is from Raspberry Pi's pico-examples (MIT
licence, see its `LICENSE`). Folder layout from the INF2004 micro T-Kernel
3.0 project template.*

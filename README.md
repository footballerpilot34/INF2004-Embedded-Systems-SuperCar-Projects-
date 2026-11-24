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
2. Open this folder. The top of `CMakeLists.txt` tells the extension which
   versions to use: **Pico SDK 2.3.1**, **Arm toolchain 15_2_Rel1**,
   picotool 2.3.1. It downloads them into `~/.pico-sdk/` the first time
   (a few minutes).
3. Install **Python 3** if you do not have it: the build runs a coding
   standard check (`tools/check_barr.py`) before it compiles.

**Without VS Code:** install `cmake`, `ninja`, Python 3, the Arm GNU
Toolchain (`arm-none-eabi-gcc`, on your `PATH`) and the Pico SDK 2.3.1
(`git clone --branch 2.3.1 https://github.com/raspberrypi/pico-sdk.git`,
then `git submodule update --init` inside it), and set `PICO_SDK_PATH` to
the SDK folder.

---

## 2. Build the `.uf2` file

**2.1 Wi-Fi hotspots (once)**

Copy `config/wifi_secrets.example.h` to `config/wifi_secrets.h` and list
the hotspots the car may join. `config/wifi_secrets.h` is git-ignored:
never commit real passwords. Without it the car makes its own network
(section 4).

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
| `app_program/<subsystem>/` | One folder per subsystem: `line_barcode/` (S1), `motion/` (S2), `obstacle/` (S3), `comms/` (S4), `imu_hump/` (S5). Each `.h` file is that module's public interface |
| `app_program/*.c` (others) | Shared helpers: GPIO interrupt dispatch, battery voltage, buzzer |
| `config/config.h` | Every pin, threshold, speed and network setting, shared by the team |
| `config/lwipopts.h` | Network stack settings |
| `lib/libwifi/dhcpserver/` | DHCP server for the recovery network (Raspberry Pi pico-examples, MIT licence) |
| `testing/` | Test records and host tests (`testing/README.md`) |
| `tools/` | MQTT console, broker setup, BARR-C checker, analysis scripts |

- **Drivers live in the subsystem that owns the hardware** (for example the
  ultrasonic driver in `app_program/obstacle/`), not in a separate driver
  folder.
- **CMake lists every source file.** A new `.c` file must be added to
  `add_executable(supercar …)` in `CMakeLists.txt`, and a new folder to
  `target_include_directories`. Keep folders at most one level below
  `app_program/`.
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
| `CMAKE_BUILD_TYPE` | `-DCMAKE_BUILD_TYPE=` | `Debug`, `Release` | Debug is what VS Code uses; Release is smaller |
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

On a Windows laptop, run Mosquitto as the broker
(`tools/mqtt_broker_setup.ps1` opens the listener and firewall once), then
type **`.\mqtt`** in PowerShell in this folder. It opens three windows:
INPUT (type commands), OUTPUT (the car's log and events, also saved to
`%USERPROFILE%\mqtt_log.txt`) and PINGS (heartbeat and telemetry). Commands
include `start`, `stop`, `ft1111`, `bt`, `calibrate`, `wheel`, `scan`, `v`,
and `set <letters> <left|right|straight|uturn|none>` to change what each
barcode letter does.

With the Mosquitto tools directly:

```sh
mosquitto_sub -h BROKER_IP -t 'inf2004/aai14/supercar/#' -v
mosquitto_pub -h BROKER_IP -t 'inf2004/aai14/supercar/cmd' -m start
```

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
- After alignment, one low-speed forward-only correction is allowed (at
  most 60 mm / 3 s, ±5° steering). Missing-line recovery searches up to
  240 mm / 8 s with ±25° heading targets.

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

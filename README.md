> **Latest measurement update:** continuous scan geometry replaces width bins
> and scanner fallbacks. Missing centre/edge/width data stays UNKNOWN/null.
> Nominal dimensions remain exclusively in bypass planning. See
> [OBSTACLE_DETECTION.md](OBSTACLE_DETECTION.md) for schema changes and limits.

> **Submission update (24 November 2026):** detection triggers at 200 mm.
> Full real-echo scanning and detailed edge/raw/calibrated-width logs are restored.
> Bypass uses the stationary measured range, with the existing +65 degree return,
> 140 mm post-contact advance, 160 mm lateral cap and bounded IR recovery.
> Normal success returns to FOLLOW_LINE; manual `v` ends STOPPED.
> See [BARR_C_REVIEW.md](BARR_C_REVIEW.md) for the repository-wide standards
> refactor, unresolved deviations and verification artifacts. Full compliance
> and physical validation of the changed trigger are not claimed.

# SuperCar connection (current firmware)

Build the current firmware with `cmake --build build --target supercar -j4`.
Flash `build/supercar.uf2`; the `robocar` program described below is the older
manual-driving firmware.

The current SuperCar tries the hotspots in `src/wifi_secrets.h` in order.
For an iPhone, enable **Allow Others to Join** and **Maximise Compatibility**
(if available), and leave the Personal Hotspot settings screen open while
connecting. Verify the advertised name and password match the saved entry.
Connect your laptop to the same hotspot, then open
**http://supercar-aai14.local** or the numeric **http://** address printed in
the USB serial monitor (`UI: open ...`).

If no hotspot connection is established within 60 seconds, the car starts a
recovery network:

- Wi-Fi: **SuperCar-AAI14**
- Password: **supercar14**
- UI: **http://192.168.4.1**

Stay connected if the computer reports no Internet. The recovery network stays
on until you select a hotspot in the UI and press **USE**, or restart the car.
With no hotspot list configured, this network starts immediately.

# RoboCar: Wi-Fi car for Robo Pico + Pico W (C / Pico SDK)

Your phone drives the car through a web page that the car serves over its own Wi-Fi.

## Files
| File | What it is |
|---|---|
| `robocar.c` | The program: motors, servo, Wi-Fi hotspot, web page |
| `CMakeLists.txt` | Build recipe (set `PICO_BOARD` to `pico_w` or `pico2_w`) |
| `lwipopts.h` | Settings for the network stack |
| `dhcpserver/` | Gives your phone an IP address when it joins (from Raspberry Pi's pico-examples, MIT licence) |
| `pico_sdk_import.cmake` | Helps CMake find the Pico SDK |
| `prebuilt/` | Ready-made `.uf2` files if you just want to test |

## Build (VS Code, easiest on Windows/Mac/Linux)
1. Install VS Code, then the **Raspberry Pi Pico** extension (publisher: Raspberry Pi).
2. Pico sidebar → **Import Project** → choose this folder → pick board **Pico W** (or Pico 2 W) → Import.
   The first time, it downloads the SDK and compiler (takes a few minutes).
3. Click **Compile** in the bottom status bar. Output: `build/robocar.uf2`.

## Build (terminal, if you already have the SDK + arm-none-eabi-gcc)
```
export PICO_SDK_PATH=/path/to/pico-sdk
mkdir build && cd build
cmake .. -DPICO_BOARD=pico_w      # or pico2_w
make -j4
```

## Flash
1. Hold **BOOTSEL** on the Pico W, plug USB into the computer, release.
2. A drive called **RPI-RP2** (Pico 2 W: **RP2350**) appears.
3. Drag `robocar.uf2` onto it. The drive disappears and the program starts. It stays on the Pico even with the power off.

## Drive
1. Unplug USB. Plug the battery into the Robo Pico and switch it **ON**.
2. Pico W LED **blinking** means it's waiting for a phone.
3. Phone → Wi-Fi → join **RoboCar** (password `robocar123`).
   If the phone says "no internet", choose stay connected and turn mobile data off.
4. Open **http://192.168.4.1** in the browser. The LED goes solid. Tap ▲.

## Ultrasonic obstacle stop
The HC-SR04 wiring used by this project is TRIG → GP16 and ECHO → GP17.
Power the sensor as specified for your module and connect its ground to Pico ground.
HC-SR04 ECHO is typically 5 V: use a voltage divider or level shifter before GP17;
never connect a 5 V ECHO signal directly to a Pico GPIO.
The car stops when an echo indicates an object closer than 15 cm. After it stops,
remove the object and press a drive button again; it will not restart automatically.

## Tweaks (top of `robocar.c`)
- `WIFI_NAME`: give it a unique name if several teams are in the same lab.
- `DRIVE_SPEED`, `TURN_SPEED`: % power.
- `ULTRASONIC_STOP_DISTANCE_CM`: distance at which the car stops.
- `LEFT_REVERSED`, `RIGHT_REVERSED`: flip one if a wheel spins backwards on ▲.
- ◀ turns it right instead? The motors are swapped. Swap the M1 and M2 motor connectors.

Safety: the motors stop automatically if no phone is connected to the car's Wi-Fi.

---

# Current SuperCar firmware: features and operation

> **Firmware distinction:** The sections above are retained for the original `robocar` program. The features below describe the newer, modular `supercar` target. The original RoboCar hotspot, drive controls and 15 cm stop behaviour should not be assumed to describe the newer firmware.

## What the current firmware includes

| Feature | Implementation and purpose |
|---|---|
| Autonomous black-line following | IR sensors detect the line; start-up searches for and aligns with the line before following it. |
| Line-sensor calibration and diagnostics | Learn black/white sensor polarity, monitor sensor changes and diagnose alignment. |
| Code 39 barcode reading | Decode barcodes, display their messages and history, and map selected letters to left/right/straight/U-turn instructions. |
| Crossroad navigation | Queue applicable barcode turn commands for the next detected crossroad. |
| Ultrasonic obstacle detection | HC-SR04 distance measurements provide forward obstacle detection and trigger the obstacle-handling state machine. |
| Servo-based obstacle scanning | Sweep the ultrasonic sensor, plot distance against angle, and report measured obstacle edges/width estimates where readings support them. |
| Obstacle bypass | Encoder-guided, multi-step manoeuvre around a detected obstacle, with attempts to recover the line and align with its original direction. |
| Bypass rehearsal | A controlled empty-floor route can be triggered using USB serial `v` or MQTT `v` for movement calibration. |
| Hump detection and measurement | IMU pitch/roll measurements support hump detection, climb handling and height-related reporting. |
| Wheel encoders | Track left/right wheel edges to support movement estimates and turning/bypass calibration. |
| Motor, servo and buzzer control | Separate modules manage wheel power, scan direction and audible signalling. |
| Web dashboard | Browser-based controls, live sensor and motor values, connection information, barcode history, scan plot, logs and IMU visualisations. |
| Wi-Fi hotspot selection and recovery | Try configured 2.4 GHz hotspots, then provide a fallback access point when the connection times out. |
| MQTT telemetry and remote commands | Publish status, sensor readings, events and logs; receive and acknowledge supported control commands. |
| USB serial and physical controls | Start, stop, calibrate, test and diagnose without relying exclusively on the browser. |
| Monitoring and protections | Battery/VSYS readings, state reporting, command gating, timeouts, stop handling and watchdog-related diagnostics. |

**Implementation versus validation:** These capabilities are present in the uploaded source. Real-world line recovery, obstacle clearance, turn smoothness, sensor accuracy and hump measurements depend on the hardware, environment and calibration. In particular, a successful bypass rehearsal does not establish reliable autonomous avoidance in every situation.

## Build and flash the newer SuperCar

From the repository root, with the Raspberry Pi Pico SDK and appropriate toolchain configured:

```bash
cmake --build build --target supercar -j4
```

Flash `build/supercar.uf2` to the Pico W using BOOTSEL as described above. The repository also includes a prebuilt `build/supercar.uf2` for convenience; **rebuild after changing source or configuration** because a prebuilt UF2 does not automatically reflect edits. The `prebuilt/robocar-*.uf2` files belong to the older RoboCar program.

## Connect to the newer dashboard

1. Configure permitted hotspot names and passwords in `src/wifi_secrets.h`, using `src/wifi_secrets_example.h` as a reference. Use a 2.4 GHz-compatible hotspot. Do not publish personal hotspot passwords to GitHub.
2. Power the car and allow it to attempt the listed networks in order. Connect the laptop/phone to the same network.
3. Open `http://supercar-aai14.local` if name resolution works, or use the numeric HTTP address printed in USB serial output.
4. If no hotspot connects within approximately 60 seconds, join **SuperCar-AAI14** using the recovery credentials documented at the start of this README and open `http://192.168.4.1`.
5. The dashboard can display available hotspot choices and allow a change using **USE**. Your computer may need to reconnect when the car changes networks.

## Driving and feature tests

- **START:** Begin line acquisition, alignment and autonomous following.
- **STOP/RESET:** Stop movement. Use this before repositioning the car or changing the test setup.
- **FEATURE TEST:** Independently enable/disable line following, collision avoidance, barcode scanning and hump/IMU handling. Disabled line following allows straight-driving tests; disabled collision avoidance changes obstacle handling, so test in a clear area.
- **WHEEL TEST:** Check motor behaviour with the car safely lifted off the floor.
- **CALIBRATE IMU:** Keep the car still and level while calibration runs.
- **IR DIAG / LEARN BLACK / LEARN WHITE:** Inspect sensor readings and teach the line sensors their observed black/white levels.
- **ENCODER MARK:** Mark two positions while rolling the car a measured 300 mm to assist encoder calibration.
- **STATIONARY SCAN TEST / SCAN NOW:** Scan without driving, inspect the plotted readings and identify missing echoes or unreliable edges.
- **Servo slider:** Unlock to adjust the servo while stationary; servo requests are restricted while the car moves.
- **MQTT broker:** Enter a reachable broker address in More controls, or configure the default in `src/config.h`.

The dashboard also reports vehicle state, Wi-Fi/MQTT connection, uptime/restart information, battery voltage, ultrasonic range, IR states, motor output, encoder counts, barcode messages and history, pending turns, scan results and logs.

### USB serial shortcuts

| Key | Action |
|---|---|
| `s` | Start |
| `x` | Stop |
| `c` | Calibrate IMU while stationary |
| `w` | Wheel test (lift the car) |
| `d` | Toggle IR diagnostics |
| `b` / `n` | Learn black / learn white |
| `t` | Barcode test |
| `l` / `r` / `u` | Test left / right / U-turn behaviour |
| `e` | Encoder distance mark |
| `g` | Stationary ultrasonic scan |
| `v` | Empty-floor obstacle bypass rehearsal |

Physical controls include GP20 for start (long hold for IMU calibration) and GP21 for stop. Commands may be ignored or deferred depending on the vehicle state; stop takes priority.

## MQTT integration

The current firmware uses lwIP's MQTT client and the topic prefix `inf2004/aai14/supercar` (configurable in `src/config.h`). A broker must be reachable on the same network as the car. Main topic groups include:

- `.../online` and `.../heartbeat`: availability and periodic health information.
- `.../telemetry`: frequent vehicle state, motors, encoder counts, IR/ultrasonic readings, servo position, IMU values and other measurements.
- `.../barcode`, `.../obstacle`, `.../hump`: feature-specific events.
- `.../log`: diagnostic log messages.
- `.../cmd` and `.../cmd/ack`: remote control requests and acknowledgements.

For example, with Mosquitto installed and a broker reachable at `BROKER_IP`:

```bash
mosquitto_sub -h BROKER_IP -t 'inf2004/aai14/supercar/#' -v
mosquitto_pub -h BROKER_IP -t 'inf2004/aai14/supercar/cmd' -m start
mosquitto_pub -h BROKER_IP -t 'inf2004/aai14/supercar/cmd' -m stop
mosquitto_pub -h BROKER_IP -t 'inf2004/aai14/supercar/cmd' -m v
```

Supported MQTT commands also include feature-test selections (`ft1111`), barcode test (`bt`), IMU calibration (`calibrate`), wheel test (`wheel`), stationary scan (`scan`) and barcode letter mapping commands. `v` invokes the same bypass rehearsal as USB serial and cannot restart an active rehearsal. See **[MQTT.md](MQTT.md)** for broker setup, complete topics, payload fields, command formats and recovery behaviour. Avoid publicly accessible brokers for real driving: unauthenticated users may be able to issue movement commands.

## Obstacle avoidance and calibration

The ultrasonic sensor detects objects ahead; a servo scan samples multiple angles to estimate obstacle boundaries. The bypass logic uses configurable geometry, wheel encoder estimates, turning stages and line reacquisition. It is not simply a fixed-duration steering command. The controlled 180 ×140 mm car / centred 150 ×150 mm obstacle test now starts from a 150 mm ultrasonic reading (60 mm setback). It plans forward-only departures first: nominal −77°, 210 mm estimated offset and +65° return, about 836.5 mm axle travel, with no reverse leg. Normal success requires BOTH IR sensors and heading alignment before FOLLOW; continuous v always ends STOPPED. See [OBSTACLE_DETECTION.md](OBSTACLE_DETECTION.md) for geometry, bounded reverse fallback and the current lack of calibrated IMU yaw.

Before relying on obstacle bypass, validate forward clearance, wheel track, encoder edges per distance, pivot slip, turning angles, lateral clearance and final line alignment on the actual surface. Scan edges at the servo limit or with missing echoes should not be treated as reliable object dimensions. Refer to **[OBSTACLE_DETECTION.md](OBSTACLE_DETECTION.md)** for the fixed-width route, parameters, protections and staged physical tests, and **[ULTRASONIC_SCAN.md](ULTRASONIC_SCAN.md)** for scan interpretation and width limitations.

## Main code modules

| Module | Responsibility |
|---|---|
| `src/supercar_main.c` | Main firmware loop, USB/physical controls and module scheduling |
| `src/config.h` | Tunable pins, thresholds, speeds, calibration and networking constants |
| `src/vehicle.c`, `src/stage.h` | Vehicle state machine and operating stages |
| `src/line_sensor.c`, `src/line_diag.c` | Line/crossroad sensing and diagnostics |
| `src/barcode.c` | Barcode decoding and command interpretation |
| `src/ultrasonic.c`, `src/servo.c`, `src/scan_plot.h` | Range sensing, servo scanning and scan representation |
| `src/obstacle.c`, `src/bypass.c` | Obstacle assessment and bypass manoeuvres |
| `src/imu.c`, `src/hump.c` | Inertial sensing and hump handling |
| `src/encoder.c`, `src/motor.c`, `src/power.c` | Wheel feedback, motor control and battery monitoring |
| `src/web.c`, `src/mqtt_comm.c` | Wireless dashboard and MQTT communications |
| `src/telemetry.c`, `src/log.c`, `src/buzzer.c` | Telemetry, diagnostics and sound |
| `src/gpio_isr.c` | GPIO interrupt dispatch |
| `encoder_monitor.py`, `tools/` | Development, monitoring and test utilities |
| `tests/` | Source-level verification and regression tests |

The main control loop is organised around a nominal **10 ms** task period, with separate module update functions and periodic telemetry. This is a cooperative loop, not a claim that the project already runs a FreeRTOS scheduler.

## Testing and safe operation

1. Test wheel direction and encoder counts with the car raised before any floor test.
2. Calibrate line sensors and IMU under the conditions in which they will be used.
3. Run stationary ultrasonic scans first, then test obstacle avoidance in a clear, supervised area with room to stop.
4. Use the `v` bypass rehearsal only with sufficient empty floor space; it deliberately moves the car.
5. Keep the stop button or stop command accessible and do not assume the software can prevent every collision.
6. Check the dashboard and MQTT logs for state transitions, missing sensor readings, timeouts and unexpected resets.
7. Keep credentials out of public commits, and avoid exposing unauthenticated movement controls on untrusted networks.

## Additional documentation

- [MQTT.md](MQTT.md) — MQTT configuration, telemetry and commands.
- [OBSTACLE_DETECTION.md](OBSTACLE_DETECTION.md) — obstacle bypass design, configuration and validation.
- [ULTRASONIC_SCAN.md](ULTRASONIC_SCAN.md) — servo scan measurements and interpretation.

Current width/recovery tuning: genuine full-angle scans retain raw/calibrated
width and measured range; separate corrected width uses a labelled 150 mm
fallback when needed. Planning width is at least 150 mm, depth 150 mm. The
parallel leg is shortened by a footprint-validated 40 mm, and post-contact
advance is 65 mm. Both IR sensors/heading remain required before normal FOLLOW.
See [OBSTACLE_DETECTION.md](OBSTACLE_DETECTION.md) for safety limits and logs.

Final line recovery now permits one low-speed, forward-only correction after
alignment: at most 60 mm / 3 s with ±5° steering, behind the obstacle's complete
turning envelope. Both IR readings and heading confirmation remain required;
normal success enters FOLLOW_LINE, while failure and dry v end STOPPED.

Latest recovery tuning: post-contact advance 140 mm with a 160 mm lateral
cap; working route angles are unchanged. Missing-line recovery searches up to
240 mm / 8 s with ±25° heading targets, restores zero, and accepts the normal
follower's stable one-or-both-black pattern. Normal FOLLOW immediately resumes
IR steering; manual v still ends STOPPED. See OBSTACLE_DETECTION.md.

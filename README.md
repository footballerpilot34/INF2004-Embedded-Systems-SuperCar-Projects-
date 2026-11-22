# SuperCar AAI14 (INF2004)

Firmware for the Robo Pico + Pico W car: line following, Code 39 barcode
commands at crossroads, hump climbing and height measurement, obstacle
detection and bypass, pick-up stop, and MQTT telemetry and commands.

## Files

| Path | What it is |
|---|---|
| `src/` | The firmware (`supercar_main.c` is the entry point; settings in `config.h`) |
| `CMakeLists.txt` | Build recipe (`PICO_BOARD` is `pico_w`) |
| `lwipopts.h` | Network stack settings (Wi-Fi, web page, MQTT) |
| `dhcpserver/` | Hands out addresses on the car's recovery network (from Raspberry Pi's pico-examples, MIT licence) |
| `pico_sdk_import.cmake` | Helps CMake find the Pico SDK |
| `tools/` | Laptop scripts: `mqtt_car.ps1` (run `.\mqtt`), `mqtt_broker_setup.ps1` |
| `mqtt.cmd` | Starts `tools/mqtt_car.ps1` from PowerShell: `.\mqtt` |
| `tests/` | Host tests for the scan, bypass, ultrasonic, barcode and web code, and the obstacle-width calibration script |
| `MQTT.md` | MQTT topics, payloads, commands and broker set-up |
| `OBSTACLE_DETECTION.md`, `ULTRASONIC_SCAN.md` | Obstacle bypass and ultrasonic scan design and test notes |

## Build and flash

- **VS Code:** with the **Raspberry Pi Pico** extension, open this folder and
  click **Compile**. Output: `build/supercar.uf2`.
- **Terminal:** `cmake --build build --target supercar -j4`.
- **Flash:** hold **BOOTSEL** on the Pico W, plug in USB, release, and drag
  `build/supercar.uf2` onto the **RPI-RP2** drive.

## Connecting

The car tries the hotspots in `src/wifi_secrets.h` in order. For an iPhone,
enable **Allow Others to Join** and **Maximise Compatibility** (if
available), and leave the Personal Hotspot settings screen open while
connecting. Verify the advertised name and password match the saved entry.

- **MQTT (main way to run the car):** see `MQTT.md`. On the laptop, run
  `.\mqtt` from PowerShell in this folder.
- **Web page:** connect the laptop to the same hotspot, then open
  **http://supercar-aai14.local** or the address printed in the USB serial
  monitor (`UI: open ...`).

If no hotspot connection is established within 60 seconds, the car starts a
recovery network:

- Wi-Fi: **SuperCar-AAI14**
- Password: **supercar14**
- UI: **http://192.168.4.1**

Stay connected if the computer reports no Internet. The recovery network stays
on until you select a hotspot in the UI and press **USE**, or restart the car.
With no hotspot list configured, this network starts immediately.

## Wiring note: ultrasonic

The HC-SR04 is wired TRIG → GP16 and ECHO → GP17. HC-SR04 ECHO is typically
5 V: use a voltage divider or level shifter before GP17; never connect a 5 V
ECHO signal directly to a Pico GPIO.

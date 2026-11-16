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

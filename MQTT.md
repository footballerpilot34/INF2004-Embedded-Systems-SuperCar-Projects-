# MQTT telemetry and commands

The car publishes its telemetry to an MQTT broker over the Wi-Fi hotspot and
takes commands from it. The code is in `src/mqtt_comm.c` and uses lwIP's MQTT
client, polled from the main loop. The settings are in `src/config.h`, under
`MQTT_*`.

## 1. Broker set-up

The broker has to be reachable from the hotspot the car joins. There are two
ways to run one.

**A laptop on the same hotspot (recommended):**

On Windows, the quick way is:

1. `winget install --id EclipseFoundation.Mosquitto -e`. This installs
   Mosquitto as a service that listens on localhost only.
2. From an administrator PowerShell, run
   `powershell -ExecutionPolicy Bypass -File tools\mqtt_broker_setup.ps1`.
   It opens the service to the network, allows TCP 1883 through the
   firewall and prints the laptop's IP addresses. Run it with `-Undo` to
   reverse it.

Or by hand:

1. Install Mosquitto (https://mosquitto.org/download/).
2. Create `mosquitto.conf` with these two lines:
   ```
   listener 1883 0.0.0.0
   allow_anonymous true
   ```
3. Run `mosquitto -c mosquitto.conf -v`. On Windows, allow TCP port 1883
   through the firewall when asked.
4. Find the laptop's IP on the hotspot with `ipconfig` or `ifconfig`, for
   example `172.20.10.2`.

**A public test broker** works if the hotspot has internet, for example
`broker.hivemq.com`. Anyone can read and publish on a public broker,
including on the command topic, so use it only for testing.

**Point the car at the broker** in one of two ways:

- On the web page, open **More controls** and enter the IP or name under
  **MQTT broker**, then press **SET**. This lasts until the car restarts.
- Set `MQTT_BROKER_HOST` in `src/config.h`. This makes it permanent.

The page shows the connection under **MQTT**, for example
`Connected to 172.20.10.2 (812 sent, 0 dropped)`.

## 2. Watching it

```
mosquitto_sub -h 172.20.10.2 -t "inf2004/aai14/supercar/#" -v
```

MQTT Explorer (a free GUI) works as well. Connect it to the same broker and
open the `inf2004/aai14/supercar` tree.

## 3. Topics

The prefix is `MQTT_TOPIC_PREFIX` = `inf2004/aai14/supercar`.

| Topic | Direction | QoS / retained | When |
|---|---|---|---|
| `…/online` | car → | 1, retained | `1` on connect. The broker sends `0` (the car's will) if the car drops off. |
| `…/heartbeat` | car → | 0 | Every 2 s (`MQTT_HEARTBEAT_MSEC`). |
| `…/telemetry` | car → | 0 | Every 250 ms (`MQTT_TELEMETRY_MSEC`). |
| `…/barcode` | car → | 1 | Each decoded barcode. |
| `…/obstacle` | car → | 1 | Each finished obstacle scan. |
| `…/hump` | car → | 1, retained | Each hump, once the car is level again. |
| `…/cmd` | → car | 1 | A command word (see section 5). |
| `…/cmd/ack` | car → | 1 | The command word, once the car has taken it. |
| `…/log` | car → | 0 | Each line of the car's log (what the web page's serial monitor shows): `{"c":"state","m":"State: READY -> FOLLOW_LINE"}`. `c` is `system`, `obstacle`, `line`, `state` or `servo`. Lines logged before MQTT connected (start-up, Wi-Fi) are sent once it does, as far back as the car keeps (64 lines). |

## 4. Payloads (JSON)

**heartbeat**

```json
{"seq":42,"up":84,"vb":4870,"sent":812,"drop":0,"conn":1,
 "wifi":"Hotspot \"Ryan\" at 172.20.10.3, signal -58 dBm, ...",
 "rr":"power-up"}
```

| Field | Meaning |
|---|---|
| `seq` | Heartbeat number. A gap means heartbeats were lost. |
| `up` | Uptime in seconds. A drop back towards 0 means the car restarted. |
| `vb` | Battery (VSYS) in mV. 0 means unavailable. |
| `sent`, `drop` | MQTT messages queued, and messages that didn't fit or were made while offline. |
| `conn` | MQTT connections so far. More than 1 means it has reconnected. |
| `wifi` | Wi-Fi status text. |
| `rr` | Reason for the last restart. |

**telemetry**

```json
{"t":84250,"st":"FOLLOW_LINE","bp":"","ml":9,"mr":9,"el":40211,"er":41570,
 "l":1,"r":1,"b":0,"rg":612,"pt":12,"rl":-4,"sv":90,"hh":0,
 "pend":"TURN LEFT"}
```

| Field | Meaning |
|---|---|
| `t` | ms since boot. |
| `st` | Car state: `READY`, `LINE_CAL`, `FOLLOW_LINE`, `FEATURE_TEST`, `TURN`, `OBSTACLE_SCAN`, `BYPASS`, `HUMP`, `BLOCKED`, `STOPPED` (also after being picked up while driving: the end of the demo), and so on. |
| `bp` | Bypass step while in `BYPASS` (`out`, `pass`, `return`, …), otherwise empty. |
| `ml`, `mr` | Left and right motor power in %. |
| `el`, `er` | Left and right encoder edges since boot (about 13.3 and 13.7 per mm). |
| `l`, `r`, `b` | Left line, right line and barcode sensor. 1 means black. |
| `rg` | Ultrasonic range in mm. -1 means nothing in range. |
| `pt`, `rl` | Pitch and roll in tenths of a degree. |
| `sv` | Servo angle in degrees. 90 is straight ahead. |
| `hh` | Height the car is on now, in mm (one decimal): how far the raised end is above the floor. About 0 on the flat. |
| `am` | Total acceleration from the IMU, in milli-g: about 1000 at rest, whichever way the car is tilted. |
| `vz` | Upward speed from the IMU, in mm/s (down is negative): acceleration above 1 g added up over the last 0.4 s. A brisk lift builds it up; a bump goes up and back to about 0. Each kick is also logged as `IMU kick (...)`. |
| `pend` | Turn waiting for the next crossroad, or empty. |

**barcode**: `{"msg":"A","cmd":"TURN LEFT","t":51230}`. `cmd` is one of
`TURN LEFT`, `TURN RIGHT`, `GO STRAIGHT`, `U-TURN` or `MESSAGE`. Every letter
from A to Z is a command. The table is `BARCODE_LETTER_COMMANDS` in
`config.h`.

**obstacle**:

```json
{"front":248,"left":-62,"right":31,"mid":-15,"width":93,
 "lst":"echo-bracketed","rst":"estimated distance jump",
 "calibrated_width":null,"left_angle_x10":-230,"right_angle_x10":150,"t":60110}
```

All distances are in mm, measured from the car. `front` is the distance
ahead. `left`, `right` and `mid` are the edges and middle across the car's
path (+ is right). `front` is measured from the ultrasonic sensor; the sensor
is 60 mm behind the bumper. `width` is the unchanged raw geometric estimate,
which can overestimate physical width. `calibrated_width` is 150 or 250 mm
only with usable physical rectangular-target calibration, otherwise `null`
(unknown). `left_angle_x10` and `right_angle_x10` are relative scan edge
angles in tenths of a degree; these remain estimates affected by the beam.
`lst` and `rst` describe the observation quality of each edge. Topics, QoS,
and the previous raw fields are unchanged. See `OBSTACLE_DETECTION.md` for
the fixed-width physical test. The current movement route assumes a 200 mm
obstacle independently of these reported measurements; `width` is never
forced to 200. The operator establishes an empty left test corridor.

**hump**:

```json
{"height_mm":4.6,"peak_tilt_x10":31,"run_height_mm":5.1,
 "run_peak_tilt_x10":35,"n":2,"t":70400}
```

`height_mm` is the hump's peak height in mm (one decimal), and
`peak_tilt_x10` its highest tilt in tenths of a degree. The `run_` fields
are the highest of the run so far (the highest hump peak), and `n` counts
the humps cleared. A new run (START or a feature test) starts them again.
The height the car is on at any moment is `hh` in the telemetry.

The height is worked out from the IMU. A hump shorter than the car only
tips it: with the caster on top, the nose rises by the wheelbase
(`HUMP_WHEELBASE_MM`, 85 mm) times sin(pitch); with one drive wheel on top,
that corner rises by the track (`WHEEL_TRACK_MM`, 125 mm) times sin(roll).
The larger of the two is the height. Speeding up, braking and jolts read
as extra tilt, so a steady crossing gives the best figure.

## 5. Commands (`…/cmd`)

These are the same words the web page sends, as plain text, in any case:

| Word | Does |
|---|---|
| `start` | Start: calibrate on the line, then follow it. |
| `stop` | Stop at once, from any state. |
| `v` | Empty-floor bypass rehearsal, identical to USB serial `v`; ends stopped. Ignored while a rehearsal or other movement is active. |
| `ftLABH` | Feature test. L, A, B and H are each `1`/`0` for line following, obstacle avoidance, barcodes and humps (IMU). `ft1111` turns all four on; `ft0001` drives straight and only climbs and measures humps. The older three-digit form (`ft111`) leaves humps on. |
| `bt` | Barcode test: drive straight and read barcodes. |
| `calibrate` | Calibrate the IMU (car still and level). |
| `wheel` | Wheel test (car lifted). |
| `scan` | Stationary obstacle scan test. |
| `clear` | Clear the barcode message, history and log. |
| `bc` + letter + `l`/`r`/`s`/`u`/`n` | Set what a barcode letter does: left, right, straight, U-turn or nothing (a message). `bcal` = A turns left. Lasts until the car restarts. |
| `bclist` | Log the letter table, one line per command. |
| `bcreset` | Put the letter table back to `BARCODE_LETTER_COMMANDS` in `config.h`. |

```
mosquitto_pub -h 172.20.10.2 -t inf2004/aai14/supercar/cmd -m v
mosquitto_pub -h 172.20.10.2 -t inf2004/aai14/supercar/cmd -m stop
```

The car answers on `…/cmd/ack` with the word it took. A broker change
(`mb…`) is only accepted from the web page, so MQTT can't cut itself off.
For `v`, the acknowledgement still echoes receipt of the command; it does not
mean an active rehearsal was restarted. A queued `stop` takes priority over `v`.
USB serial `v` remains supported. Use an empty, physically checked test corridor
as described in `OBSTACLE_DETECTION.md`.

## 6. Recovery

- **Wi-Fi drops:** MQTT disconnects, and the hotspot code rejoins on its
  own.
- **Connection fails or drops:** after any failure, or if the broker goes
  quiet, the car tries again every 5 s (`MQTT_RETRY_MSEC`), for ever. A
  broker name is looked up again on each try.
- **Keepalive:** the keepalive is 10 s, so the broker notices a dead car
  within about 15 s and publishes `online` = `0`.
- **Driving is unaffected:** if the send buffer is full, a message is
  dropped and counted in `drop`, rather than holding up the car.

## 7. The laptop script (`.\mqtt`)

From PowerShell in the repo folder, `.\mqtt` (tools/mqtt_car.ps1) checks the
broker, tells the car this laptop's address, and opens three windows:

- **INPUT** (the one you started it in): type commands. Besides the words
  above, `set A left` / `set AEI right` sets barcode letters
  (left/right/straight/uturn/none), `letters` lists them, `letters reset`
  restores config.h, `broker` resends the laptop's address, `q` quits.
- **OUTPUT**: what the car says: its log (state changes, Wi-Fi, barcodes,
  obstacles, humps, ...), the barcode/obstacle/hump events, online/offline
  and command replies. It saves everything, pings included, to
  `%USERPROFILE%\mqtt_log.txt`.
- **PINGS**: the constant telemetry (once a second) and heartbeat.

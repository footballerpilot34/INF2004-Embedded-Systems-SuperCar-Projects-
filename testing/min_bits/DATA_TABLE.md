# Data representation table (change requirement 1)

Every application data field: its valid range, the bits that range needs,
and how it is stored now. "Before" is commit `94ebfcc`. Fields kept wider
than their minimum are listed with the reason.

How fields are stored:

- **Bit-field.** A module's flags and small state values are members of one
  `static struct` with C bit-fields (`bool x : 1`, `enum_t y : n`). Only
  the main loop uses them.
- **Mask.** One `volatile uint8_t` or `uint32_t` with explicit `1u << n`
  masks. Used where an interrupt shares the data (see section 6).
- **Packed frame.** An MQTT field written with explicit masks and shifts
  (`bitpack.c`), LSB first, no padding. C structs are never sent as they
  are, so padding, bit-field order and byte order cannot change a frame
  (BARR-C 5.3.a, 5.3.b).

On this toolchain (`arm-none-eabi`, short enums) an `enum` variable
already took 1 byte before the change. Packing enums therefore saves
bits, not 3 bytes each.

## 1. Motion control and encoder data

| Field | Where | Valid range | Bits needed | Now | Before |
|---|---|---|---|---|---|
| Motor power, left and right | `motor.c` `g_left`, `g_right` | -100..100 % | 8 (signed) | `int8_t`, held at ±100 when stored | `int32_t` |
| Watched encoder pins | `encoder.c` `g_watch_pins[10]` (flash) | GPIO 0..29 | 5 | `uint8_t` | `uint32_t` |
| Motor stall | `fault.c` | yes/no | 1 | Mask: bit 0 of the fault byte | (none) |
| Encoder fault | `fault.c` | yes/no | 1 | Mask: bit 1 of the fault byte | (none) |
| Bypass move: heading | `bypass.c` `move_t` (64 kept) | ±180.0° in 0.1° | 12 | `int16_t` in 0.1° | `float` |
| Bypass move: length | `move_t` | 0..about 700 mm | 10 | `uint16_t` mm | `uint32_t` |
| Bypass move: sideways start and end | `move_t` | about ±1000 mm | 11 | `int16_t` mm | `float` ×2 |
| Bypass phase (and 3 "next phase" values) | `bypass.c` `g_bp` | 13 phases | 4 each | Bit-field `: 4` ×4 | enum ×4 |
| Bypass flags (14) | `g_bp` | yes/no | 1 each | Bit-field `: 1` | `bool` ×14 |

Kept wider, with reasons:

- **Encoder edge counts** `g_count[2]` (32 bits). The count only goes up and
  wraps; the users take differences, so it needs the full word. An
  interrupt writes it, and a 32-bit read is atomic. Telemetry sends only
  the low 16 bits; see section 8.
- **Edge timestamps and intervals** (32-bit µs). The hardware timer is
  32 bits, and intervals are differences of it.
- **Motor start-boost deadlines** (64-bit µs). The SDK's `time_us_64()`
  timebase.
- **Live bypass odometry and geometry** (`float`, about 20 values). These
  feed continuous trigonometry every tick. Rounding them would change the
  tuned route, so they stay `float`. Only the 64-entry move stack was
  packed.

## 2. Line and barcode sensing

| Field | Where | Valid range | Bits needed | Now | Before |
|---|---|---|---|---|---|
| Line sensors on black, left and right | `line_state_t` (`line_sensor.h`) | yes/no | 1 each | Bit-field `: 1` ×2 | `bool` ×2 |
| Raw DOUT levels | `line_raw_t` | high/low | 1 each | Bit-field `: 1` ×2 | `bool` ×2 |
| "Black is high", left and right; crossroad detector state (3) | `line_sensor.c` `g_ls` | yes/no | 1 each | Bit-field ×5 | `bool` ×5 |
| IR diagnostic flags (3) | `line_diag.c` `g_ld` | yes/no | 1 each | Bit-field ×3 | `bool` ×3 |
| Which sensor last saw the line | `vehicle.c` `g_v.line_last` | 3 values | 2 | Bit-field `: 2` | enum |
| Steering | `g_v.steer` | 5 values | 3 | Bit-field `: 3` | enum |
| Barcode edge colour (ring of 256) | `barcode.c` `g_edge_black` | black/white | 1 per edge | Mask: `uint32_t[8]`, 1 bit per edge (interrupt) | `bool[256]` (256 B) |
| Barcode scan results (2) | `g_bc` | 3 values | 2 each | Bit-field `: 2` | enum |
| Barcode command | `g_bc.command` | 5 values | 3 | Bit-field `: 3` | enum |
| Barcode flags (6) | `g_bc` | yes/no | 1 each | Bit-field ×6 | `bool` ×6 |
| Barcode message character (MQTT) | barcode frame | 44 Code 39 symbols | 6 | 6 bits per character in the frame | 8-bit JSON text |

Kept wider, with reasons:

- **Edge-ring timestamps and positions** (`uint32_t`). These are the 32-bit
  µs timer and the encoder position, written by the interrupt.
- **Element widths** `g_elem_usec`, `g_elements` (`uint32_t`). A bar slid
  past by hand can take longer than 65 ms, which is more than 16 bits of µs.
- **Element distances** `g_elem_dist` (`uint32_t`). These would fit in 16
  bits. They were left alone to avoid re-tuning the decoder without the
  car; this is a possible further saving of 400 B.
- **Message and letter table** (`char`). Kept as text: they are C strings
  passed to the web page.

## 3. Obstacle detection

| Field | Where | Valid range | Bits needed | Now | Before |
|---|---|---|---|---|---|
| Scan point angle | `obstacle_point_t` (`g_points[64]`) | ±30° | 7 (signed) | `int8_t` | `int32_t` |
| Scan point range | `obstacle_point_t` | 0..8190 mm, or "no echo" | 13 | Bit-field `uint16_t : 13`; 8191 = no echo (`OBSTACLE_POINT_NO_RANGE`) | `uint32_t` |
| Scan point kind | `obstacle_point_t` | 4 values | 2 | Bit-field `uint16_t : 2` | enum |
| Scan queue angles | `obstacle.c` `g_queue[64]` | ±30° | 7 (signed) | `int8_t` | `int32_t` |
| Scan result flags (11) | `obstacle_profile_t` | yes/no | 1 each | Bit-field `: 1` | `bool` ×11 |
| Scan flags (2) | `obstacle.c` `g_ob` | yes/no | 1 each | Bit-field ×2 | `bool` ×2 |
| Ultrasonic flags: armed, waiting, ready, no echo | `ultrasonic.c` `g_us_flags` | yes/no | 1 each | Mask: one `volatile uint8_t` (interrupt; see 6) | `bool` ×3 |
| No echo (fault) | `fault.c` | yes/no | 1 | Mask: bit 3 of the fault byte | (none) |
| Servo angle | `servo.c` `g_angle` | 0..180°, or -1 (not set) | 9 (signed) | `int16_t` | `int32_t` |
| Servo released | `g_sv` | yes/no | 1 | Bit-field | `bool` |

Kept wider, with reasons:

- **Edge statuses** `left_status` and `right_status` (6 values, 1-byte enum).
  The scan passes them by pointer (`&p->left_status`), and a bit-field
  has no address.
- **Echo edge times** (`uint32_t`). The interrupt writes them as whole
  words, so reads are atomic.
- **Width and edge distances in `obstacle_profile_t`** (`int32_t`).
  There is only one profile, and they are mixed into `float` geometry.

## 4. Hump estimation

| Field | Where | Valid range | Bits needed | Now | Before |
|---|---|---|---|---|---|
| Recent heights (20, one per tick) | `hump.c` `g_recent_x10` | ±102.3 mm in 0.1 mm (reported that way) | 11 | `int16_t` in 0.1 mm | `float` |
| Index into them | `g_recent_next` | 0..19 | 5 | `uint8_t` | `uint32_t` |
| Pitch, roll and their zero points | `imu.c` | ±180.0° in 0.1° | 12 | `int16_t` | `int32_t` |
| Pitch and roll filter state | `imu.c` | ±180.00° in 0.01° | 15 | `int16_t` | `int32_t` |
| Total acceleration | `imu.c` `g_accel_mg` | 0..3547 mg (±2 g per axis) | 12 | `int16_t` | `int32_t` |
| Raw axes (3 accel, 3 mag) | `imu_axes_t` | ±2048 mg; about ±8900 mgauss | 12; 15 | `int16_t` ×6 | `int32_t` ×6 |
| IMU flags (4) | `g_imu` | yes/no | 1 each | Bit-field ×4 | `bool` ×4 |
| IMU fault | `fault.c` | yes/no | 1 | Mask: bit 2 | (none) |
| Hump and pick-up flags (7) | `vehicle.c` `g_v` | yes/no | 1 each | Bit-field | `bool` ×7 |

Kept wider, with a reason: **lift window** `g_lift[20]` (`int32_t`). Excess
mg × ms per sample can exceed ±32767.

## 5. Vehicle and controller states

| Field | Where | Valid range | Bits needed | Now | Before |
|---|---|---|---|---|---|
| Vehicle state; state after a turn | `vehicle.c` `g_v` | 12 states | 4 | Bit-field `: 4` ×2 | enum ×2 |
| Turn phase | `g_v` | 4 values | 2 | Bit-field `: 2` | enum |
| Turn command; pending turn | `g_v` | 5 values | 3 | Bit-field `: 3` ×2 | enum ×2 |
| Vehicle flags (19, incl. 4 feature switches) | `g_v` | yes/no | 1 each | Bit-field | `bool` ×19 |
| MQTT client state | `mqtt_comm.c` `g_mq` | 5 states | 3 | Bit-field `: 3` | enum |
| MQTT flags (5) | `g_mq` | yes/no | 1 each | Bit-field | `bool` ×5 |
| Pending web/MQTT command | `web.c` `g_w` | 17 commands | 5 | Bit-field `: 5` | enum |
| Wi-Fi mode | `g_w` | 5 modes | 3 | Bit-field `: 3` | enum |
| Web flags (13) | `g_w`, `g_wh` | yes/no | 1 each | Bit-field | `bool` ×13 |
| Start-up and test flags (5) | `app_main.c` `g_app` | yes/no | 1 each | Bit-field | `bool` ×5 |
| Power flags (3) | `power.c` `g_pw` | yes/no | 1 each | Bit-field | `bool` ×3 |
| Restart reason | `web.c` `g_restart_code` | 8 kinds × 20 loop stages | 3 + 5 | One byte: kind in bits 0-2, stage in bits 3-7 | text only |
| Faults | `fault.c` `g_faults` | 8 conditions | 8 | One byte, one bit each (`FAULT_...` in `fault.h`) | (none) |

The packed structs, before and after (bytes of RAM):

| Module | Before | After |
|---|---|---|
| `vehicle.c` (`g_v`) | 33 | 6 |
| `bypass.c` (`g_bp`) | 17 | 4 |
| `web.c` (`g_w`, `g_wh`) | 15 | 4 |
| `mqtt_comm.c` (`g_mq`) | 6 | 1 |
| `barcode.c` (`g_bc`) | 9 | 2 |
| `line_sensor.c`, `line_diag.c`, `imu.c`, `obstacle.c`, `servo.c`, `power.c`, `app_main.c`, `ultrasonic.c` | 26 | 8 |

## 6. Data shared between an interrupt and the main loop

There is one control loop on core 0 and no RTOS. Core 1 is not used. lwIP
is polled, so its callbacks also run in the main loop. The only
concurrency is the GPIO and timer interrupts.

| Data | Written by | Read by | How access is safe |
|---|---|---|---|
| Encoder counts, times (`uint32_t`) | GPIO interrupt | main loop | Single writer; a 32-bit load is atomic on the RP2040 |
| Barcode edge ring: times, positions | interrupt | main loop | Single writer; the head index is written last, so an edge is complete before it counts |
| Barcode edge colours (1 bit per edge) | interrupt (read-modify-write of a word) | main loop (read only) | Only the interrupt writes (the GPIO edge **or** the analog timer, never both), so its read-modify-write cannot lose another writer's bit |
| Ultrasonic flags (one byte, 4 bits) | main loop | echo interrupt (reads `ARMED`) | The main loop's read-modify-write runs with interrupts off (`flags_write()`); the interrupt only reads, a byte load is atomic |
| Fault byte (8 bits) | `fault_set()`/`fault_clear()`, from any context | anyone | Every read-modify-write runs with interrupts off (`save_and_disable_interrupts()`); `fault_get()` is a single byte load |
| Echo edge times (`uint32_t`) | echo interrupt | main loop | Single writer, 32-bit words |

The bit-field structs (`g_v`, `g_bp`, `g_w`, …) are read and written only
by the main loop. This was checked: no interrupt handler uses them.

## 7. Inter-task queue messages

Not applicable. The firmware has no RTOS, no tasks and no queues (see
`TEAM.md` §4). The nearest equivalent is the web/MQTT command hand-off,
which is the 5-bit `g_w.pending` in the same thread.

## 8. MQTT telemetry and command payloads

Byte 0 of every frame is the header: type in bits 0-3 (1 telemetry,
2 heartbeat, 3 barcode, 4 obstacle, 5 hump, 6 log, 7 command, 8 ack) and
version `0xA` in bits 4-7. The fields follow in this order, packed LSB
first. "Held" means a value out of range is stored as the nearest end of
the range. "Wraps" means only the low bits are sent.

**Telemetry**: 24 bytes (8 + 182 bits). Before, it was JSON, 143–168
bytes.

| Field | Bits | Range / unit |
|---|---|---|
| time | 32 | ms since boot |
| vehicle state | 4 | 12 states |
| bypass phase | 4 | 13 phases (0 outside BYPASS) |
| motor left, right | 8 + 8 signed | -100..100 % |
| encoder left, right | 16 + 16 | edge count, wraps (the receiver unwraps) |
| line left, line right, barcode on black | 1 + 1 + 1 | yes/no |
| range | 12 | 1..4095 mm, 0 = no echo, held |
| pitch, roll | 12 + 12 signed | ±204.7° in 0.1°, held |
| servo | 8 | 0..180° |
| hump height | 11 signed | ±102.3 mm in 0.1 mm, held |
| total acceleration | 13 | 0..8191 mg, held |
| lift speed | 12 signed | ±2047 mm/s, held |
| pending turn | 3 | 5 commands |
| faults | 8 | fault byte |

**Heartbeat**: 28 bytes (8 + 214 bits). Before, it was JSON, 57–210
bytes.

| Field | Bits | Range / unit |
|---|---|---|
| sequence | 16 | wraps |
| uptime | 24 | s (194 days), held |
| battery (VSYS) | 13 | 0..8191 mV, held |
| messages sent, dropped | 16 + 16 | wrap |
| broker connections | 8 | held at 255 |
| Wi-Fi mode | 3 | 5 modes |
| hotspot | 3 | index in `HOTSPOT_LIST` |
| IP address | 32 | IPv4 |
| signal | 8 signed | dBm |
| ping answered | 1 | yes/no |
| ping round trip | 10 | 0..1023 ms, held |
| ping age | 8 | 0..255 s, held |
| hotspot links lost | 8 | held at 255 |
| restart reason | 8 | kind (3) + loop stage (5) |
| faults | 8 | fault byte |
| longest loop pass | 16 | µs since the last beat, held |
| loop overruns | 16 | passes over 10 ms since boot |

The heartbeat's Wi-Fi line, which used to include the hotspot name, is
now sent as numbers. The web page still shows the text.

**Barcode**: 6 bytes + 6 bits per character. Fields: time 32, command 3,
length 5 (0..31), then each character as its Code 39 index (6 bits; 63 =
not a Code 39 character).

**Obstacle**: 14 bytes. Fields: time 32, front 12 (0 = none), left edge,
right edge and middle 13 signed each (-4096 = none), width 12 (0 = none),
width from confirmed edges 1, left and right edge status 3 + 3. Before,
it was JSON, 78–243 bytes.

**Hump**: 12 bytes. Fields: time 32, height 11 signed, tilt 12 signed,
highest height this run 11 signed, highest tilt 12 signed (all ×10),
count 8 (held at 255).

**Log**: 2 bytes + the line. The category takes 3 bits of byte 1. The
line itself is text for people to read, so it is sent as UTF-8 text
without the JSON wrapper.

**Command** (laptop → car): 2 bytes. Fields: opcode 5 (17 commands). Two
commands take a third byte:

- set a barcode letter: letter 5 (A = 0) + action 3 (none, left, right,
  straight, U-turn);
- feature test: four 1-bit switches (line, avoid, barcode, hump).

The car turns the frame back into the web page's command word, so MQTT
and the page share one command path. A text payload is ignored and
logged.

**Ack**: the accepted command frame, with type 8 in the header.

**Online**: still the single character `1` or `0`. One byte is the
smallest MQTT payload that is not empty. The offline value is the broker
"will", which lwIP sends as a C string, so it cannot be a `0x00` byte.

Not MQTT, and left as text:

- the web page's HTTP JSON (`web.c`);
- the Teleplot stream on USB serial (`telemetry.c`).

Both are views for people while debugging, not messages between parts of
the system.

The same layouts are implemented in three places, which must stay in step:

- the firmware: `app_program/comms/mqtt_frame.c`;
- the Python reference: `tools/mqtt_frame.py`;
- the laptop windows: `tools/mqtt_frames.ps1`.

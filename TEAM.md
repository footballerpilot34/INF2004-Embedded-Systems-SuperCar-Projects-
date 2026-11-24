# Team and subsystems

Team AAI14, INF2004 SuperCar. This file says **who owns what**: subsystems,
pins, peripherals, tasks and the interfaces between subsystems.

> **How this project differs from the template.** The firmware is built with
> the Pico SDK and CMake, **not micro T-Kernel**. There is one bare-metal
> control loop on core 0 (10 ms tick, `app_program/app_main.c`); each module
> has a `*_task()` step function that the loop calls in a fixed order. So
> sections 4 and 6 describe that loop instead of kernel tasks and objects,
> and interfaces (section 5) are function calls, not kernel objects.

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

Owners are as assigned at the Week 6 design review. **To fill in:** student
IDs and GitHub usernames, and confirm each owner (several people worked
across subsystems; see `git log`).

| # | Subsystem (what it does) | Owner | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|---|
| 1 | Line following, crossroads, Code 39 barcodes, start-up line calibration | Ryan | | | `app_program/line_barcode/` | `testing/line_barcode/` |
| 2 | Motors (PWM) and wheel encoders | Saketh | | | `app_program/motion/` | `testing/motion/` |
| 3 | Ultrasonic ranging, servo scan, obstacle bypass | Aruna, Akash | | | `app_program/obstacle/` | `testing/obstacle/` |
| 4 | Wi-Fi, MQTT, web dashboard, telemetry, log | Felix | | | `app_program/comms/` | `testing/comms/` |
| 5 | IMU (LSM303DLHC), hump detection and height, pick-up check | Rui Feng | | | `app_program/imu_hump/` | `testing/imu_hump/` |

The subsystem folders keep descriptive names instead of `subsystem1/` …
`subsystem5/`; the number in this table is the subsystem number used in
the rest of this file and in test IDs.

**Shared files**, changed only by agreement of the team:

| File | What it is |
|---|---|
| `app_program/app_main.c` | Start-up, the 10 ms control loop, buttons, USB serial commands, watchdog |
| `app_program/vehicle.c/.h` | Vehicle state machine: READY, LINE_CAL, FOLLOW_LINE, TURN, OBSTACLE_SCAN, BYPASS, HUMP, BLOCKED, STOPPED … (the Week 6 design review gave the vehicle task to Aruna/Akash) |
| `app_program/stage.h` | Loop stage IDs, so a watchdog restart reports where the loop froze |
| `app_program/gpio_isr.c/.h` | The single GPIO interrupt callback, passing edges to subsystems 1, 2 and 3 |
| `app_program/power.c/.h` | VSYS battery voltage (ADC) |
| `app_program/buzzer.c/.h` | Robo Pico piezo buzzer (beeps for calibration and barcodes) |
| `config/config.h` | Every pin, threshold, speed and network setting |
| `config/lwipopts.h` | lwIP network stack settings |
| `TEAM.md` | This file |

## 2. Pins

From `config/config.h`. One owner per pin; claim a pin here before wiring it.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25 | *reserved* | Pico W radio |
| GP29 | shared (`power.c`) | VSYS / 3 on ADC3; also the radio's pin, so it is read only after Wi-Fi starts |
| GP0 | S2 | Right wheel encoder A |
| GP1 | S2 | Right wheel encoder B |
| GP2 | S5 | IMU SDA (I2C1) |
| GP3 | S5 | IMU SCL (I2C1) |
| GP4 | S1 | Right line IR sensor (digital) |
| GP5 | | Free |
| GP6 | S1 | Left line IR sensor (digital) |
| GP7 | S2 | Left wheel encoder A |
| GP8 | S2 | Left motor M1A (PWM slice 4) |
| GP9 | S2 | Left motor M1B (PWM slice 4) |
| GP10 | S2 | Right motor M2A (PWM slice 5) |
| GP11 | S2 | Right motor M2B (PWM slice 5) |
| GP12 | S3 | Ultrasonic pan servo (PWM slice 6) |
| GP13 | | Free |
| GP14 | | Free |
| GP15 | | Free |
| GP16 | S3 | Ultrasonic TRIG |
| GP17 | S3 | Ultrasonic ECHO (3.3 V level) |
| GP18 | | Not used by the firmware |
| GP19 | | Free |
| GP20 | shared (`app_main.c`) | START button (Robo Pico, active low; hold 2 s = IMU calibration) |
| GP21 | shared (`app_main.c`) | STOP button (Robo Pico, active low) |
| GP22 | shared (`buzzer.c`) | Piezo buzzer (PWM slice 3) |
| GP26 | S1 | Barcode IR sensor, digital output |
| GP27 | S1 | Barcode IR sensor, analog output (ADC1); only read when `BARCODE_USE_ANALOG` is true (currently false) |
| GP28 | S2 | Left wheel encoder B |

## 3. Peripherals

Each peripheral is driven by one module. All modules run in the same
thread (the main loop), so no two of them can drive a peripheral at once.

| Peripheral | Owner subsystem | Owning module / step | Pins |
|---|---|---|---|
| I2C1 (100 kHz) | S5 | `imu.c` / `imu_task()` | GP2, GP3 |
| ADC | shared + S1 | `power.c` / `power_task()` reads VSYS (ADC3). `barcode.c` also uses the ADC (ADC1) in analog mode, which is off; if it is turned on, the two must agree on `adc_select_input()` | GP29, (GP27) |
| PWM slices 4, 5 | S2 | `motor.c` / `motor_set()` | GP8–GP11 |
| PWM slice 6 | S3 | `servo.c` / `servo_set_angle()` | GP12 |
| PWM slice 3 | shared | `buzzer.c` / `buzzer_task()` | GP22 |
| GPIO interrupts (bank 0) | shared | `gpio_isr.c` passes edges to `ultrasonic_echo_isr()` (S3), `barcode_edge_isr()` (S1), `encoder_edge_isr()` (S2) | GP17, GP26, encoder pins |
| Repeating timer | S1 | `barcode.c`, analog sampling only (off) | — |
| Watchdog | shared | `app_main.c` (3 s) | — |
| USB CDC serial | shared | `app_main.c` (commands), `log.c` (printf) | USB |
| Wi-Fi radio, lwIP, MQTT, mDNS, DHCP server | S4 | `web.c` / `web_task()`, `mqtt_comm.c` / `mqtt_comm_task()` (`pico_cyw43_arch_lwip_poll`: polled from the main loop) | GP23–25, GP29 |

**Motors:** `motor_set()` is called by the vehicle state machine (including
`bypass_step()`, which runs inside `vehicle_task()`), by `app_main.c` to
brake (STOP, wheel test, and when the ultrasonic reading is stale), and by
`line_diag.c`. All of these run in the main loop.

## 4. Tasks and execution order

There is no RTOS, so there are no task priorities. One loop on **core 0**
runs every **10 ms** (`TICK_USEC` in `config/config.h`) and calls each step
in this order; earlier steps see their inputs sooner. **Core 1 is not
used.** Interrupts (GPIO edges) run on core 0.

| Order | Step | Subsystem |
|---|---|---|
| 1 | `web_task()` (Wi-Fi, web server) | S4 |
| 2 | `imu_auto_calibrate()` (when parked) | shared / S5 |
| 3 | `mqtt_comm_task()` | S4 |
| 4 | `power_task()` | shared |
| 5 | Buttons, USB serial, web/MQTT commands → `command_run()` | shared |
| 6 | Servo requests from the dashboard | S3 |
| 7 | `ultrasonic_task()` (only once the servo has settled) | S3 |
| 8 | `imu_task()` (reads at 50 Hz) | S5 |
| 9 | `barcode_task()` → `barcode_report()` | S1 |
| 10 | `vehicle_task()` (state machine; calls `line_steer`, `hump_task`, `obstacle_scan_task`, `bypass_step` …) | shared |
| 11 | Stationary scan test (`obstacle_scan_task()`) | S3 |
| 12 | `line_diag_task()` | S1 |
| 13 | `buzzer_task()` | shared |
| 14 | `telemetry_task()` (every 5th tick, 50 ms) | S4 |

If the loop stops for 3 s, the watchdog restarts the car and the next boot
reports the last stage reached (`stage.h`).

## 5. Interfaces between subsystems

Interfaces are function calls declared in each subsystem's headers. The
provider owns the function; the user only calls it.

| ID | From → To | Mechanism | Function(s) | Data / meaning |
|---|---|---|---|---|
| IF-01 | S1 → vehicle | function call | `line_sensor_read()` | `line_state_t`: left/right sensor on black |
| IF-02 | S1 → vehicle | function call | `barcode_task()`, then `vehicle_barcode_command()` (through `barcode_report()` in `app_main.c`) | Decoded letter → turn command for the next crossroad |
| IF-03 | S1 → vehicle | function call | `barcode_is_black()`, `barcode_is_capturing()`, `barcode_end_scan()` | Side sensor state for crossroads and pick-up |
| IF-04 | vehicle, S3, S1, shared → S2 | function call | `motor_set()`, `motor_left_speed()`, `motor_right_speed()` | Wheel power −100…100 % |
| IF-05 | S2 → S1, S3, S5, vehicle, S4 | function call | `encoder_count()`, `encoder_position()` | Wheel edges / distance (barcode widths, bypass odometry, hump distance) |
| IF-06 | S3 → vehicle | function call | `ultrasonic_range_mm()`, `ultrasonic_measurement_ready()` | Forward range in mm, freshness |
| IF-07 | S3 → vehicle | function call | `obstacle_scan_start()`, `obstacle_scan_task()`, `obstacle_profile()` | Stationary servo scan and its result |
| IF-08 | vehicle → S3 | function call | `bypass_begin()`, `bypass_step()`, `bypass_cancel()` | Drive round the obstacle; DONE / FAILED / RUNNING |
| IF-09 | S5 → vehicle | function call | `imu_total_tilt_x10()`, `imu_pitch_x10()`, `imu_roll_x10()`, `imu_accel_mg()`, `imu_lift_speed_mm_s()` | Tilt, total acceleration, upward speed |
| IF-10 | S5 → vehicle, S4 | function call | `hump_task()`, `hump_height_x10()`, `hump_finish()`, `hump_run_height_x10()` | Height the car is on; hump peaks |
| IF-11 | vehicle → S4 | function call | `vehicle_state()`, `vehicle_state_name()`, `vehicle_pending_turn()` | State and pending turn for telemetry and the dashboard |
| IF-12 | S4 → vehicle | function call | `web_command_submit()` (MQTT and web), `web_command_take()` → `command_run()` in `app_main.c` → `vehicle_start()`, `vehicle_stop()`, `vehicle_feature_test()` | Start, stop and test commands |
| IF-13 | all → S4 | function call + ring buffer | `log_line()`, `log_state()`, `log_obstacle()`, `log_printf()`; read with `log_get()`, `log_next_seq()` | Log lines (64 kept) for the web page and the MQTT `…/log` topic |
| IF-14 | shared → S1, S2, S3 | GPIO interrupt | `barcode_edge_isr()`, `encoder_edge_isr()`, `ultrasonic_echo_isr()` | Pin edges, called from `gpio_dispatch_isr()` |
| IF-15 | shared → S4 | function call | `power_vsys_mv()` | Battery voltage for the heartbeat and dashboard |

**To fill in:** the integration test that covers each interface
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Not applicable: the project does not use micro T-Kernel, so it creates no
tasks, semaphores, mailboxes or other kernel objects. All application data
is static (no heap allocation after start-up); lwIP uses its own fixed pool
(`config/lwipopts.h`).

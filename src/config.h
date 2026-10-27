/** @file config.h
 *
 * @brief Pin assignments and tuning values for the SuperCar firmware.
 *
 * @par
 * Every value that may need changing on the real car lives here. Values
 * marked "VERIFY" are assumptions that must be checked on the hardware
 * (use the 'w' wheel test and the Teleplot graphs).
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>

// ------------------------------------------------------------------------
// Pins (current wiring).
// ------------------------------------------------------------------------
#define PIN_LINE_LEFT               28u     // Left line IR DOUT (Grove 7).
#define PIN_LINE_RIGHT              0u      // Right line IR DOUT (Grove 1).
#define PIN_BARCODE                 26u     // Barcode IR DOUT.
#define PIN_IMU_SDA                 2u      // MPU-6050 SDA (I2C1).
#define PIN_IMU_SCL                 3u      // MPU-6050 SCL (I2C1).
#define PIN_US_TRIG                 16u     // Ultrasonic TRIG.
#define PIN_US_ECHO                 17u     // Ultrasonic ECHO (3.3 V).
#define PIN_SERVO                   12u     // Ultrasonic pan servo.
#define PIN_BUTTON_START            20u     // Robo Pico button, active low.
#define PIN_BUTTON_STOP             21u     // Robo Pico button, active low.
#define PIN_BUZZER                  22u     // Robo Pico piezo buzzer.

// ------------------------------------------------------------------------
// Wireless UI: the car makes its own Wi-Fi network; open the page at
// http://192.168.4.1 on a phone or laptop that has joined it.
// ------------------------------------------------------------------------
#define WIFI_NAME                   "SuperCar-AAI14"
#define WIFI_PASSWORD               "supercar14"    // At least 8 characters.

// Wheel test (stage 2) showed LEFT and RIGHT were swapped, so the LEFT
// wheel is on M1 (GP8/GP9) and the RIGHT wheel on M2 (GP10/GP11).
// VERIFY: re-run the 'w' wheel test; flip a REVERSED flag if a wheel
// spins backwards.
#define PIN_LEFT_MOTOR_A            8u      // M1A
#define PIN_LEFT_MOTOR_B            9u      // M1B
#define PIN_RIGHT_MOTOR_A           10u     // M2A
#define PIN_RIGHT_MOTOR_B           11u     // M2B
#define LEFT_MOTOR_REVERSED         true
#define RIGHT_MOTOR_REVERSED        true

// ------------------------------------------------------------------------
// IR sensors.
// ------------------------------------------------------------------------
// MEASURED with IR DIAG (LEARN BLACK / LEARN WHITE): both line sensors
// read HIGH over black and LOW over white.
// VERIFY the barcode sensor the same way (same module type assumed).
#define LINE_BLACK_IS_HIGH          true
#define BARCODE_BLACK_IS_HIGH       true

// The two line sensors are the limits of the line. While following, the
// car always keeps moving forward:
// - both on the line, or both lost together: straight on;
// - one on the line: arc towards it (inner wheel slowed);
// - neither, after one was last on the line: keep arcing that way.
#define LINE_LOST_MSEC              3000u   // No line this long: stop.

// START calibration (closed loop, bounded):
// 1. Sweep: swing left/right, each swing LINE_CAL_SWING_MSEC longer than
//    the last, until EACH sensor has seen both black and white.
// 2. Align: pivot until both sensors are on black for 30 ms.
// Any timeout stops the car with a reason; only a manual START retries.
#define LINE_CAL_SWING_MSEC         1000u   // First swing; then 2000, 3000..
#define LINE_CAL_SWEEP_MAX_MSEC     16000u  // Sweep must finish in this.
#define LINE_CAL_ALIGN_MAX_MSEC     5000u   // Align must finish in this.
#define LINE_CAL_ALIGN_TICKS        3u      // Both on black this long.

// ------------------------------------------------------------------------
// Speeds, in percent of full power (-100 to +100).
// ------------------------------------------------------------------------
#define SPEED_CRUISE                8       // Line following (halved from
                                            // 15 to make steering calmer).
#define SPEED_CREEP                 6       // Near an obstacle (halved).
#define SPEED_SWEEP                 6       // START calibration pivot
                                            // (25% slower than 8).
#define SPEED_KICK                  12      // Brief push when a pivot
#define KICK_MSEC                   80u     // starts or reverses, to get
                                            // the wheels moving.
#define SPEED_HUMP                  15      // Climbing a hump (kept higher:
                                            // the slope needs torque).
#define TURN_INNER_PERCENT          0       // Inner wheel speed in a turn,
                                            // as a percent of the outer.
                                            // 0 = inner wheel stops, so
                                            // the car keeps moving forward
                                            // (negative values spin it on
                                            // the spot and it oscillates).

// Steering filter: a sensor must read the same for this many 10 ms ticks
// before the steering reacts, so flicker at the line edge is ignored.
#define LINE_DEBOUNCE_TICKS         2u

// After calibration, drive straight this long before steering, so the car
// gets moving instead of correcting from a standstill.
#define LINE_START_STRAIGHT_MSEC    300u

// ------------------------------------------------------------------------
// Obstacle handling (ultrasonic).
// ------------------------------------------------------------------------
#define OBSTACLE_SLOW_MM            400u    // Slow to creep speed.
#define OBSTACLE_STOP_MM            250u    // Possible obstacle: probe it.
#define OBSTACLE_HARD_STOP_MM       80u     // Wall: stop for certain.
#define OBSTACLE_CLEAR_MM           300u    // Clear again (hysteresis).
#define OBSTACLE_PROBE_MSEC         4000u   // Longest creep while probing.
#define OBSTACLE_CLEAR_MSEC         200u    // Clear this long to resume.

#define US_PING_PERIOD_USEC         60000u  // Pings at least 60 ms apart.
#define US_ECHO_TIMEOUT_USEC        30000u  // About 5 m.
#define US_TRIGGER_USEC             10u
#define US_STALE_MSEC               200u    // Range older than this is
                                            // treated as unknown.

// ------------------------------------------------------------------------
// Hump detection (MPU-6050 pitch).
// ------------------------------------------------------------------------
// VERIFY with gyro_test: which accelerometer axis points forward, which
// gyro axis turns when the nose tips up, and the signs that make nose-up
// read positive on the "pitch_deg" graph.
#define IMU_FORWARD_ACCEL_AXIS      0u      // 0 = X, 1 = Y, 2 = Z.
#define IMU_PITCH_GYRO_AXIS         1u      // 0 = X, 1 = Y, 2 = Z.
#define IMU_ACCEL_PITCH_SIGN        1
#define IMU_GYRO_PITCH_SIGN         (-1)

#define HUMP_ENTER_PITCH_X10        80      // 8.0 deg nose up: on a hump.
#define HUMP_LEVEL_PITCH_X10        30      // Within 3.0 deg: level again.
#define HUMP_LEVEL_MSEC             300u    // Level this long: hump done.
#define HUMP_MIN_MSEC               500u    // Shortest possible hump.
#define HUMP_MAX_MSEC               8000u   // Longer than this: stop.
#define HUMP_IGNORE_RANGE_MSEC      500u    // Ignore ultrasonic after a
                                            // hump (it may see the floor).

// ------------------------------------------------------------------------
// Barcode (Code 39).
// ------------------------------------------------------------------------
#define BARCODE_END_GAP_MIN_USEC    300000u // No edge this long ends a scan.
#define BARCODE_END_GAP_FACTOR      3u      // ...or 3x the widest element.
#define BARCODE_MESSAGE_MAX         16u     // Longest message kept.

// ------------------------------------------------------------------------
// Timing.
// ------------------------------------------------------------------------
#define TICK_USEC                   10000u  // 10 ms control period.
#define TELEMETRY_EVERY_TICKS       5u      // Teleplot points every 50 ms.
#define BUTTON_DEBOUNCE_TICKS       3u      // 30 ms START debounce.
#define BUTTON_LONG_PRESS_TICKS     200u    // Hold START 2 s: calibrate.
#define CAL_DONE_BEEP_MSEC          150u    // Beep when calibration ends.

#endif /* CONFIG_H */

/*** end of file ***/

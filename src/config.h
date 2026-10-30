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
#define PIN_BARCODE                 26u     // Barcode IR DOUT (Grove 5).
#define PIN_BARCODE_AO              27u     // Barcode IR AO (Grove 5), ADC1.
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
// car always keeps moving forward and never stops for white (only for an
// obstacle, or STOP):
// - both on the line, or both lost together: straight on;
// - one on the line: arc towards it (inner wheel slowed);
// - neither, after one was last on the line: keep arcing that way.

// START calibration: find the line from anywhere near it, repeating
// until it is found (STOP always works):
// 1. Pivot one way (LINE_CAL_PIVOT_LEFT) until either sensor sees black,
//    for up to LINE_CAL_PIVOT_MAX_MSEC.
// 2. Still no line: drive straight ahead until either sensor sees black
//    (or an obstacle is ahead), then go back to step 1.
// 3. Align: pivot until both sensors are on black for 30 ms, then follow.
//    If that fails within LINE_CAL_ALIGN_MAX_MSEC, go back to step 1.
// Success also needs EACH sensor to have DETECTED black during
// calibration: switched from white to black, so a dead or stuck sensor
// cannot pass. Starting on the line, the car turns off and back once to
// prove it.
#define LINE_CAL_PIVOT_LEFT         true    // Search pivot: true = left.
#define LINE_CAL_PIVOT_MAX_MSEC     8000u   // Pivot this long (7-10 s), then
                                            // drive forward instead.
#define LINE_CAL_DRIVE_SPEED        8       // Forward search speed.
#define LINE_CAL_ALIGN_MAX_MSEC     5000u   // Align must finish in this.
#define LINE_CAL_ALIGN_TICKS        3u      // Both on black this long.
#define LINE_CAL_ALIGN_TRIES        3u      // Failed aligns before giving
                                            // up and naming the sensor that
                                            // never detected black.

// ------------------------------------------------------------------------
// Speeds, in percent of full power (-100 to +100).
// ------------------------------------------------------------------------
#define SPEED_CRUISE                8       // Line following (halved from
                                            // 15 to make steering calmer).
#define SPEED_CREEP                 6       // Near an obstacle (halved).
// START calibration pivots are PULSED: the motors stall below about
// 10-15% when turning on the spot, so instead of a weak steady 6%, each
// pivot is short bursts at a power that really moves the wheels, with
// braked pauses between them. Average turning speed stays slow:
// slower turning -> shorter ON or longer OFF; stalls -> raise the speed.
#define PIVOT_PULSE_SPEED           20      // Burst power (moves the car).
#define PIVOT_PULSE_ON_MSEC         60u     // Burst length.
#define PIVOT_PULSE_OFF_MSEC        120u    // Braked pause between bursts.
#define SPEED_HUMP                  15      // Climbing a hump (kept higher:
                                            // the slope needs torque).
#define TURN_INNER_PERCENT          0       // Inner wheel speed in a turn,
                                            // as a percent of the outer.
                                            // 0 = inner wheel stops, so
                                            // the car keeps moving forward
                                            // (negative values spin it on
                                            // the spot and it oscillates).

// Turn delay: a sensor that goes WHITE must stay white this long before
// the car turns; if it goes back to black first, the movement does not
// change. A sensor that sees BLACK again counts at once.
#define LINE_TURN_DELAY_MSEC        200u

// After calibration, drive straight this long before steering, so the car
// gets moving instead of correcting from a standstill.
#define LINE_START_STRAIGHT_MSEC    300u

// ------------------------------------------------------------------------
// Obstacle handling (ultrasonic).
// ------------------------------------------------------------------------
// Stop at once: an echo from closer than OBSTACLE_STOP_MM brakes the car
// (BLOCKED) straight away; it resumes once the way is clear. false = the
// hump/wall PROBE instead (creep closer, climb if the nose tips up). With
// true the car stops in front of a hump too.
#define OBSTACLE_STOP_AT_ONCE       true
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
// Barcode (Code 39) on the side IR sensor; barcodes are 20 mm beside the
// line, so the sensor must sit far enough out that it never sees the line.
// ------------------------------------------------------------------------
#define BARCODE_END_GAP_MIN_USEC    300000u // No edge this long ends a scan.
#define BARCODE_END_GAP_FACTOR      3u      // ...or 3x the widest element.
#define BARCODE_MESSAGE_MAX         16u     // Longest message kept.
#define BARCODE_GLITCH_USEC         2000u   // A colour shorter than this is
                                            // sensor flicker, not a bar.

// The module's digital output (DOUT) misses thin bars: its comparator
// only switches at one fixed level, and a thin bar only dims the sensor's
// spot a little. So the analog output (AO) is sampled instead and edges
// are found in software: whenever the darkness rises BARCODE_AO_DELTA
// above the last dip, a bar starts; whenever it falls BARCODE_AO_DELTA
// below the last peak, a space starts. A thin bar's small dip counts
// as long as it is bigger than BARCODE_AO_DELTA.
// MEASURED (Teleplot, car still): white paper ~166, thick barcode bar
// ~2350, track tape ~3055, flicker about +/-15. A bar must also rise
// DELTA/2 above the tracked paper-white level, so noise can never start
// a bar or hold the reader on "black" over white.
// Tuning: hold the sensor still over a THIN bar; DELTA must be well below
// (thin bar level - white level).
#define BARCODE_USE_ANALOG          true    // false = old DOUT reading.
#define BARCODE_AO_BLACK_IS_HIGH    true    // VERIFY: the web "level"
                                            // must RISE over black.
#define BARCODE_AO_SAMPLE_USEC      200u    // 5 kHz sampling.
#define BARCODE_AO_OVERSAMPLE       8u      // ADC reads averaged per sample
                                            // (noise / ~2.8; 16 us of CPU).
#define BARCODE_AO_DELTA            80      // Edge size, in ADC counts
                                            // (0-4095). Lower: fainter bars
                                            // seen, but more false edges.

// Bar widths are measured in time, so while a barcode is being read the
// car ignores the line sensors and drives straight at one steady speed.
#define SPEED_SCAN                  8       // Steady speed while reading.
#define BARCODE_SCAN_MAX_MSEC       5000u   // Longest straight drive for
                                            // one scan, then steer again.

// Messages that are COMMANDS (exact match; Code 39 is upper case). They
// trump the line sensors. Anything else is shown as a plain message.
// VERIFY against the real barcodes and change the texts to match.
#define BARCODE_TEXT_LEFT           "LEFT"
#define BARCODE_TEXT_LEFT_SHORT     "L"
#define BARCODE_TEXT_RIGHT          "RIGHT"
#define BARCODE_TEXT_RIGHT_SHORT    "R"
#define BARCODE_TEXT_UTURN          "UTURN"
#define BARCODE_TEXT_UTURN_SHORT    "U"

// Barcode turns: pivot (pulsed, like calibration) off the line, then
// until a line sensor finds a line, then until both are on it; then
// follow it. With no line there, a left/right turn stops pivoting after
// TURN_90_MSEC and drives on, latching onto the next line it meets.
// VERIFY TURN_90_MSEC with the TEST LEFT/RIGHT buttons with the car
// stopped: it should turn about 90 degrees and stop.
#define TURN_90_MSEC                2000u   // Pivot time for about 90 deg.
#define TURN_UTURN_MAX_MSEC         6000u   // U-turn gives up after this.
#define TURN_UTURN_LEFT             true    // U-turn pivot: true = left.
#define TURN_ALIGN_MAX_MSEC         1500u   // Both-on-line must take less.

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

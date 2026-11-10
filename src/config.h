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
// Pin finder (2026-11-06): the LEFT wheel's encoder is on GP6 + GP19
// (3140 + 3142 edges, healthy quadrature); the RIGHT one on GP4 + GP5,
// but only GP5 counted (157 edges, GP4 0): VERIFY its wiring.
#define PIN_ENC_LEFT_A              6u      // Left wheel encoder A.
#define PIN_ENC_LEFT_B              19u     // Left wheel encoder B (moved
                                            // from GP7).
#define PIN_ENC_RIGHT_A             4u      // Right wheel encoder A.
#define PIN_ENC_RIGHT_B             5u      // Right wheel encoder B.
// The pin finder watches the encoder pins and every free GPIO and shows
// each pin's edges on the web page. Turn it off once the wiring is right.
#define ENCODER_PIN_FINDER          false

// Ultrasonic pan servo (as in robocar.c): centred at start, and turned
// from the web page's slider within SERVO_RANGE_DEG either side.
#define SERVO_CENTER_DEG            90      // Facing straight ahead.
#define SERVO_RANGE_DEG             30      // Limit either side (60-120).
#define SERVO_MIN_USEC              500     // Pulse width at 0 degrees.
#define SERVO_MAX_USEC              2500    // Pulse width at 180 degrees.

// ------------------------------------------------------------------------
// Wireless UI.
// Hotspot mode: src/wifi_secrets.h (copy src/wifi_secrets_example.h; git
// ignores it, so passwords stay off GitHub) lists hotspots (e.g. phone
// hotspots) in HOTSPOT_LIST. The car tries each in turn, or the one on the
// page, and the page is at http://WIFI_HOST_NAME.local or the IP it logs.
// Each is tried for up to WIFI_TRY_MSEC, round and round until one
// answers (also when the link drops); the car makes no network of its own.
// Own-network mode, only if no hotspot is listed: join WIFI_NAME and open
// http://192.168.4.1.
// ------------------------------------------------------------------------
#define WIFI_NAME                   "SuperCar-AAI14"
#define WIFI_PASSWORD               "supercar14"    // At least 8 characters.
#define WIFI_HOST_NAME              "supercar-aai14"    // Must match
                                                        // CYW43_HOST_NAME in
                                                        // CMakeLists.txt.
#define WIFI_TRY_MSEC               10000u  // Longest try at one hotspot...
#define WIFI_GET_IP_MSEC            45000u  // ...once on it but still
                                            // waiting for an IP address.
#define WIFI_FAIL_SETTLE_MSEC       1500u   // A failure is only believed
                                            // this long into a try.
#define WIFI_STILL_LOG_MSEC         30000u  // After the first time round the
                                            // list, "still looking" this
                                            // often instead of every try.
#define WIFI_CHECK_MSEC             250u    // How often the link is checked.
#define WIFI_PING_MSEC              2000u   // Ping the hotspot this often...
#define WIFI_PING_LOST_MSEC         10000u  // ...no reply this long: rejoin
                                            // (only if it ever answered).

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"           // HOTSPOT_LIST.
#endif

// The older one-hotspot form still works.
#if defined(HOTSPOT_SSID) && !defined(HOTSPOT_LIST)
#define HOTSPOT_LIST                { HOTSPOT_SSID, HOTSPOT_PASSWORD }
#endif

#ifndef HOTSPOT_AUTH
#define HOTSPOT_AUTH                CYW43_AUTH_WPA2_AES_PSK
#endif

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

// START calibration (user, 2026-11-06): turn on the spot at half the
// driving speed (LINE_CAL_SPEED_PERCENT of SPEED_CRUISE, as an average of
// PIVOT_PULSE_SPEED bursts) until both line sensors are on black, then
// follow. Direction, chosen every tick:
// - one line sensor on black: turn towards it;
// - neither: the side (barcode) sensor on black -> turn left (its side);
//   otherwise keep turning the same way (LINE_CAL_PIVOT_LEFT at first);
// - both on black: skip calibration (at the start) or stop turning.
// It turns until the line is found or STOP is pressed.
#define LINE_CAL_PIVOT_LEFT         true    // First turn: true = left.
#define LINE_CAL_SPEED_PERCENT      50      // % of SPEED_CRUISE: 9 -> 4%.
#define LINE_CAL_ALIGN_TICKS        3u      // Both on black this long (after
                                            // turning) to finish.

// ------------------------------------------------------------------------
// Speeds, in percent of full power (-100 to +100).
// ------------------------------------------------------------------------
#define SPEED_CRUISE                9       // 8 + 1: at 8 the car jerked
                                            // (too little power,
                                            // 2026-11-10).
#define SPEED_CREEP                 6       // 5 + 1, same reason.
// START calibration pivots are PULSED: the motors stall below about
// 10-15% when turning on the spot, so instead of a weak steady 6%, each
// pivot is short bursts at a power that really moves the wheels, with
// braked pauses between them. Average turning speed stays slow:
// slower turning -> shorter ON or longer OFF; stalls -> raise the speed.
#define PIVOT_PULSE_SPEED           20      // Burst power, -20% from 25.
#define PIVOT_PULSE_ON_MSEC         60u     // Burst length.
#define PIVOT_PULSE_OFF_MSEC        180u    // Braked pause between bursts
                                            // (was 120: turning speed
                                            // 75%, burst power kept).
#define SPEED_HUMP                  23      // Climbing a hump, +25% from 18
                                            // (22.5, rounded).
// Line-following turns keep BOTH wheels rolling forward, so the car never
// pauses to turn: the outer wheel speeds up and the inner wheel slows
// (never 0, which would brake it and pivot the car on that wheel). The
// turning rate is about the same as braking the inner wheel at the base
// speed, but the car keeps moving forward at about the base speed.
// Too gentle for sharp curves: raise OUTER or lower INNER.
#define TURN_OUTER_PERCENT          150     // Outer wheel, % of base speed.
#define TURN_INNER_PERCENT          60      // Inner wheel, % of base speed.
// Hard turn, once BOTH sensors have slid off the line: pulls the car back
// onto it quickly instead of drifting alongside it. Inner wheel is kept
// just above 0 (rolls freely, never brakes). Still drifts off on curves:
// raise HARD_OUTER; overshoots the line: lower it.
#define TURN_HARD_OUTER_PERCENT     180
#define TURN_HARD_INNER_PERCENT     20

// Dirty-track filter. Each line sensor keeps a score: +1 tick for every
// tick it reads black, -1 for every tick it reads white, held between 0
// and LINE_TURN_DELAY_MSEC. So short flickers cancel out instead of
// steering the car:
// - grey footprints on white blip "black" now and then; a sensor only
//   counts as black once black has beaten white by LINE_BLACK_CONFIRM_MSEC;
// - scratches on the tape blip "white"; a sensor on the line only counts
//   as white once the score has drained to 0 (about LINE_TURN_DELAY_MSEC
//   of mostly-white).
// Still jittery: raise both. Too slow on curves: lower them.
#define LINE_TURN_DELAY_MSEC        120u
#define LINE_BLACK_CONFIRM_MSEC     60u

// Grey patches. Real tape reads SOLID black; a grey patch sits near the
// sensor's switching level and flickers. While a sensor is white, each
// white reading cancels this many black readings, so only nearly solid
// black is taken as the line (3: anything under about 75% black is
// rejected). Patches still caught: raise it. Real line missed: lower it.
#define LINE_GREY_WHITE_WEIGHT      3u

// Line lost (both sensors white): black on either sensor counts after
// only this long (instead of LINE_BLACK_CONFIRM_MSEC), so a sensor that
// sweeps across the line is caught and the car turns towards it rather
// than turning past it.
#define LINE_LOST_BLACK_CONFIRM_MSEC 20u

// Backtrack: when BOTH sensors lose the line, the car reverses along the
// path it just drove (its recorded wheel speeds, played backwards) to the
// point where BOTH sensors were last on the line, then follows on from
// there. Once per loss: if the line still is not found, it turns hard
// towards the side the line was last seen. Paths longer than MAX are not
// kept.
#define LINE_BACKTRACK_ENABLE       true
#define LINE_BACKTRACK_MAX_MSEC     5000u
#define LINE_BACKTRACK_BRAKE_MSEC   80u     // Stop rolling before reversing.
// Reverse at this % of the recorded power, over a correspondingly shorter
// time, so it covers the same path but more firmly. Lands short or long
// of the line: adjust it.
#define LINE_BACKTRACK_SPEED_PERCENT 150u
// Recorded ticks averaged together while reversing (odd), so the small
// corrections in the recording do not make the reverse wobble.
#define LINE_BACKTRACK_SMOOTH_TICKS 5u

// Turning towards a sensor on black always happens at once, whichever
// way the car was turning. Only straightening up waits until the last
// steering change is this old, so the car cannot flip back and forth. Raise to calm it further; lower if
// it overshoots the line after a turn.
#define LINE_STEER_HOLD_MSEC        150u

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
#define US_ECHO_MIN_USEC            100u    // Shorter echoes (under about
                                            // 2 cm) are glitches: ignored.
#define US_STALE_MSEC               200u    // Range older than this is
                                            // treated as unknown.

// LSM303DLHC pitch uses Y-forward / Z-up axes (see IMU mounting diagram).
// Tilt is low-pass filtered; thresholds provide hysteresis against noise.
#define HUMP_ENTER_PITCH_X10        80      // 8.0 deg nose up: on a hump.
#define HUMP_LEVEL_PITCH_X10        30      // Within 3.0 deg: level again.
#define HUMP_LEVEL_MSEC             700u    // Level after tilt excursion: done.
#define HUMP_MIN_MSEC               500u    // Shortest possible hump.
#define HUMP_MAX_MSEC               8000u   // Longer than this: stop.
#define HUMP_IGNORE_RANGE_MSEC      500u    // Ignore ultrasonic after a
                                            // hump (it may see the floor).

// ------------------------------------------------------------------------
// Barcode (Code 39) on the side IR sensor; barcodes are 20 mm beside the
// line, so the sensor must sit far enough out that it never sees the line.
// ------------------------------------------------------------------------
#define BARCODE_END_GAP_MIN_USEC    300000u // No edge this long ON WHITE
                                            // ends a scan...
#define BARCODE_BLACK_MAX_USEC      2000000u // ...on black, only this long
                                            // (sensor left on a branch or
                                            // the line).
#define BARCODE_HAND_END_GAP_USEC   1500000u // BARCODE TEST (STATIC): a scan
                                            // slid by hand ends only after
                                            // this long on white.
// Bar widths measured in DISTANCE (wheel encoders, encoder.c) instead of
// time, so the car slowing or pausing over a barcode no longer stretches
// its bars (2026-11-06). If the encoders turned less than
// BARCODE_MIN_SCAN_TICKS over a scan (not connected, or the car was
// pushed by something else), it falls back to time and says so in the log.
#define BARCODE_USE_ENCODERS        true
#define BARCODE_MIN_SCAN_TICKS      20u     // Both wheels' ticks, per scan.
#define BARCODE_STALL_USEC          5000000u // No edge and no ending for
                                            // this long: end the scan.
#define BARCODE_END_GAP_FACTOR      5u      // ...or 5x the widest element.
                                            // Must exceed the 3:1 wide:narrow
                                            // ratio, or the first wide space
                                            // (after a narrow bar) can end
                                            // the scan. Used until a wide
                                            // element has been seen...
#define BARCODE_END_GAP_FAST        2u      // ...then 2x is enough (nothing
                                            // inside is wider than 3 narrow
                                            // units), so a scan ends soon
                                            // after the last bar.
#define BARCODE_WIDE_SEEN_X10       20u     // Wide seen: widest >= 2.0x
                                            // the narrowest.
#define BARCODE_MESSAGE_MAX        16u     // Longest message kept.
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
// 2026-11-06: back to DOUT. The user re-tuned the module's potentiometer
// so DOUT switches on every bar, thin ones included, and the sensor sits
// slightly raised, which shrinks and varies the analog swing (fixed DELTA
// then gave inconsistent reads). true = analog reading as described above.
#define BARCODE_USE_ANALOG          false   // false = DOUT (GP26) reading.
#define BARCODE_AO_BLACK_IS_HIGH    true    // VERIFY: the web "level"
                                            // must RISE over black.
#define BARCODE_AO_SAMPLE_USEC      200u    // 5 kHz sampling.
#define BARCODE_AO_OVERSAMPLE       8u      // ADC reads averaged per sample
                                            // (noise / ~2.8; 16 us of CPU).
#define BARCODE_AO_DELTA            500     // Edge size, in ADC counts
                                            // (0-4095). Lower: fainter bars
                                            // seen, but more false edges.
                                            // 80 split solid bars into
                                            // 7-20 ms pieces while driving
                                            // (2026-11-03); 500 is ~1/4 of
                                            // the white-to-bar swing.

// Bar widths are measured in time, so from the first bar until the scan
// ends the car drives strictly straight at one steady speed, holding BOTH
// line sensors on the line: no line-following turns, reversing or hard
// arcs, and only an obstacle (or a hump) interrupts it. If one sensor
// slips off, the wheel on the far side runs SCAN_TRIM_EXTRA faster to
// bring it back; the other wheel stays at SPEED_SCAN. Slowing a wheel
// instead (80% of 8 = 6) stalled it and the car crawled over the barcode
// (2026-11-06). Drifts off the line during a scan: raise SCAN_TRIM_EXTRA;
// wobbles: lower it.
#define SPEED_SCAN                  8       // Steady speed while reading
                                            // (7 + 1, ~38 mm/s; 6 jerked,
                                            // 7 still did, 2026-11-10).
#define SCAN_TRIM_EXTRA             2       // Far-side wheel: SPEED_SCAN + 2.

// Messages that are COMMANDS (exact match; Code 39 is upper case).
// Anything else is shown as a plain message. LEFT and RIGHT are kept
// until the next crossroad and carried out there; a U-turn is at once.
// Track barcodes (user, 2026-11-03): 'A' = turn left at the next
// crossroad, 'Z' = turn right at the next crossroad.
#define BARCODE_TEXT_LEFT           "A"
#define BARCODE_TEXT_LEFT_SHORT     "L"
#define BARCODE_TEXT_RIGHT          "Z"
#define BARCODE_TEXT_RIGHT_SHORT    "R"
#define BARCODE_TEXT_UTURN          "UTURN"
#define BARCODE_TEXT_UTURN_SHORT    "U"

// Barcode turns: pivot (pulsed, like calibration) off the line, then
// until a line sensor finds a line, then until both are on it; then
// follow it. With no line there, a left/right turn stops pivoting after
// TURN_90_MSEC and drives on, latching onto the next line it meets.
// VERIFY TURN_90_MSEC with the TEST LEFT/RIGHT buttons with the car
// stopped: it should turn about 90 degrees and stop.
#define TURN_90_MSEC                2700u   // Pivot time for about 90 deg
                                            // (x1.33: 75% turn speed).
#define TURN_UTURN_MAX_MSEC         8000u   // U-turn gives up after this.
#define TURN_UTURN_LEFT             true    // U-turn pivot: true = left.
#define TURN_ALIGN_MAX_MSEC         2000u   // Both-on-line must take less.

// Crossroads (+), detected in line_sensor.c. The two line sensors stay
// black across a crossing line, so the crossroad is seen by the LEFT side
// (barcode) sensor instead: the left branch is one long black mark (18 mm
// wide). A black mark that lasted LINE_JUNCTION_MIN_MSEC to _MAX_MSEC is
// a crossroad (judged when it ends), and a pending LEFT/RIGHT turn is
// carried out there. Detection is ON ONLY once a barcode has been read
// and its turn is waiting (a barcode always comes before its crossroad),
// so a barcode's own bars, even a big print's ~19 mm wide bars, are never
// a crossroad. A branch is 18 mm (~470 ms at SPEED_SCAN 8, ~38 mm/s). The
// pivot point is the wheel axle, behind the sensors, so the car first
// drives on for JUNCTION_ADVANCE_MSEC to bring the axle over the crossing.
#define LINE_JUNCTION_MIN_MSEC      340u    // ~13 mm at SPEED_SCAN 8.
#define LINE_JUNCTION_MAX_MSEC      1050u   // ~40 mm: longer black (the
                                            // mat past a paper edge) is
                                            // not a branch (2026-11-06:
                                            // false crossroads).
#define LINE_JUNCTION_GAP_MSEC      20u     // Shorter white inside a mark
                                            // (glare, scuff) is ignored.
#define JUNCTION_ADVANCE_MSEC       1180u   // VERIFY: about the line-sensor
                                            // to axle distance at
                                            // SPEED_SCAN (~32 mm/s).
// A + crossroad or a sharp bend? The side sensor crosses a line at both.
// After the mark the car follows on for JUNCTION_CONFIRM_MSEC (part of
// the ADVANCE above, so it must be shorter): a line sensor white for
// JUNCTION_BEND_WHITE_MSEC means the line ended ahead (a bend: the turn
// waits for the real crossroad); both still on black means a + crossroad.
#define JUNCTION_CONFIRM_MSEC       400u    // ~14 mm at cruise speed.
#define JUNCTION_BEND_WHITE_MSEC    150u    // Shorter white is a wobble.

// ------------------------------------------------------------------------
// Supply watch (power.c): VSYS, the battery voltage the Pico gets, shown
// on the page with its lowest value since power-up. A 1-cell LiPo is
// about 4.2 V full and 3.7 V half; below about 3.5 V under load the
// motors jerk and the Pico can brown out and restart.
// ------------------------------------------------------------------------
#define POWER_SAMPLE_MSEC           1000u   // Read the supply this often.
#define POWER_LOW_MV                3500u   // Below this: "battery low".
#define POWER_VALID_MIN_MV          2500u   // Below this the reading is
                                            // wrong (the Pico would be
                                            // off): shown as unavailable.
#define POWER_LOW_HYST_MV           200u    // Warn again once it has been
                                            // this far above, then dips.

// ------------------------------------------------------------------------
// Timing.
// ------------------------------------------------------------------------
#define TICK_USEC                   10000u  // 10 ms control period.
#define TELEMETRY_EVERY_TICKS       5u      // Teleplot points every 50 ms.
#define BUTTON_DEBOUNCE_TICKS       3u      // 30 ms START debounce.
#define BUTTON_LONG_PRESS_TICKS     200u    // Hold START 2 s: calibrate.
#define CAL_DONE_BEEP_MSEC          150u    // Beep when calibration ends.
#define BARCODE_BEEP_MSEC           80u     // Soft beep: barcode read.
#define BUZZER_SOFT_DUTY_PERCENT    5u     // Soft beep loudness (PWM duty;
                                            // 50 = normal beep). Lower is
                                            // quieter.
 
#endif /* CONFIG_H */

/*** end of file ***/

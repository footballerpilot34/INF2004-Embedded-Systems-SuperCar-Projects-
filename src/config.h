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
// 2026-11-13: GP26 is on BOTH Grove 5 (GP6/GP26) and Grove 6 (GP26/GP27).
// Tested on the car: the left line sensor's DOUT reached GP26 (its cable
// puts DOUT on the port's second pin), and so does the barcode sensor's
// (its LED followed the left line sensor). Two DOUTs on one pin cannot
// be told apart: the left line sensor's DOUT must be moved onto the GP6
// wire (and its GP26 wire left unconnected) for these pins to be right.
#define PIN_LINE_LEFT               6u      // Left line IR DOUT (Grove 5).
#define PIN_LINE_RIGHT              4u      // Right line IR DOUT (Grove 3).
                                            // VERIFY: may be GP5 instead.
#define PIN_BARCODE                 26u     // Barcode IR DOUT (Grove 6).
#define PIN_BARCODE_AO              27u     // Barcode IR AO (Grove 6), ADC1.
#define PIN_IMU_SDA                 2u      // MPU-6050 SDA (I2C1).
#define PIN_IMU_SCL                 3u      // MPU-6050 SCL (I2C1).
#define PIN_US_TRIG                 16u     // Ultrasonic TRIG.
#define PIN_US_ECHO                 17u     // Ultrasonic ECHO (3.3 V).
#define PIN_SERVO                   12u     // Ultrasonic pan servo.
#define PIN_BUTTON_START            20u     // Robo Pico button, active low.
#define PIN_BUTTON_STOP             21u     // Robo Pico button, active low.
#define PIN_BUZZER                  22u     // Robo Pico piezo buzzer.
// Rewired: both signal wires of each encoder use its Grove port.
// Counts include both channels; A/B order does not affect unsigned counts.
#define PIN_ENC_LEFT_A              7u      // Left encoder, Grove 7.
#define PIN_ENC_LEFT_B              28u     // Left encoder, Grove 7.
#define PIN_ENC_RIGHT_A             0u      // Right encoder, Grove 1.
#define PIN_ENC_RIGHT_B             1u      // Right encoder, Grove 1.
// The pin finder watches the encoder pins and every free GPIO and shows
// each pin's edges on the web page. Turn it off once the wiring is right.
#define ENCODER_PIN_FINDER          true

// Ultrasonic pan servo: 60 = left, 90 = forward, 120 = right (confirmed).
// Centred while driving; automatic scans own it while braked. Web slider
// control is available only while parked.
#define SERVO_CENTER_DEG            90      // Facing straight ahead.
#define SERVO_RANGE_DEG             30      // Limit either side (60-120).
#define SERVO_MIN_USEC              500     // Pulse width at 0 degrees.
#define SERVO_MAX_USEC              2500    // Pulse width at 180 degrees.
#define SERVO_SETTLE_MSEC           300u    // Verify on the mounted servo.

// ------------------------------------------------------------------------
// Wireless UI.
// Hotspot mode: src/wifi_secrets.h (copy src/wifi_secrets_example.h; git
// ignores it, so passwords stay off GitHub) lists hotspots (e.g. phone
// hotspots) in HOTSPOT_LIST. The car tries each in turn, or the one on the
// page, and the page is at http://WIFI_HOST_NAME.local or the IP it logs.
// Each is tried for up to WIFI_TRY_MSEC, round and round until one
// answers (also when the link drops). After WIFI_RECOVERY_MSEC without a
// connection, join WIFI_NAME and open http://192.168.4.1 to recover the UI;
// select a hotspot and press USE to retry. No hotspots listed: own network.
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
#define WIFI_RECOVERY_MSEC          60000u  // Offline this long: recovery AP.
#define WIFI_RECOVERY_RETRY_MSEC    90000u  // Recovery AP with nobody on it
                                            // this long: retry the hotspot.
#define WIFI_PING_MSEC              2000u   // Ping the hotspot this often...
                                            // for diagnostics, not link control.

// MQTT telemetry and commands (mqtt_comm.c). Topics are under
// MQTT_TOPIC_PREFIX; see mqtt_comm.h. The broker must be reachable from
// the hotspot: a laptop on the same hotspot running Mosquitto (listening
// on all addresses, anonymous allowed), or a public test broker if the
// hotspot has internet. Set it here, or at run time on the web page
// (More controls -> MQTT broker); "" = off until set there.
#define MQTT_ENABLED                true
#define MQTT_BROKER_HOST            ""      // IP address or host name.
#define MQTT_BROKER_PORT            1883u
#define MQTT_CLIENT_ID              "supercar-aai14"
#define MQTT_TOPIC_PREFIX           "inf2004/aai14/supercar"
#define MQTT_KEEPALIVE_S            10u     // Broker drops us after 1.5x.
#define MQTT_RETRY_MSEC             5000u   // Wait between attempts.
#define MQTT_TELEMETRY_MSEC         250u    // Live readings, 4 per second.
#define MQTT_HEARTBEAT_MSEC         2000u   // Heartbeat.

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
#define LEFT_MOTOR_REVERSED         false   // Flipped 2026-11-13: after the
#define RIGHT_MOTOR_REVERSED        false   // rewiring, WHEEL TEST spun both
                                            // wheels backwards.

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
// Barcode turns at a crossroad (TURN state) pivot harder: at 20% / 180 ms
// the car sometimes stalled mid-turn (2026-11-13). Calibration keeps the
// gentler settings above. Overshoots the line: lower the speed.
#define TURN_PULSE_SPEED            30      // Burst power in a turn.
#define TURN_PULSE_OFF_MSEC         120u    // Shorter pause in a turn.
// Climbing a hump: speed up gently from SPEED_CRUISE to SPEED_HUMP over
// HUMP_RAMP_MSEC. Was 23 at once (2026-11-19): on the 4-5 mm course hump
// that jerk read as 7-8 deg of tilt (the hump itself is ~3.4 deg) and
// doubled the measured height.
#define SPEED_HUMP                  12      // Most power on a hump.
#define HUMP_RAMP_MSEC              300u    // Cruise to SPEED_HUMP in this,
                                            // and back down after.
// Extra power is for the climb only: once the tilt has fallen this far
// below its peak, the car is over the top and eases back to cruise.
// Holding 12 down the far side sped the car up, then it jerked as it
// dropped to cruise on the flat (2026-11-19).
#define HUMP_TOP_DROP_X10           15      // 1.5 deg below the peak.
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
// TEMPORARY: stop at 25 cm (Felix's was 40 cm; 20 and 30 cm tried on
// 2026-11-17: 30 cm made the bypass too wide). The +/-30 deg scan then
// sees about +/-145 mm sideways. All three move together (gaps kept).
#define OBSTACLE_SLOW_MM            350u    // Slow before the stop.
#define OBSTACLE_STOP_MM            250u    // Brake at or below 25 cm.
#define OBSTACLE_HARD_STOP_MM       80u     // Wall: stop for certain.
#define OBSTACLE_CLEAR_MM           300u    // Clear again (hysteresis).
#define OBSTACLE_PROBE_MSEC         4000u   // Longest creep while probing.
#define OBSTACLE_CLEAR_MSEC         200u    // Clear this long to resume.

// Confirmed test setup; scan settings require tuning on the real sensor.
#define CAR_LENGTH_MM              160u
#define CAR_WIDTH_MM               130u
#define OBSTACLE_EXPECTED_WIDTH_MM  300u    // Width only; depth is UNKNOWN.
#define OBSTACLE_SCAN_COARSE_DEG    10
#define OBSTACLE_SCAN_FINE_DEG      2
#define OBSTACLE_SCAN_SAMPLES       3u
#define OBSTACLE_SCAN_EDGE_JUMP_MM  100u   // Minimum near-to-far depth jump;
                                        // also maximum far-streak depth span.
#define OBSTACLE_SCAN_EDGE_CONFIRM_RAYS 3u // Clearance safety only; NOT edge detection.
#define OBSTACLE_SCAN_STABLE_SPREAD_MM 40u // Echo agreement for clearance safety.
#define OBSTACLE_BYPASS_MARGIN_MM   20u    // Candidate path margin; validate on floor.
#define OBSTACLE_SCAN_TIMEOUT_MSEC  30000u

// ------------------------------------------------------------------------
// Obstacle bypass (bypass.c): go round the obstacle and back onto the
// same line, which runs straight on behind it. The way out is a smooth
// CURVE, steered by running the wheels at different speeds (both forward):
// heading 0 at first, turning more and more as the car nears the obstacle,
// reaching the side-step at its front face. The curve is recorded on a
// STACK as short moves (heading, mm); coming back pops them in reverse,
// mirrored, so the return starts steep and flattens out onto the line.
//   1 SCAN:   the full 60-120 deg scan picks the side (obstacle.c);
//   2 OUT:    the curve, pushed every BYPASS_SEGMENT_MM. Something closer
//             than the known face: re-scan only towards that side (90 -> 60
//             deg for left, 90 -> 120 for right) and plan a new curve;
//   3 PASS:   straighten up (pushed) and drive on, the sensor watching the
//             obstacle, until it has ended (the straight part: not pushed);
//   4 RETURN: pop each move, mirrored; the line found (line following on):
//             line up on it and clear the stack;
//   5 Stack empty: turn straight; back on the original path.
// Distances and heading come from the wheel encoders. STOP ends it at
// once; any step that cannot finish brakes the car.
// ------------------------------------------------------------------------
#define BYPASS_ENABLED              true    // false: stay BLOCKED (old way).
// Wheel encoders, measured 2026-11-17 rolling 300 mm along a straight
// edge: LEFT 3992, RIGHT 4113 edges.
#define ENC_LEFT_EDGES_PER_M        13307u
#define ENC_RIGHT_EDGES_PER_M       13710u
#define WHEEL_TRACK_MM              125u    // Between the wheel centres.
// The curve out: sideways offset grows as (distance travelled / distance
// to the face) squared, so the heading starts at 0 and steepens near it.
#define BYPASS_CURVE_END_PCT        90u     // The side-step is done this
                                            // far (%) to the face; then
                                            // straight past it.
// Too close to curve round (a bigger obstacle seen late): reverse
// straight, holding the heading, until this much is clear ahead (or
// BYPASS_BACKUP_MAX_MM reversed), then carry on.
#define BYPASS_BACKUP_TRIGGER_MM    150u    // Nearer than this: back up.
#define BYPASS_BACKUP_CLEAR_MM      200u    // Back up until this is clear,
#define BYPASS_BACKUP_MAX_MM        150u    // or this far at most.
#define BYPASS_BACKUP_MAX_COUNT     3u      // Back-ups per bypass at most.
// Coming back, the sensor keeps watching the obstacle's side; while it
// is still there the car holds straight instead of turning in. Once it
// has gone (BYPASS_SIDE_CLEAR_COUNT far echoes), this much further,
// then the turn back carries on.
#define BYPASS_RETURN_GUARD_EXTRA_MM 80u
#define BYPASS_SEGMENT_MM           20u     // One stack move per this.
#define BYPASS_STEER_KP_X10         5       // Wheel speed difference per
                                            // degree of heading error (x10).
#define BYPASS_STEER_MAX            5       // Largest wheel difference (%):
                                            // inner wheel never below
#define BYPASS_WHEEL_MIN            7       // this (was 5: stalled; the
                                            // motors stall at 6 or less),
                                            // so it keeps driving
                                            // forward (a stopped wheel is
                                            // dragged back, and the
                                            // encoders count that as
                                            // forward: wrong heading).
// Heading comes from LEFT minus RIGHT wheel travel. If the encoder or
// motor called "left" is really on the right, the car curves the wrong
// way round and never reaches the side-step: flip this.
#define BYPASS_ODO_SWAP_LR          false
#define BYPASS_POSE_LOG_MSEC        500u    // Log position this often.
#define BYPASS_LOOKAHEAD_MM         100u    // Steer back onto the curve
                                            // over about this distance.
#define BYPASS_PIVOT_SLIP_X100      100u    // Pivot counts x this / 100:
                                            // turns short -> raise it.
#define BYPASS_PIVOT_SPEED          17      // Steady pivot power (was 30:
                                            // too violent; stalls below
                                            // about 15; 20 -> 18 -> 17).
#define BYPASS_DRIVE_SPEED          12      // Straight-line power (14 ->
                                            // 13 -> 12; 12 - 5 keeps the
                                            // inner wheel at 7, above
                                            // stalling).
// 10 cm less each (2026-11-17, small test space).
#define BYPASS_SIDE_MARGIN_MM       30u     // Gap kept between the car's
                                            // side and the obstacle's edge
                                            // (clearance = half the car's
                                            // width + this).
#define BYPASS_BEAM_CORRECTION_DEG  4       // Move scanned edges inwards by
                                            // this (the beam is ~15 deg
                                            // wide: an 8.5 cm bottle scans
                                            // as 19.3 cm). Fitted to round
                                            // bottles at ~25 cm; 0 = off.
#define BYPASS_MIN_SHIFT_MM         50u     // Sideways shift: at least (was
                                            // 150; the turning clearance
                                            // already gives about 120),
#define BYPASS_MAX_SHIFT_MM         300u    // and at most (was 400).
#define BYPASS_ASSUMED_DEPTH_MM     50u     // Depth driven past before the
                                            // sideways sensor is trusted
                                            // (was 150).
#define BYPASS_PASS_MAX_MM          700u    // Give up passing after this
                                            // (was 800).
#define BYPASS_SIDE_CLEAR_COUNT     3u      // Far side echoes in a row =
                                            // obstacle ended.
#define BYPASS_STACK_MAX            64u     // Moves pushed at most
                                            // (20 mm each).
#define BYPASS_RESCAN_MARGIN_MM     40u     // Going out, an echo this much
                                            // closer than the known face:
                                            // something new, re-scan.
#define BYPASS_RESCAN_NEAR_MM       400u    // A re-scan echo counts as the
                                            // obstacle only within this.
#define BYPASS_RESCAN_MIN_MM        50u     // Drive at least this far on
                                            // each move first.
#define BYPASS_RESCAN_STEP_DEG      5       // Half-sweep step.
#define BYPASS_EDGE_JUMP_MM         100u    // Echo this much further (or
                                            // none) = the edge.
#define BYPASS_EDGE_TURN_DEG        20      // Edge past the half-sweep:
                                            // turn this much more, re-scan.
#define BYPASS_HEADING_MAX_DEG      75      // Never turn further off line.
#define BYPASS_EXPLORE_SCAN_LIMIT   true    // No side usable but the
                                            // obstacle runs past the sweep:
                                            // turn that way, look again.
#define BYPASS_LINE_ADVANCE_MM      40u     // Line found: bring the axle
                                            // over it before aligning.
#define BYPASS_ALIGN_MAX_DEG        80u     // Align pivot gives up after.
#define BYPASS_FRONT_STOP_MM        120u    // Something this close ahead
                                            // while driving out/in: stop.
#define BYPASS_STEP_MAX_MSEC        15000u  // Any one step taking longer
                                            // brakes the car (stuck).

// DISPLAY ONLY: provisional effective-beam fit to the supplied bottle scans.
// Re-measure on this sensor/target; never use this correction for clearance.
#ifndef OBSTACLE_WIDTH_CALIBRATION_ENABLED
#define OBSTACLE_WIDTH_CALIBRATION_ENABLED true
#endif
#ifndef OBSTACLE_WIDTH_BEAM_HALF_DEG_X10
#define OBSTACLE_WIDTH_BEAM_HALF_DEG_X10 98 // 9.8 degrees, effective half-angle.
#endif
#ifndef OBSTACLE_WIDTH_CAL_MIN_MM
#define OBSTACLE_WIDTH_CAL_MIN_MM    200u
#endif
#ifndef OBSTACLE_WIDTH_CAL_MAX_MM
#define OBSTACLE_WIDTH_CAL_MAX_MM    320u
#endif

#define US_PING_PERIOD_USEC         60000u  // Pings at least 60 ms apart.
#define US_ECHO_TIMEOUT_USEC        30000u  // About 5 m.
#define US_TRIGGER_USEC             10u
#define US_ECHO_MIN_USEC            100u    // Shorter echoes (under about
                                            // 2 cm) are glitches: ignored.
#define US_STALE_MSEC               200u    // Range older than this is
                                            // treated as unknown.

// LSM303DLHC pitch: the sensor's -Y is forward, Z up (imu.c flips it).
// Tilt is low-pass filtered; thresholds provide hysteresis against noise.
// A hump is TOTAL TILT, sqrt(pitch^2 + roll^2), either way. Measured on
// the course hump, car still (2026-11-19; caster at the front):
// - caster on top:          pitch +7.4, roll -0.1 deg (nose up);
// - one drive wheel on top: pitch -2.0, roll +4.5 deg (one rear corner up).
// Driving over it, the drive-wheel case is what shows: pitch -2..-5 and
// roll +4..+11 for 2-3 s. Still on the flat: within 0.4 deg. HUMP ignores
// the ultrasonic, so a false hump is unsafe: keep clear of the noise.
#define HUMP_ENTER_PITCH_X10        30      // 3.0 deg tilt: on a hump.
#define HUMP_LEVEL_PITCH_X10        15      // Within 1.5 deg: level again.
#define HUMP_ENTER_MSEC             300u    // Tilted this long: a hump. A
                                            // jolt tilts the reading for
                                            // less; the hump lasts 2-3 s.
// Drive axle to caster ball contact (measured 2026-11-19). The caster
// raised by h tips the car by asin(h / wheelbase); one drive wheel raised
// by h rolls it by asin(h / WHEEL_TRACK_MM). hump.c uses both.
#define HUMP_WHEELBASE_MM           85u
// A hump's height is the highest height HELD this long: one bounce (the
// wheel dropping off the edge) read up to twice the still height.
#define HUMP_STEADY_MSEC            200u
#define HUMP_SETTLE_MSEC            1000u   // No hump looked for this long
                                            // after setting off: pulling
                                            // away read as -3 deg for 258
                                            // ms on the flat (2026-11-19).
#define HUMP_LEVEL_MSEC             400u    // Level after tilt excursion: done.
#define HUMP_PASS_MM                200u    // Or driven this far on it
                                            // (rest of the hump + the
                                            // 85 mm wheelbase; 250 ran on
                                            // far past it): done, unless
                                            // still tilted. Speeding up
                                            // reads as tilt and could keep
                                            // it "not level" for ever.
#define HUMP_MIN_MSEC               500u    // Shortest possible hump.
#define HUMP_MAX_MSEC               8000u   // Longer than this: stop.
// Picked up: lifted, the three IR sensors (left, right, barcode) see no
// floor, which reads as black on all three at once. Measured 2026-11-19:
// all three black from the moment of lifting, in every hold (level,
// tipped forward, tipped sideways); on the floor never all three, except
// for ~0.35 s over a crossroad branch. Wheel speed (only ~35% up when
// lifted, less than rolling off a hump) and tilt (none in a level lift)
// are not used.
#define PICKUP_FLOORLESS_MSEC       800u    // All three black this long.

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
#define BARCODE_FIRST_MARK_MAX_MSEC 340u   // A scan whose FIRST black mark
                                            // lasts this long is dropped
                                            // (a branch, not a bar). Was
                                            // LINE_JUNCTION_MIN_MSEC; kept
                                            // separate so crossroads can
                                            // go lower (2026-11-13).
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
// TEMPORARY (2026-11-13): log every black/white change of the barcode
// sensor (Line / Barcode tab), to check its sensitivity. Set false after.
#define BARCODE_DEBUG_LOG           false
#define BARCODE_DEBUG_MAX_PER_SEC   10u     // Lines a second, then a count.
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

// Every single-letter barcode is a command, carried out at the next
// crossroad (user, 2026-11-18). One character per letter, A to Z:
//   L = turn left, R = turn right, S = go straight, U = U-turn.
// Fixed by the user: A = left, F = U-turn, Z = right. The rest cycle
// L R S U, with L, R, S and U standing for themselves. Anything longer
// than one letter is a plain message.
//                                   ABCDEFGHIJKLMNOPQRSTUVWXYZ
#define BARCODE_LETTER_COMMANDS     "LRSULUSULRSLLRSULRSUURSULR"

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
#define TURN_UTURN_SKIP_LINES       1u      // At a + crossroad: side
                                            // branches to pivot past.
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
#define LINE_JUNCTION_MIN_MSEC      150u    // ~5 mm at SPEED_SCAN 8 (was
                                            // 340, ~13 mm): the crossing
                                            // line can be thinner than a
                                            // big print's wide bar; bars
                                            // are no risk (see above).
#define LINE_JUNCTION_MAX_MSEC      1050u   // ~40 mm: longer black (the
                                            // mat past a paper edge) is
                                            // not a branch (2026-11-06:
                                            // false crossroads).
#define LINE_JUNCTION_GAP_MSEC      20u     // Shorter white inside a mark
                                            // (glare, scuff) is ignored.
#define JUNCTION_ADVANCE_MSEC       1180u   // VERIFY: about the line-sensor
                                            // to axle distance at
                                            // SPEED_SCAN (~32 mm/s).

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
#define WATCHDOG_MSEC               3000u   // Loop stuck this long: the
                                            // Pico restarts itself.
// Idle power saving (Wi-Fi doze with no page open, servo released while
// idle). OFF while the car runs from a USB power bank: a bank switches
// itself off when the draw is too low, and these made it do so sooner
// (2026-11-17). Turn on for a LiPo or AA pack, which do not auto-sleep.
#define IDLE_POWER_SAVING           false
#define SERVO_RELEASE_MSEC          2000u   // Idle and settled this long:
                                            // stop holding the servo.
#define WIFI_PM_IDLE_MSEC           15000u  // No page request this long:
                                            // Wi-Fi power saving on.
#define TICK_USEC                   10000u  // 10 ms control period.
#define TELEMETRY_EVERY_TICKS       5u      // Teleplot points every 50 ms.
#define BUTTON_DEBOUNCE_TICKS       3u      // 30 ms START debounce.
#define BUTTON_LONG_PRESS_TICKS     200u    // Hold START 2 s: calibrate.
#define BARCODE_BEEP_MSEC           80u     // Soft beep: barcode read.
#define BUZZER_SOFT_DUTY_PERCENT    5u     // Soft beep loudness (PWM duty;
                                            // 50 = normal beep). Lower is
                                            // quieter.
 
#endif /* CONFIG_H */

/*** end of file ***/

/** @file vehicle.c
 *
 * @brief Mission state machine: line following, obstacle stop, humps.
 *
 * @par Obstacle stop
 * With OBSTACLE_STOP_AT_ONCE, an echo from closer than OBSTACLE_STOP_MM
 * brakes the car at once (BLOCKED, resumes when clear); no PROBE is done.
 *
 * @par Telling a hump from a wall (OBSTACLE_STOP_AT_ONCE false)
 * The ultrasonic sensor alone cannot tell them apart: both look like
 * "something close ahead". So a close reading does not stop the car at
 * once; it enters PROBE and creeps forward while watching pitch:
 * - the nose tips up (pitch above HUMP_ENTER): it is a hump, so climb it
 *   (HUMP) and ignore the ultrasonic until the car is level again;
 * - the range keeps shrinking to OBSTACLE_HARD_STOP_MM, or the probe takes
 *   longer than OBSTACLE_PROBE_MSEC, with no pitch: it is a wall, so brake
 *   (BLOCKED) until the way is clear again.
 *
 * @par Line following
 * Both line sensors sit over the line when the car is centred:
 * - both on the line: go straight;
 * - only the left on the line: the line has moved left, so arc left
 *   (left wheel slower, right wheel faster, both still forward); only the
 *   right: arc right;
 * - neither, when both had been on the line together: the line is
 *   between the sensors, so keep going straight;
 * - once turning, keep turning that way until both sensors are on the
 *   line, so turns are completed;
 * - neither: reverse along the recorded path back to where both sensors
 *   were last on the line (LINE_BACKTRACK_ENABLE, once per loss), then
 *   follow on;
 *   if that does not find it, arc hard towards the side the line slid out
 *   past until a sensor finds it.
 * The car keeps its current movement until a change is confirmed, so a
 * dirty track (grey footprints, scratched tape) does not make it jitter:
 * each sensor follows the majority of its recent readings (see
 * line_filtered()), and each steering change is held for at least
 * LINE_STEER_HOLD_MSEC. It never pivots on
 * the spot and never stops for white; only an obstacle (PROBE/BLOCKED)
 * or STOP stops it.
 *
 * @par START calibration (LINE_CAL): search until the line is found
 * The car may start anywhere near the line. It repeats, until the line is
 * found or STOP is pressed:
 * 1. PIVOT: pivot one way (LINE_CAL_PIVOT_LEFT) in short bursts with
 *    braked pauses (see pivot()), until either sensor sees black, for up to
 *    LINE_CAL_PIVOT_MAX_MSEC.
 * 2. DRIVE: no line found, so drive straight ahead until either sensor
 *    sees black, then go back to PIVOT. An obstacle ahead also ends the
 *    drive (back to PIVOT, which turns away from it).
 * 3. ALIGN: pivot towards the sensor on black until both sensors are on
 *    black for LINE_CAL_ALIGN_TICKS, then follow the line. Overshooting
 *    (both on white) pivots back; failing within LINE_CAL_ALIGN_MAX_MSEC
 *    goes back to PIVOT.
 * Success factor: EACH sensor must have DETECTED black during LINE_CAL,
 * that is, switched from white to black. Merely sitting on black at the
 * start does not count, so a stuck sensor cannot pass: a car placed on
 * the line turns off it and back once. A sensor that still has not
 * detected black after LINE_CAL_ALIGN_TRIES failed aligns stops the car
 * with its name.
 *
 * @par Barcodes
 * While a barcode is being read (barcode_is_capturing()), FOLLOW_LINE
 * ignores the line sensors and drives straight at SPEED_SCAN, because bar
 * widths are measured in time; obstacles and humps are still checked. A
 * decoded COMMAND trumps the line sensors (TURN state):
 * 1. LEAVE: pivot until both line sensors are off the current line;
 * 2. FIND:  keep pivoting until a sensor finds a line. A left/right turn
 *    gives up after TURN_90_MSEC (about 90 degrees) and drives on; a
 *    U-turn finds the same line again behind it;
 * 3. ALIGN: keep pivoting until both sensors are on it, then follow.
 * Pressed with the car stopped (test keys), the car turns and stops
 * again, so TURN_90_MSEC can be tuned on the bench.
 *
 * @par BARCODE TEST
 * Drives straight at SPEED_SCAN, ignoring the line sensors, with barcode
 * reading on: decoded messages and commands are shown, but commands do
 * not turn the car. It runs until STOP, or stops by itself at an
 * obstacle closer than OBSTACLE_STOP_MM.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/stdlib.h"

#include "barcode.h"
#include "config.h"
#include "imu.h"
#include "line_sensor.h"
#include "log.h"
#include "motor.h"
#include "ultrasonic.h"
#include "vehicle.h"


#define USEC_PER_MSEC   1000u
#define PERCENT         100

// Line filter score limits, in control ticks.
#define MSEC_TO_TICKS(msec)  (((msec) * USEC_PER_MSEC) / TICK_USEC)
#define LINE_SCORE_FULL      MSEC_TO_TICKS(LINE_TURN_DELAY_MSEC)
#define LINE_SCORE_CONFIRM   MSEC_TO_TICKS(LINE_BLACK_CONFIRM_MSEC)
#define PATH_MAX_TICKS       MSEC_TO_TICKS(LINE_BACKTRACK_MAX_MSEC)


static vehicle_state_t g_state        = VEHICLE_READY;
static uint32_t        g_entered_msec = 0u;     // When g_state began.
static uint32_t        g_level_msec   = 0u;     // Level since (in HUMP).
static uint32_t        g_clear_msec   = 0u;     // Clear since (BLOCKED).
static uint32_t        g_ignore_until = 0u;     // Ignore range until.
static int32_t         g_hump_peak_tilt_x10 = 0;
static bool            gb_level       = false;
static bool            gb_clear       = false;
static bool            gb_hump_tilted = false;
static bool            gb_last_left   = true;   // Line last seen left.

// Which sensors last saw the line, so the car knows which limit it left by.
typedef enum
{
    LINE_LAST_BOTH,
    LINE_LAST_LEFT,
    LINE_LAST_RIGHT
} line_last_t;

static line_last_t     g_line_last    = LINE_LAST_BOTH;

// Debounced line state used for steering.
static line_state_t    g_line          = { true, true };
static uint32_t        g_left_score    = LINE_SCORE_FULL;  // Net black,
static uint32_t        g_right_score   = LINE_SCORE_FULL;  // in ticks.
static uint32_t        g_left_black_msec  = 0u; // Left turned black at.
static uint32_t        g_right_black_msec = 0u; // Right turned black at.

// Steering: a gentle arc while one sensor is still on the line, a hard
// arc once both are off it. Going straight or reversing the turn is held
// back for LINE_STEER_HOLD_MSEC after each change (see steer_allowed()).
typedef enum
{
    STEER_STRAIGHT,
    STEER_LEFT,
    STEER_RIGHT,
    STEER_HARD_LEFT,
    STEER_HARD_RIGHT
} steer_t;

static steer_t         g_steer         = STEER_STRAIGHT;
static uint32_t        g_steer_msec    = 0u;    // Steering last changed at.

// Backtrack: the wheel speeds driven since both sensors were last on the
// line, so a car that has lost it can reverse along the same path back.
static int8_t          g_path_left[PATH_MAX_TICKS];
static int8_t          g_path_right[PATH_MAX_TICKS];
static uint32_t        g_path_len      = 0u;
static bool            gb_path_ok      = false; // Path leads back to black.
static bool            gb_backtracking = false;
static bool            gb_backtracked  = false; // Used up for this loss.
static uint32_t        g_back_msec     = 0u;    // Reverse began at.
static uint32_t        g_back_pos_x100 = 0u;    // Replay position, ticks
                                                // x 100 (counts down).
static line_state_t    g_raw_line      = { false, false };

// START calibration: search phases.
typedef enum
{
    CAL_PIVOT,      // Pivot one way looking for the line.
    CAL_DRIVE,      // Drive straight ahead looking for the line.
    CAL_ALIGN       // Line found: pivot until both sensors are on black.
} cal_phase_t;

static cal_phase_t     g_cal_phase     = CAL_PIVOT;
static uint32_t        g_phase_msec    = 0u;    // Current phase began.
static uint32_t        g_search_round  = 0u;    // PIVOT phases so far.
static uint32_t        g_centred_ticks = 0u;
static uint32_t        g_align_fails   = 0u;
static bool            gb_left_detect  = false; // Left went white -> black.
static bool            gb_right_detect = false; // Right went white -> black.
static bool            gb_left_was     = false; // Left black last tick.
static bool            gb_right_was    = false; // Right black last tick.
static bool            gb_pivoting     = false;
static bool            gb_pivot_left   = true;
static uint32_t        g_pivot_msec    = 0u;    // Pivot began/reversed.

// Barcode reading and barcode turns.
typedef enum
{
    TURN_LEAVE,     // Pivot until both line sensors are off the line.
    TURN_FIND,      // Pivot until a line sensor finds a line.
    TURN_ALIGN      // Pivot until both line sensors are on it.
} turn_phase_t;

static bool              gb_launch      = true;  // Straight start allowed.
static bool              gb_scanning    = false; // Driving straight to read.
static uint32_t          g_scan_msec    = 0u;    // Reading began at.
static barcode_command_t g_turn_cmd     = BARCODE_CMD_NONE;
static vehicle_state_t   g_turn_after   = VEHICLE_FOLLOW_LINE;
static turn_phase_t      g_turn_phase   = TURN_LEAVE;
static uint32_t          g_turn_msec    = 0u;    // Current phase began.
static bool              gb_turn_left_on = true; // Left was last on black.


static void     state_enter(vehicle_state_t state, uint32_t now_msec);
static void     turn_step(uint32_t now_msec);
static void     turn_phase_enter(turn_phase_t phase, uint32_t now_msec);
static void     turn_finish(uint32_t now_msec, line_state_t line,
                            line_last_t last, char const * p_how);
static bool     scan_is_on(uint32_t now_msec);
static void     line_steer(int32_t speed, uint32_t now_msec);
static void     line_cal_step(uint32_t now_msec);
static void     cal_phase_enter(cal_phase_t phase, uint32_t now_msec);
static void     line_align_step(uint32_t now_msec, line_state_t line);
static void     pivot(bool b_left, uint32_t now_msec);
static line_state_t line_filtered(uint32_t now_msec);
static bool     line_score_step(uint32_t * p_score, bool b_raw_black,
                                bool b_black);
static bool     line_trusted(bool b_black, uint32_t black_msec,
                             uint32_t now_msec);
static void     line_filter_reset(line_state_t line, uint32_t now_msec);
static int32_t  steer_side(steer_t steer);
static bool     backtrack_step(line_state_t line, uint32_t now_msec);
static void     path_record(int32_t left, int32_t right, line_state_t line);
static void     path_reset(bool b_ok);
static bool     steer_allowed(steer_t want, uint32_t now_msec);
static uint32_t now_msec_get(void);
static void     follow_line_step(uint32_t now_msec, uint32_t range_mm,
                                 int32_t pitch_x10);
static void     probe_step(uint32_t now_msec, uint32_t range_mm,
                           int32_t pitch_x10);
static void     hump_step(uint32_t now_msec, int32_t pitch_x10);
static void     blocked_step(uint32_t now_msec, uint32_t range_mm);
static void     barcode_test_step(uint32_t now_msec, uint32_t range_mm);


/*!
 * @brief Start in READY with the motors braked.
 */
void
vehicle_init (void)
{
    state_enter(VEHICLE_READY, now_msec_get());
}   /* vehicle_init() */


/*!
 * @brief Start: search for the line (LINE_CAL), then follow it.
 *
 * @par
 * Works from READY or STOPPED only. The car may be placed anywhere near
 * the line.
 */
void
vehicle_start (void)
{
    if ((VEHICLE_READY == g_state) || (VEHICLE_STOPPED == g_state))
    {
        state_enter(VEHICLE_LINE_CAL, now_msec_get());
    }
}   /* vehicle_start() */


/*!
 * @brief Stop at once, from any state. START is needed to move again.
 */
void
vehicle_stop (void)
{
    state_enter(VEHICLE_STOPPED, now_msec_get());
}   /* vehicle_stop() */


/*!
 * @brief Act on a barcode command; it trumps the line sensors.
 *
 * @par
 * While following, the car turns and then follows the line it finds.
 * When stopped (a TEST button), it turns and stops again, to tune
 * TURN_90_MSEC. In any other state the command is ignored and logged.
 *
 * @param[in] command The command; BARCODE_CMD_NONE does nothing.
 */
void
vehicle_barcode_command (barcode_command_t command)
{
    uint32_t now_msec = now_msec_get();

    if (BARCODE_CMD_NONE == command)
    {
        // A plain message: keep following.
    }
    else if (VEHICLE_FOLLOW_LINE == g_state)
    {
        g_turn_cmd   = command;
        g_turn_after = VEHICLE_FOLLOW_LINE;
        state_enter(VEHICLE_TURN, now_msec);
    }
    else if ((VEHICLE_READY == g_state) || (VEHICLE_STOPPED == g_state))
    {
        g_turn_cmd   = command;
        g_turn_after = VEHICLE_STOPPED;
        state_enter(VEHICLE_TURN, now_msec);
    }
    else
    {
        log_printf("%s ignored in %s.", barcode_command_name(command),
                   vehicle_state_name(g_state));
    }
}   /* vehicle_barcode_command() */


/*!
 * @brief Start BARCODE TEST: drive straight and read barcodes.
 *
 * @par
 * Works from READY or STOPPED only. STOP ends it.
 */
void
vehicle_barcode_test (void)
{
    if ((VEHICLE_READY == g_state) || (VEHICLE_STOPPED == g_state))
    {
        log_printf("BARCODE TEST: driving straight, line sensors ignored. "
                   "STOP to end.");
        state_enter(VEHICLE_BARCODE_TEST, now_msec_get());
    }
    else
    {
        log_printf("Stop the car before BARCODE TEST.");
    }
}   /* vehicle_barcode_test() */


/*!
 * @brief Run one step of the state machine. Call every control tick,
 *        after the sensor tasks.
 */
void
vehicle_task (void)
{
    uint32_t now_msec  = now_msec_get();
    uint32_t range_mm  = ultrasonic_range_mm();
    int32_t  tilt_x10  = imu_max_tilt_x10();

    switch (g_state)
    {
    case VEHICLE_LINE_CAL:
        line_cal_step(now_msec);
    break;

    case VEHICLE_FOLLOW_LINE:
        follow_line_step(now_msec, range_mm, tilt_x10);
    break;

    case VEHICLE_PROBE:
        probe_step(now_msec, range_mm, tilt_x10);
    break;

    case VEHICLE_HUMP:
        hump_step(now_msec, tilt_x10);
    break;

    case VEHICLE_BLOCKED:
        blocked_step(now_msec, range_mm);
    break;

    case VEHICLE_TURN:
        turn_step(now_msec);
    break;

    case VEHICLE_BARCODE_TEST:
        barcode_test_step(now_msec, range_mm);
    break;

    case VEHICLE_READY:
    case VEHICLE_STOPPED:
    default:
        motor_set(0, 0);
    break;
    }
}   /* vehicle_task() */


/*!
 * @brief Report the current state.
 *
 * @return The state.
 */
vehicle_state_t
vehicle_state (void)
{
    return (g_state);
}   /* vehicle_state() */


/*!
 * @brief Name a state for telemetry.
 *
 * @param[in] state The state.
 *
 * @return A constant string.
 */
char const *
vehicle_state_name (vehicle_state_t state)
{
    char const * p_name = "UNKNOWN";

    switch (state)
    {
    case VEHICLE_READY:
        p_name = "READY";
    break;

    case VEHICLE_FOLLOW_LINE:
        p_name = "FOLLOW_LINE";
    break;

    case VEHICLE_PROBE:
        p_name = "PROBE";
    break;

    case VEHICLE_HUMP:
        p_name = "HUMP";
    break;

    case VEHICLE_BLOCKED:
        p_name = "BLOCKED";
    break;

    case VEHICLE_STOPPED:
        p_name = "STOPPED";
    break;

    case VEHICLE_LINE_CAL:
        p_name = "LINE_CAL";
    break;

    case VEHICLE_TURN:
        p_name = "TURN";
    break;

    case VEHICLE_BARCODE_TEST:
        p_name = "BARCODE_TEST";
    break;

    default:
        p_name = "UNKNOWN";
    break;
    }

    return (p_name);
}   /* vehicle_state_name() */


/*!
 * @brief Report the maximum positive pitch seen during the latest hump.
 */
int32_t
vehicle_hump_peak_tilt_x10 (void)
{
    return (g_hump_peak_tilt_x10);
}   /* vehicle_hump_peak_tilt_x10() */


/*!
 * @brief FOLLOW_LINE: steer along the line; react to humps and obstacles.
 *
 * @param[in] now_msec  The current time.
 * @param[in] range_mm  The forward range.
 * @param[in] tilt_x10 The larger-magnitude pitch or roll, in tenths degree.
 */
static void
follow_line_step (uint32_t now_msec, uint32_t range_mm, int32_t tilt_x10)
{
    bool b_range_used = ((int32_t) (now_msec - g_ignore_until) >= 0);

    if ((tilt_x10 >= HUMP_ENTER_PITCH_X10) ||
        (tilt_x10 <= -HUMP_ENTER_PITCH_X10))
    {
        state_enter(VEHICLE_HUMP, now_msec);
    }
    else if (b_range_used && (range_mm <= OBSTACLE_STOP_MM))
    {
        if (OBSTACLE_STOP_AT_ONCE)
        {
            log_printf("Obstacle at %" PRIu32 " mm -> stopped.", range_mm);
            state_enter(VEHICLE_BLOCKED, now_msec);
        }
        else
        {
            state_enter(VEHICLE_PROBE, now_msec);
        }
    }
    else if (scan_is_on(now_msec))
    {
        motor_set(SPEED_SCAN, SPEED_SCAN);  // Reading a barcode: steady.
        path_reset(false);                  // Not steering: no way back.
    }
    else if (b_range_used && (range_mm <= OBSTACLE_SLOW_MM))
    {
        line_steer(SPEED_CREEP, now_msec);
    }
    else
    {
        line_steer(SPEED_CRUISE, now_msec);
    }
}   /* follow_line_step() */


/*!
 * @brief Decide whether to drive straight for a barcode being read.
 *
 * @par
 * True from the first bar until the scan ends, but for no longer than
 * BARCODE_SCAN_MAX_MSEC, so a stray black patch cannot keep the car off
 * the line for long.
 *
 * @param[in] now_msec The current time.
 *
 * @return True to drive straight at SPEED_SCAN this tick.
 */
static bool
scan_is_on (uint32_t now_msec)
{
    bool b_on = false;

    if (!barcode_is_capturing())
    {
        gb_scanning = false;
    }
    else if (!gb_scanning)
    {
        gb_scanning = true;
        g_scan_msec = now_msec;
        b_on        = true;
    }
    else
    {
        b_on = ((now_msec - g_scan_msec) < BARCODE_SCAN_MAX_MSEC);
    }

    return (b_on);
}   /* scan_is_on() */


/*!
 * @brief TURN: carry out a barcode command, then follow the line found.
 *
 * @param[in] now_msec The current time.
 */
static void
turn_step (uint32_t now_msec)
{
    line_state_t line     = line_sensor_read();
    bool         b_black  = (line.b_left_black || line.b_right_black);
    bool         b_uturn  = (BARCODE_CMD_UTURN == g_turn_cmd);
    bool         b_left   = (b_uturn ? TURN_UTURN_LEFT
                                     : (BARCODE_CMD_LEFT == g_turn_cmd));
    uint32_t     max_msec = (b_uturn ? TURN_UTURN_MAX_MSEC : TURN_90_MSEC);
    uint32_t     in_turn  = now_msec - g_entered_msec;
    line_state_t no_line  = { false, false };

    if ((TURN_ALIGN != g_turn_phase) && (in_turn > max_msec))
    {
        turn_finish(now_msec, no_line, LINE_LAST_BOTH,
                    (b_uturn ? "no line found, driving on"
                             : "about 90 deg, no line, driving on"));
    }
    else if (TURN_LEAVE == g_turn_phase)
    {
        if (!b_black)
        {
            turn_phase_enter(TURN_FIND, now_msec);
        }
        pivot(b_left, now_msec);
    }
    else if (TURN_FIND == g_turn_phase)
    {
        if (b_black)
        {
            gb_turn_left_on = line.b_left_black;
            turn_phase_enter(TURN_ALIGN, now_msec);
        }
        pivot(b_left, now_msec);
    }
    else if (line.b_left_black && line.b_right_black)
    {
        turn_finish(now_msec, line, LINE_LAST_BOTH, "on the line");
    }
    else if (!b_black)
    {
        // Pivoted past the line: it is on the side last seen.
        turn_finish(now_msec, line,
                    (gb_turn_left_on ? LINE_LAST_LEFT : LINE_LAST_RIGHT),
                    "line found");
    }
    else if ((now_msec - g_turn_msec) > TURN_ALIGN_MAX_MSEC)
    {
        turn_finish(now_msec, line,
                    (line.b_left_black ? LINE_LAST_LEFT : LINE_LAST_RIGHT),
                    "line found");
    }
    else
    {
        gb_turn_left_on = line.b_left_black;
        pivot(b_left, now_msec);
    }
}   /* turn_step() */


/*!
 * @brief Start a TURN phase.
 *
 * @param[in] phase    The new phase.
 * @param[in] now_msec The current time.
 */
static void
turn_phase_enter (turn_phase_t phase, uint32_t now_msec)
{
    g_turn_phase = phase;
    g_turn_msec  = now_msec;
}   /* turn_phase_enter() */


/*!
 * @brief End a barcode turn: hand the line state to the steering and
 *        follow on (or stop again, after a TEST turn).
 *
 * @param[in] now_msec The current time.
 * @param[in] line     The line state to steer from.
 * @param[in] last     Where the line was last seen.
 * @param[in] p_how    How the turn ended, for the log.
 */
static void
turn_finish (uint32_t now_msec, line_state_t line, line_last_t last,
             char const * p_how)
{
    log_printf("%s done (%s, %" PRIu32 " ms).",
               barcode_command_name(g_turn_cmd), p_how,
               (now_msec - g_entered_msec));

    line_filter_reset(line, now_msec);
    g_line_last    = last;
    gb_last_left   = (LINE_LAST_LEFT == last);
    state_enter(g_turn_after, now_msec);
}   /* turn_finish() */


/*!
 * @brief PROBE: creep towards the object and decide hump or wall.
 *
 * @param[in] now_msec  The current time.
 * @param[in] range_mm  The forward range.
 * @param[in] tilt_x10 The larger-magnitude pitch or roll, in tenths degree.
 */
static void
probe_step (uint32_t now_msec, uint32_t range_mm, int32_t tilt_x10)
{
    if ((tilt_x10 >= HUMP_ENTER_PITCH_X10) ||
        (tilt_x10 <= -HUMP_ENTER_PITCH_X10))
    {
        log_printf("Probe: nose tipped up -> HUMP, climbing.\n");
        state_enter(VEHICLE_HUMP, now_msec);
    }
    else if (range_mm > OBSTACLE_CLEAR_MM)
    {
        state_enter(VEHICLE_FOLLOW_LINE, now_msec);
    }
    else if ((range_mm <= OBSTACLE_HARD_STOP_MM) ||
             ((now_msec - g_entered_msec) > OBSTACLE_PROBE_MSEC))
    {
        log_printf("Probe: no climb -> WALL, stopped at %" PRIu32 " mm.\n",
               range_mm);
        state_enter(VEHICLE_BLOCKED, now_msec);
    }
    else
    {
        line_steer(SPEED_CREEP, now_msec);
    }
}   /* probe_step() */


/*!
 * @brief HUMP: climb and descend, ignoring the ultrasonic.
 *
 * @param[in] now_msec  The current time.
 * @param[in] tilt_x10 The larger-magnitude pitch or roll, in tenths degree.
 */
static void
hump_step (uint32_t now_msec, int32_t tilt_x10)
{
    int32_t  magnitude = ((tilt_x10 < 0) ? -tilt_x10 : tilt_x10);
    uint32_t in_hump   = now_msec - g_entered_msec;

    // A single raised wheel may produce roll rather than pitch. Track an
    // excursion on either axis, then finish only after both return near base.
    if (magnitude >= HUMP_ENTER_PITCH_X10)
    {
        gb_hump_tilted = true;
        if (magnitude > g_hump_peak_tilt_x10)
        {
            g_hump_peak_tilt_x10 = magnitude;
        }
    }

    if (!gb_hump_tilted || (magnitude > HUMP_LEVEL_PITCH_X10))
    {
        gb_level = false;
    }
    else if (!gb_level)
    {
        gb_level     = true;
        g_level_msec = now_msec;
    }
    else
    {
        // Still level.
    }

    if (gb_hump_tilted && gb_level && (in_hump > HUMP_MIN_MSEC) &&
        ((now_msec - g_level_msec) > HUMP_LEVEL_MSEC))
    {
        log_printf("Hump cleared after %" PRIu32 " ms.\n", in_hump);
        g_ignore_until = now_msec + HUMP_IGNORE_RANGE_MSEC;
        state_enter(VEHICLE_FOLLOW_LINE, now_msec);
    }
    else if (in_hump > HUMP_MAX_MSEC)
    {
        log_printf("Hump took too long (stalled?) -> BLOCKED.\n");
        state_enter(VEHICLE_BLOCKED, now_msec);
    }
    else
    {
        line_steer(SPEED_HUMP, now_msec);
    }
}   /* hump_step() */


/*!
 * @brief BLOCKED: stay braked until the way is clear for a while.
 *
 * @param[in] now_msec The current time.
 * @param[in] range_mm The forward range.
 */
static void
blocked_step (uint32_t now_msec, uint32_t range_mm)
{
    motor_set(0, 0);

    if (range_mm <= OBSTACLE_CLEAR_MM)
    {
        gb_clear = false;
    }
    else if (!gb_clear)
    {
        gb_clear     = true;
        g_clear_msec = now_msec;
    }
    else if ((now_msec - g_clear_msec) > OBSTACLE_CLEAR_MSEC)
    {
        log_printf("Obstacle cleared -> resuming.\n");
        state_enter(VEHICLE_FOLLOW_LINE, now_msec);
    }
    else
    {
        // Clear, but not for long enough yet.
    }
}   /* blocked_step() */


/*!
 * @brief BARCODE TEST: drive straight; stop at an obstacle.
 *
 * @param[in] now_msec The current time.
 * @param[in] range_mm The forward range.
 */
static void
barcode_test_step (uint32_t now_msec, uint32_t range_mm)
{
    if (range_mm <= OBSTACLE_STOP_MM)
    {
        log_printf("BARCODE TEST: obstacle at %" PRIu32 " mm -> stopped.",
                   range_mm);
        state_enter(VEHICLE_STOPPED, now_msec);
    }
    else
    {
        motor_set(SPEED_SCAN, SPEED_SCAN);
    }
}   /* barcode_test_step() */


/*!
 * @brief Change state, set up the new state's timers and barcode reading.
 *
 * @param[in] state    The new state.
 * @param[in] now_msec The current time.
 */
static void
state_enter (vehicle_state_t state, uint32_t now_msec)
{
    if (state != g_state)
    {
        log_printf("State: %s -> %s", vehicle_state_name(g_state),
                   vehicle_state_name(state));
    }

    // After a barcode turn the car is already lined up: no straight start.
    gb_launch      = (VEHICLE_TURN != g_state);
    g_state        = state;
    g_entered_msec = now_msec;
    path_reset(false);                  // Valid again once on the line.
    gb_level       = false;
    gb_clear       = false;
    if (VEHICLE_HUMP == state)
    {
        g_hump_peak_tilt_x10 = 0;
        gb_hump_tilted = false;
    }

    if (VEHICLE_LINE_CAL == state)
    {
        line_state_t start = line_sensor_read();

        g_search_round  = 0u;
        g_align_fails   = 0u;
        gb_left_detect  = false;
        gb_right_detect = false;
        gb_left_was     = start.b_left_black;   // Black at the start does
        gb_right_was    = start.b_right_black;  // not count as detected.
        cal_phase_enter(CAL_PIVOT, now_msec);
    }

    // Barcodes are only read while following the line normally, or in
    // BARCODE TEST.
    barcode_set_enabled((VEHICLE_FOLLOW_LINE == state) ||
                        (VEHICLE_BARCODE_TEST == state));

    if ((VEHICLE_FOLLOW_LINE != state) && (VEHICLE_PROBE != state) &&
        (VEHICLE_HUMP != state) && (VEHICLE_LINE_CAL != state))
    {
        motor_set(0, 0);
    }
}   /* state_enter() */


/*!
 * @brief LINE_CAL: search for the line until it is found, then align.
 *
 * @param[in] now_msec The current time.
 */
static void
line_cal_step (uint32_t now_msec)
{
    line_state_t line     = line_sensor_read();
    bool         b_black  = (line.b_left_black || line.b_right_black);
    uint32_t     in_phase = now_msec - g_phase_msec;

    // A detection is a change from white to black, so a stuck sensor (or
    // one that started on black and never moved) has not detected black.
    if (line.b_left_black && (!gb_left_was) && (!gb_left_detect))
    {
        gb_left_detect = true;
        log_printf("LINE_CAL: LEFT sensor detected black.");
    }
    if (line.b_right_black && (!gb_right_was) && (!gb_right_detect))
    {
        gb_right_detect = true;
        log_printf("LINE_CAL: RIGHT sensor detected black.");
    }
    gb_left_was  = line.b_left_black;
    gb_right_was = line.b_right_black;

    switch (g_cal_phase)
    {
    case CAL_PIVOT:
        if (b_black)
        {
            log_printf("LINE_CAL: line found (%s sensor on black) -> "
                       "aligning.", (line.b_left_black ? "LEFT" : "RIGHT"));
            cal_phase_enter(CAL_ALIGN, now_msec);
        }
        else if (in_phase > LINE_CAL_PIVOT_MAX_MSEC)
        {
            log_printf("LINE_CAL: no line after pivoting -> driving forward "
                       "until a sensor sees black.");
            cal_phase_enter(CAL_DRIVE, now_msec);
        }
        else
        {
            pivot(LINE_CAL_PIVOT_LEFT, now_msec);
        }
    break;

    case CAL_DRIVE:
        if (b_black)
        {
            log_printf("LINE_CAL: black seen while driving -> searching "
                       "again from here.");
            cal_phase_enter(CAL_PIVOT, now_msec);
        }
        else if (ultrasonic_range_mm() <= OBSTACLE_STOP_MM)
        {
            log_printf("LINE_CAL: obstacle ahead while driving -> pivoting "
                       "instead.");
            cal_phase_enter(CAL_PIVOT, now_msec);
        }
        else
        {
            motor_set(LINE_CAL_DRIVE_SPEED, LINE_CAL_DRIVE_SPEED);
        }
    break;

    case CAL_ALIGN:
        line_align_step(now_msec, line);
    break;

    default:
        cal_phase_enter(CAL_PIVOT, now_msec);
    break;
    }
}   /* line_cal_step() */


/*!
 * @brief Start a LINE_CAL phase: brake, restart the pivot kick, log it.
 *
 * @param[in] phase    The new phase.
 * @param[in] now_msec The current time.
 */
static void
cal_phase_enter (cal_phase_t phase, uint32_t now_msec)
{
    motor_set(0, 0);
    gb_pivoting     = false;    // The next pivot starts with a kick.
    g_cal_phase     = phase;
    g_phase_msec    = now_msec;
    g_centred_ticks = 0u;

    if (CAL_PIVOT == phase)
    {
        g_search_round++;
        log_printf("LINE_CAL search %" PRIu32 ": pivoting %s until a sensor "
                   "sees black...", g_search_round,
                   (LINE_CAL_PIVOT_LEFT ? "left" : "right"));
    }
}   /* cal_phase_enter() */


/*!
 * @brief ALIGN: pivot until both sensors are on black, then follow.
 *
 * @param[in] now_msec The current time.
 * @param[in] line     The current line state.
 */
static void
line_align_step (uint32_t now_msec, line_state_t line)
{
    bool b_both_detected = (gb_left_detect && gb_right_detect);

    if ((now_msec - g_phase_msec) > LINE_CAL_ALIGN_MAX_MSEC)
    {
        g_align_fails++;

        if (g_align_fails >= LINE_CAL_ALIGN_TRIES)
        {
            log_printf("LINE_CAL FAILED: LEFT %s, RIGHT %s. Check it in IR "
                       "DIAG.",
                       (gb_left_detect ? "detected black"
                                       : "NEVER detected black"),
                       (gb_right_detect ? "detected black"
                                        : "NEVER detected black"));
            state_enter(VEHICLE_STOPPED, now_msec);
        }
        else
        {
            log_printf("LINE_CAL: could not align on the line -> searching "
                       "again.");
            cal_phase_enter(CAL_PIVOT, now_msec);
        }
    }
    else if (line.b_left_black && line.b_right_black && b_both_detected)
    {
        motor_set(0, 0);
        gb_pivoting = false;
        g_centred_ticks++;

        if (g_centred_ticks >= LINE_CAL_ALIGN_TICKS)
        {
            log_printf("LINE_CAL OK: both sensors detected black and are on "
                       "the line -> following.");
            line_filter_reset(line, now_msec);
            g_line_last          = LINE_LAST_BOTH;
            state_enter(VEHICLE_FOLLOW_LINE, now_msec);
        }
    }
    else if (line.b_left_black && line.b_right_black)
    {
        // On the line, but a sensor has not yet detected black (it may
        // have started on it): turn off the line so it can prove it.
        g_centred_ticks = 0u;
        pivot(LINE_CAL_PIVOT_LEFT, now_msec);
    }
    else if (line.b_left_black)
    {
        g_centred_ticks = 0u;
        pivot(true, now_msec);              // Line is left: pivot left.
    }
    else if (line.b_right_black)
    {
        g_centred_ticks = 0u;
        pivot(false, now_msec);             // Line is right: pivot right.
    }
    else
    {
        // Both on white: the pivot overshot the line, so turn back.
        g_centred_ticks = 0u;
        pivot(!LINE_CAL_PIVOT_LEFT, now_msec);
    }
}   /* line_align_step() */


/*!
 * @brief Pivot on the spot in pulses: PIVOT_PULSE_ON_MSEC at
 *        PIVOT_PULSE_SPEED, then PIVOT_PULSE_OFF_MSEC braked, repeating.
 *
 * @par
 * The motors stall at the low steady power a slow pivot would need, so
 * each burst uses a power that really turns the wheels and the braked
 * pauses keep the average turning speed slow. A new pivot, or a change
 * of direction, always starts with a burst.
 *
 * @param[in] b_left   True to pivot left (left wheel back, right forward).
 * @param[in] now_msec The current time.
 */
static void
pivot (bool b_left, uint32_t now_msec)
{
    uint32_t period     = PIVOT_PULSE_ON_MSEC + PIVOT_PULSE_OFF_MSEC;
    bool     b_bursting = false;

    if ((!gb_pivoting) || (b_left != gb_pivot_left))
    {
        gb_pivoting   = true;
        gb_pivot_left = b_left;
        g_pivot_msec  = now_msec;
    }

    b_bursting = (((now_msec - g_pivot_msec) % period) < PIVOT_PULSE_ON_MSEC);

    if (!b_bursting)
    {
        motor_set(0, 0);    // Braked pause between bursts.
    }
    else if (b_left)
    {
        motor_set(-PIVOT_PULSE_SPEED, PIVOT_PULSE_SPEED);
    }
    else
    {
        motor_set(PIVOT_PULSE_SPEED, -PIVOT_PULSE_SPEED);
    }
}   /* pivot() */


/*!
 * @brief Steer along the line at a given speed, never stopping.
 *
 * @par
 * Turns keep both wheels rolling forward, so the car never pauses or
 * pivots to turn. While one sensor is still on the line the car arcs
 * gently (TURN_OUTER/INNER_PERCENT); once both are off it, it arcs hard
 * (TURN_HARD_OUTER/INNER_PERCENT) to get back on the line quickly.
 *
 * @param[in] speed    The base (straight-line) speed, in percent.
 * @param[in] now_msec The current time.
 */
static void
line_steer (int32_t speed, uint32_t now_msec)
{
    line_state_t line     = line_filtered(now_msec);
    bool         b_left_ok  = line_trusted(line.b_left_black,
                                           g_left_black_msec, now_msec);
    bool         b_right_ok = line_trusted(line.b_right_black,
                                           g_right_black_msec, now_msec);
    int32_t      outer    = (speed * TURN_OUTER_PERCENT) / PERCENT;
    int32_t      inner    = (speed * TURN_INNER_PERCENT) / PERCENT;
    int32_t      hard_out = (speed * TURN_HARD_OUTER_PERCENT) / PERCENT;
    int32_t      hard_in  = (speed * TURN_HARD_INNER_PERCENT) / PERCENT;
    steer_t      want     = STEER_STRAIGHT;
    bool         b_launch = ((VEHICLE_FOLLOW_LINE == g_state) &&
                             ((now_msec - g_entered_msec) <
                              LINE_START_STRAIGHT_MSEC));
    int32_t      left     = speed;
    int32_t      right    = speed;

    // Both sensors lost the line: reverse along the recorded path to where
    // both were last on it (once per loss), then follow on from there.
    if (LINE_BACKTRACK_ENABLE && !gb_backtracking && !gb_backtracked &&
        gb_path_ok && (g_path_len > 0u) && !b_launch &&
        (VEHICLE_FOLLOW_LINE == g_state) &&
        !line.b_left_black && !line.b_right_black)
    {
        log_printf("Line lost: reversing %" PRIu32 " ms of path to where "
                   "both sensors were on it.",
                   (g_path_len * TICK_USEC) / USEC_PER_MSEC);
        gb_backtracking = true;
        gb_backtracked  = true;
        g_back_msec     = now_msec;
        g_back_pos_x100 = g_path_len * (uint32_t) PERCENT;
    }

    if (gb_backtracking && backtrack_step(line, now_msec))
    {
        return;
    }

    // Dark patches: the line is continuous, so new black on the side AWAY
    // from where it has been tracked is suspect until it has lasted
    // LINE_NEW_BLACK_TRUST_MSEC. A suspect sensor is neither followed nor
    // fled from: the car goes straight (or keeps hunting, if it had
    // already lost the line), so a real line there is still reached.
    bool b_suspect = ((line.b_right_black && !line.b_left_black &&
                       !b_right_ok && (LINE_LAST_LEFT == g_line_last)) ||
                      (line.b_left_black && !line.b_right_black &&
                       !b_left_ok && (LINE_LAST_RIGHT == g_line_last)));
    bool b_hunting = ((STEER_HARD_LEFT == g_steer) ||
                      (STEER_HARD_RIGHT == g_steer));

    if (b_launch)
    {
        // Just started: get moving first.
    }
    else if (b_suspect && b_hunting)
    {
        want = g_steer;                 // Keep turning back to the line.
    }
    else if (b_suspect)
    {
        // Straight on until the new black proves itself or goes away.
    }
    else if (line.b_left_black && line.b_right_black)
    {
        // Centred: straight on. Until BOTH blacks are trusted, keep
        // remembering the side the line was tracked on, in case the new
        // one is a patch.
        if (b_left_ok && b_right_ok)
        {
            g_line_last = LINE_LAST_BOTH;
        }
    }
    else if (line.b_left_black)
    {
        gb_last_left = true;
        g_line_last  = LINE_LAST_LEFT;
        want = STEER_LEFT;              // Line moved left: arc left.
    }
    else if (line.b_right_black)
    {
        gb_last_left = false;
        g_line_last  = LINE_LAST_RIGHT;
        want = STEER_RIGHT;             // Line moved right: arc right.
    }
    else if (LINE_LAST_LEFT == g_line_last)
    {
        // Neither sensor on the line: turn hard towards the limit the
        // line was last seen at. Never stop, never pivot.
        want = STEER_HARD_LEFT;         // Line slid out left.
    }
    else if (LINE_LAST_RIGHT == g_line_last)
    {
        want = STEER_HARD_RIGHT;        // Line slid out right.
    }
    else
    {
        // Line is between the sensors.
    }

    // Finish turns: once turning, keep turning that way until BOTH
    // sensors are on the line (a lost line is handled by the reverse).
    if (!b_launch && (0 != steer_side(g_steer)) &&
        (steer_side(want) != steer_side(g_steer)) &&
        !(line.b_left_black && line.b_right_black))
    {
        want = g_steer;
    }

    if ((want != g_steer) && (b_launch || steer_allowed(want, now_msec)))
    {
        g_steer      = want;
        g_steer_msec = now_msec;
    }

    // Speed 0 brakes a wheel, so an inner wheel never drops to 0.
    if (speed > 0)
    {
        inner   = ((inner < 1) ? 1 : inner);
        hard_in = ((hard_in < 1) ? 1 : hard_in);
    }

    if (STEER_LEFT == g_steer)
    {
        left  = inner;
        right = outer;
    }
    else if (STEER_RIGHT == g_steer)
    {
        left  = outer;
        right = inner;
    }
    else if (STEER_HARD_LEFT == g_steer)
    {
        left  = hard_in;
        right = hard_out;
    }
    else if (STEER_HARD_RIGHT == g_steer)
    {
        left  = hard_out;
        right = hard_in;
    }
    else
    {
        // Straight: both at the base speed.
    }

    motor_set(left, right);
    path_record(left, right, line);
}   /* line_steer() */


/*!
 * @brief Reverse one tick along the recorded path.
 *
 * @par
 * First brakes for LINE_BACKTRACK_BRAKE_MSEC so the car is not still
 * rolling forward. Then plays the recorded wheel speeds back newest
 * first, negated, so the car retraces its path backwards:
 * - at LINE_BACKTRACK_SPEED_PERCENT of the recorded power, using up the
 *   path that much faster, so the distance stays the same but the wheels
 *   are driven firmly (low power is where these motors are least exact);
 * - averaged over LINE_BACKTRACK_SMOOTH_TICKS recorded ticks, so the
 *   small steering corrections in the recording do not make it wobble.
 * Stops once both sensors are on black again, or the path is used up
 * (the car is back where both were last on the line); the car then
 * follows the line from there.
 *
 * @param[in] line     The filtered line state.
 * @param[in] now_msec The current time.
 *
 * @return True if the car reversed (or braked) this tick; false once
 *         it is done.
 */
static bool
backtrack_step (line_state_t line, uint32_t now_msec)
{
    bool     b_reversed = true;
    uint32_t centre     = 0u;
    uint32_t first      = 0u;
    uint32_t last       = 0u;
    int32_t  sum_left   = 0;
    int32_t  sum_right  = 0;
    int32_t  count      = 0;
    int32_t  pct        = (int32_t) LINE_BACKTRACK_SPEED_PERCENT;

    if ((line.b_left_black && line.b_right_black) || (0u == g_back_pos_x100))
    {
        log_printf("Reverse done (%s).",
                   ((0u == g_back_pos_x100) ? "back where both were on it" :
                                              "both on the line"));
        gb_backtracking = false;
        g_path_len      = 0u;
        g_steer         = STEER_STRAIGHT;   // Steer afresh from here.
        g_steer_msec    = now_msec;
        b_reversed      = false;
    }
    else if ((now_msec - g_back_msec) < LINE_BACKTRACK_BRAKE_MSEC)
    {
        motor_set(0, 0);                    // Stop rolling forward first.
    }
    else
    {
        g_back_pos_x100 = ((g_back_pos_x100 > LINE_BACKTRACK_SPEED_PERCENT) ?
                           (g_back_pos_x100 - LINE_BACKTRACK_SPEED_PERCENT) :
                           0u);

        // Average the recorded ticks around the current position.
        centre = g_back_pos_x100 / (uint32_t) PERCENT;
        first  = ((centre > (LINE_BACKTRACK_SMOOTH_TICKS / 2u)) ?
                  (centre - (LINE_BACKTRACK_SMOOTH_TICKS / 2u)) : 0u);
        last   = centre + (LINE_BACKTRACK_SMOOTH_TICKS / 2u);
        last   = ((last < g_path_len) ? last : (g_path_len - 1u));

        for (uint32_t idx = first; idx <= last; idx++)
        {
            sum_left  += g_path_left[idx];
            sum_right += g_path_right[idx];
            count++;
        }

        motor_set(-(sum_left * pct) / (count * PERCENT),
                  -(sum_right * pct) / (count * PERCENT));
    }

    return (b_reversed);
}   /* backtrack_step() */


/*!
 * @brief Record one tick of wheel speeds for a later backtrack.
 *
 * @par
 * While BOTH sensors really are on the line (raw AND filtered black, so a
 * dark patch does not count), the path restarts from here. Otherwise each
 * tick's speeds are added; a path longer than LINE_BACKTRACK_MAX_MSEC is
 * dropped, as it can no longer lead all the way back.
 *
 * @param[in] left  Left wheel speed driven this tick.
 * @param[in] right Right wheel speed driven this tick.
 * @param[in] line  The filtered line state.
 */
static void
path_record (int32_t left, int32_t right, line_state_t line)
{
    bool const b_on_line = (line.b_left_black && g_raw_line.b_left_black &&
                            line.b_right_black && g_raw_line.b_right_black);

    if (VEHICLE_FOLLOW_LINE != g_state)
    {
        path_reset(false);
    }
    else if (b_on_line)
    {
        path_reset(true);
    }
    else if (g_path_len < PATH_MAX_TICKS)
    {
        // NOTE: Wheel speeds are within +/-100, so they fit in int8_t.
        g_path_left[g_path_len]  = (int8_t) left;
        g_path_right[g_path_len] = (int8_t) right;
        g_path_len++;
    }
    else
    {
        gb_path_ok = false;             // Too long: no way back.
    }
}   /* path_record() */


/*!
 * @brief Start a new, empty backtrack path.
 *
 * @param[in] b_ok True if the car is on the line now, so a path recorded
 *                 from here leads back to it.
 */
static void
path_reset (bool b_ok)
{
    g_path_len      = 0u;
    gb_path_ok      = b_ok;
    gb_backtracking = false;

    if (b_ok)
    {
        gb_backtracked = false;         // Line found: next loss may reverse.
    }
}   /* path_reset() */


/*!
 * @brief Report which way a steering command turns.
 *
 * @param[in] steer The steering command.
 *
 * @return -1 for left, +1 for right, 0 for straight.
 */
static int32_t
steer_side (steer_t steer)
{
    int32_t side = 0;

    if ((STEER_LEFT == steer) || (STEER_HARD_LEFT == steer))
    {
        side = -1;
    }
    else if ((STEER_RIGHT == steer) || (STEER_HARD_RIGHT == steer))
    {
        side = 1;
    }
    else
    {
        // Straight.
    }

    return (side);
}   /* steer_side() */


/*!
 * @brief Decide whether the steering may change now.
 *
 * @par
 * Turning towards the line is never delayed: from straight into a turn,
 * or harder/softer on the same side, happens at once, so the car stays
 * close to the line. Straightening up or reversing the turn waits until
 * the last change has been held for LINE_STEER_HOLD_MSEC, so dirt cannot
 * make the car flip back and forth.
 *
 * @param[in] want     The steering the sensors ask for.
 * @param[in] now_msec The current time.
 *
 * @return True if the steering may change to want.
 */
static bool
steer_allowed (steer_t want, uint32_t now_msec)
{
    int32_t const now_side  = steer_side(g_steer);
    int32_t const want_side = steer_side(want);

    return ((0 == now_side) || (want_side == now_side) ||
            ((now_msec - g_steer_msec) >= LINE_STEER_HOLD_MSEC));
}   /* steer_allowed() */


/*!
 * @brief Read the line sensors through the dirty-track filter.
 *
 * @par
 * Each sensor's score rises one tick for every black reading and falls
 * for every white reading, so the filter follows the majority, not
 * single readings. A sensor becomes black once its score reaches
 * LINE_BLACK_CONFIRM_MSEC and white again only once it has drained to 0;
 * between the two it keeps its last state.
 *
 * @par Grey patches
 * Real tape reads solid black; grey sits near the sensor's switching
 * level and flickers. So while a sensor is white, each white reading
 * takes away LINE_GREY_WHITE_WEIGHT black readings, and only nearly solid
 * black can confirm. Also records when each sensor turned black, for
 * line_trusted(). Call once per control tick (line_steer() does).
 *
 * @param[in] now_msec The current time.
 *
 * @return The line state the steering should act on.
 */
static line_state_t
line_filtered (uint32_t now_msec)
{
    line_state_t raw          = line_sensor_read();
    bool const   b_left_was   = g_line.b_left_black;

    g_raw_line = raw;
    bool const   b_right_was  = g_line.b_right_black;

    g_line.b_left_black  = line_score_step(&g_left_score, raw.b_left_black,
                                           b_left_was);
    g_line.b_right_black = line_score_step(&g_right_score,
                                           raw.b_right_black, b_right_was);

    if (g_line.b_left_black && !b_left_was)
    {
        g_left_black_msec = now_msec;
    }
    if (g_line.b_right_black && !b_right_was)
    {
        g_right_black_msec = now_msec;
    }

    return (g_line);
}   /* line_filtered() */


/*!
 * @brief Update one sensor's filter score with one reading.
 *
 * @param[in,out] p_score     The sensor's score, in ticks.
 * @param[in]     b_raw_black This tick's raw reading.
 * @param[in]     b_black     The sensor's filtered state so far.
 *
 * @return The sensor's new filtered state.
 */
static bool
line_score_step (uint32_t * p_score, bool b_raw_black, bool b_black)
{
    uint32_t const limit  = (b_black ? LINE_SCORE_FULL : LINE_SCORE_CONFIRM);
    uint32_t const weight = (b_black ? 1u : LINE_GREY_WHITE_WEIGHT);

    if (b_raw_black)
    {
        if (*p_score < limit)
        {
            (*p_score)++;
        }
    }
    else if (*p_score > weight)
    {
        *p_score -= weight;
    }
    else
    {
        *p_score = 0u;
    }

    if (!b_black && (*p_score >= LINE_SCORE_CONFIRM))
    {
        b_black = true;
    }
    else if (0u == *p_score)
    {
        b_black = false;
    }
    else
    {
        // Between the thresholds: keep the last state.
    }

    return (b_black);
}   /* line_score_step() */


/*!
 * @brief Check whether a sensor's black has lasted long enough to be
 *        trusted as the line rather than a dark patch.
 *
 * @param[in] b_black    The sensor's filtered state.
 * @param[in] black_msec When it turned black.
 * @param[in] now_msec   The current time.
 *
 * @return True if black for at least LINE_NEW_BLACK_TRUST_MSEC.
 */
static bool
line_trusted (bool b_black, uint32_t black_msec, uint32_t now_msec)
{
    return (b_black &&
            ((now_msec - black_msec) >= LINE_NEW_BLACK_TRUST_MSEC));
}   /* line_trusted() */


/*!
 * @brief Set the filtered line state outright, e.g. after lining up on
 *        the line, and restart the steering hold.
 *
 * @param[in] line     The line state to start from.
 * @param[in] now_msec The current time.
 */
static void
line_filter_reset (line_state_t line, uint32_t now_msec)
{
    g_line        = line;
    g_left_score  = (line.b_left_black ? LINE_SCORE_FULL : 0u);
    g_right_score = (line.b_right_black ? LINE_SCORE_FULL : 0u);

    // Lined up on the real line: its black is trusted at once.
    g_left_black_msec  = now_msec - LINE_NEW_BLACK_TRUST_MSEC;
    g_right_black_msec = now_msec - LINE_NEW_BLACK_TRUST_MSEC;
    g_steer       = STEER_STRAIGHT;
    g_steer_msec  = now_msec;
}   /* line_filter_reset() */


/*!
 * @brief Read the time in milliseconds since boot.
 *
 * @return Milliseconds; wraps after about 49 days.
 */
static uint32_t
now_msec_get (void)
{
    // NOTE: Truncating to 32 bits is intended; all time differences use
    // unsigned subtraction, which is correct across the wrap.
    return ((uint32_t) (time_us_64() / USEC_PER_MSEC));
}   /* now_msec_get() */

/*** end of file ***/

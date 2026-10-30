/** @file vehicle.c
 *
 * @brief Mission state machine: line following, obstacle stop, humps.
 *
 * @par Telling a hump from a wall
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
 * - only the left on the line: the line has moved left, so turn left
 *   (left wheel slower or backwards); only the right: turn right;
 * - neither, when both had been on the line together: the line is
 *   between the sensors, so keep going straight;
 * - neither, when one sensor was last on the line: the line slid out past
 *   that side, so keep arcing that way until a sensor finds it.
 * The car keeps its current movement until a change is confirmed: a
 * sensor going white must stay white for LINE_TURN_DELAY_MSEC before the
 * car turns (back to black sooner: no change at all). It never pivots on
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
static bool            gb_left_going   = false; // Left white, unconfirmed.
static bool            gb_right_going  = false; // Right white, unconfirmed.
static uint32_t        g_left_white    = 0u;    // Left turned white at.
static uint32_t        g_right_white   = 0u;    // Right turned white at.

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


static void     state_enter(vehicle_state_t state, uint32_t now_msec);
static void     line_steer(int32_t speed, uint32_t now_msec);
static void     line_cal_step(uint32_t now_msec);
static void     cal_phase_enter(cal_phase_t phase, uint32_t now_msec);
static void     line_align_step(uint32_t now_msec, line_state_t line);
static void     pivot(bool b_left, uint32_t now_msec);
static line_state_t line_filtered(uint32_t now_msec);
static uint32_t now_msec_get(void);
static void     follow_line_step(uint32_t now_msec, uint32_t range_mm,
                                 int32_t pitch_x10);
static void     probe_step(uint32_t now_msec, uint32_t range_mm,
                           int32_t pitch_x10);
static void     hump_step(uint32_t now_msec, int32_t pitch_x10);
static void     blocked_step(uint32_t now_msec, uint32_t range_mm);


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
        state_enter(VEHICLE_PROBE, now_msec);
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

    g_state        = state;
    g_entered_msec = now_msec;
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

    // Barcodes are only read while following the line normally.
    barcode_set_enabled(VEHICLE_FOLLOW_LINE == state);

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
            g_line.b_left_black  = true;
            g_line.b_right_black = true;
            gb_left_going        = false;
            gb_right_going       = false;
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
 * @param[in] speed    The outer-wheel speed, in percent.
 * @param[in] now_msec The current time.
 */
static void
line_steer (int32_t speed, uint32_t now_msec)
{
    line_state_t line     = line_filtered(now_msec);
    int32_t      inner    = (speed * TURN_INNER_PERCENT) / PERCENT;
    bool         b_launch = ((VEHICLE_FOLLOW_LINE == g_state) &&
                             ((now_msec - g_entered_msec) <
                              LINE_START_STRAIGHT_MSEC));

    if (b_launch)
    {
        motor_set(speed, speed);        // Just started: get moving first.
    }
    else if (line.b_left_black && line.b_right_black)
    {
        g_line_last = LINE_LAST_BOTH;
        motor_set(speed, speed);        // Centred: straight on.
    }
    else if (line.b_left_black)
    {
        gb_last_left = true;
        g_line_last  = LINE_LAST_LEFT;
        motor_set(inner, speed);        // Line moved left: arc left.
    }
    else if (line.b_right_black)
    {
        gb_last_left = false;
        g_line_last  = LINE_LAST_RIGHT;
        motor_set(speed, inner);        // Line moved right: arc right.
    }
    else if (LINE_LAST_LEFT == g_line_last)
    {
        // Neither sensor on the line: keep going towards the limit the
        // line was last seen at. Never stop, never pivot.
        motor_set(inner, speed);        // Line slid out left: arc left.
    }
    else if (LINE_LAST_RIGHT == g_line_last)
    {
        motor_set(speed, inner);        // Line slid out right: arc right.
    }
    else
    {
        motor_set(speed, speed);        // Line is between the sensors.
    }
}   /* line_steer() */


/*!
 * @brief Read the line sensors with the turn delay applied.
 *
 * @par
 * Black counts at once. White only counts once it has lasted
 * LINE_TURN_DELAY_MSEC; if the sensor sees black again before that, the
 * white is ignored and the car's movement does not change. Call once per
 * control tick (line_steer() does).
 *
 * @param[in] now_msec The current time.
 *
 * @return The line state the steering should act on.
 */
static line_state_t
line_filtered (uint32_t now_msec)
{
    line_state_t raw = line_sensor_read();

    if (raw.b_left_black)
    {
        g_line.b_left_black = true;
        gb_left_going       = false;
    }
    else if (!gb_left_going)
    {
        gb_left_going = true;           // Just went white: start the delay.
        g_left_white  = now_msec;
    }
    else if ((now_msec - g_left_white) >= LINE_TURN_DELAY_MSEC)
    {
        g_line.b_left_black = false;    // Still white after the delay.
    }
    else
    {
        // White, but not for long enough yet: no change.
    }

    if (raw.b_right_black)
    {
        g_line.b_right_black = true;
        gb_right_going       = false;
    }
    else if (!gb_right_going)
    {
        gb_right_going = true;
        g_right_white  = now_msec;
    }
    else if ((now_msec - g_right_white) >= LINE_TURN_DELAY_MSEC)
    {
        g_line.b_right_black = false;
    }
    else
    {
        // White, but not for long enough yet: no change.
    }

    return (g_line);
}   /* line_filtered() */


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

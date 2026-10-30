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
 * The car never pivots on the spot while following, so it keeps moving
 * forward; it stops if no sensor sees the line for LINE_LOST_MSEC.
 *
 * @par START calibration (LINE_CAL): closed loop and bounded
 * 1. Sweep: swing left and right at a steady speed, each swing longer
 *    (wider) than the last. The sweep is judged only when a swing ends,
 *    and only after at least one full left and one full right swing; it
 *    passes once EACH sensor has seen both black and white, otherwise the
 *    next, wider swing runs (LINE_CAL_SWEEP_MAX_MSEC limit). Each pivot
 *    starts with a short SPEED_KICK push so the wheels actually turn,
 *    then continues at SPEED_SWEEP.
 * 2. Align: pivot towards the line until both sensors are on black for
 *    LINE_CAL_ALIGN_TICKS (LINE_CAL_ALIGN_MAX_MSEC limit).
 * Any failure stops the car with the reason; nothing retries by itself.
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
static bool            gb_level       = false;
static bool            gb_clear       = false;
static bool            gb_lost        = false;  // Line lost (searching).
static uint32_t        g_lost_msec    = 0u;     // Lost since.
static bool            gb_last_left   = true;   // Line last seen left.

// Which sensors last saw the line, so the car knows which limit it left by.
typedef enum
{
    LINE_LAST_BOTH,
    LINE_LAST_LEFT,
    LINE_LAST_RIGHT
} line_last_t;

static line_last_t     g_line_last    = LINE_LAST_BOTH;

// START sweep: which levels each sensor has seen.
static bool            gb_left_on     = false;
static bool            gb_left_off    = false;
static bool            gb_right_on    = false;
static bool            gb_right_off   = false;

// Debounced line state used for steering.
static line_state_t    g_line          = { true, true };
static uint32_t        g_left_ticks    = 0u;    // Left raw != filtered for.
static uint32_t        g_right_ticks   = 0u;    // Right raw != filtered for.

// START calibration: swings, alignment and the current pivot.
static bool            gb_aligning     = false;
static uint32_t        g_align_msec    = 0u;    // Alignment began.
static uint32_t        g_centred_ticks = 0u;
static uint32_t        g_swing_start   = 0u;
static uint32_t        g_swing_count   = 0u;
static bool            gb_swing_left   = true;
static bool            gb_pivoting     = false;
static bool            gb_pivot_left   = true;
static uint32_t        g_pivot_msec    = 0u;    // Pivot began/reversed.


static void     state_enter(vehicle_state_t state, uint32_t now_msec);
static void     line_steer(int32_t speed, uint32_t now_msec);
static void     line_cal_step(uint32_t now_msec);
static void     line_align_step(uint32_t now_msec, line_state_t line);
static bool     line_swing_step(uint32_t now_msec);
static void     pivot(bool b_left, uint32_t now_msec);
static char const * seen_text(bool b_saw_black, bool b_saw_white);
static line_state_t line_filtered(void);
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
 * @brief Start: run the 4 s line sweep, then follow the line.
 *
 * @par
 * Works from READY or STOPPED only. Place the car centred on the line
 * first.
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
    int32_t  pitch_x10 = imu_pitch_x10();

    switch (g_state)
    {
    case VEHICLE_LINE_CAL:
        line_cal_step(now_msec);
    break;

    case VEHICLE_FOLLOW_LINE:
        follow_line_step(now_msec, range_mm, pitch_x10);
    break;

    case VEHICLE_PROBE:
        probe_step(now_msec, range_mm, pitch_x10);
    break;

    case VEHICLE_HUMP:
        hump_step(now_msec, pitch_x10);
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
 * @brief FOLLOW_LINE: steer along the line; react to humps and obstacles.
 *
 * @param[in] now_msec  The current time.
 * @param[in] range_mm  The forward range.
 * @param[in] pitch_x10 The pitch, in tenths of a degree.
 */
static void
follow_line_step (uint32_t now_msec, uint32_t range_mm, int32_t pitch_x10)
{
    bool b_range_used = ((int32_t) (now_msec - g_ignore_until) >= 0);

    if (pitch_x10 >= HUMP_ENTER_PITCH_X10)
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
 * @param[in] pitch_x10 The pitch, in tenths of a degree.
 */
static void
probe_step (uint32_t now_msec, uint32_t range_mm, int32_t pitch_x10)
{
    if (pitch_x10 >= HUMP_ENTER_PITCH_X10)
    {
        log_printf("Probe: nose tipped up -> HUMP, climbing.\n");
        state_enter(VEHICLE_HUMP, now_msec);
    }
    else if (range_mm > OBSTACLE_CLEAR_MM)
    {
        state_enter(VEHICLE_FOLLOW_LINE, now_msec);
    }
    else if ((range_mm <= OBSTACLE_HARD_STOP_MM) ||
             ((now_msec - g_entered_msec) > OBSTACLE_PROBE_MSEC) ||
             (!imu_is_ok()))
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
 * @param[in] pitch_x10 The pitch, in tenths of a degree.
 */
static void
hump_step (uint32_t now_msec, int32_t pitch_x10)
{
    int32_t  magnitude = ((pitch_x10 < 0) ? -pitch_x10 : pitch_x10);
    uint32_t in_hump   = now_msec - g_entered_msec;

    // Track how long the car has been level.
    if (magnitude > HUMP_LEVEL_PITCH_X10)
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

    if (gb_level && (in_hump > HUMP_MIN_MSEC) &&
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
    gb_lost        = false;

    if (VEHICLE_LINE_CAL == state)
    {
        gb_left_on   = false;
        gb_left_off  = false;
        gb_right_on  = false;
        gb_right_off = false;

        gb_aligning     = false;
        gb_pivoting     = false;
        g_centred_ticks = 0u;
        g_swing_count   = 0u;
        g_swing_start   = now_msec;
        gb_swing_left   = true;
        log_printf("LINE_CAL: sweeping until each sensor sees black and "
                   "white...");
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
 * @brief LINE_CAL: sweep until each sensor has seen black and white, then
 *        align on the line.
 *
 * @param[in] now_msec The current time.
 */
static void
line_cal_step (uint32_t now_msec)
{
    line_state_t line       = line_sensor_read();
    uint32_t     elapsed    = now_msec - g_entered_msec;
    bool         b_left_ok  = false;
    bool         b_right_ok = false;

    gb_left_on   = (gb_left_on || line.b_left_black);
    gb_left_off  = (gb_left_off || (!line.b_left_black));
    gb_right_on  = (gb_right_on || line.b_right_black);
    gb_right_off = (gb_right_off || (!line.b_right_black));
    b_left_ok    = (gb_left_on && gb_left_off);
    b_right_ok   = (gb_right_on && gb_right_off);

    // Remember which side the line was last on, to align towards it.
    if (line.b_left_black != line.b_right_black)
    {
        gb_last_left = line.b_left_black;
    }

    if (gb_aligning)
    {
        line_align_step(now_msec, line);
    }
    else if (elapsed > LINE_CAL_SWEEP_MAX_MSEC)
    {
        motor_set(0, 0);
        log_printf("LINE_CAL FAILED (sweep timed out): LEFT %s, RIGHT %s.",
                   seen_text(gb_left_on, gb_left_off),
                   seen_text(gb_right_on, gb_right_off));
        log_printf("If the car barely turned: run WHEEL TEST, then raise "
                   "SPEED_KICK / SPEED_SWEEP.");
        state_enter(VEHICLE_STOPPED, now_msec);
    }
    else if (line_swing_step(now_msec))
    {
        // A swing has just ended: judge the sweep only now, and only
        // after at least one full swing each way.
        log_printf("LINE_CAL swing %" PRIu32 " (%s) done: LEFT %s, RIGHT %s.",
                   g_swing_count, (gb_swing_left ? "right" : "left"),
                   seen_text(gb_left_on, gb_left_off),
                   seen_text(gb_right_on, gb_right_off));

        if (b_left_ok && b_right_ok &&
            (g_swing_count >= LINE_CAL_MIN_SWINGS))
        {
            log_printf("LINE_CAL sweep OK: both sensors saw black and "
                       "white. Aligning...");
            gb_aligning   = true;
            g_align_msec  = now_msec;
            g_swing_count = 0u;
            g_swing_start = now_msec;
            gb_swing_left = gb_last_left;
        }
    }
    else
    {
        // Mid-swing: keep sweeping.
    }
}   /* line_cal_step() */


/*!
 * @brief Swing left and right, each swing LINE_CAL_SWING_MSEC longer than
 *        the last, so the sweep widens until the sensors cross the line.
 *
 * @par
 * Swings alternate around the start: 1 s left, 2 s right (ending 1 s
 * right of the start), 3 s left (ending 2 s left), and so on.
 *
 * @param[in] now_msec The current time.
 *
 * @return True on the tick a swing ends (and the next one begins).
 */
static bool
line_swing_step (uint32_t now_msec)
{
    uint32_t swing_msec  = (g_swing_count + 1u) * LINE_CAL_SWING_MSEC;
    bool     b_swing_end = ((now_msec - g_swing_start) > swing_msec);

    if (b_swing_end)
    {
        gb_swing_left = !gb_swing_left;
        g_swing_start = now_msec;
        g_swing_count++;
    }

    pivot(gb_swing_left, now_msec);

    return (b_swing_end);
}   /* line_swing_step() */


/*!
 * @brief Pivot until both sensors are on black, then start following.
 *
 * @param[in] now_msec The current time.
 * @param[in] line     The current line state.
 */
static void
line_align_step (uint32_t now_msec, line_state_t line)
{
    if ((now_msec - g_align_msec) > LINE_CAL_ALIGN_MAX_MSEC)
    {
        motor_set(0, 0);
        log_printf("LINE_CAL FAILED: could not align both sensors on the "
                   "line in time.");
        state_enter(VEHICLE_STOPPED, now_msec);
    }
    else if (line.b_left_black && line.b_right_black)
    {
        motor_set(0, 0);
        gb_pivoting = false;
        g_centred_ticks++;

        if (g_centred_ticks >= LINE_CAL_ALIGN_TICKS)
        {
            log_printf("LINE_CAL OK: both sensors on the line -> "
                       "following.");
            g_line.b_left_black  = true;
            g_line.b_right_black = true;
            g_line_last          = LINE_LAST_BOTH;
            state_enter(VEHICLE_FOLLOW_LINE, now_msec);
        }
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
        g_centred_ticks = 0u;
        (void) line_swing_step(now_msec);   // Lost it: widening swings.
    }
}   /* line_align_step() */


/*!
 * @brief Pivot on the spot, with a short SPEED_KICK push whenever the
 *        pivot starts or changes direction.
 *
 * @param[in] b_left   True to pivot left (left wheel back, right forward).
 * @param[in] now_msec The current time.
 */
static void
pivot (bool b_left, uint32_t now_msec)
{
    int32_t speed = SPEED_SWEEP;

    if ((!gb_pivoting) || (b_left != gb_pivot_left))
    {
        gb_pivoting   = true;
        gb_pivot_left = b_left;
        g_pivot_msec  = now_msec;
    }

    if ((now_msec - g_pivot_msec) < KICK_MSEC)
    {
        speed = SPEED_KICK;
    }

    if (b_left)
    {
        motor_set(-speed, speed);
    }
    else
    {
        motor_set(speed, -speed);
    }
}   /* pivot() */


/*!
 * @brief Describe what one sensor saw during the sweep.
 *
 * @param[in] b_saw_black True if it ever read black.
 * @param[in] b_saw_white True if it ever read white.
 *
 * @return A short constant string.
 */
static char const *
seen_text (bool b_saw_black, bool b_saw_white)
{
    char const * p_text = "OK";

    if ((!b_saw_black) && (!b_saw_white))
    {
        p_text = "never changed";
    }
    else if (!b_saw_black)
    {
        p_text = "never saw black";
    }
    else if (!b_saw_white)
    {
        p_text = "never saw white";
    }
    else
    {
        // Saw both: OK.
    }

    return (p_text);
}   /* seen_text() */


/*!
 * @brief Steer along the line at a given speed; search if it is lost.
 *
 * @param[in] speed    The outer-wheel speed, in percent.
 * @param[in] now_msec The current time.
 */
static void
line_steer (int32_t speed, uint32_t now_msec)
{
    line_state_t line     = line_filtered();
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
        gb_lost     = false;
        g_line_last = LINE_LAST_BOTH;
        motor_set(speed, speed);        // Centred: straight on.
    }
    else if (line.b_left_black)
    {
        gb_lost      = false;
        gb_last_left = true;
        g_line_last  = LINE_LAST_LEFT;
        motor_set(inner, speed);        // Line moved left: arc left.
    }
    else if (line.b_right_black)
    {
        gb_lost      = false;
        gb_last_left = false;
        g_line_last  = LINE_LAST_RIGHT;
        motor_set(speed, inner);        // Line moved right: arc right.
    }
    else
    {
        // Neither sensor on the line: keep moving towards the limit the
        // line was last seen at, never pivoting on the spot.
        if (!gb_lost)
        {
            gb_lost     = true;
            g_lost_msec = now_msec;
        }

        if ((now_msec - g_lost_msec) > LINE_LOST_MSEC)
        {
            log_printf("Line lost for 3 s -> stopped. Put the car back on "
                       "the line and press START.");
            state_enter(VEHICLE_STOPPED, now_msec);
        }
        else if (LINE_LAST_LEFT == g_line_last)
        {
            motor_set(inner, speed);    // Line slid out left: arc left.
        }
        else if (LINE_LAST_RIGHT == g_line_last)
        {
            motor_set(speed, inner);    // Line slid out right: arc right.
        }
        else
        {
            motor_set(speed, speed);    // Line is between the sensors.
        }
    }
}   /* line_steer() */


/*!
 * @brief Read the line sensors, ignoring changes shorter than
 *        LINE_DEBOUNCE_TICKS.
 *
 * @par
 * Call once per control tick (line_steer() does).
 *
 * @return The debounced line state.
 */
static line_state_t
line_filtered (void)
{
    line_state_t raw = line_sensor_read();

    if (raw.b_left_black == g_line.b_left_black)
    {
        g_left_ticks = 0u;
    }
    else
    {
        g_left_ticks++;
        if (g_left_ticks >= LINE_DEBOUNCE_TICKS)
        {
            g_line.b_left_black = raw.b_left_black;
            g_left_ticks        = 0u;
        }
    }

    if (raw.b_right_black == g_line.b_right_black)
    {
        g_right_ticks = 0u;
    }
    else
    {
        g_right_ticks++;
        if (g_right_ticks >= LINE_DEBOUNCE_TICKS)
        {
            g_line.b_right_black = raw.b_right_black;
            g_right_ticks        = 0u;
        }
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

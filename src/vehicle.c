/** @file vehicle.c
 *
 * @brief Mission state machine: line following, obstacle stop, humps.
 *
 * @par Obstacle stop
 * With OBSTACLE_STOP_AT_ONCE, an echo from closer than OBSTACLE_STOP_MM
 * brakes the car at once, scans while stopped, then waits in BLOCKED until
 * clear. Bypass and line recovery await movement calibration; no PROBE is done.
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
 * - whatever the car is doing, the moment one sensor sees black it turns
 *   towards that sensor;
 * - neither: reverse along the recorded path back to where both sensors
 *   were last on the line (LINE_BACKTRACK_ENABLE, once per loss), stopping
 *   as soon as a sensor sees black; if that does not find it, arc hard
 *   towards the side the line slid out past until a sensor finds it.
 * The car keeps its current movement until a change is confirmed, so a
 * dirty track (grey footprints, scratched tape) does not make it jitter:
 * each sensor follows the majority of its recent readings (see
 * line_filtered()), and each steering change is held for at least
 * LINE_STEER_HOLD_MSEC. It never pivots on
 * the spot and never stops for white; only an obstacle (PROBE/BLOCKED)
 * or STOP stops it.
 *
 * @par START calibration (LINE_CAL): drive forward onto the line
 * Both line sensors already on black: no calibration, follow at once.
 * One on black: drive forward, arcing towards it, until both are on
 * black for LINE_CAL_ALIGN_TICKS, then follow. Both on white: START does
 * not move the car; it stays READY. If both go white while arcing, it
 * keeps arcing for LINE_CAL_LOST_MSEC, then stops (READY).
 *
 * @par Barcodes
 * From the first bar until the scan ends (barcode_is_capturing()),
 * FOLLOW_LINE drives strictly straight at SPEED_SCAN, holding both line
 * sensors on the line with only a small speed-neutral trim
 * (scan_hold_line()), because bar widths are measured in time. Normal
 * line-following steering is suspended; only an obstacle (or a hump)
 * interrupts the scan.
 * A decoded command (every letter A-Z: left, right, straight or U-turn,
 * BARCODE_LETTER_COMMANDS) is kept as the pending turn and carried out at
 * the next crossroad, which line_junction_detect() finds from the side sensor
 * (junction_act()); a newer command replaces it. Crossroads are only looked for
 * while such a turn is waiting, so no barcode bar is ever taken for one. A
 * U-turn there pivots past the side branch (TURN_UTURN_SKIP_LINES) to the
 * line it came along. A turn (TURN state) runs:
 * 0. ADVANCE (crossroad turns only): drive straight for
 *    JUNCTION_ADVANCE_MSEC, so the wheel axle is over the crossing;
 * 1. LEAVE: pivot until both line sensors are off the current line;
 * 2. FIND:  keep pivoting until a sensor finds a line. A left/right turn
 *    gives up after TURN_90_MSEC (about 90 degrees) and drives on; a
 *    U-turn finds the same line again behind it;
 * 3. ALIGN: keep pivoting until both sensors are on it, then follow.
 * Pressed with the car stopped (test keys), the car turns and stops
 * again, so TURN_90_MSEC can be tuned on the bench.
 *
 * @par BARCODE TEST
 * Drives straight, ignoring the line sensors, at the same speeds as START
 * (SPEED_CRUISE, SPEED_SCAN while a barcode is read, SPEED_CREEP near an
 * obstacle), with barcode reading on: decoded messages and commands are
 * shown. A command (left, right or U-turn) becomes the pending turn and
 * is carried out at the next crossroad, exactly as when following; after
 * the turn the car drives straight again (still BARCODE TEST). It runs until
 * STOP, or stops by itself at an obstacle closer than OBSTACLE_STOP_MM.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/stdlib.h"

#include "barcode.h"
#include "config.h"
#include "encoder.h"
#include "hump.h"
#include "imu.h"
#include "line_sensor.h"
#include "log.h"
#include "motor.h"
#include "obstacle.h"
#include "bypass.h"
#include "ultrasonic.h"
#include "vehicle.h"

#define USEC_PER_MSEC 1000u
#define PERCENT 100

// Line filter score limits, in control ticks.
#define MSEC_TO_TICKS(msec) (((msec) * USEC_PER_MSEC) / TICK_USEC)
#define LINE_SCORE_FULL MSEC_TO_TICKS(LINE_TURN_DELAY_MSEC)
#define LINE_SCORE_CONFIRM MSEC_TO_TICKS(LINE_BLACK_CONFIRM_MSEC)
#define LINE_SCORE_CAPTURE MSEC_TO_TICKS(LINE_LOST_BLACK_CONFIRM_MSEC)
#define PATH_MAX_TICKS MSEC_TO_TICKS(LINE_BACKTRACK_MAX_MSEC)

// Calibration speed: LINE_CAL_SPEED_PERCENT of the following speed. Pivots
// still burst at PIVOT_PULSE_SPEED (lower stalls), with braked pauses
// long enough that the AVERAGE is CAL_SPEED.
#define CAL_SPEED ((SPEED_CRUISE * LINE_CAL_SPEED_PERCENT) / PERCENT)
#define CAL_PIVOT_OFF_MSEC                                                     \
    ((PIVOT_PULSE_ON_MSEC * (uint32_t)(PIVOT_PULSE_SPEED - CAL_SPEED)) /       \
     (uint32_t)CAL_SPEED)

static bool            gb_bypass_dry_run    = false;
static vehicle_state_t g_state              = VEHICLE_READY;
static uint32_t        g_entered_msec       = 0u; // When g_state began.
static uint32_t        g_level_msec         = 0u; // Level since (in HUMP).
static uint32_t        g_clear_msec         = 0u; // Clear since (BLOCKED).
static uint32_t        g_ignore_until       = 0u; // Ignore range until.
static int32_t         g_hump_peak_tilt_x10 = 0;
static bool            gb_level             = false;
static bool            gb_clear             = false;
static bool            gb_hump_tilted       = false;
static uint32_t        g_hump_off_msec      = 0u;    // When HUMP last ended.
static bool            gb_floorless         = false; // No floor seen, since
static uint32_t        g_floorless_msec     = 0u;    // this time.
static bool            gb_hump_over         = false; // Past the top of it.
static uint32_t        g_top_msec           = 0u;    // When it went over.
static int32_t         g_top_speed          = 0;     // Speed then.
static int32_t         g_hump_speed         = 0;     // Speed on it now.
static int32_t         g_hump_extra         = 0;     // Power added for stalls.
static uint32_t        g_boost_msec         = 0u;    // Last stall boost.
static uint32_t        g_stall_edges        = 0u;    // Edges at window start.
static uint32_t        g_stall_window       = 0u;    // Window start.
static uint32_t        g_stall_since        = 0u;    // Stalled since.
static bool            gb_stalled           = false; // Last window stalled.
static int32_t         g_exit_speed         = 0;     // Speed as HUMP ended.
static bool            gb_tilting           = false; // Over HUMP_ENTER, since
static uint32_t        g_tilt_msec          = 0u;    // g_tilt_msec.
static bool            gb_last_left         = true;  // Line last seen left.

// Which sensors last saw the line, so the car knows which limit it left by.
typedef enum
{
    LINE_LAST_BOTH,
    LINE_LAST_LEFT,
    LINE_LAST_RIGHT
} line_last_t;

static line_last_t g_line_last = LINE_LAST_BOTH;

// Debounced line state used for steering.
static line_state_t g_line        = {true, true};
static uint32_t     g_left_score  = LINE_SCORE_FULL; // Net black,
static uint32_t     g_right_score = LINE_SCORE_FULL; // in ticks.

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

static steer_t  g_steer      = STEER_STRAIGHT;
static uint32_t g_steer_msec = 0u; // Steering last changed at.

// Backtrack: the wheel speeds driven since both sensors were last on the
// line, so a car that has lost it can reverse along the same path back.
static int8_t       g_path_left[PATH_MAX_TICKS];
static int8_t       g_path_right[PATH_MAX_TICKS];
static uint32_t     g_path_len      = 0u;
static bool         gb_path_ok      = false; // Path leads back to black.
static bool         gb_backtracking = false;
static bool         gb_backtracked  = false; // Used up for this loss.
static uint32_t     g_back_msec     = 0u;    // Reverse began at.
static uint32_t     g_back_pos_x100 = 0u;    // Replay position, ticks
                                             // x 100 (counts down).
static line_state_t g_raw_line      = {false, false};

// START calibration.
static uint32_t g_centred_ticks  = 0u;    // Both on black this long.
static bool     gb_cal_left      = true;  // Turning left (calibration).
static bool     gb_cal_turned    = false; // Has arced at all.
static uint32_t g_cal_black_msec = 0u;    // A line sensor last on
                                          // black (calibration).
static bool     gb_pivoting      = false;
static bool     gb_pivot_left    = true;
static uint32_t g_pivot_msec     = 0u; // Pivot began/reversed.

// Barcode reading and barcode turns.
typedef enum
{
    TURN_ADVANCE, // Crossroad: drive on until the axle is over it.
    TURN_LEAVE,   // Pivot until both line sensors are off the line.
    TURN_FIND,    // Pivot until a line sensor finds a line.
    TURN_ALIGN    // Pivot until both line sensors are on it.
} turn_phase_t;

static bool              gb_follow_from_bypass;
static bool              gb_launch          = true; // Straight start allowed.
static barcode_command_t g_turn_cmd         = BARCODE_CMD_NONE;
static vehicle_state_t   g_turn_after       = VEHICLE_FOLLOW_LINE;
static turn_phase_t      g_turn_phase       = TURN_LEAVE;
static uint32_t          g_turn_msec        = 0u;   // Current phase began.
static uint32_t          g_pivot_start_msec = 0u;   // LEAVE began (pivot).
static bool              gb_turn_left_on    = true; // Left was last on black.
static bool              gb_turn_advance = false;   // Next turn: ADVANCE first.
static uint32_t          g_uturn_skip    = 0u; // Lines still to pivot past.
static bool              gb_junction_deferred = false; // Crossroad waiting
                                                       // for a decode.
static bool              gb_feat_line         = true;  // Follow the line (else
                                                       // drive straight).
static bool              gb_feat_avoid        = true; // Scan and go round (else
                                                      // stop and wait).
static bool              gb_feat_barcode      = true; // Read barcodes.
static bool              gb_feat_hump         = true; // Climb and measure humps
                                                      // (else tilt is ignored).
static barcode_command_t g_pending_turn       = BARCODE_CMD_NONE; // At the next
                                                                  // crossroad.

static void state_enter(vehicle_state_t state, uint32_t now_msec);
static void junction_seen(uint32_t now_msec);
static void junction_act(uint32_t now_msec, uint32_t already_msec);
static void turn_step(uint32_t now_msec);
static void turn_phase_enter(turn_phase_t phase, uint32_t now_msec);
static void turn_finish(uint32_t now_msec, line_state_t line, line_last_t last,
                        char const * p_how);
static void scan_hold_line(void);
static void line_steer(int32_t speed, uint32_t now_msec);
static void line_cal_step(uint32_t now_msec);
static void pivot(bool b_left, uint32_t now_msec);
static line_state_t line_filtered(void);
static bool line_score_step(uint32_t * p_score, bool b_raw_black, bool b_black,
                            uint32_t confirm);
static void line_filter_reset(line_state_t line, uint32_t now_msec);
static bool backtrack_step(line_state_t line, uint32_t now_msec);
static void path_record(int32_t left, int32_t right, line_state_t line);
static void path_reset(bool b_ok);
static bool steer_allowed(steer_t want, uint32_t now_msec);
static uint32_t now_msec_get(void);
static void     follow_line_step(uint32_t now_msec, uint32_t range_mm,
                                 int32_t pitch_x10);
static void probe_step(uint32_t now_msec, uint32_t range_mm, int32_t pitch_x10);
static void hump_step(uint32_t now_msec, int32_t pitch_x10);
static bool hump_tilt_held(uint32_t now_msec, int32_t tilt_x10);
static void stall_update(uint32_t now_msec);
static uint32_t stall_msec(uint32_t now_msec);
static int32_t  hump_speed(uint32_t now_msec, uint32_t in_hump_msec);
static int32_t  cruise_speed(uint32_t now_msec);
static bool     is_parked(vehicle_state_t state);
static bool     line_under_car(void);
static void     cal_arc(bool b_left);
static bool     picked_up(uint32_t now_msec);
static void     log_tenths(char const * p_what, int32_t value_x10);
static void     blocked_step(uint32_t now_msec, uint32_t range_mm);
static void     barcode_test_step(uint32_t now_msec, uint32_t range_mm,
                                  int32_t tilt_x10);

/*!
 * @brief Start in READY with the motors braked.
 */
void
vehicle_init(void)
{
    state_enter(VEHICLE_READY, now_msec_get());
} /* vehicle_init() */

// Shared rehearsal: recover/follow when started on line; empty floor ends
// stopped.
/** @brief Start the manual bypass rehearsal subject to existing guards. */
void
vehicle_bypass_test(void)
{
    uint32_t now = now_msec_get();
    if ((!is_parked(g_state)) || (obstacle_scan_active()))
    {
        log_obstacle(
            "BYPASS DRY RUN: stop the car and finish/cancel the scan first.");
        return;
    }
    if (bypass_test_begin(now))
    {
        gb_feat_line      = false;
        gb_feat_barcode   = false;
        gb_feat_hump      = false;
        gb_bypass_dry_run = true;
        state_enter(VEHICLE_BYPASS, now);
    }
}

/*!
 * @brief Start: search for the line (LINE_CAL), then follow it.
 *
 * @par
 * Works from READY or STOPPED only. The car may be placed anywhere near
 * the line.
 */
void
vehicle_start(void)
{
    if ((is_parked(g_state)) && (!line_under_car()))
    {
        // Both line sensors on white: stay put (line_under_car() logs).
    }
    else if (is_parked(g_state))
    {
        gb_feat_line    = true; // A full run: every feature on.
        gb_feat_avoid   = true;
        gb_feat_barcode = true;
        gb_feat_hump    = true;
        g_pending_turn  = BARCODE_CMD_NONE; // A new run: no old turn.
        hump_new_run();                     // Nor the last run's humps.
        state_enter(VEHICLE_LINE_CAL, now_msec_get());
    }
} /* vehicle_start() */

/*!
 * @brief Start a FEATURE TEST run: each feature on or off.
 *
 * @par
 * - b_line:    follow the line with the IR sensors (from LINE_CAL, as
 *              START); off: drive straight ahead (BARCODE_TEST).
 * - b_avoid:   at an obstacle, scan and go round it (bypass.c); off:
 *              stop, and drive on once it has gone (BLOCKED).
 * - b_barcode: read barcodes (and beep and act on them); off: ignore.
 * - b_hump:    when the nose tips up, climb the hump (HUMP) and measure
 *              it (hump.c), with or without the line; off: tilt ignored.
 * Works from READY or STOPPED only. STOP ends it.
 *
 * @param[in] b_line    Follow the line.
 * @param[in] b_avoid   Go round obstacles.
 * @param[in] b_barcode Read barcodes.
 * @param[in] b_hump    Climb and measure humps.
 */
void
vehicle_feature_test(bool b_line, bool b_avoid, bool b_barcode, bool b_hump)
{
    if (((is_parked(g_state)) && b_line) && (!line_under_car()))
    {
        // Both line sensors on white: stay put (line_under_car() logs).
    }
    else if (is_parked(g_state))
    {
        gb_feat_line    = b_line;
        gb_feat_avoid   = b_avoid;
        gb_feat_barcode = b_barcode;
        gb_feat_hump    = b_hump;
        g_pending_turn  = BARCODE_CMD_NONE;
        hump_new_run();
        log_state("FEATURE TEST: line %s, obstacles %s, barcodes %s, "
                  "humps %s.",
                  (b_line ? "FOLLOWED" : "ignored (straight)"),
                  (b_avoid ? "AVOIDED" : "stop and wait"),
                  (b_barcode ? "READ" : "ignored"),
                  (b_hump ? "CLIMBED" : "ignored"));
        state_enter((b_line ? VEHICLE_LINE_CAL : VEHICLE_BARCODE_TEST),
                    now_msec_get());
    }
    else
    {
        log_printf("Stop the car before FEATURE TEST.");
    }
} /* vehicle_feature_test() */

/*!
 * @brief Stop at once, from any state. START is needed to move again.
 */
void
vehicle_stop(void)
{
    state_enter(VEHICLE_STOPPED, now_msec_get());
} /* vehicle_stop() */

/*!
 * @brief Act on a barcode command.
 *
 * @par
 * While on a run (following, BARCODE TEST, or held up by an obstacle or
 * hump), every command (left, right, straight, U-turn) becomes the
 * pending turn,
 * carried out at the next crossroad (a newer command replaces it). A test
 * key with the car stopped turns it at once and
 * stops again, to tune TURN_90_MSEC. A barcode decoded once the car has
 * stopped is only logged: it never moves the car.
 *
 * @param[in] command The command; BARCODE_CMD_NONE does nothing.
 * @param[in] b_test  True for a test key, false for a read barcode.
 */
void
vehicle_barcode_command(barcode_command_t command, bool b_test)
{
    uint32_t now_msec  = now_msec_get();
    bool     b_stopped = is_parked(g_state);
    bool     b_moving  = ((((((VEHICLE_FOLLOW_LINE == g_state) ||
                              (VEHICLE_BARCODE_TEST == g_state)) ||
                             (VEHICLE_BARCODE_STATIC == g_state)) ||
                            (VEHICLE_BLOCKED == g_state)) ||
                           (VEHICLE_PROBE == g_state)) ||
                          (VEHICLE_HUMP == g_state));

    if (BARCODE_CMD_NONE == command)
    {
        // A plain message: keep following.
    }
    else if ((b_test && b_stopped) && (BARCODE_CMD_STRAIGHT == command))
    {
        log_line("GO STRAIGHT: nothing to turn on the bench.");
    }
    else if (b_test && b_stopped)
    {
        // Test key on the bench: turn at once, then stop again.
        g_turn_cmd      = command;
        g_turn_after    = VEHICLE_STOPPED;
        gb_turn_advance = false;
        state_enter(VEHICLE_TURN, now_msec);
    }
    else if (b_moving)
    {
        // Also while held up (obstacle, hump): the run carries on after.
        g_pending_turn = command;
        log_line("Pending: %s at the next crossroad.",
                 barcode_command_name(command));
    }
    else
    {
        // A barcode read as the car stopped never moves it.
        log_line("%s not acted on in %s.", barcode_command_name(command),
                 vehicle_state_name(g_state));
    }
} /* vehicle_barcode_command() */

/*!
 * @brief The side sensor's mark (after a barcode) is the crossroad: act
 *        on it at once.
 *
 * @par
 * There is no bend check: on the car it rejected real crossroads more
 * often than it caught bends (2026-11-13).
 *
 * @param[in] now_msec The current time.
 */
static void
junction_seen(uint32_t now_msec)
{
    junction_act(now_msec, 0u);
} /* junction_seen() */

/*!
 * @brief A crossroad was just reached: carry out the pending turn, if any.
 *
 * @par
 * Only called while following the line or in BARCODE TEST; the car goes
 * back to that state after the turn.
 *
 * @param[in] now_msec     The current time.
 * @param[in] already_msec How long the car has driven on since the mark
 *                         (spent confirming it); the turn's ADVANCE
 *                         phase is that much shorter.
 */
static void
junction_act(uint32_t now_msec, uint32_t already_msec)
{
    if (BARCODE_CMD_STRAIGHT == g_pending_turn)
    {
        log_line("Crossroad: GO STRAIGHT, carrying on.");
        g_pending_turn = BARCODE_CMD_NONE;
    }
    else if (BARCODE_CMD_NONE != g_pending_turn)
    {
        log_line("Crossroad: %s now.", barcode_command_name(g_pending_turn));
        g_turn_cmd      = g_pending_turn;
        g_pending_turn  = BARCODE_CMD_NONE;
        g_turn_after    = g_state;
        gb_turn_advance = true;
        state_enter(VEHICLE_TURN, now_msec);
        g_turn_msec -= already_msec; // ADVANCE counts from the mark.
    }
    else
    {
        log_line("Crossroad seen in %s, no turn pending: straight on.",
                 vehicle_state_name(g_state));
    }
} /* junction_act() */

/*!
 * @brief Report the turn waiting for the next crossroad.
 *
 * @return The pending command, or BARCODE_CMD_NONE.
 */
barcode_command_t
vehicle_pending_turn(void)
{
    return (g_pending_turn);
} /* vehicle_pending_turn() */

/*!
 * @brief Start BARCODE TEST: drive straight and read barcodes.
 *
 * @par
 * Works from READY or STOPPED only. STOP ends it.
 */
void
vehicle_barcode_test(void)
{
    // Kept for the serial key: a feature test with only barcodes on.
    vehicle_feature_test(false, false, true, false);
} /* vehicle_barcode_test() */

/*!
 * @brief Start BARCODE TEST (STATIC): stand still and read barcodes slid
 *        slowly past the side sensor by hand.
 *
 * @par
 * Works from READY or STOPPED only. STOP ends it. A read command is kept
 * as the pending turn, as in a real run, so the page shows it.
 */
void
vehicle_barcode_static(void)
{
    if (is_parked(g_state))
    {
        log_state("BARCODE TEST (STATIC): motors off. Slide a barcode "
                  "slowly past the side sensor. STOP to end.");
        g_pending_turn = BARCODE_CMD_NONE; // A new test: no old turn.
        state_enter(VEHICLE_BARCODE_STATIC, now_msec_get());
    }
    else
    {
        log_printf("Stop the car before BARCODE TEST (STATIC).");
    }
} /* vehicle_barcode_static() */

/*!
 * @brief Run one step of the state machine. Call every control tick,
 *        after the sensor tasks.
 */
void
vehicle_task(void)
{
    uint32_t now_msec = now_msec_get();
    uint32_t range_mm = ultrasonic_range_mm();
    int32_t  tilt_x10 = imu_total_tilt_x10(); // Humps (config.h).
    bool     b_reading =
        ((VEHICLE_FOLLOW_LINE == g_state) || (VEHICLE_BARCODE_TEST == g_state));
    uint32_t        mark_msec = 0u;
    line_junction_t junction  = LINE_JUNCTION_NONE;

    // Lifted off the floor while driving: the end of the demo.
    if ((vehicle_is_moving()) && (picked_up(now_msec)))
    {
        log_state("Picked up (no floor under any IR sensor for %" PRIu32
                  " ms) -> stopped. End of the demo.",
                  (now_msec - g_floorless_msec));
        state_enter(VEHICLE_PICKED_UP, now_msec);
        motor_set(0, 0);
        return;
    }

    stall_update(now_msec);

    // Hump height: add up the climb while driving forwards.
    hump_task(
        ((((VEHICLE_FOLLOW_LINE == g_state) || (VEHICLE_PROBE == g_state)) ||
          (VEHICLE_HUMP == g_state)) ||
         (VEHICLE_BARCODE_TEST == g_state)),
        (VEHICLE_HUMP == g_state));

    // Apply the immediate stop before junctions, calibration, turns or
    // tilt handling can issue another movement command.
    if (((((((OBSTACLE_STOP_AT_ONCE && (vehicle_is_moving())) &&
             (VEHICLE_BLOCKED != g_state)) &&
            (VEHICLE_OBSTACLE_SCAN != g_state)) &&
           (VEHICLE_BYPASS != g_state)) &&
          (VEHICLE_HUMP != g_state)) &&
         (((int32_t)(now_msec - g_ignore_until)) >= 0)) &&
        (range_mm <= OBSTACLE_STOP_MM))
    {
        log_obstacle("Obstacle at %" PRIu32 " mm -> stopped.", range_mm);
        state_enter((gb_feat_avoid ? VEHICLE_OBSTACLE_SCAN : VEHICLE_BLOCKED),
                    now_msec);
        if (VEHICLE_OBSTACLE_SCAN == g_state)
        {
            obstacle_scan_start(range_mm, now_msec);
        }
        return;
    }

    // A crossroad found last tick while a barcode was being read: the
    // barcode has been decoded and reported since, so act on it now.
    if (gb_junction_deferred)
    {
        gb_junction_deferred = false;
        if (b_reading)
        {
            junction_seen(now_msec);
        }
    }

    // Crossroads are only looked for once a barcode has been read and its
    // turn is waiting (a barcode always comes before its crossroad). So a
    // barcode's own bars, however wide the print, are never taken for a
    // crossroad, and nor is any mark when no turn is due.
    junction = line_junction_detect(
        barcode_is_black(), (b_reading && (BARCODE_CMD_NONE != g_pending_turn)),
        now_msec, &mark_msec);
    if (((LINE_JUNCTION_FOUND == junction) && (barcode_is_capturing())) &&
        ((barcode_scan_size()) > 1u))
    {
        // A turn is waiting but a new barcode is being read (the last
        // crossroad was missed): this mark is one of its bars.
        log_line("Side sensor: %" PRIu32 " ms black mark is a barcode bar, "
                 "not a crossroad.",
                 mark_msec);
    }
    else if (LINE_JUNCTION_FOUND == junction)
    {
        log_line("Side sensor: %" PRIu32 " ms black mark -> crossroad.",
                 mark_msec);
        if (barcode_end_scan())
        {
            gb_junction_deferred = true; // Decode first, then turn.
        }
        else
        {
            junction_seen(now_msec);
        }
    }
    else if (LINE_JUNCTION_TOO_LONG == junction)
    {
        log_line("Side sensor: %" PRIu32 " ms black mark, too long for a "
                 "crossroad branch: ignored.",
                 mark_msec);
    }
    else
    {
        // No mark ended this tick.
    }

    switch (g_state)
    {
        case VEHICLE_OBSTACLE_SCAN:
            motor_set(0, 0);
            obstacle_scan_task(now_msec);
            if (!obstacle_scan_active())
            {
                state_enter((bypass_begin(obstacle_profile(), now_msec, true)
                                 ? VEHICLE_BYPASS
                                 : VEHICLE_BLOCKED),
                            now_msec);
            }
            break;

        case VEHICLE_BYPASS:
        {
            bypass_result_t result = bypass_step(now_msec, range_mm);

            if (BYPASS_DONE == result)
            {
                // Recheck a live follower-valid reading, not just cached
                // recovery ticks.
                line_state_t recovered = line_sensor_read();
                if ((!gb_bypass_dry_run) &&
                    (!((recovered.b_left_black) || (recovered.b_right_black))))
                {
                    log_obstacle(
                        "BYPASS: line lost at FOLLOW handoff; stopping.");
                    state_enter(VEHICLE_STOPPED, now_msec);
                    break;
                }
                if (!gb_bypass_dry_run)
                {
                    gb_feat_line = true;
                    line_filter_reset(recovered, now_msec);
                    g_line_last =
                        (recovered.b_left_black) && (recovered.b_right_black)
                            ? LINE_LAST_BOTH
                        : recovered.b_left_black ? LINE_LAST_LEFT
                                                 : LINE_LAST_RIGHT;
                    if (g_line_last != LINE_LAST_BOTH)
                    {
                        gb_last_left = recovered.b_left_black;
                    }
                }
                state_enter(gb_bypass_dry_run ? VEHICLE_STOPPED
                                              : VEHICLE_FOLLOW_LINE,
                            now_msec);
            }
            else if (BYPASS_FAILED == result)
            {
                log_obstacle("BYPASS recovery failed: STOPPED; line/heading or "
                             "safety not confirmed.");
                state_enter(VEHICLE_STOPPED, now_msec);
            }
            else
            {
                // Still going round.
            }
        }
        break;

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
            barcode_test_step(now_msec, range_mm, tilt_x10);
            break;

        case VEHICLE_BARCODE_STATIC: // Barcodes are read by barcode_task().
        case VEHICLE_READY:
        case VEHICLE_STOPPED:
        case VEHICLE_PICKED_UP:
        default:
            motor_set(0, 0);
            break;
    }
} /* vehicle_task() */

/*!
 * @brief Report the current state.
 *
 * @return The state.
 */
vehicle_state_t
vehicle_state(void)
{
    return (g_state);
} /* vehicle_state() */

/*!
 * @brief Report whether the car may be driving (any state but READY,
 *        STOPPED and BARCODE TEST (STATIC)). The servo is locked then.
 *
 * @return True if the car may be moving.
 */
bool
vehicle_is_moving(void)
{
    return ((!is_parked(g_state)) && (VEHICLE_BARCODE_STATIC != g_state));
} /* vehicle_is_moving() */

/*!
 * @brief Name a state for telemetry.
 *
 * @param[in] state The state.
 *
 * @return A constant string.
 */
char const *
vehicle_state_name(vehicle_state_t state)
{
    char const * p_name = "UNKNOWN";

    switch (state)
    {
        case VEHICLE_OBSTACLE_SCAN:
            p_name = "OBSTACLE_SCAN";
            break;

        case VEHICLE_BYPASS:
            p_name = "BYPASS";
            break;

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

        case VEHICLE_PICKED_UP:
            p_name = "PICKED_UP";
            break;

        case VEHICLE_LINE_CAL:
            p_name = "LINE_CAL";
            break;

        case VEHICLE_TURN:
            p_name = "TURN";
            break;

        case VEHICLE_BARCODE_TEST:
            p_name = "FEATURE_TEST";
            break;

        case VEHICLE_BARCODE_STATIC:
            p_name = "BARCODE_STATIC";
            break;

        default:
            p_name = "UNKNOWN";
            break;
    }

    return (p_name);
} /* vehicle_state_name() */

/*!
 * @brief Report the maximum positive pitch seen during the latest hump.
 */
int32_t
vehicle_hump_peak_tilt_x10(void)
{
    return (g_hump_peak_tilt_x10);
} /* vehicle_hump_peak_tilt_x10() */

/*!
 * @brief FOLLOW_LINE: steer along the line; react to humps and obstacles.
 *
 * @param[in] now_msec  The current time.
 * @param[in] range_mm  The forward range.
 * @param[in] tilt_x10 The larger-magnitude pitch or roll, in tenths degree.
 */
static void
follow_line_step(uint32_t now_msec, uint32_t range_mm, int32_t tilt_x10)
{
    bool b_range_used = (((int32_t)(now_msec - g_ignore_until)) >= 0);

    if (hump_tilt_held(now_msec, tilt_x10))
    {
        state_enter(VEHICLE_HUMP, now_msec);
    }
    else if (b_range_used && (range_mm <= OBSTACLE_STOP_MM))
    {
        if (OBSTACLE_STOP_AT_ONCE)
        {
            log_obstacle("Obstacle at %" PRIu32 " mm -> stopped.", range_mm);
            state_enter(VEHICLE_BLOCKED, now_msec);
        }
        else
        {
            state_enter(VEHICLE_PROBE, now_msec);
        }
    }
    else if (barcode_is_capturing())
    {
        scan_hold_line();  // Reading a barcode: straight.
        path_reset(false); // Not following: no way back.
    }
    else if (b_range_used && (range_mm <= OBSTACLE_SLOW_MM))
    {
        line_steer(SPEED_CREEP, now_msec);
    }
    else
    {
        line_steer(cruise_speed(now_msec), now_msec);
    }
} /* follow_line_step() */

/*!
 * @brief While a barcode is read: drive strictly straight at SPEED_SCAN
 *        with both line sensors held on the line.
 *
 * @par
 * Both sensors on black: both wheels at SPEED_SCAN. One sensor slipped
 * off: a small trim towards the line: the far-side wheel runs
 * SCAN_TRIM_EXTRA faster, the other stays at SPEED_SCAN (never slower:
 * below SPEED_SCAN the motors stall). Both off: keep trimming towards the
 * side the line was last seen. Never the line-following arcs, hard turns
 * or reversing.
 */
static void
scan_hold_line(void)
{
    line_state_t line  = line_filtered(); // Keeps the filter up to date.
    int32_t      fast  = SPEED_SCAN + SCAN_TRIM_EXTRA;
    int32_t      slow  = SPEED_SCAN;
    int32_t      left  = SPEED_SCAN;
    int32_t      right = SPEED_SCAN;

    if ((line.b_left_black) && (line.b_right_black))
    {
        g_line_last = LINE_LAST_BOTH; // Centred: straight.
    }
    else if (line.b_left_black)
    {
        gb_last_left = true;
        g_line_last  = LINE_LAST_LEFT;
        left         = slow; // Line is to the left.
        right        = fast;
    }
    else if (line.b_right_black)
    {
        gb_last_left = false;
        g_line_last  = LINE_LAST_RIGHT;
        left         = fast; // Line is to the right.
        right        = slow;
    }
    else if (LINE_LAST_LEFT == g_line_last)
    {
        left  = slow;
        right = fast;
    }
    else if (LINE_LAST_RIGHT == g_line_last)
    {
        left  = fast;
        right = slow;
    }
    else
    {
        // Line between the sensors: straight.
    }

    motor_set(left, right);
} /* scan_hold_line() */

/*!
 * @brief TURN: carry out a barcode command, then follow the line found.
 *
 * @param[in] now_msec The current time.
 */
static void
turn_step(uint32_t now_msec)
{
    line_state_t line    = line_sensor_read();
    bool         b_black = ((line.b_left_black) || (line.b_right_black));
    bool         b_uturn = (BARCODE_CMD_UTURN == g_turn_cmd);
    bool         b_left =
        (b_uturn ? TURN_UTURN_LEFT : (BARCODE_CMD_LEFT == g_turn_cmd));
    uint32_t     max_msec = (b_uturn ? TURN_UTURN_MAX_MSEC : TURN_90_MSEC);
    uint32_t     in_turn  = now_msec - g_pivot_start_msec; // Pivot only.
    line_state_t no_line  = {false, false};

    if (TURN_ADVANCE == g_turn_phase)
    {
        // Line sensors ignored: the crossing line would steer the car.
        if ((now_msec - g_turn_msec) >= JUNCTION_ADVANCE_MSEC)
        {
            turn_phase_enter(TURN_LEAVE, now_msec);
        }
        motor_set(SPEED_SCAN, SPEED_SCAN);
    }
    else if ((TURN_ALIGN != g_turn_phase) && (in_turn > max_msec))
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
        if (b_black && (g_uturn_skip > 0u))
        {
            // U-turn at a crossroad: this is a side branch, not the line
            // behind. Pivot on past it.
            g_uturn_skip--;
            log_line("U-turn: side branch passed; looking for the next "
                     "line.");
            turn_phase_enter(TURN_LEAVE, now_msec);
            g_pivot_start_msec = now_msec - in_turn; // Keep timing.
        }
        else if (b_black)
        {
            gb_turn_left_on = line.b_left_black;
            turn_phase_enter(TURN_ALIGN, now_msec);
        }
        else
        {
            // Still looking.
        }
        pivot(b_left, now_msec);
    }
    else if ((line.b_left_black) && (line.b_right_black))
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
} /* turn_step() */

/*!
 * @brief Start a TURN phase.
 *
 * @param[in] phase    The new phase.
 * @param[in] now_msec The current time.
 */
static void
turn_phase_enter(turn_phase_t phase, uint32_t now_msec)
{
    g_turn_phase = phase;
    g_turn_msec  = now_msec;

    if (TURN_LEAVE == phase)
    {
        g_pivot_start_msec = now_msec; // TURN_90_MSEC counts from here.
    }
} /* turn_phase_enter() */

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
turn_finish(uint32_t now_msec, line_state_t line, line_last_t last,
            char const * p_how)
{
    log_line("%s done (%s, %" PRIu32 " ms).", barcode_command_name(g_turn_cmd),
             p_how, (now_msec - g_entered_msec));

    line_filter_reset(line, now_msec);
    g_line_last  = last;
    gb_last_left = (LINE_LAST_LEFT == last);
    state_enter(g_turn_after, now_msec);
} /* turn_finish() */

/*!
 * @brief PROBE: creep towards the object and decide hump or wall.
 *
 * @param[in] now_msec  The current time.
 * @param[in] range_mm  The forward range.
 * @param[in] tilt_x10 The larger-magnitude pitch or roll, in tenths degree.
 */
static void
probe_step(uint32_t now_msec, uint32_t range_mm, int32_t tilt_x10)
{
    if (hump_tilt_held(now_msec, tilt_x10))
    {
        state_enter(VEHICLE_HUMP, now_msec);
    }
    else if (range_mm > OBSTACLE_CLEAR_MM)
    {
        state_enter(VEHICLE_FOLLOW_LINE, now_msec);
    }
    else if ((range_mm <= OBSTACLE_HARD_STOP_MM) ||
             ((now_msec - g_entered_msec) > OBSTACLE_PROBE_MSEC))
    {
        log_obstacle("Probe: no climb -> WALL, stopped at %" PRIu32 " mm.\n",
                     range_mm);
        state_enter(VEHICLE_BLOCKED, now_msec);
    }
    else
    {
        line_steer(SPEED_CREEP, now_msec);
    }
} /* probe_step() */

/*!
 * @brief Decide whether the car is on a hump: raised (any way) by
 *        HUMP_ENTER_HEIGHT for HUMP_ENTER_MSEC without a break (a jolt
 *        lifts the reading for less), or stalled on a slope: wheels still
 *        for HUMP_STALL_MSEC while tilted by HUMP_STALL_TILT.
 *
 * @param[in] now_msec The current time.
 * @param[in] tilt_x10 The total tilt, in tenths of a degree.
 *
 * @return True once the tilt has lasted (logged with pitch and roll).
 */
static bool
hump_tilt_held(uint32_t now_msec, int32_t tilt_x10)
{
    bool b_held     = false;
    bool b_settling = ((VEHICLE_PROBE != g_state) &&
                       ((now_msec - g_entered_msec) < HUMP_SETTLE_MSEC));

    // Pulling away (or setting off again after a turn or a hump) tilts
    // the reading for a while: no hump is looked for until it settles.
    if (((gb_feat_hump && (!b_settling)) &&
         (tilt_x10 >= HUMP_STALL_TILT_X10)) &&
        ((stall_msec(now_msec)) >= HUMP_STALL_MSEC))
    {
        b_held = true; // Stuck on the ramp: power now (config.h).
        log_obstacle("Stalled on a slope for %" PRIu32 " ms -> HUMP, "
                     "more power.",
                     stall_msec(now_msec));
        log_tenths("  pitch", imu_pitch_x10());
        log_tenths("  roll", imu_roll_x10());
    }
    else if (((!gb_feat_hump) || b_settling) ||
             ((hump_height_x10()) < HUMP_ENTER_HEIGHT_X10))
    {
        gb_tilting = false;
    }
    else if (!gb_tilting)
    {
        gb_tilting  = true;
        g_tilt_msec = now_msec;
    }
    else if ((now_msec - g_tilt_msec) >= HUMP_ENTER_MSEC)
    {
        b_held = true;
        log_obstacle("Raised %" PRId32 ".%" PRId32 " mm for %" PRIu32
                     " ms -> HUMP, climbing.",
                     ((hump_height_x10()) / 10), ((hump_height_x10()) % 10),
                     (now_msec - g_tilt_msec));
        log_tenths("  pitch", imu_pitch_x10());
        log_tenths("  roll", imu_roll_x10());
    }
    else
    {
        // Tilted, but not for long enough yet.
    }

    return (b_held);
} /* hump_tilt_held() */

/*!
 * @brief Measure whether the wheels have (nearly) stopped while driven:
 *        both wheels' encoder edges per HUMP_STALL_WINDOW_MSEC below
 *        HUMP_STALL_EDGES. Call every control tick.
 *
 * @param[in] now_msec The current time.
 */
static void
stall_update(uint32_t now_msec)
{
    if ((now_msec - g_stall_window) >= HUMP_STALL_WINDOW_MSEC)
    {
        uint32_t edges =
            (encoder_count(PIN_ENC_LEFT_A)) + (encoder_count(PIN_ENC_RIGHT_A));
        bool b_drive =
            (((motor_left_speed()) > 0) || ((motor_right_speed()) > 0));
        bool b_still =
            (b_drive && ((edges - g_stall_edges) < HUMP_STALL_EDGES));

        if (b_still && (!gb_stalled))
        {
            g_stall_since = g_stall_window; // Since this window began.
        }
        gb_stalled     = b_still;
        g_stall_edges  = edges;
        g_stall_window = now_msec;
    }
} /* stall_update() */

/*!
 * @brief Report how long the wheels have been stalled while driven.
 *
 * @param[in] now_msec The current time.
 *
 * @return ms; 0 if they are turning (or not driven).
 */
static uint32_t
stall_msec(uint32_t now_msec)
{
    return (gb_stalled ? (now_msec - g_stall_since) : 0u);
} /* stall_msec() */

/*!
 * @brief Log an angle in tenths of a degree as degrees, e.g. "-3.4 deg".
 *
 * @param[in] p_what    The label.
 * @param[in] value_x10 The angle, in tenths of a degree.
 */
static void
log_tenths(char const * p_what, int32_t value_x10)
{
    int32_t size = ((value_x10 < 0) ? -value_x10 : value_x10);

    log_obstacle("%s %s%" PRId32 ".%" PRId32 " deg", p_what,
                 ((value_x10 < 0) ? "-" : ""), (size / 10), (size % 10));
} /* log_tenths() */

/*!
 * @brief HUMP: climb and descend, ignoring the ultrasonic.
 *
 * @param[in] now_msec  The current time.
 * @param[in] tilt_x10 The larger-magnitude pitch or roll, in tenths degree.
 */
static void
hump_step(uint32_t now_msec, int32_t tilt_x10)
{
    int32_t  magnitude = ((tilt_x10 < 0) ? -tilt_x10 : tilt_x10);
    uint32_t in_hump   = now_msec - g_entered_msec;
    bool     b_stalled = ((stall_msec(now_msec)) >= HUMP_STALL_MSEC);

    // Stalled again after "over the top" (a wobble looked like the
    // crest): it is still climbing, so power back up (2026-11-20).
    if (gb_hump_over && b_stalled)
    {
        gb_hump_over = false;
        g_boost_msec = now_msec;
        log_obstacle("Hump: stalled again -> climbing power.");
    }

    // Total tilt (config.h); finish only once it is back near level.
    if ((magnitude >= HUMP_ENTER_PITCH_X10) ||
        ((hump_height_x10()) >= HUMP_ENTER_HEIGHT_X10))
    {
        gb_hump_tilted = true;
        if (magnitude > g_hump_peak_tilt_x10)
        {
            g_hump_peak_tilt_x10 = magnitude;
        }
    }

    // Tilt falling from its peak while the wheels turn: over the top, so
    // ease off (config.h).
    if ((((!gb_hump_over) && gb_hump_tilted) &&
         (0u == (stall_msec(now_msec)))) &&
        ((magnitude + HUMP_TOP_DROP_X10) <= g_hump_peak_tilt_x10))
    {
        gb_hump_over = true;
        g_top_msec   = now_msec;
        g_top_speed  = g_hump_speed;
        log_obstacle("Hump: over the top after %" PRIu32 " ms -> easing "
                     "back to cruise.",
                     in_hump);
    }

    if ((!gb_hump_tilted) || (magnitude > HUMP_LEVEL_PITCH_X10))
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

    if (((gb_hump_tilted && (!b_stalled)) && (in_hump > HUMP_MIN_MSEC)) &&
        ((gb_level && ((now_msec - g_level_msec) > HUMP_LEVEL_MSEC)) ||
         (((hump_on_mm()) >= ((int32_t)HUMP_PASS_MM)) &&
          (magnitude < HUMP_ENTER_PITCH_X10))))
    {
        hump_finish(g_hump_peak_tilt_x10);
        log_obstacle("Hump cleared after %" PRIu32 " ms, %" PRId32 " mm "
                     "driven: peak height %" PRId32 ".%" PRId32 " mm, "
                     "peak tilt %" PRId32 ".%" PRId32 " deg.",
                     in_hump, hump_on_mm(), ((hump_last_height_x10()) / 10),
                     ((hump_last_height_x10()) % 10),
                     (g_hump_peak_tilt_x10 / 10), (g_hump_peak_tilt_x10 % 10));
        log_obstacle("Highest this run: %" PRId32 ".%" PRId32 " mm (%" PRIu32
                     " humps).\n",
                     ((hump_run_height_x10()) / 10),
                     ((hump_run_height_x10()) % 10), hump_count());
        g_ignore_until  = now_msec + HUMP_IGNORE_RANGE_MSEC;
        g_hump_off_msec = now_msec; // Slow back down gently from here.
        g_exit_speed    = g_hump_speed;
        state_enter((gb_feat_line ? VEHICLE_FOLLOW_LINE : VEHICLE_BARCODE_TEST),
                    now_msec);
    }
    else if ((in_hump > HUMP_MAX_MSEC) ||
             ((g_hump_speed >= SPEED_HUMP_MAX) &&
              ((stall_msec(now_msec)) >= HUMP_GIVE_UP_MSEC)))
    {
        log_obstacle("Hump: still stuck at power %" PRId32 " after %" PRIu32
                     " ms -> BLOCKED.\n",
                     g_hump_speed, in_hump);
        state_enter(VEHICLE_BLOCKED, now_msec);
    }
    else if (b_stalled)
    {
        int32_t speed = hump_speed(now_msec, in_hump);

        // Stuck: push with both wheels, no steering, until they turn (a
        // hard arc left one wheel at 1% on the slope).
        motor_set(speed, speed);
    }
    else if (gb_feat_line)
    {
        line_steer(hump_speed(now_msec, in_hump), now_msec);
    }
    else
    {
        int32_t speed = hump_speed(now_msec, in_hump);

        motor_set(speed, speed); // Feature test: straight over.
    }
} /* hump_step() */

/*!
 * @brief The speed on a hump: up from SPEED_CRUISE to SPEED_HUMP over
 *        HUMP_RAMP_UP_MSEC for the climb, more while still stalled (up to
 *        SPEED_HUMP_MAX); once over the top, back down to SPEED_CRUISE
 *        over HUMP_RAMP_MSEC (config.h): the far side is downhill.
 *
 * @param[in] now_msec     The current time.
 * @param[in] in_hump_msec How long the car has been in HUMP.
 *
 * @return The motor power, in %.
 */
static int32_t
hump_speed(uint32_t now_msec, uint32_t in_hump_msec)
{
    if (gb_hump_over)
    {
        uint32_t span = now_msec - g_top_msec;
        uint32_t done = ((span < HUMP_RAMP_MSEC) ? span : HUMP_RAMP_MSEC);

        g_hump_speed =
            g_top_speed - (((g_top_speed - SPEED_CRUISE) * ((int32_t)done)) /
                           ((int32_t)HUMP_RAMP_MSEC));
    }
    else
    {
        uint32_t done =
            ((in_hump_msec < HUMP_RAMP_UP_MSEC) ? in_hump_msec
                                                : HUMP_RAMP_UP_MSEC);

        // Still not moving: more power, a step at a time (config.h).
        if ((((stall_msec(now_msec)) >= HUMP_STALL_MSEC) &&
             ((now_msec - g_boost_msec) >= HUMP_STALL_MSEC)) &&
            ((SPEED_HUMP + g_hump_extra) < SPEED_HUMP_MAX))
        {
            g_boost_msec = now_msec;
            g_hump_extra += HUMP_STALL_STEP;
            log_obstacle("Hump: still stalled -> power %" PRId32 ".",
                         SPEED_HUMP + g_hump_extra);
        }

        g_hump_speed =
            (SPEED_CRUISE + (((SPEED_HUMP - SPEED_CRUISE) * ((int32_t)done)) /
                             ((int32_t)HUMP_RAMP_UP_MSEC))) +
            g_hump_extra;
        if (g_hump_speed > SPEED_HUMP_MAX)
        {
            g_hump_speed = SPEED_HUMP_MAX;
        }
    }

    return (g_hump_speed);
} /* hump_speed() */

/*!
 * @brief The cruising speed: SPEED_CRUISE, except just after a hump left
 *        above it, when it comes down from that speed over
 *        HUMP_RAMP_MSEC. Dropping straight from 12 to 9 jerked the car
 *        (2026-11-19).
 *
 * @param[in] now_msec The current time.
 *
 * @return The motor power, in %.
 */
static int32_t
cruise_speed(uint32_t now_msec)
{
    uint32_t since = now_msec - g_hump_off_msec;
    int32_t  speed = SPEED_CRUISE;

    if (((0u != g_hump_off_msec) && (since < HUMP_RAMP_MSEC)) &&
        (g_exit_speed > SPEED_CRUISE))
    {
        speed =
            g_exit_speed - (((g_exit_speed - SPEED_CRUISE) * ((int32_t)since)) /
                            ((int32_t)HUMP_RAMP_MSEC));
    }

    return (speed);
} /* cruise_speed() */

/*!
 * @brief Report whether a state is a parked one: braked, waiting for
 *        START (READY, STOPPED, PICKED_UP).
 *
 * @param[in] state The state.
 *
 * @return True if parked.
 */
static bool
is_parked(vehicle_state_t state)
{
    return (((VEHICLE_READY == state) || (VEHICLE_STOPPED == state)) ||
            (VEHICLE_PICKED_UP == state));
} /* is_parked() */

/*!
 * @brief Decide whether the car has been picked up: all three IR sensors
 *        (left, right, barcode) black for PICKUP_FLOORLESS_MSEC without a
 *        break. Lifted, they see no floor (config.h).
 *
 * @param[in] now_msec The current time.
 *
 * @return True once the floor has been missing long enough.
 */
static bool
picked_up(uint32_t now_msec)
{
    line_state_t line = line_sensor_read();
    bool         b_none =
        (((line.b_left_black) && (line.b_right_black)) && (barcode_is_black()));
    bool b_found = false;

    if (!b_none)
    {
        gb_floorless = false;
    }
    else if (!gb_floorless)
    {
        gb_floorless     = true;
        g_floorless_msec = now_msec;
    }
    else
    {
        b_found = ((now_msec - g_floorless_msec) >= PICKUP_FLOORLESS_MSEC);
    }

    return (b_found);
} /* picked_up() */

/*!
 * @brief BLOCKED: stay braked until the way is clear for a while.
 *
 * @param[in] now_msec The current time.
 * @param[in] range_mm The forward range.
 */
static void
blocked_step(uint32_t now_msec, uint32_t range_mm)
{
    motor_set(0, 0);

    if ((ULTRASONIC_NO_RANGE == range_mm) || (range_mm <= OBSTACLE_CLEAR_MM))
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
        log_obstacle("Obstacle cleared -> resuming.\n");
        state_enter((gb_feat_line ? VEHICLE_FOLLOW_LINE : VEHICLE_BARCODE_TEST),
                    now_msec);
    }
    else
    {
        // Clear, but not for long enough yet.
    }
} /* blocked_step() */

/*!
 * @brief BARCODE TEST: drive straight at the same speeds as START (cruise,
 *        scan speed while a barcode is read, creep near an obstacle); stop
 *        at an obstacle; climb a hump (HUMP, straight). Only the line
 *        sensors are ignored.
 *
 * @param[in] now_msec The current time.
 * @param[in] range_mm The forward range.
 * @param[in] tilt_x10 The larger-magnitude pitch or roll, in tenths degree.
 */
static void
barcode_test_step(uint32_t now_msec, uint32_t range_mm, int32_t tilt_x10)
{
    bool b_range_used = (((int32_t)(now_msec - g_ignore_until)) >= 0);

    if (hump_tilt_held(now_msec, tilt_x10))
    {
        state_enter(VEHICLE_HUMP, now_msec);
    }
    else if (!b_range_used)
    {
        int32_t speed = cruise_speed(now_msec);

        motor_set(speed, speed); // Just off a hump.
    }
    else if (range_mm <= OBSTACLE_STOP_MM)
    {
        log_obstacle("FEATURE TEST: obstacle at %" PRIu32 " mm -> stopped.",
                     range_mm);
        state_enter(VEHICLE_STOPPED, now_msec);
    }
    else if (barcode_is_capturing())
    {
        motor_set(SPEED_SCAN, SPEED_SCAN); // As START while reading.
    }
    else if (range_mm <= OBSTACLE_SLOW_MM)
    {
        motor_set(SPEED_CREEP, SPEED_CREEP);
    }
    else
    {
        int32_t speed = cruise_speed(now_msec);

        motor_set(speed, speed);
    }
} /* barcode_test_step() */

/*!
 * @brief Change state, set up the new state's timers and barcode reading.
 *
 * @param[in] state    The new state.
 * @param[in] now_msec The current time.
 */
static void
state_enter(vehicle_state_t state, uint32_t now_msec)
{
    if (((VEHICLE_OBSTACLE_SCAN == g_state) &&
         (VEHICLE_OBSTACLE_SCAN != state)) &&
        (obstacle_scan_active()))
    {
        obstacle_scan_cancel();
    }
    if ((VEHICLE_LINE_CAL == state) || (VEHICLE_BARCODE_TEST == state))
    {
        obstacle_scan_cancel(); // New mission: discard the previous profile.
    }
    if ((VEHICLE_BYPASS == g_state) && (VEHICLE_BYPASS != state))
    {
        bypass_cancel();
    }
    if (state != g_state)
    {
        log_state("State: %s -> %s", vehicle_state_name(g_state),
                  vehicle_state_name(state));
    }

    if (state != VEHICLE_BYPASS)
    {
        gb_bypass_dry_run = false;
    }
    // After a barcode turn the car is already lined up: no straight start.
    gb_follow_from_bypass =
        ((g_state == VEHICLE_BYPASS) && (state == VEHICLE_FOLLOW_LINE));
    gb_launch      = (VEHICLE_TURN != g_state);
    g_state        = state;
    g_entered_msec = now_msec;
    path_reset(false); // Valid again once on the line.
    gb_level     = false;
    gb_clear     = false;
    gb_tilting   = false; // A tilt must build up afresh.
    gb_floorless = false; // So must a missing floor.
    if (VEHICLE_HUMP == state)
    {
        g_hump_peak_tilt_x10 = 0;
        gb_hump_tilted       = false;
        gb_hump_over         = false;
        g_hump_speed         = SPEED_CRUISE;
        g_hump_extra         = 0;
        g_boost_msec         = now_msec; // First boost a stall later.
    }

    if (VEHICLE_LINE_CAL == state)
    {
        gb_cal_left      = true; // Until a sensor says.
        gb_cal_turned    = false;
        g_cal_black_msec = now_msec;
        gb_pivoting      = false; // First burst starts fresh.
        g_centred_ticks  = 0u;
    }

    if (VEHICLE_TURN == state)
    {
        // Every turn starts from its first phase, not where the last
        // one ended (ALIGN would end it at once, without turning).
        turn_phase_enter((gb_turn_advance ? TURN_ADVANCE : TURN_LEAVE),
                         now_msec);
        // At a + crossroad a U-turn pivots past one side branch first.
        g_uturn_skip = (((BARCODE_CMD_UTURN == g_turn_cmd) && gb_turn_advance)
                            ? TURN_UTURN_SKIP_LINES
                            : 0u);
    }

    // Barcodes are only read while following the line normally, or in
    // either BARCODE TEST.
    barcode_set_hand_mode(VEHICLE_BARCODE_STATIC == state);
    barcode_set_enabled(gb_feat_barcode && (((VEHICLE_FOLLOW_LINE == state) ||
                                             (VEHICLE_BARCODE_TEST == state)) ||
                                            (VEHICLE_BARCODE_STATIC == state)));

    if ((((VEHICLE_FOLLOW_LINE != state) && (VEHICLE_PROBE != state)) &&
         (VEHICLE_HUMP != state)) &&
        (VEHICLE_LINE_CAL != state))
    {
        motor_set(0, 0);
    }
} /* state_enter() */

/*!
 * @brief LINE_CAL: drive forward onto the line, then follow it.
 *
 * @par
 * Decided again every tick, by the two line sensors:
 * - both on black: done (at once if the car has not moved: no
 *   calibration needed); straight on while confirming;
 * - one on black: forward, arcing towards it (cal_arc());
 * - neither: before moving, stop and stay READY; after arcing (overshot),
 *   keep arcing the same way for LINE_CAL_LOST_MSEC, then stop (READY).
 *
 * @param[in] now_msec The current time.
 */
static void
line_cal_step(uint32_t now_msec)
{
    line_state_t line = line_sensor_read();

    if ((line.b_left_black) && (line.b_right_black))
    {
        motor_set(LINE_CAL_SPEED, LINE_CAL_SPEED); // Straight on.
        g_centred_ticks++;

        if ((!gb_cal_turned) || (g_centred_ticks >= LINE_CAL_ALIGN_TICKS))
        {
            log_line("%s", (gb_cal_turned
                                ? "LINE_CAL OK: both sensors on the line -> "
                                  "following."
                                : "LINE_CAL: both sensors already on the "
                                  "line -> skipping calibration."));
            line_filter_reset(line, now_msec);
            g_line_last = LINE_LAST_BOTH;
            state_enter(VEHICLE_FOLLOW_LINE, now_msec);
        }
    }
    else if ((line.b_left_black) || (line.b_right_black))
    {
        bool b_left = line.b_left_black; // Towards the black sensor.

        if ((!gb_cal_turned) || (b_left != gb_cal_left))
        {
            log_line("LINE_CAL: %s sensor on black -> forward, arcing %s.",
                     (b_left ? "left" : "right"), (b_left ? "left" : "right"));
        }

        g_centred_ticks  = 0u;
        g_cal_black_msec = now_msec;
        gb_cal_left      = b_left;
        gb_cal_turned    = true;
        cal_arc(b_left);
    }
    else if (!gb_cal_turned)
    {
        log_line("LINE_CAL: both line sensors on white -> not moving.");
        state_enter(VEHICLE_READY, now_msec);
    }
    else if ((now_msec - g_cal_black_msec) > LINE_CAL_LOST_MSEC)
    {
        log_line("LINE_CAL: line lost for %" PRIu32 " ms -> stopped "
                 "(READY). Put the car by the line and START again.",
                 (now_msec - g_cal_black_msec));
        state_enter(VEHICLE_READY, now_msec);
    }
    else
    {
        g_centred_ticks = 0u;
        cal_arc(gb_cal_left); // Overshot: keep arcing.
    }
} /* line_cal_step() */

/*!
 * @brief LINE_CAL: drive forward in an arc, the outer wheel at
 *        LINE_CAL_SPEED and the inner one at LINE_CAL_INNER_PERCENT of it
 *        (forward: 0 would brake it).
 *
 * @param[in] b_left True to curve left (left wheel slower).
 */
static void
cal_arc(bool b_left)
{
    int32_t inner = (LINE_CAL_SPEED * LINE_CAL_INNER_PERCENT) / PERCENT;

    if (inner < 1)
    {
        inner = 1;
    }
    motor_set((b_left ? inner : LINE_CAL_SPEED),
              (b_left ? LINE_CAL_SPEED : inner));
} /* cal_arc() */

/*!
 * @brief Check, before START or a line feature test moves the car, that a
 *        line sensor is on black. If both are on white, log it, and park
 *        the car in READY.
 *
 * @return True if at least one line sensor is on black.
 */
static bool
line_under_car(void)
{
    line_state_t line    = line_sensor_read();
    bool         b_found = ((line.b_left_black) || (line.b_right_black));

    if (!b_found)
    {
        log_line("START: both line sensors on white -> not moving. Put "
                 "the car on the line, or with one sensor on it.");
        if (VEHICLE_READY != g_state)
        {
            state_enter(VEHICLE_READY, now_msec_get());
        }
    }

    return (b_found);
} /* line_under_car() */

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
pivot(bool b_left, uint32_t now_msec)
{
    bool     b_turn = (VEHICLE_TURN == g_state); // Barcode turn.
    uint32_t off =
        ((VEHICLE_LINE_CAL == g_state)
             ? CAL_PIVOT_OFF_MSEC
             : (b_turn ? TURN_PULSE_OFF_MSEC : PIVOT_PULSE_OFF_MSEC));
    int32_t  speed      = (b_turn ? TURN_PULSE_SPEED : PIVOT_PULSE_SPEED);
    uint32_t period     = PIVOT_PULSE_ON_MSEC + off;
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
        motor_set(0, 0); // Braked pause between bursts.
    }
    else if (b_left)
    {
        motor_set(-speed, speed);
    }
    else
    {
        motor_set(speed, -speed);
    }
} /* pivot() */

/*!
 * @brief Steer along the line at a given speed, never stopping.
 *
 * @par
 * Turns keep both wheels rolling forward, so the car never pauses or
 * pivots to turn. Whenever exactly one sensor is on black, the car arcs
 * gently towards THAT sensor (TURN_OUTER/INNER_PERCENT), at once, whatever
 * it was doing before. Only when both are off the line does it arc hard
 * (TURN_HARD_OUTER/INNER_PERCENT) towards the side the line was last
 * seen on, and the first black either sensor sees then is captured
 * quickly (LINE_LOST_BLACK_CONFIRM_MSEC) so the car turns towards it
 * instead of sweeping past it.
 *
 * @param[in] speed    The base (straight-line) speed, in percent.
 * @param[in] now_msec The current time.
 */
static void
line_steer(int32_t speed, uint32_t now_msec)
{
    line_state_t line     = line_filtered();
    int32_t      outer    = (speed * TURN_OUTER_PERCENT) / PERCENT;
    int32_t      inner    = (speed * TURN_INNER_PERCENT) / PERCENT;
    int32_t      hard_out = (speed * TURN_HARD_OUTER_PERCENT) / PERCENT;
    int32_t      hard_in  = (speed * TURN_HARD_INNER_PERCENT) / PERCENT;
    steer_t      want     = STEER_STRAIGHT;
    bool         b_launch =
        (((!gb_follow_from_bypass) && (VEHICLE_FOLLOW_LINE == g_state)) &&
         ((now_msec - g_entered_msec) < LINE_START_STRAIGHT_MSEC));
    int32_t left  = speed;
    int32_t right = speed;

    // Both sensors lost the line: reverse along the recorded path to where
    // both were last on it (once per loss), then follow on from there.
    if ((((((((LINE_BACKTRACK_ENABLE && (!gb_backtracking)) &&
              (!gb_backtracked)) &&
             gb_path_ok) &&
            (g_path_len > 0u)) &&
           (!b_launch)) &&
          (VEHICLE_FOLLOW_LINE == g_state)) &&
         (!line.b_left_black)) &&
        (!line.b_right_black))
    {
        log_line("Line lost: reversing %" PRIu32 " ms of path to where "
                 "both sensors were on it.",
                 (g_path_len * TICK_USEC) / USEC_PER_MSEC);
        gb_backtracking = true;
        gb_backtracked  = true;
        g_back_msec     = now_msec;
        g_back_pos_x100 = g_path_len * ((uint32_t)PERCENT);
    }

    if (gb_backtracking && (backtrack_step(line, now_msec)))
    {
        return;
    }

    if (b_launch)
    {
        // Just started: get moving first.
    }
    else if ((line.b_left_black) && (line.b_right_black))
    {
        g_line_last = LINE_LAST_BOTH; // Centred: straight on.
    }
    else if (line.b_left_black)
    {
        gb_last_left = true;
        g_line_last  = LINE_LAST_LEFT;
        want         = STEER_LEFT; // Line moved left: arc left.
    }
    else if (line.b_right_black)
    {
        gb_last_left = false;
        g_line_last  = LINE_LAST_RIGHT;
        want         = STEER_RIGHT; // Line moved right: arc right.
    }
    else if (LINE_LAST_LEFT == g_line_last)
    {
        // Neither sensor on the line: turn hard towards the limit the
        // line was last seen at. Never stop, never pivot.
        want = STEER_HARD_LEFT; // Line slid out left.
    }
    else if (LINE_LAST_RIGHT == g_line_last)
    {
        want = STEER_HARD_RIGHT; // Line slid out right.
    }
    else
    {
        // Line is between the sensors.
    }

    if ((want != g_steer) && (b_launch || (steer_allowed(want, now_msec))))
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
} /* line_steer() */

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
 * Stops the moment either sensor sees black (the car then turns towards
 * it), or once the path is used up (the car is back where both were last
 * on the line).
 *
 * @param[in] line     The filtered line state.
 * @param[in] now_msec The current time.
 *
 * @return True if the car reversed (or braked) this tick; false once
 *         it is done.
 */
static bool
backtrack_step(line_state_t line, uint32_t now_msec)
{
    bool     b_reversed = true;
    uint32_t centre     = 0u;
    uint32_t first      = 0u;
    uint32_t last       = 0u;
    int32_t  sum_left   = 0;
    int32_t  sum_right  = 0;
    int32_t  count      = 0;
    int32_t  pct        = (int32_t)LINE_BACKTRACK_SPEED_PERCENT;

    if (((line.b_left_black) || (line.b_right_black)) ||
        (0u == g_back_pos_x100))
    {
        log_line("Reverse done (%s).",
                 ((0u == g_back_pos_x100) ? "back where both were on it"
                                          : "a sensor found black"));
        gb_backtracking = false;
        g_path_len      = 0u;
        g_steer         = STEER_STRAIGHT; // Steer afresh from here.
        g_steer_msec    = now_msec;
        b_reversed      = false;
    }
    else if ((now_msec - g_back_msec) < LINE_BACKTRACK_BRAKE_MSEC)
    {
        motor_set(0, 0); // Stop rolling forward first.
    }
    else
    {
        g_back_pos_x100 =
            ((g_back_pos_x100 > LINE_BACKTRACK_SPEED_PERCENT)
                 ? (g_back_pos_x100 - LINE_BACKTRACK_SPEED_PERCENT)
                 : 0u);

        // Average the recorded ticks around the current position.
        centre = g_back_pos_x100 / ((uint32_t)PERCENT);
        first  = ((centre > (LINE_BACKTRACK_SMOOTH_TICKS / 2u))
                      ? (centre - (LINE_BACKTRACK_SMOOTH_TICKS / 2u))
                      : 0u);
        last   = centre + (LINE_BACKTRACK_SMOOTH_TICKS / 2u);
        last   = ((last < g_path_len) ? last : (g_path_len - 1u));

        for (uint32_t idx = first; idx <= last; idx++)
        {
            sum_left += g_path_left[idx];
            sum_right += g_path_right[idx];
            count++;
        }

        motor_set((-(sum_left * pct)) / (count * PERCENT),
                  (-(sum_right * pct)) / (count * PERCENT));
    }

    return (b_reversed);
} /* backtrack_step() */

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
path_record(int32_t left, int32_t right, line_state_t line)
{
    bool const b_on_line =
        ((((line.b_left_black) && (g_raw_line.b_left_black)) &&
          (line.b_right_black)) &&
         (g_raw_line.b_right_black));

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
        g_path_left[g_path_len]  = (int8_t)left;
        g_path_right[g_path_len] = (int8_t)right;
        g_path_len++;
    }
    else
    {
        gb_path_ok = false; // Too long: no way back.
    }
} /* path_record() */

/*!
 * @brief Start a new, empty backtrack path.
 *
 * @param[in] b_ok True if the car is on the line now, so a path recorded
 *                 from here leads back to it.
 */
static void
path_reset(bool b_ok)
{
    g_path_len      = 0u;
    gb_path_ok      = b_ok;
    gb_backtracking = false;

    if (b_ok)
    {
        gb_backtracked = false; // Line found: next loss may reverse.
    }
} /* path_reset() */

/*!
 * @brief Decide whether the steering may change now.
 *
 * @par
 * Turning towards a sensor on black is never delayed, whichever way the
 * car was turning before. Only straightening up waits until the last
 * change has been held for LINE_STEER_HOLD_MSEC, so dirt cannot make the
 * car flip between turning and straight.
 *
 * @param[in] want     The steering the sensors ask for.
 * @param[in] now_msec The current time.
 *
 * @return True if the steering may change to want.
 */
static bool
steer_allowed(steer_t want, uint32_t now_msec)
{
    return ((STEER_STRAIGHT != want) ||
            ((now_msec - g_steer_msec) >= LINE_STEER_HOLD_MSEC));
} /* steer_allowed() */

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
 * black can confirm.
 *
 * @par Line lost
 * While BOTH sensors are white, black needs only
 * LINE_LOST_BLACK_CONFIRM_MSEC, so a sensor sweeping across the line is
 * captured at once. Call once per control tick (line_steer() does).
 *
 * @return The line state the steering should act on.
 */
static line_state_t
line_filtered(void)
{
    line_state_t   raw         = line_sensor_read();
    bool const     b_left_was  = g_line.b_left_black;
    bool const     b_right_was = g_line.b_right_black;
    uint32_t const confirm =
        ((b_left_was || b_right_was) ? LINE_SCORE_CONFIRM : LINE_SCORE_CAPTURE);

    g_raw_line = raw;

    g_line.b_left_black =
        line_score_step(&g_left_score, raw.b_left_black, b_left_was, confirm);
    g_line.b_right_black = line_score_step(&g_right_score, raw.b_right_black,
                                           b_right_was, confirm);

    return (g_line);
} /* line_filtered() */

/*!
 * @brief Update one sensor's filter score with one reading.
 *
 * @param[in,out] p_score     The sensor's score, in ticks.
 * @param[in]     b_raw_black This tick's raw reading.
 * @param[in]     b_black     The sensor's filtered state so far.
 * @param[in]     confirm     Score needed to turn black, in ticks.
 *
 * @return The sensor's new filtered state.
 */
static bool
line_score_step(uint32_t * p_score, bool b_raw_black, bool b_black,
                uint32_t confirm)
{
    uint32_t const limit  = (b_black ? LINE_SCORE_FULL : confirm);
    uint32_t const weight = (b_black ? 1u : LINE_GREY_WHITE_WEIGHT);

    if (b_raw_black)
    {
        if ((*p_score) < limit)
        {
            (*p_score)++;
        }
    }
    else if ((*p_score) > weight)
    {
        *p_score -= weight;
    }
    else
    {
        *p_score = 0u;
    }

    if ((!b_black) && ((*p_score) >= confirm))
    {
        b_black = true;
    }
    else if (0u == (*p_score))
    {
        b_black = false;
    }
    else
    {
        // Between the thresholds: keep the last state.
    }

    return (b_black);
} /* line_score_step() */

/*!
 * @brief Set the filtered line state outright, e.g. after lining up on
 *        the line, and restart the steering hold.
 *
 * @param[in] line     The line state to start from.
 * @param[in] now_msec The current time.
 */
static void
line_filter_reset(line_state_t line, uint32_t now_msec)
{
    g_line        = line;
    g_left_score  = (line.b_left_black ? LINE_SCORE_FULL : 0u);
    g_right_score = (line.b_right_black ? LINE_SCORE_FULL : 0u);
    g_steer       = STEER_STRAIGHT;
    g_steer_msec  = now_msec;
} /* line_filter_reset() */

/*!
 * @brief Read the time in milliseconds since boot.
 *
 * @return Milliseconds; wraps after about 49 days.
 */
static uint32_t
now_msec_get(void)
{
    // NOTE: Truncating to 32 bits is intended; all time differences use
    // unsigned subtraction, which is correct across the wrap.
    return ((uint32_t)((time_us_64()) / USEC_PER_MSEC));
} /* now_msec_get() */

/*** end of file ***/

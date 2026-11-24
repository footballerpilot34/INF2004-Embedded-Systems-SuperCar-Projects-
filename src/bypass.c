/** @file bypass.c
 *
 * @brief Bypass and line recovery using existing wheel/encoder controls.
 *
 * Default: controlled fixed-width test. Scan normally, pivot left, travel
 * diagonally to the configured centre offset, pivot forward, clear the assumed
 * 100 mm depth and whole vehicle, then pivot right and reacquire the line. Empty corridor is an
 * operator test assumption, never inferred from missing echoes. The USB/MQTT
 * dry run rehearses the route without manufacturing a scan result.
 *
 * Coordinates: z forward along the original line, x right, heading right
 * positive. Existing encoder calibration supplies distance and pivot angle;
 * steering holds a heading using the existing wheel-speed correction.
 * STOP, deadlines, close echoes and failed line acquisition brake the car.
 *
 * The previous scan-based curve/stack controller is retained for
 * BYPASS_FIXED_WIDTH_TEST=false. Its route verification still applies there.
 */

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "bypass.h"
#include "config.h"
#include "bypass_geometry.h"
#include "encoder.h"
#include "line_sensor.h"
#include "log.h"
#include "motor.h"
#include "servo.h"
#include "ultrasonic.h"


#define DEG_PER_RAD         57.29578f
#define SIDE_SAMPLE_MSEC    100u    // One sideways reading per this.
#define SIDE_NEAR_EXTRA_MM  150u    // Echo nearer than lateral + this =
                                    // the obstacle is still alongside.
#define ALIGN_MIN_DEG       15.0f   // Turn this far before trusting "both
                                    // on the line" (the line just found).
#define ALIGN_EXTRA_DEG     20.0f   // Align gives up this far past straight.
#define PIVOT_DONE_DEG      1.5f    // Pivot within this of the target: done.
#define STRAIGHT_DEG        3.0f    // Within this of the line: straight.
#define MIN_CURVE_MM        60.0f   // A curve never shorter than this.
_Static_assert(BYPASS_AXLE_TO_FRONT_MM <= CAR_LENGTH_MM, "Axle must lie inside chassis");
_Static_assert(BYPASS_RETURN_MIN_DEG > BYPASS_ALIGN_TOL_DEG && BYPASS_RETURN_MAX_DEG < 90,
               "Return must make finite forward and lateral progress");
_Static_assert(BYPASS_RETURN_MIN_DEG <= BYPASS_RETURN_MAX_DEG &&
               BYPASS_RETURN_MIN_DEG > BYPASS_DRIVE_HEADING_TOL_DEG,
               "Return search and tracking corridor must be finite");
_Static_assert(BYPASS_DEPARTURE_MIN_DEG > BYPASS_DRIVE_HEADING_TOL_DEG &&
               BYPASS_DEPARTURE_MIN_DEG <= BYPASS_DEPARTURE_MAX_DEG &&
               BYPASS_DEPARTURE_MAX_DEG + BYPASS_DRIVE_HEADING_TOL_DEG < 90,
               "Departure must remain forward and within heading bounds");
_Static_assert(BYPASS_GEOMETRY_STEP_MM > 0 && BYPASS_GEOMETRY_STEP_DEG > 0,
               "Geometry sampling must advance");
_Static_assert(BYPASS_PIVOT_LEFT_X100 > 0 && BYPASS_PIVOT_RIGHT_X100 > 0,
               "Pivot calibration must be positive");
#define NO_SIDE             UINT32_MAX  // side_shift(): unusable.
// Reserve the corner radius, including the length, while turning.
#define SIDE_CLEARANCE_MM   ((uint32_t)ceilf(hypotf(CAR_LENGTH_MM / 2.0f, CAR_WIDTH_MM / 2.0f)) + BYPASS_SIDE_MARGIN_MM)
// Tightest turn the steering makes: outer and inner wheel at base +/- the
// largest difference (inner never below BYPASS_WHEEL_MIN).
#define STEER_OUTER         ((float) (BYPASS_DRIVE_SPEED + BYPASS_STEER_MAX))
#define STEER_INNER_RAW     (BYPASS_DRIVE_SPEED - BYPASS_STEER_MAX)
#define STEER_INNER         ((float) ((STEER_INNER_RAW < BYPASS_WHEEL_MIN) ? \
                                      BYPASS_WHEEL_MIN : STEER_INNER_RAW))
#define TURN_RADIUS_MM      ((((float) WHEEL_TRACK_MM / 2.0f) *          \
                              (STEER_OUTER + STEER_INNER)) /            \
                             (STEER_OUTER - STEER_INNER))
#define HEADING_RUNAWAY_DEG 20      // Past the heading limit by this: the
                                    // steering is fighting itself.

typedef enum
{
    PHASE_IDLE,
    PHASE_PIVOT,        // Turning on the spot to g_target_hdg, then g_after.
    PHASE_OUT,          // Curving out (pushed).
    PHASE_RESCAN,       // Braked, half-sweep towards the side.
    PHASE_PASS,         // Straightening (pushed), then straight past it.
    PHASE_RETURN,       // Steering the popped moves, mirrored.
    PHASE_ADVANCE,      // Line found: axle over it.
    PHASE_ALIGN,        // Turning straight onto the line.
    PHASE_FINISH,       // Stack empty and straight: done.
    PHASE_BRAKE,        // Settle translation before interpreting counts as a pivot.
    PHASE_PLAN,         // Validate the fixed route while braked.
    PHASE_BACKUP        // Reversing straight: too close to turn.
} phase_t;

typedef struct
{
    float    heading_deg;   // Mean direction of the move (from the line).
    uint32_t dist_mm;       // How far.
    float    x_from_mm;     // Sideways position at its start,
    float    x_to_mm;       // and at its end.
} move_t;


static phase_t  g_phase       = PHASE_IDLE;
static phase_t  g_brake_then  = PHASE_IDLE;
static phase_t  g_after       = PHASE_IDLE;  // Phase after a pivot.
static phase_t  g_backup_then = PHASE_IDLE;  // Phase after a back-up.
static float    g_backup_hdg  = 0.0f;   // Heading held reversing.
static uint32_t g_backups     = 0u;     // Back-ups so far.
static bool     gb_ret_hold   = false;  // Return paused: still beside it.
static float    g_hold_end_mm = 0.0f;   // Hold ends here (0: not yet).
static bool     gb_left       = false;  // Going round on the left side.
static bool     gb_use_line   = true;   // Come back onto a line.
static float    g_heading     = 0.0f;   // Degrees from the line, + right.
static int32_t  g_turn_direction = 1;   // Last pivot direction, including braking.
static float    g_target_hdg  = 0.0f;   // Pivot target.
static float    g_x           = 0.0f;   // Position (mm).
static float    g_z           = 0.0f;
static float    g_target_x    = 0.0f;   // Sideways target (signed).
static float    g_front_z     = 0.0f;   // The obstacle's nearest face.
static uint32_t g_clearance   = 0u;     // Edge to car centre, sideways.
static float    g_cx0         = 0.0f;   // Current curve: start x, z,
static float    g_cz0         = 0.0f;
static float    g_clen        = 0.0f;   // and length along z.
static move_t   g_stack[BYPASS_STACK_MAX];
static uint32_t g_sp          = 0u;     // Moves on the stack.
static move_t   g_ret;                  // The move being driven back.
static float    g_ret_done    = 0.0f;   // mm of it driven so far.
static float    g_seg_mm      = 0.0f;   // Move being recorded: mm,
static float    g_seg_hsum    = 0.0f;   // and heading x mm (for the mean),
static float    g_seg_x0      = 0.0f;   // and x where it started.
static float    g_phase_mm    = 0.0f;   // Driven in this phase.
static float    g_pass_hold_x = 0.0f;   // PASS keeps to this x.
static bool     gb_pass_straight = false; // PASS: straightened up yet.
static uint32_t g_phase_msec  = 0u;
static uint32_t g_last_left   = 0u;     // Encoder counts last tick.
static uint32_t g_last_right  = 0u;
static int32_t  g_scan_rel    = 0;      // Re-scan: servo angle (relative).
static int32_t  g_near_rel    = 0;      // Last ray that still saw it.
static uint32_t g_near_mm     = 0u;
static uint32_t g_ahead_mm    = ULTRASONIC_NO_RANGE;  // Re-scan, ahead.
static bool     gb_near_seen  = false;
static uint32_t g_pass_min_mm = 0u;     // Pass at least this far.
static uint32_t g_pass_end_mm = 0u;     // PASS ends at this distance.
static bool     gb_side_seen  = false;  // Obstacle seen alongside.
static uint32_t g_side_clear  = 0u;     // Far echoes in a row.
static uint32_t g_side_next   = 0u;     // Next sideways sample (ms).
static uint32_t g_line_ticks  = 0u;     // Consecutive line observations.
static uint32_t g_side_sequence = 0u;   // Never count a ping twice.
static bool     gb_fixed_test = false;
static float    g_fixed_clear_z;       // Required forward progress before return.
static bypass_geometry_t g_geometry;
static uint32_t g_plan_angle;
static bool gb_plan_reverse;
static float    g_steer_residual;
static bool     gb_brake_reverse;
static bool     gb_odo_moved;          // Count changes, including residual brake motion.
static bool     gb_turn_settling;
static uint32_t g_turn_quiet_msec;
static float    g_line_contact_x;      // Bound sideways movement after finding black.
static float    g_line_advance_hdg;
static uint32_t g_pose_next   = 0u;     // Next pose log (ms).


static bool     fixed_begin(obstacle_profile_t const * profile,
                             uint32_t now_msec, bool use_line);
static bypass_result_t fixed_step(uint32_t now_msec, uint32_t range_mm);
static bool fixed_start(uint32_t front_mm, float left, float right,
                        uint32_t now_msec, bool use_line);
static bool fixed_turn_step(uint32_t now_msec);
static int32_t  beam_corrected_edge(obstacle_profile_t const * p_profile,
                                    bool b_left);
static uint32_t side_shift(obstacle_profile_t const * p_profile,
                           bool b_left);
static bool     explore_start(obstacle_profile_t const * p_profile,
                              uint32_t now_msec, bool b_use_line);
static void     reset_state(bool b_left, bool b_use_line);
static void     curve_start(uint32_t now_msec);
static void     start_pivot(float target_deg, phase_t after,
                            uint32_t now_msec);
static bool     record(bool b_flush);
static void     return_next(void);
static void     return_steer(void);
static void     return_guard(uint32_t now_msec, uint32_t range_mm);
static uint32_t side_near_mm(void);
static bool     start_backup(phase_t then, uint32_t range_mm,
                             uint32_t now_msec);
static void     backup_step(uint32_t now_msec, uint32_t range_mm);
static void     trim_stack(float back_mm);
static void     reverse(float heading_deg);
static bypass_result_t out_step(uint32_t now_msec, uint32_t range_mm);
static bypass_result_t rescan_step(uint32_t now_msec, uint32_t range_mm);
static bypass_result_t pass_step(uint32_t now_msec, uint32_t range_mm);
static void     phase_enter(phase_t phase, uint32_t now_msec);
static void     odo_update(void);
static void     steer(float heading_deg);
static void     pivot(bool b_left);
static bool     front_within(uint32_t range_mm, uint32_t limit_mm);
static float    clampf(float value, float limit);


/*!
 * @brief Plan a bypass from the full scan and start it.
 *
 * @param[in] p_profile  The full scan (obstacle_profile()).
 * @param[in] now_msec   The time now.
 * @param[in] b_use_line True to come back onto the line; false to come
 *                       back onto the original path by the stack alone.
 *
 * @return True if started; false (logged) if no side is usable.
 */
bool
bypass_begin (obstacle_profile_t const * p_profile, uint32_t now_msec,
              bool b_use_line)
{
    gb_fixed_test = BYPASS_FIXED_WIDTH_TEST;
    if (gb_fixed_test) { return fixed_begin(p_profile, now_msec, b_use_line); }
    uint32_t left_shift  = side_shift(p_profile, true);
    uint32_t right_shift = side_shift(p_profile, false);
    bool     b_go_left   = (left_shift < right_shift);
    uint32_t shift       = (b_go_left ? left_shift : right_shift);
    bool     b_ok        = false;

    if (!BYPASS_ENABLED)
    {
        log_obstacle("BYPASS off (BYPASS_ENABLED): staying blocked.");
    }
    else if (!p_profile->finished || p_profile->timed_out ||
             p_profile->cancelled)
    {
        log_obstacle("BYPASS: the scan did not finish: staying blocked.");
    }
    else if (SENSOR_TO_BUMPER_MM < 0 || SENSOR_TO_BUMPER_MM > (int)CAR_LENGTH_MM)
    {
        log_obstacle("BYPASS: measure SENSOR_TO_BUMPER_MM first; staying blocked.");
    }
    else if ((NO_SIDE == shift) && BYPASS_EXPLORE_SCAN_LIMIT &&
             ((OBSTACLE_EDGE_SCAN_LIMIT == p_profile->left_status) ||
              (OBSTACLE_EDGE_SCAN_LIMIT == p_profile->right_status)))
    {
        // No side usable as scanned, but the obstacle runs on past the end
        // of the sweep on one side: turn that way and look again.
        b_ok = explore_start(p_profile, now_msec, b_use_line);
    }
    else if (NO_SIDE == shift)
    {
        log_obstacle("BYPASS: no side has a seen edge and a clear way: "
                     "staying blocked.");
    }
    else if (shift > BYPASS_MAX_SHIFT_MM)
    {
        log_obstacle("BYPASS: needs a %" PRIu32 " mm side-step, over the "
                     "%u mm limit: staying blocked.", shift,
                     BYPASS_MAX_SHIFT_MM);
    }
    // The existing smoothstep curve has peak curvature 6*shift/length^2
    // at departure. Reject it if the available room demands a tighter
    // turn than the configured wheel speeds support. Reserve corners and
    // bumper setback BEFORE the front plane; do not reverse into an unseen path.
    else if ((float)p_profile->front_mm <= SENSOR_TO_BUMPER_MM +
                 SIDE_CLEARANCE_MM + sqrtf(6.0f * shift * TURN_RADIUS_MM))
    {
        log_obstacle("BYPASS: insufficient room for existing curve; staying blocked.");
    }
    else
    {
        reset_state(b_go_left, b_use_line);
        shift      = ((shift < BYPASS_MIN_SHIFT_MM) ? BYPASS_MIN_SHIFT_MM
                                                    : shift);
        g_target_x = (gb_left ? -(float) shift : (float) shift);
        g_front_z  = (float) p_profile->front_mm;

        log_obstacle("BYPASS: going round on the %s, side-step %" PRIu32
                     " mm.", (gb_left ? "LEFT" : "RIGHT"), shift);
        servo_set_angle(SERVO_CENTER_DEG);
        curve_start(now_msec);
        b_ok = true;
    }

    return (b_ok);
}   /* bypass_begin() */


/*!
 * @brief Run one step of the bypass. Call every tick while it runs.
 *
 * @param[in] now_msec The time now.
 * @param[in] range_mm The latest ultrasonic range (where the servo aims).
 *
 * @return BYPASS_RUNNING, BYPASS_DONE or BYPASS_FAILED (braked).
 */
bypass_result_t
bypass_step (uint32_t now_msec, uint32_t range_mm)
{
    bypass_result_t result = BYPASS_RUNNING;
    line_state_t    line   = line_sensor_read();
    bool            b_line = (line.b_left_black || line.b_right_black);

    odo_update();       // Position and heading, from this tick's counts.
    if (gb_fixed_test && (gb_turn_settling || g_phase==PHASE_BRAKE) && gb_odo_moved) {
        // Track quiet time even while a fresh ultrasonic sample is pending.
        g_turn_quiet_msec = now_msec;
    }
    bool confirm_line = b_line;
    if (gb_fixed_test && (g_phase == PHASE_ALIGN || g_phase == PHASE_FINISH)) {
        confirm_line = line.b_left_black && line.b_right_black &&
                       fabsf(g_heading) <= BYPASS_ALIGN_TOL_DEG;
    }
    if (confirm_line) { if (g_line_ticks < BYPASS_LINE_CONFIRM_TICKS) { ++g_line_ticks; } }
    else { g_line_ticks = 0u; }

    if (((int32_t) (now_msec - g_pose_next) >= 0) &&
        ((PHASE_OUT == g_phase) || (PHASE_PASS == g_phase) ||
         (PHASE_RETURN == g_phase) || (PHASE_BACKUP == g_phase)))
    {
        g_pose_next = now_msec + BYPASS_POSE_LOG_MSEC;
        log_obstacle("BYPASS pose (%s): x=%ld z=%ld mm, heading %ld deg, "
                     "wheels L%ld R%ld.", bypass_phase_name(),
                     (long) lroundf(g_x), (long) lroundf(g_z),
                     (long) lroundf(g_heading), (long) motor_left_speed(),
                     (long) motor_right_speed());
    }

    float heading_limit=gb_fixed_test ? BYPASS_DEPARTURE_MAX_DEG : BYPASS_HEADING_MAX_DEG;
    if (fabsf(g_heading) > heading_limit+HEADING_RUNAWAY_DEG)
    {
        // Steering one way but turning the other: the left/right sense
        // of the encoders or motors is swapped (BYPASS_ODO_SWAP_LR).
        log_obstacle("BYPASS: heading ran away to %ld deg; left/right "
                     "swapped? (BYPASS_ODO_SWAP_LR): stopping.",
                     (long) lroundf(g_heading));
        result = BYPASS_FAILED;
    }
    else if ((PHASE_RESCAN != g_phase) &&
        ((now_msec - g_phase_msec) > BYPASS_STEP_MAX_MSEC))
    {
        log_obstacle("BYPASS: step %s took over %u ms (stuck?): stopping.",
                     bypass_phase_name(), BYPASS_STEP_MAX_MSEC);
        result = BYPASS_FAILED;
    }
    else if (gb_fixed_test)
    {
        result = fixed_step(now_msec, range_mm);
    }
    else if (!servo_is_settled() || !ultrasonic_measurement_ready())
    {
        motor_set(0, 0); // Wait for a fresh reading at the selected angle.
    }
    else if ((PHASE_OUT == g_phase || PHASE_RETURN == g_phase) &&
             (range_mm == ULTRASONIC_NO_RANGE || range_mm == 0u))
    {
        result = BYPASS_FAILED;
    }
    else if (PHASE_PIVOT == g_phase)
    {
        float error = g_target_hdg - g_heading;

        if (fabsf(error) <= PIVOT_DONE_DEG)
        {
            phase_enter(g_after, now_msec);
        }
        else
        {
            pivot(error < 0.0f);
        }
    }
    else if (PHASE_OUT == g_phase)
    {
        result = out_step(now_msec, range_mm);
    }
    else if (PHASE_RESCAN == g_phase)
    {
        result = rescan_step(now_msec, range_mm);
    }
    else if (PHASE_BACKUP == g_phase)
    {
        backup_step(now_msec, range_mm);
    }
    else if (PHASE_PASS == g_phase)
    {
        result = pass_step(now_msec, range_mm);
    }
    else if (PHASE_RETURN == g_phase)
    {
        return_guard(now_msec, range_mm);   // Still beside it?

        if (gb_use_line && !gb_ret_hold &&
            g_line_ticks >= BYPASS_LINE_CONFIRM_TICKS)
        {
            g_sp = 0u;                      // On the line: forget the rest.
            log_obstacle("BYPASS: line found on the way back; stack "
                         "cleared.");
            phase_enter(PHASE_ADVANCE, now_msec);
        }
        else if (front_within(range_mm, OBSTACLE_HARD_STOP_MM))
        {
            log_obstacle("BYPASS: %" PRIu32 " mm from it while coming "
                         "back: stopping.", range_mm);
            result = BYPASS_FAILED;
        }
        else if (gb_ret_hold)
        {
            steer(0.0f);                    // Still beside it: straight.
        }
        else if (g_ret_done >= (float) g_ret.dist_mm)
        {
            if (0u == g_sp)
            {
                start_pivot(0.0f, PHASE_FINISH, now_msec);
            }
            else
            {
                return_next();              // On to the next, no stop.
                return_steer();
            }
        }
        else
        {
            return_steer();
        }
    }
    else if (PHASE_ADVANCE == g_phase)
    {
        steer(g_heading);                   // Keep the heading.
        if (g_phase_mm >= (float) BYPASS_LINE_ADVANCE_MM)
        {
            g_target_hdg = g_heading;       // Where the align starts.
            phase_enter(PHASE_ALIGN, now_msec);
        }
    }
    else if (PHASE_ALIGN == g_phase)
    {
        float size   = fabsf(g_target_hdg);
        float turned = fabsf(g_heading - g_target_hdg);

        if ((g_line_ticks >= BYPASS_LINE_CONFIRM_TICKS) &&
            ((fabsf(g_heading) <= STRAIGHT_DEG) ||
             ((turned >= ALIGN_MIN_DEG) && line.b_left_black && line.b_right_black)))
        {
            log_obstacle("BYPASS done: back on the line.");
            result = BYPASS_DONE;
        }
        else if (fabsf(g_heading) <= STRAIGHT_DEG && b_line)
        {
            motor_set(0, 0); // Wait for consecutive line confirmation.
        }
        else if (turned >= (size + ALIGN_EXTRA_DEG) ||
                 fabsf(g_heading) <= STRAIGHT_DEG)
        {
            log_obstacle("BYPASS: line not confirmed after alignment; stopping.");
            result = BYPASS_FAILED;
        }
        else
        {
            pivot(g_heading > 0.0f);        // Back towards straight.
        }
    }
    else if (PHASE_FINISH == g_phase)
    {
        motor_set(0, 0);
        if (!gb_use_line || g_line_ticks >= BYPASS_LINE_CONFIRM_TICKS) {
            log_obstacle("BYPASS done: original path confirmed.");
            result = BYPASS_DONE;
        } else if (!b_line) {
            log_obstacle("BYPASS: returned path has no line; stopping.");
            result = BYPASS_FAILED;
        }
    }
    else
    {
        result = BYPASS_FAILED;     // Not started.
    }

    if (result==BYPASS_DONE && gb_use_line &&
        (fabsf(g_heading)>BYPASS_ALIGN_TOL_DEG || !line.b_left_black || !line.b_right_black)) {
        log_obstacle("BYPASS: final heading and BOTH IR sensors not confirmed; stopping.");
        result=BYPASS_FAILED;
    }
    if (BYPASS_RUNNING != result)
    {
        motor_set(0, 0);
        servo_set_angle(SERVO_CENTER_DEG);
        g_phase = PHASE_IDLE;
    }

    return (result);
}   /* bypass_step() */


// Fixed rectangle test, using the existing pivot, steering and encoder feedback.
// The operator, not a missing echo, establishes that the left corridor is empty.
static bool
fixed_begin(obstacle_profile_t const * profile, uint32_t now_msec, bool use_line)
{
    motor_set(0, 0);
    uint32_t count = 0u;
    obstacle_point_t const * points = obstacle_scan_points(&count);
    bool centre_stable = false;
    bool close_echo = false;
    for (uint32_t i = 0u; i < count; ++i) {
        if ((points[i].kind == OBSTACLE_RETURN_RANGE ||
             points[i].kind == OBSTACLE_RETURN_WEAK_RANGE) &&
            points[i].range_mm > 0u && points[i].range_mm <= BYPASS_FRONT_STOP_MM) {
            close_echo = true;
        }
        if (points[i].angle_deg == 0 && points[i].kind == OBSTACLE_RETURN_RANGE &&
            points[i].range_mm > 0u && points[i].range_mm != ULTRASONIC_NO_RANGE) {
            centre_stable = true;
        }
    }
    if (!BYPASS_ENABLED || !profile->finished || profile->timed_out ||
        profile->cancelled || !profile->centre_seen || !centre_stable || close_echo ||
        profile->front_mm == 0u || profile->front_mm > OBSTACLE_CLEAR_MM ||
        SENSOR_TO_BUMPER_MM < 0 || SENSOR_TO_BUMPER_MM > (int)CAR_LENGTH_MM) {
        log_obstacle("BYPASS fixed test: incomplete/implausible forward scan; blocked.");
        return false;
    }
    // Confirmed controlled scenario: 200 mm rectangle centred on the line.
    // Scan assessment remains required, but no unapproved edge-based planner runs.
    return fixed_start(profile->front_mm,-(float)BYPASS_FIXED_WIDTH_MM/2.0f,
                       BYPASS_FIXED_WIDTH_MM/2.0f,now_msec,use_line);
}

// Line-aware rehearsal: no scan/profile is fabricated or published.
bool
bypass_test_begin(uint32_t now_msec)
{
    if (!BYPASS_FIXED_WIDTH_TEST) return false;
    gb_fixed_test=true;
    // v is always a continuous dry run, even if parked on a black mark.
    bool use_line=false;
    log_obstacle("BYPASS rehearsal: continuous dry run; end STOPPED");
    return fixed_start(OBSTACLE_STOP_MM,-(float)BYPASS_FIXED_WIDTH_MM/2.0f,
                       BYPASS_FIXED_WIDTH_MM/2.0f,now_msec,use_line);
}

bool bypass_test_uses_line(void) { return gb_use_line; }

static bool
fixed_start(uint32_t front_mm, float left, float right,
            uint32_t now_msec, bool use_line)
{
    if (!BYPASS_ENABLED || SENSOR_TO_BUMPER_MM < 0 ||
        SENSOR_TO_BUMPER_MM > (int)CAR_LENGTH_MM || left >= right ||
        front_mm <= (uint32_t)SENSOR_TO_BUMPER_MM) { return false; }
    reset_state(true, use_line);
    g_heading=BYPASS_INITIAL_HEADING_DEG;
    log_obstacle("BYPASS: encoder heading; track zero assumes initial -10 deg. No calibrated IMU yaw.");
    log_obstacle("BYPASS controlled test: depth=%u mm; empty LEFT corridor "
                 "assumed by operator, NOT verified by scan.", BYPASS_ASSUMED_DEPTH_MM);
    g_front_z = ((float)front_mm+BYPASS_AXLE_TO_FRONT_MM-SENSOR_TO_BUMPER_MM)*
                cosf(BYPASS_INITIAL_HEADING_DEG/DEG_PER_RAD)-BYPASS_AXLE_TO_FRONT_MM;
    bypass_geometry_init(&g_geometry, g_front_z, left, right);
    if (g_geometry.offset > BYPASS_MAX_PLAN_OFFSET_MM) { return false; }
    g_geometry.offset=BYPASS_COMPACT_OFFSET_MM;
    g_geometry.return_deg=BYPASS_COMPACT_RETURN_DEG;
    g_geometry.return_mm=g_geometry.offset/sinf(g_geometry.return_deg/DEG_PER_RAD);
    g_target_x = -g_geometry.offset;
    g_fixed_clear_z = g_geometry.clear_y;
    // Prefer the first safe forward-only angle before considering reverse.
    g_plan_angle=BYPASS_DEPARTURE_MIN_DEG;
    gb_plan_reverse=false;
    phase_enter(PHASE_PLAN, now_msec);
    return true;
}

// Brake on reaching/crossing the target instead of reversing and repeatedly
// retriggering the motor's startup boost. Pulse only near the target.
static bool
fixed_turn_step(uint32_t now_msec)
{
    float error = g_target_hdg - g_heading;
    if (gb_turn_settling || g_turn_direction * error <= PIVOT_DONE_DEG) {
        motor_set(0, 0);
        if (!gb_turn_settling || gb_odo_moved) {
            g_turn_quiet_msec = now_msec;
        }
        gb_turn_settling = true;
        // Keep the pivot phase/direction while braking so residual encoder
        // ticks cannot be misinterpreted as forward travel in the next leg.
        return now_msec - g_turn_quiet_msec >= BYPASS_TURN_SETTLE_MSEC;
    }
    uint32_t pulse = (now_msec - g_phase_msec) %
                     (PIVOT_PULSE_ON_MSEC + PIVOT_PULSE_OFF_MSEC);
    if (fabsf(error) <= BYPASS_TURN_SLOW_DEG && pulse >= PIVOT_PULSE_ON_MSEC) {
        motor_set(0, 0);
    } else { pivot(g_turn_direction < 0); }
    return false;
}

static bypass_result_t
fixed_step(uint32_t now_msec, uint32_t range_mm)
{
    if (!servo_is_settled() || !ultrasonic_measurement_ready()) {
        motor_set(0, 0);
        return BYPASS_RUNNING; // Outer per-phase deadline still applies.
    }
    // No-echo permits travel only under the declared empty-corridor test
    // assumption. Any actual close echo stops; zero is an invalid range.
    if (range_mm == 0u || front_within(range_mm, BYPASS_FRONT_STOP_MM)) {
        return BYPASS_FAILED;
    }
    if (g_phase == PHASE_PLAN) {
        motor_set(0, 0);
        if (bypass_geometry_candidate(&g_geometry,(float)g_plan_angle)) {
            log_obstacle("BYPASS plan: offset=%ld mm, departure=%ld deg/%ld mm, "
                         "pass=%ld mm, return=%ld deg/%ld mm, path=%ld mm.",
                         (long)lroundf(g_geometry.offset), (long)lroundf(g_geometry.departure),
                         (long)lroundf(g_geometry.departure_mm),
                         (long)lroundf(g_fixed_clear_z-g_geometry.departure_y),
                         (long)lroundf(g_geometry.return_deg), (long)lroundf(g_geometry.return_mm),
                         (long)lroundf(g_geometry.translation_mm));
            if(g_geometry.reverse_mm>0) {
                log_obstacle("BYPASS reverse=%ld mm; rear corridor must be clear.",
                             (long)lroundf(g_geometry.reverse_mm));
                g_backup_hdg=g_heading;
                phase_enter(PHASE_BACKUP,now_msec);
            } else {
                log_obstacle("BYPASS forward-only translation; no reverse leg.");
                start_pivot(-g_geometry.departure, PHASE_OUT, now_msec);
            }
        } else if (!gb_plan_reverse) {
            if(g_plan_angle<BYPASS_DEPARTURE_MAX_DEG) ++g_plan_angle;
            else {
                // Every supported forward candidate failed the footprint check.
                gb_plan_reverse=true;
                g_plan_angle=BYPASS_COMPACT_DEPARTURE_DEG;
                g_geometry.reverse_mm=BYPASS_REVERSE_PLAN_STEP_MM;
            }
        } else if (g_geometry.reverse_mm+BYPASS_REVERSE_PLAN_STEP_MM<=BYPASS_REVERSE_PLAN_MAX_MM) {
            g_geometry.reverse_mm+=BYPASS_REVERSE_PLAN_STEP_MM;
        } else {
            log_obstacle("BYPASS: no collision-safe route within bounded reverse allowance.");
            return BYPASS_FAILED;
        }
        return BYPASS_RUNNING;
    }
    if (bypass_geometry_gap(&g_geometry, g_x, g_z, g_heading) < bypass_geometry_margin()) {
        return BYPASS_FAILED;
    }
    if (g_phase == PHASE_BACKUP) {
        if(fabsf(g_heading-g_backup_hdg)>BYPASS_DRIVE_HEADING_TOL_DEG) return BYPASS_FAILED;
        if(g_phase_mm>=g_geometry.reverse_mm) start_pivot(-g_geometry.departure,PHASE_OUT,now_msec);
        else reverse(g_backup_hdg);
    } else if (g_phase == PHASE_BRAKE) {
        motor_set(0, 0);
        if (gb_odo_moved) { g_turn_quiet_msec=now_msec; }
        if (now_msec-g_turn_quiet_msec >= BYPASS_TURN_SETTLE_MSEC) {
            g_turn_direction=(g_target_hdg < g_heading) ? -1 : 1;
            phase_enter(g_brake_then,now_msec);
        }
    } else if (g_phase == PHASE_PIVOT) {
        if (fixed_turn_step(now_msec)) {
            if (fabsf(g_target_hdg - g_heading) > BYPASS_DRIVE_HEADING_TOL_DEG) {
                return BYPASS_FAILED; // Excessive overshoot needs calibration.
            }
            phase_enter(g_after, now_msec);
        }
    } else if (g_phase == PHASE_OUT) {
        if (fabsf(g_heading+g_geometry.departure) > BYPASS_DRIVE_HEADING_TOL_DEG) {
            return BYPASS_FAILED;
        }
        if (g_x <= g_target_x) {
            start_pivot(0.0f, PHASE_PASS, now_msec);
        } else if (g_phase_mm > g_geometry.departure_mm + BYPASS_RETURN_EXTRA_MM) {
            return BYPASS_FAILED;
        } else { steer(-g_geometry.departure); }
    } else if (g_phase == PHASE_PASS) {
        float low=(g_heading-BYPASS_ALIGN_TOL_DEG)/DEG_PER_RAD;
        float high=(g_heading+BYPASS_ALIGN_TOL_DEG)/DEG_PER_RAD;
        float rear=CAR_LENGTH_MM-BYPASS_AXLE_TO_FRONT_MM;
        float q0=CAR_WIDTH_MM/2.0f*cosf(low)+(sinf(low)>=0 ? BYPASS_AXLE_TO_FRONT_MM : -rear)*sinf(low);
        float q1=CAR_WIDTH_MM/2.0f*cosf(high)+(sinf(high)>=0 ? BYPASS_AXLE_TO_FRONT_MM : -rear)*sinf(high);
        if (-g_x+g_geometry.left-fmaxf(q0,q1)-BYPASS_POSITION_UNCERTAINTY_MM < BYPASS_MIN_SIDE_GAP_MM ||
            fabsf(g_heading) > BYPASS_DRIVE_HEADING_TOL_DEG ||
            fabsf(g_x-g_target_x) > BYPASS_LATERAL_TOL_MM ||
            g_phase_mm > BYPASS_PASS_MAX_MM) { return BYPASS_FAILED; }
        if (g_z >= g_fixed_clear_z) {
            start_pivot(g_geometry.return_deg, PHASE_RETURN, now_msec);
        } else { steer(0.0f); }
    } else if (g_phase == PHASE_RETURN) {
        bool on_line = g_line_ticks > 0u;
        if (fabsf(g_heading-g_geometry.return_deg) > BYPASS_DRIVE_HEADING_TOL_DEG) {
            return BYPASS_FAILED;
        }
        if ((now_msec - g_phase_msec) > BYPASS_LINE_TIMEOUT_MSEC ||
            g_phase_mm > fabsf(g_target_x) /
                sinf((g_geometry.return_deg - BYPASS_ALIGN_TOL_DEG) / DEG_PER_RAD) +
                BYPASS_RETURN_EXTRA_MM) {
            return BYPASS_FAILED;
        }
        if (gb_use_line && on_line) {
            motor_set(0, 0); // Brake on first detection; confirm while stopped.
            if (g_line_ticks == 1u) { g_line_contact_x = g_x; }
            if (g_line_ticks >= BYPASS_LINE_CONFIRM_TICKS) {
                g_line_advance_hdg = g_heading;
                phase_enter(PHASE_ADVANCE, now_msec);
            }
        } else if (!gb_use_line && g_x >= 0.0f) {
            start_pivot(0.0f, PHASE_FINISH, now_msec);
        } else { steer(g_geometry.return_deg); }
    } else if (g_phase == PHASE_ADVANCE) {
        // Existing sensor-to-axle placement correction, adjustable on the floor.
        if (g_phase_mm >= BYPASS_LINE_SENSOR_FORWARD_MM ||
            g_x - g_line_contact_x >= BYPASS_LINE_CROSS_MAX_MM) {
            phase_enter(PHASE_ALIGN, now_msec);
        } else { steer(g_line_advance_hdg); }
    } else if (g_phase == PHASE_ALIGN) {
        if ((now_msec - g_phase_msec) > BYPASS_LINE_TIMEOUT_MSEC) {
            return BYPASS_FAILED;
        }
        // Finish and settle the heading correction even if black appears
        // during rotation. Line contact alone never completes the manoeuvre.
        if (fixed_turn_step(now_msec)) {
            if (fabsf(g_heading) > BYPASS_ALIGN_TOL_DEG) { return BYPASS_FAILED; }
            if (g_line_ticks >= BYPASS_LINE_CONFIRM_TICKS) { return BYPASS_DONE; }
            // No aligned pair: remain braked until timeout, never search by
            // rotating past forward or reverse direction to chase the line.
        }
    } else if (g_phase == PHASE_FINISH) {
        motor_set(0, 0);
        if (fabsf(g_heading) > BYPASS_ALIGN_TOL_DEG) { return BYPASS_FAILED; }
        if (!gb_use_line || g_line_ticks >= BYPASS_LINE_CONFIRM_TICKS) {
            return BYPASS_DONE;
        }
        if ((now_msec - g_phase_msec) > BYPASS_LINE_TIMEOUT_MSEC) {
            return BYPASS_FAILED;
        }
    } else { return BYPASS_FAILED; }
    return BYPASS_RUNNING;
}


/*!
 * @brief Abandon a bypass (STOP, or the run ended): brake and recentre.
 */
void
bypass_cancel (void)
{
    if (PHASE_IDLE != g_phase)
    {
        motor_set(0, 0);
        servo_set_angle(SERVO_CENTER_DEG);
        log_obstacle("BYPASS cancelled in %s.", bypass_phase_name());
        g_phase = PHASE_IDLE;
    }
}   /* bypass_cancel() */


/*!
 * @brief Name the current step, for the log and page.
 *
 * @return A constant string.
 */
char const *
bypass_phase_name (void)
{
    static char const * const names[] =
    {
        "idle", "pivot", "out", "re-scan", "pass", "return", "advance",
        "align", "finish", "brake", "plan", "back-up"
    };

    return (names[g_phase]);
}   /* bypass_phase_name() */


/*!
 * @brief The obstacle's edge on one side, corrected for the ultrasonic
 *        beam's width.
 *
 * @par
 * The beam is about 15 deg wide, so it still hears the obstacle when its
 * centre line already points past the edge: scanned edges sit too far
 * out (an 8.5 cm bottle scanned as 19.3 cm wide). The edge's angle (the
 * midpoint of its bracket) is moved inwards by BYPASS_BEAM_CORRECTION_DEG
 * and the edge put at that angle at its forward depth. It is never moved
 * past the obstacle's middle. 0 deg leaves the scanned edge as it is.
 *
 * @param[in] p_profile The scan.
 * @param[in] b_left    True for the left edge.
 *
 * @return The edge's x in mm (+ right).
 */
static int32_t
beam_corrected_edge (obstacle_profile_t const * p_profile, bool b_left)
{
    int32_t raw_x   = (b_left ? p_profile->left_mm : p_profile->right_mm);
    float   angle   = (float) (b_left ? p_profile->left_angle_x10
                                      : p_profile->right_angle_x10) / 10.0f;
    float   forward = (float) (b_left ? p_profile->left_edge.forward_mm
                                      : p_profile->right_edge.forward_mm);
    float   fixed   = angle + (b_left ? (float) BYPASS_BEAM_CORRECTION_DEG
                                      : -(float) BYPASS_BEAM_CORRECTION_DEG);
    int32_t x       = (int32_t) lroundf(forward * tanf(fixed / DEG_PER_RAD));
    int32_t middle  = p_profile->middle_x_mm;

    if ((0 == BYPASS_BEAM_CORRECTION_DEG) || (forward < 1.0f))
    {
        x = raw_x;                          // Off, or no depth to use.
    }
    else if (b_left ? (x > middle) : (x < middle))
    {
        x = middle;                         // Never past its middle.
    }
    else
    {
        // Corrected.
    }

    if (x != raw_x)
    {
        log_obstacle("BYPASS: %s edge %ld mm, beam-corrected to %ld mm.",
                     (b_left ? "left" : "right"), (long) raw_x, (long) x);
    }

    return (x);
}   /* beam_corrected_edge() */


/*!
 * @brief Plan one side from the full scan: the sideways shift that clears
 *        its edge, if the edge was seen and the way past it is clear.
 *
 * @par
 * The edge must have been seen: a confirmed or estimated jump from near
 * to far echoes, or echoes stopping (open space beyond). The way past it
 * is checked: no echo beyond the edge may lie inside the car's corridor
 * (shift plus half the car plus the margin) closer ahead than the car
 * drives while passing. The shift is the edge plus the car's turning
 * clearance (half the car's width plus BYPASS_SIDE_MARGIN_MM).
 *
 * @param[in] p_profile The scan.
 * @param[in] b_left    True for the left side.
 *
 * @return The shift in mm, or NO_SIDE if this side cannot be used.
 */
static uint32_t
side_shift (obstacle_profile_t const * p_profile, bool b_left)
{
    obstacle_edge_status_t status = (b_left ? p_profile->left_status
                                            : p_profile->right_status);
    int32_t  edge_x    = beam_corrected_edge(p_profile, b_left);
    int32_t  far_angle = (b_left ? p_profile->left_edge.far_angle_deg
                                 : p_profile->right_edge.far_angle_deg);
    uint32_t clearance = SIDE_CLEARANCE_MM;
    int32_t  outward   = (b_left ? -edge_x : edge_x);
    uint32_t width = p_profile->calibrated_width_valid ?
                     p_profile->calibrated_width_mm : 250u;
    if (outward < (int32_t)(width / 2u)) { outward = (int32_t)(width / 2u); }
    uint32_t shift     = ((outward > 0) ? (uint32_t) outward : 0u) +
                         clearance;
    float    corridor  = (float) (shift + (CAR_WIDTH_MM / 2u) +
                                  BYPASS_SIDE_MARGIN_MM);
    float    reach     = (float) (p_profile->front_mm +
                                  BYPASS_ASSUMED_DEPTH_MM + CAR_LENGTH_MM);
    uint32_t count     = 0u;
    obstacle_point_t const * p_points = obstacle_scan_points(&count);
    bool     b_bounded = (b_left ? p_profile->left_bounded
                                 : p_profile->right_bounded);
    bool     b_clear   = true;

    // A timeout is not evidence of a free route. Require a measured edge
    // and an angular view reaching the outer side of the reserved corridor.
    if (!b_bounded || status != OBSTACLE_EDGE_CONFIRMED ||
        p_profile->front_mm * tanf(SERVO_RANGE_DEG / DEG_PER_RAD) < corridor) {
        return NO_SIDE;
    }
    uint32_t verified = 0u;
    for (uint32_t idx = 0u; (idx < count) && b_clear; idx++)
    {
        obstacle_point_t const * p_pt = &p_points[idx];
        bool  b_beyond = (b_left ? (p_pt->angle_deg <= far_angle)
                                 : (p_pt->angle_deg >= far_angle));
        float rad      = (float) p_pt->angle_deg / DEG_PER_RAD;
        float px       = fabsf((float) p_pt->range_mm * sinf(rad));
        float py       = (float) p_pt->range_mm * cosf(rad);

        if (b_beyond) {
            if (p_pt->kind != OBSTACLE_RETURN_RANGE || p_pt->range_mm == 0u ||
                p_pt->range_mm == ULTRASONIC_NO_RANGE) { return NO_SIDE; }
            ++verified;
        }
        if (b_beyond && ((OBSTACLE_RETURN_RANGE == p_pt->kind) ||
                         (OBSTACLE_RETURN_WEAK_RANGE == p_pt->kind)) &&
            (px < corridor) && (py < reach))
        {
            log_obstacle("BYPASS: %s way blocked: echo at %ld deg, x=%ld "
                         "y=%ld mm (corridor %ld mm).",
                         (b_left ? "left" : "right"),
                         (long) p_pt->angle_deg, (long) lroundf(px),
                         (long) lroundf(py), (long) lroundf(corridor));
            b_clear = false;
        }
    }

    if (b_clear)
    {
        log_obstacle("BYPASS: %s side usable: side-step %" PRIu32 " mm.",
                     (b_left ? "left" : "right"), shift);
    }

    return (b_clear && verified >= OBSTACLE_SCAN_EDGE_CONFIRM_RAYS ? shift : NO_SIDE);
}   /* side_shift() */


/*!
 * @brief Start a bypass towards a side where the obstacle ran past the end
 *        of the sweep: turn BYPASS_EDGE_TURN_DEG that way and half-sweep.
 *
 * @par
 * The sideways target starts at the furthest point seen plus the turning
 * clearance; rescan_step() moves it out if the edge is further. If both
 * sides ran past the sweep, the side whose last ray was furthest (more
 * likely to end soon) is tried.
 *
 * @param[in] p_profile  The full scan.
 * @param[in] now_msec   The time now.
 * @param[in] b_use_line As bypass_begin().
 *
 * @return True if started; false if the furthest point seen is already
 *         past BYPASS_MAX_SHIFT_MM.
 */
static bool
explore_start (obstacle_profile_t const * p_profile, uint32_t now_msec,
               bool b_use_line)
{
    bool    b_left_lim  = (OBSTACLE_EDGE_SCAN_LIMIT == p_profile->left_status);
    bool    b_right_lim = (OBSTACLE_EDGE_SCAN_LIMIT ==
                           p_profile->right_status);
    bool    b_left      = (b_left_lim &&
                           ((!b_right_lim) ||
                            (p_profile->left_ray_mm >=
                             p_profile->right_ray_mm)));
    int32_t seen_x      = (b_left ? p_profile->left_observed_mm
                                  : p_profile->right_observed_mm);
    bool    b_ok        = false;

    reset_state(b_left, b_use_line);
    g_target_x = (float) seen_x + (gb_left ? -(float) g_clearance
                                           : (float) g_clearance);
    g_front_z  = (float) p_profile->front_mm;

    if (fabsf(g_target_x) > (float) BYPASS_MAX_SHIFT_MM)
    {
        log_obstacle("BYPASS: the %s side runs past %ld mm already: staying "
                     "blocked.", (gb_left ? "left" : "right"),
                     (long) seen_x);
    }
    else
    {
        log_obstacle("BYPASS: the obstacle runs past the %s end of the "
                     "sweep; turning %d deg that way to look again.",
                     (gb_left ? "left" : "right"), BYPASS_EDGE_TURN_DEG);
        start_pivot((float) (gb_left ? -BYPASS_EDGE_TURN_DEG
                                     : BYPASS_EDGE_TURN_DEG),
                    PHASE_RESCAN, now_msec);
        b_ok = true;
    }

    return (b_ok);
}   /* explore_start() */


/*!
 * @brief Clear everything for a new bypass towards one side.
 *
 * @param[in] b_left     Going round on the left.
 * @param[in] b_use_line Come back onto a line.
 */
static void
reset_state (bool b_left, bool b_use_line)
{
    gb_left      = b_left;
    gb_use_line  = b_use_line;
    g_heading    = 0.0f;
    g_x          = 0.0f;
    g_z          = 0.0f;
    g_sp         = 0u;
    g_seg_mm     = 0.0f;
    g_seg_hsum   = 0.0f;
    g_seg_x0     = 0.0f;
    g_backups    = 0u;
    g_line_ticks = 0u;
    g_clearance  = SIDE_CLEARANCE_MM;
    g_last_left  = encoder_count(PIN_ENC_LEFT_A);
    g_last_right = encoder_count(PIN_ENC_RIGHT_A);
}   /* reset_state() */


/*!
 * @brief Plan a curve from where the car is now to the sideways target,
 *        reached BYPASS_CURVE_END_PCT of the way to the obstacle's front
 *        face (PASS then straightens up and drives on), and start it.
 *
 * @param[in] now_msec The time now.
 */
static void
curve_start (uint32_t now_msec)
{
    g_cx0  = g_x;
    g_cz0  = g_z;
    // Reserve the sensor setback and full corner allowance before the face.
    g_clen = g_front_z - g_z - SENSOR_TO_BUMPER_MM - SIDE_CLEARANCE_MM;
    g_clen = ((g_clen < MIN_CURVE_MM) ? MIN_CURVE_MM : g_clen);

    log_obstacle("BYPASS: curve %s: %ld mm sideways over %ld mm ahead.",
                 (gb_left ? "left" : "right"),
                 (long) lroundf(fabsf(g_target_x - g_x)),
                 (long) lroundf(g_clen));
    servo_set_angle(SERVO_CENTER_DEG);
    phase_enter(PHASE_OUT, now_msec);
}   /* curve_start() */


/*!
 * @brief OUT: follow the curve, pushing a move every BYPASS_SEGMENT_MM.
 *
 * @par
 * Along the curve the wanted offset is an S: x0 + dx * (3u^2 - 2u^3)
 * (u = progress along z, 0..1). Its slope 6 * dx * u (1 - u) / length
 * starts at 0, peaks at 1.5 * dx / length halfway, and is 0 again at the
 * end: the car turns gently out and arrives straight. The steering also
 * pulls the car back onto the curve (over BYPASS_LOOKAHEAD_MM). Reaching
 * the target (or the face) ends it. An echo closer than the known face (by
 * BYPASS_RESCAN_MARGIN_MM) is something new: push, then re-scan (backing
 * up first if under BYPASS_BACKUP_TRIGGER_MM). Within the hard stop: back
 * up, then re-scan.
 *
 * @param[in] now_msec The time now.
 * @param[in] range_mm The latest range, straight ahead.
 *
 * @return BYPASS_RUNNING or BYPASS_FAILED.
 */
static bypass_result_t
out_step (uint32_t now_msec, uint32_t range_mm)
{
    bypass_result_t result = BYPASS_RUNNING;
    float dx      = g_target_x - g_cx0;
    float u       = clampf((g_z - g_cz0) / g_clen, 1.0f);
    float want_x  = g_cx0 + (dx * u * u * (3.0f - (2.0f * u)));
    float slope   = (6.0f * dx * u * (1.0f - u)) / g_clen;
    float heading = (atanf(slope) + atanf((want_x - g_x) /
                                          (float) BYPASS_LOOKAHEAD_MM)) *
                    DEG_PER_RAD;
    float face    = g_front_z - g_z;        // Straight-line room to the face.
    // Straightening up from the current heading carries the car further
    // out by about R (1 - cos heading): count that in, so it lands on the
    // side-step instead of overshooting it.
    float drift   = TURN_RADIUS_MM * (1.0f - cosf(g_heading / DEG_PER_RAD));
    bool  b_there = (gb_left ? ((g_x - drift) <= g_target_x)
                             : ((g_x + drift) >= g_target_x));

    u = ((u < 0.0f) ? 0.0f : u);

    if (front_within(range_mm, BYPASS_FRONT_STOP_MM) || !record(false))
    {
        result = BYPASS_FAILED;
    }
    else if (b_there || (u >= 1.0f))
    {
        if (!record(true))
        {
            result = BYPASS_FAILED;
        }
        else
        {
            log_obstacle("BYPASS: out at x=%ld z=%ld mm, heading %ld deg; "
                         "%" PRIu32 " moves.", (long) lroundf(g_x),
                         (long) lroundf(g_z), (long) lroundf(g_heading),
                         g_sp);
            phase_enter(PHASE_PASS, now_msec);
        }
    }
    else if (front_within(range_mm, OBSTACLE_HARD_STOP_MM))
    {
        log_obstacle("BYPASS: %" PRIu32 " mm ahead while going out.",
                     range_mm);
        if (!record(true) ||
            !start_backup(PHASE_RESCAN, range_mm, now_msec))
        {
            result = BYPASS_FAILED;
        }
    }
    else if ((g_phase_mm >= (float) BYPASS_RESCAN_MIN_MM) &&
             (face > (float) BYPASS_RESCAN_MARGIN_MM) &&
             front_within(range_mm, (uint32_t) lroundf(face) -
                                    BYPASS_RESCAN_MARGIN_MM))
    {
        // A slanted beam only reads further than the face, never nearer:
        // a nearer echo is something the scan did not see.
        log_obstacle("BYPASS: %" PRIu32 " mm ahead, nearer than the face "
                     "(%ld mm): re-scanning towards the %s.", range_mm,
                     (long) lroundf(face), (gb_left ? "left" : "right"));
        if (!record(true))
        {
            result = BYPASS_FAILED;
        }
        else if (range_mm < BYPASS_BACKUP_TRIGGER_MM)
        {
            // Too close to curve round whatever it finds: room first.
            if (!start_backup(PHASE_RESCAN, range_mm, now_msec))
            {
                result = BYPASS_FAILED;
            }
        }
        else
        {
            phase_enter(PHASE_RESCAN, now_msec);
        }
    }
    else
    {
        heading = clampf(heading, (float) BYPASS_HEADING_MAX_DEG);
        steer(heading);
    }

    return (result);
}   /* out_step() */


/*!
 * @brief Pivot on the spot to a heading (degrees from the line), then
 *        enter a phase.
 *
 * @param[in] target_deg The heading wanted.
 * @param[in] after      The phase once there.
 * @param[in] now_msec   The time now.
 */
static void
start_pivot (float target_deg, phase_t after, uint32_t now_msec)
{
    g_target_hdg = target_deg;
    g_turn_direction = (target_deg < g_heading) ? -1 : 1;
    g_after      = after;
    if (gb_fixed_test && (g_phase==PHASE_OUT || g_phase==PHASE_PASS ||
                          g_phase==PHASE_RETURN || g_phase==PHASE_ADVANCE || g_phase==PHASE_BACKUP)) {
        g_brake_then=PHASE_PIVOT;
        phase_enter(PHASE_BRAKE,now_msec);
    } else { phase_enter(PHASE_PIVOT, now_msec); }
}   /* start_pivot() */


/*!
 * @brief Record the curve: push the move driven so far (its mean heading
 *        and length) once it reaches BYPASS_SEGMENT_MM, or now (flush).
 *
 * @param[in] b_flush True to push whatever is recorded (end of a curve).
 *
 * @return False if the stack is full (the bypass then fails).
 */
static bool
record (bool b_flush)
{
    bool b_ok = true;

    if ((g_seg_mm >= (float) BYPASS_SEGMENT_MM) ||
        (b_flush && (g_seg_mm >= 1.0f)))
    {
        if (g_sp >= BYPASS_STACK_MAX)
        {
            log_obstacle("BYPASS: more than %u moves needed: stopping.",
                         BYPASS_STACK_MAX);
            b_ok = false;
        }
        else
        {
            g_stack[g_sp].heading_deg = g_seg_hsum / g_seg_mm;
            g_stack[g_sp].dist_mm     = (uint32_t) lroundf(g_seg_mm);
            g_stack[g_sp].x_from_mm   = g_seg_x0;
            g_stack[g_sp].x_to_mm     = g_x;
            g_sp++;
        }
        g_seg_mm   = 0.0f;
        g_seg_hsum = 0.0f;
        g_seg_x0   = g_x;
    }

    return (b_ok);
}   /* record() */


/*!
 * @brief Pop the next move to drive back (mirrored by the caller).
 */
static void
return_next (void)
{
    g_sp--;
    g_ret      = g_stack[g_sp];
    g_ret_done = 0.0f;
}   /* return_next() */


/*!
 * @brief RETURN: steer the popped move mirrored, held on the mirrored path.
 *
 * @par
 * Going out, the move took the car from x_from to x_to; coming back over
 * the same distance it should go from x_to to x_from, at the mirrored
 * heading. As on the way out, the steering also pulls the car onto that
 * path (over BYPASS_LOOKAHEAD_MM), so lag in following the headings does
 * not add up into a sideways error at the end.
 */
static void
return_steer (void)
{
    float frac   = ((g_ret.dist_mm > 0u) ?
                    clampf(g_ret_done / (float) g_ret.dist_mm, 1.0f) : 1.0f);
    float want_x = g_ret.x_to_mm + ((g_ret.x_from_mm - g_ret.x_to_mm) * frac);
    float pull   = atanf((want_x - g_x) / (float) BYPASS_LOOKAHEAD_MM) *
                   DEG_PER_RAD;

    steer(clampf(-g_ret.heading_deg + pull, (float) BYPASS_HEADING_MAX_DEG));
}   /* return_steer() */


/*!
 * @brief The sideways echo (sensor SERVO_RANGE_DEG towards the obstacle)
 *        that still means the obstacle is alongside.
 *
 * @return The limit in mm.
 */
static uint32_t
side_near_mm (void)
{
    float beam = (float) SERVO_RANGE_DEG / DEG_PER_RAD;

    return ((uint32_t) lroundf((float) g_clearance / sinf(beam)) +
            SIDE_NEAR_EXTRA_MM);
}   /* side_near_mm() */


/*!
 * @brief RETURN: watch the obstacle's side; hold straight while it is
 *        still there.
 *
 * @par
 * An echo within side_near_mm() means the obstacle is still beside or
 * ahead of the car on that side: turning in would hit it, so the return
 * pauses (gb_ret_hold) and the car drives straight on. Once
 * BYPASS_SIDE_CLEAR_COUNT far (or no) echoes come in a row, it drives
 * BYPASS_RETURN_GUARD_EXTRA_MM more, then the return carries on where
 * it paused.
 *
 * @param[in] now_msec The time now.
 * @param[in] range_mm The latest range (sideways).
 */
static void
return_guard (uint32_t now_msec, uint32_t range_mm)
{
    if (servo_is_settled() && ultrasonic_measurement_ready() &&
        ((int32_t) (now_msec - g_side_next) >= 0) &&
        ultrasonic_sample_sequence() != g_side_sequence)
    {
        g_side_sequence = ultrasonic_sample_sequence();
        g_side_next = now_msec + SIDE_SAMPLE_MSEC;

        if ((ULTRASONIC_NO_RANGE != range_mm) && (range_mm > 0u) &&
            (range_mm <= side_near_mm()))
        {
            if (!gb_ret_hold)
            {
                log_obstacle("BYPASS: obstacle still beside (%" PRIu32
                             " mm): holding straight.", range_mm);
            }
            gb_ret_hold   = true;
            g_hold_end_mm = 0.0f;
            g_side_clear  = 0u;
        }
        else if (range_mm == ULTRASONIC_NO_RANGE || range_mm == 0u) {
            g_side_clear = 0u;
            if (gb_ret_hold) { g_hold_end_mm = 0.0f; }
        }
        else if (gb_ret_hold && (0.0f == g_hold_end_mm))
        {
            g_side_clear++;
            if (g_side_clear >= BYPASS_SIDE_CLEAR_COUNT)
            {
                g_hold_end_mm = g_phase_mm +
                                (float) BYPASS_RETURN_GUARD_EXTRA_MM;
            }
        }
        else
        {
            // Clear, and not holding (or already counting down).
        }
    }

    if (gb_ret_hold && (g_hold_end_mm > 0.0f) &&
        (g_phase_mm >= g_hold_end_mm))
    {
        gb_ret_hold = false;
        log_obstacle("BYPASS: clear of it at x=%ld z=%ld mm; turning "
                     "back again.", (long) lroundf(g_x),
                     (long) lroundf(g_z));
    }
}   /* return_guard() */


/*!
 * @brief Start reversing straight back, to make room to turn.
 *
 * @param[in] then     The phase once backed up (PHASE_RESCAN, or
 *                     PHASE_OUT to plan the curve from there).
 * @param[in] range_mm How close it is, for the log.
 * @param[in] now_msec The time now.
 *
 * @return False if it has backed up BYPASS_BACKUP_MAX_COUNT times
 *         already (the bypass then fails).
 */
static bool
start_backup (phase_t then, uint32_t range_mm, uint32_t now_msec)
{
    (void)then;
    (void)range_mm;
    (void)now_msec;
    log_obstacle("BYPASS: too close; rear path unobserved, stopping instead of reversing.");
    return false;
}   /* start_backup() */


/*!
 * @brief BACKUP: reverse, holding the heading, until
 *        BYPASS_BACKUP_CLEAR_MM is clear ahead or BYPASS_BACKUP_MAX_MM
 *        has been reversed; then undo that much of the recorded path and
 *        carry on.
 *
 * @param[in] now_msec The time now.
 * @param[in] range_mm The latest range, straight ahead.
 */
static void
backup_step (uint32_t now_msec, uint32_t range_mm)
{
    bool b_clear = (servo_is_settled() && ultrasonic_measurement_ready() &&
                    ((ULTRASONIC_NO_RANGE == range_mm) ||
                     (range_mm >= BYPASS_BACKUP_CLEAR_MM)));

    if ((g_phase_mm >= 1.0f) &&
        (b_clear || (g_phase_mm >= (float) BYPASS_BACKUP_MAX_MM)))
    {
        motor_set(0, 0);
        trim_stack(g_phase_mm);
        log_obstacle("BYPASS: backed up %ld mm, to x=%ld z=%ld mm.",
                     (long) lroundf(g_phase_mm), (long) lroundf(g_x),
                     (long) lroundf(g_z));
        if (PHASE_OUT == g_backup_then)
        {
            curve_start(now_msec);
        }
        else
        {
            phase_enter(g_backup_then, now_msec);
        }
    }
    else
    {
        reverse(g_backup_hdg);
    }
}   /* backup_step() */


/*!
 * @brief Take a reversed distance off the top of the stack.
 *
 * @par
 * Reversing along the heading retraces the last moves driven, so they
 * no longer need undoing on the way back. Whole moves are dropped and
 * the last one shortened; the return's steering takes up what is left.
 *
 * @param[in] back_mm The distance reversed.
 */
static void
trim_stack (float back_mm)
{
    float left = back_mm;

    while ((left > 0.0f) && (g_sp > 0u))
    {
        move_t * p_top = &g_stack[g_sp - 1u];
        float    dist  = (float) p_top->dist_mm;

        if (dist <= left)
        {
            left -= dist;
            g_sp--;
        }
        else
        {
            p_top->x_to_mm -= (p_top->x_to_mm - p_top->x_from_mm) *
                              (left / dist);
            p_top->dist_mm  = (uint32_t) lroundf(dist - left);
            left = 0.0f;
        }
    }

    g_seg_mm   = 0.0f;
    g_seg_hsum = 0.0f;
    g_seg_x0   = g_x;
}   /* trim_stack() */


/*!
 * @brief RE-SCAN: braked, sweep from straight ahead towards the chosen side
 *        only, one fresh echo per BYPASS_RESCAN_STEP_DEG, for the next
 *        edge; then move the target out if needed and plan a new curve.
 *
 * @par
 * Only echoes within BYPASS_RESCAN_NEAR_MM count as the obstacle (a face
 * seen at a slant can bounce the sound away, so the first ray may read
 * the background). The edge is where the echo jumps BYPASS_EDGE_JUMP_MM
 * further (or goes missing, or past the near limit) after a ray that saw
 * the obstacle; that last near ray is its corner. Nothing near even
 * straight ahead: the way is clear, carry on. The obstacle across the
 * whole half-sweep: turn BYPASS_EDGE_TURN_DEG further and sweep again.
 *
 * @param[in] now_msec The time now.
 * @param[in] range_mm The latest range at the current servo angle.
 *
 * @return BYPASS_RUNNING or BYPASS_FAILED.
 */
static bypass_result_t
rescan_step (uint32_t now_msec, uint32_t range_mm)
{
    bypass_result_t result = BYPASS_RUNNING;
    int32_t         step   = (gb_left ? -BYPASS_RESCAN_STEP_DEG
                                      : BYPASS_RESCAN_STEP_DEG);
    bool            b_near = ((ULTRASONIC_NO_RANGE != range_mm) &&
                              (range_mm > 0u) &&
                              (range_mm <= BYPASS_RESCAN_NEAR_MM));
    bool            b_edge = false;

    motor_set(0, 0);

    if ((now_msec - g_phase_msec) > BYPASS_STEP_MAX_MSEC)
    {
        log_obstacle("BYPASS: re-scan took over %u ms: stopping.",
                     BYPASS_STEP_MAX_MSEC);
        return (BYPASS_FAILED);
    }

    if (!(servo_is_settled() && ultrasonic_measurement_ready()))
    {
        return (BYPASS_RUNNING);            // Wait for a fresh echo here.
    }

    if (0 == g_scan_rel)
    {
        g_ahead_mm = range_mm;              // Straight ahead, this stop.
    }

    if (b_near && ((!gb_near_seen) ||
                   (range_mm < (g_near_mm + BYPASS_EDGE_JUMP_MM))))
    {
        gb_near_seen = true;                // Still the obstacle.
        g_near_rel   = g_scan_rel;
        g_near_mm    = range_mm;
    }
    else
    {
        b_edge = true;                      // Far or none: past the edge.
    }

    if (b_edge && gb_near_seen)
    {
        // The last ray that saw it touched the corner with the edge of its
        // beam, not its centre: move it inwards by the beam correction
        // (never past straight ahead).
        float rel  = (float) g_near_rel +
                     (gb_left ? (float) BYPASS_BEAM_CORRECTION_DEG
                              : -(float) BYPASS_BEAM_CORRECTION_DEG);
        float rad  = ((gb_left ? (rel > 0.0f) : (rel < 0.0f)) ? 0.0f : rel) /
                     DEG_PER_RAD;
        float ex   = (float) g_near_mm * sinf(rad);
        float ey   = (float) g_near_mm * cosf(rad);
        float hrad = g_heading / DEG_PER_RAD;
        float cx   = g_x + (ex * cosf(hrad)) + (ey * sinf(hrad));
        float cz   = g_z - (ex * sinf(hrad)) + (ey * cosf(hrad));
        float want = cx + (gb_left ? -(float) g_clearance
                                   : (float) g_clearance);

        log_obstacle("BYPASS re-scan: edge at %ld deg, %" PRIu32 " mm "
                     "(corner x=%ld z=%ld mm).", (long) g_near_rel,
                     g_near_mm, (long) lroundf(cx), (long) lroundf(cz));
        if (gb_left ? (want < g_target_x) : (want > g_target_x))
        {
            g_target_x = want;              // Further out than planned.
        }
        g_front_z = cz;

        if (fabsf(g_target_x) > (float) BYPASS_MAX_SHIFT_MM)
        {
            log_obstacle("BYPASS: would need %ld mm sideways, over the %u "
                         "mm limit: stopping.", (long) lroundf(g_target_x),
                         BYPASS_MAX_SHIFT_MM);
            result = BYPASS_FAILED;
        }
        else if (front_within(g_ahead_mm, BYPASS_BACKUP_TRIGGER_MM))
        {
            // Too close to curve out from here: room first.
            if (!start_backup(PHASE_OUT, g_ahead_mm, now_msec))
            {
                result = BYPASS_FAILED;
            }
        }
        else
        {
            curve_start(now_msec);
        }
    }
    else if (b_edge)
    {
        // Nothing near even straight ahead: the car already points past
        // the edge, so the way is clear. Carry on from what is known.
        log_obstacle("BYPASS re-scan: clear straight ahead; carrying on.");
        curve_start(now_msec);
    }
    else if ((gb_left ? -g_scan_rel : g_scan_rel) >= SERVO_RANGE_DEG)
    {
        // The obstacle fills the whole half-sweep: turn further that way
        // and sweep again.
        float next = g_heading + (float) (gb_left ? -BYPASS_EDGE_TURN_DEG
                                                  : BYPASS_EDGE_TURN_DEG);

        if (fabsf(next) > (float) BYPASS_HEADING_MAX_DEG)
        {
            log_obstacle("BYPASS: no edge within %d deg of the line: "
                         "stopping.", BYPASS_HEADING_MAX_DEG);
            result = BYPASS_FAILED;
        }
        else
        {
            log_obstacle("BYPASS re-scan: no edge yet; turning %d deg "
                         "further.", BYPASS_EDGE_TURN_DEG);
            start_pivot(next, PHASE_RESCAN, now_msec);
        }
    }
    else
    {
        g_scan_rel += step;                 // Next ray outwards.
        servo_set_angle(SERVO_CENTER_DEG + g_scan_rel);
    }

    return (result);
}   /* rescan_step() */


/*!
 * @brief PASS: straighten up, then drive on watching the obstacle
 *        sideways until the car is past it.
 *
 * @par
 * While still turned the moves are pushed (their sideways drift must be
 * undone); once straight, they are not (along the line). The sensor aims
 * SERVO_RANGE_DEG off straight ahead, so it sees the obstacle's side
 * lateral / tan(angle) ahead of the nose. Once seen, BYPASS_SIDE_CLEAR_COUNT
 * far (or no) echoes in a row mean its rear has gone by; the car drives
 * that far again plus its own length. Never seen: the assumed depth.
 *
 * @param[in] now_msec The time now.
 * @param[in] range_mm The latest range (sideways).
 *
 * @return BYPASS_RUNNING or BYPASS_FAILED.
 */
static bypass_result_t
pass_step (uint32_t now_msec, uint32_t range_mm)
{
    bypass_result_t result   = BYPASS_RUNNING;
    uint32_t        moved    = (uint32_t) lroundf(g_phase_mm);
    float           beam     = (float) SERVO_RANGE_DEG / DEG_PER_RAD;
    uint32_t        near_mm  = side_near_mm();
    uint32_t        ahead_mm = (uint32_t) lroundf((float) g_clearance /
                                                  tanf(beam));
    bool            b_turned = !gb_pass_straight;

    if (!gb_pass_straight)
    {
        // Straightening up: steer to heading 0 (wherever that leaves the
        // car sideways) and keep recording, so this drift is undone too.
        if (!record(false))
        {
            return (BYPASS_FAILED);
        }
        if (fabsf(g_heading) <= STRAIGHT_DEG)
        {
            if (!record(true))              // Straight now: close the last.
            {
                return (BYPASS_FAILED);
            }
            gb_pass_straight = true;
            g_pass_hold_x    = g_x;         // Hold THIS line from now on.
            b_turned         = false;
        }
        steer(0.0f);
    }
    else
    {
        // Straight past it: hold this x, and record nothing (along the
        // line, so it needs no undoing).
        g_seg_mm   = 0.0f;
        g_seg_hsum = 0.0f;
        g_seg_x0   = g_x;
        steer(atanf((g_pass_hold_x - g_x) / (float) BYPASS_LOOKAHEAD_MM) *
              DEG_PER_RAD);
    }

    if (servo_is_settled() && ultrasonic_measurement_ready() &&
        ((int32_t) (now_msec - g_side_next) >= 0) &&
        ultrasonic_sample_sequence() != g_side_sequence)
    {
        g_side_sequence = ultrasonic_sample_sequence();
        g_side_next = now_msec + SIDE_SAMPLE_MSEC;

        if ((ULTRASONIC_NO_RANGE != range_mm) && range_mm > 0u && (range_mm <= near_mm))
        {
            gb_side_seen = true;            // Still alongside.
            g_side_clear = 0u;
            g_pass_end_mm = 0u;
        }
        else if (range_mm == ULTRASONIC_NO_RANGE || range_mm == 0u) {
            g_side_clear = 0u;
            g_pass_end_mm = 0u;
        }
        else if (gb_side_seen && g_pass_end_mm == 0u)
        {
            g_side_clear++;
            if (g_side_clear >= BYPASS_SIDE_CLEAR_COUNT)
            {
                g_pass_end_mm = moved + ahead_mm + CAR_LENGTH_MM + BYPASS_SIDE_MARGIN_MM;
                log_obstacle("BYPASS: obstacle ends ~%" PRIu32 " mm on; "
                             "passing to %" PRIu32 " mm.", ahead_mm,
                             g_pass_end_mm);
            }
        }
        else
        {
            // Not seen yet.
        }
    }

    if ((0u == g_pass_end_mm) && !gb_side_seen && (moved >= g_pass_min_mm))
    {
        log_obstacle("BYPASS: no side observation; cannot confirm obstacle passed.");
        return BYPASS_FAILED;
    }

    if ((0u != g_pass_end_mm) && (moved >= g_pass_end_mm) &&
        (moved >= g_pass_min_mm) && !b_turned)
    {
        servo_set_angle(SERVO_CENTER_DEG);
        log_obstacle("BYPASS: past it at x=%ld z=%ld mm; coming back over "
                     "%" PRIu32 " moves.", (long) lroundf(g_x),
                     (long) lroundf(g_z), g_sp);
        if (0u == g_sp)
        {
            start_pivot(0.0f, PHASE_FINISH, now_msec);
        }
        else
        {
            return_next();
            phase_enter(PHASE_RETURN, now_msec);
        }
    }
    else if (moved > BYPASS_PASS_MAX_MM)
    {
        log_obstacle("BYPASS: still alongside after %u mm: stopping.",
                     BYPASS_PASS_MAX_MM);
        result = BYPASS_FAILED;
    }
    else
    {
        // Still passing.
    }

    return (result);
}   /* pass_step() */


/*!
 * @brief Start a phase: note the time, reset the phase distance, and set
 *        up what the phase needs.
 *
 * @param[in] phase    The new phase.
 * @param[in] now_msec The time now.
 */
static void
phase_enter (phase_t phase, uint32_t now_msec)
{
    if (gb_fixed_test && phase==PHASE_BRAKE) gb_brake_reverse=(g_phase==PHASE_BACKUP);
    if (gb_fixed_test && phase==PHASE_ALIGN && g_phase==PHASE_ADVANCE) {
        g_brake_then=PHASE_ALIGN;
        phase=PHASE_BRAKE;
    }
    if ((PHASE_PIVOT == phase) || (PHASE_RESCAN == phase) ||
        (PHASE_BACKUP == phase))
    {
        motor_set(0, 0);    // From a standstill (motor.c boosts the start).
    }
    g_line_ticks = 0u;
    g_side_sequence = ultrasonic_sample_sequence();
    g_phase      = phase;
    g_phase_msec = now_msec;
    g_phase_mm   = 0.0f;

    if (gb_fixed_test)
    {
        motor_set(0, 0);
        gb_turn_settling = false;
        g_steer_residual=0;
        gb_odo_moved = false;
        if (phase==PHASE_BRAKE) { g_turn_quiet_msec=now_msec; }
        servo_set_angle(SERVO_CENTER_DEG);
        ultrasonic_invalidate(); // A new body heading needs a new forward ping.
        if (phase == PHASE_ALIGN) {
            g_target_hdg = 0.0f;
            g_turn_direction = (g_heading > 0.0f) ? -1 : 1;
        }
        log_obstacle("BYPASS fixed test: %s, heading %ld deg.",
                     bypass_phase_name(), (long)lroundf(g_heading));
        return;
    }
    if (PHASE_PASS == phase)
    {
        // Aim at the obstacle's side: it is on the other side from the one
        // being passed on (+ angle faces right).
        servo_set_angle(gb_left ? (SERVO_CENTER_DEG + SERVO_RANGE_DEG)
                                : (SERVO_CENTER_DEG - SERVO_RANGE_DEG));
        g_pass_hold_x = g_x;
        gb_pass_straight = false;
        gb_side_seen  = false;
        g_side_clear  = 0u;
        g_side_next   = now_msec;
        g_pass_end_mm = 0u;
        g_pass_min_mm = (uint32_t) lroundf(((g_front_z > g_z) ?
                                            (g_front_z - g_z) : 0.0f)) +
                        BYPASS_ASSUMED_DEPTH_MM + CAR_LENGTH_MM + BYPASS_SIDE_MARGIN_MM;
    }
    else if (PHASE_RETURN == phase)
    {
        servo_set_angle(gb_left ? (SERVO_CENTER_DEG + SERVO_RANGE_DEG)
                                : (SERVO_CENTER_DEG - SERVO_RANGE_DEG));
        gb_ret_hold   = false;
        g_hold_end_mm = 0.0f;
        g_side_clear  = 0u;
        g_side_next   = now_msec;
    }
    else if (PHASE_ADVANCE == phase)
    {
        servo_set_angle(SERVO_CENTER_DEG);
    }
    else if (PHASE_BACKUP == phase)
    {
        g_backup_hdg = g_heading;
        servo_set_angle(SERVO_CENTER_DEG);
        ultrasonic_invalidate();            // A fresh echo, straight on.
    }
    else if (PHASE_RESCAN == phase)
    {
        g_scan_rel   = 0;
        g_ahead_mm   = ULTRASONIC_NO_RANGE;
        gb_near_seen = false;
        servo_set_angle(SERVO_CENTER_DEG);
        ultrasonic_invalidate();            // A fresh echo, straight on.
    }
    else
    {
        // Nothing more to set up.
    }

    log_obstacle("BYPASS step: %s (heading %ld deg).", bypass_phase_name(),
                 (long) lroundf(g_heading));
}   /* phase_enter() */


/*!
 * @brief Update position and heading from this tick's encoder counts.
 *
 * @par
 * Pivoting (one wheel back, one forward), the heading turns by the mean
 * wheel travel round half the track (corrected by BYPASS_PIVOT_SLIP_X100)
 * and the position stays. Driving (both forward), the heading turns by
 * (left - right) / track and the position moves by the mean travel along
 * the mean heading; the distance also builds the move being recorded.
 * Braked (re-scan), nothing moves.
 */
static void
odo_update (void)
{
    uint32_t left   = encoder_count(PIN_ENC_LEFT_A);
    uint32_t right  = encoder_count(PIN_ENC_RIGHT_A);
    float    dl     = ((float) (left - g_last_left) * 1000.0f) /
                      (float) ENC_LEFT_EDGES_PER_M;
    float    dr     = ((float) (right - g_last_right) * 1000.0f) /
                      (float) ENC_RIGHT_EDGES_PER_M;
    float    mean   = (dl + dr) / 2.0f;

    gb_odo_moved = (left != g_last_left || right != g_last_right);
    g_last_left  = left;
    g_last_right = right;
    float pivot_scale = gb_fixed_test ?
        (float)(g_turn_direction < 0 ? BYPASS_PIVOT_LEFT_X100 : BYPASS_PIVOT_RIGHT_X100) :
        (float)BYPASS_PIVOT_SLIP_X100;

    if (PHASE_PIVOT == g_phase)
    {
        float turn = ((mean / ((float) WHEEL_TRACK_MM / 2.0f)) *
                      DEG_PER_RAD * 100.0f) / pivot_scale;

        if(gb_fixed_test) {
            float signed_mean=g_turn_direction*(dl-dr)/2;
            float mid=(g_heading+g_turn_direction*turn/2)/DEG_PER_RAD;
            g_x+=signed_mean*sinf(mid);
            g_z+=signed_mean*cosf(mid);
        }
        g_heading += gb_fixed_test ? g_turn_direction * turn :
                     ((g_target_hdg < g_heading) ? -turn : turn);
    }
    else if (PHASE_ALIGN == g_phase)
    {
        float turn = ((mean / ((float) WHEEL_TRACK_MM / 2.0f)) *
                      DEG_PER_RAD * 100.0f) / pivot_scale;

        if(gb_fixed_test) {
            float signed_mean=g_turn_direction*(dl-dr)/2;
            float mid=(g_heading+g_turn_direction*turn/2)/DEG_PER_RAD;
            g_x+=signed_mean*sinf(mid);
            g_z+=signed_mean*cosf(mid);
        }
        g_heading += gb_fixed_test ? g_turn_direction * turn :
                     ((g_heading > 0.0f) ? -turn : turn);
    }
    else if ((PHASE_OUT == g_phase) || (PHASE_PASS == g_phase) ||
             (PHASE_RETURN == g_phase) || (PHASE_ADVANCE == g_phase) ||
             (gb_fixed_test && PHASE_BRAKE == g_phase && !gb_brake_reverse))
    {
        float turn = (((BYPASS_ODO_SWAP_LR ? (dr - dl) : (dl - dr)) /
                       (float) WHEEL_TRACK_MM) * DEG_PER_RAD);
        float mid  = (g_heading + (turn / 2.0f)) / DEG_PER_RAD;

        g_x        += mean * sinf(mid);
        g_z        += mean * cosf(mid);
        g_heading  += turn;
        g_phase_mm += mean;
        g_seg_mm   += mean;
        g_seg_hsum += mean * (mid * DEG_PER_RAD);
        if ((PHASE_RETURN == g_phase) && !gb_ret_hold)
        {
            g_ret_done += mean;
        }
    }
    else if (PHASE_BACKUP == g_phase || (gb_fixed_test && PHASE_BRAKE==g_phase && gb_brake_reverse))
    {
        // Reversing (the encoders count it as forward): position goes
        // back along the heading, and the turn's sense flips.
        float turn = -(((BYPASS_ODO_SWAP_LR ? (dr - dl) : (dl - dr)) /
                        (float) WHEEL_TRACK_MM) * DEG_PER_RAD);
        float mid  = (g_heading + (turn / 2.0f)) / DEG_PER_RAD;

        g_x        -= mean * sinf(mid);
        g_z        -= mean * cosf(mid);
        g_heading  += turn;
        g_phase_mm += mean;
    }
    else
    {
        // Braked: nothing moved.
    }
}   /* odo_update() */


/*!
 * @brief Drive forward at BYPASS_DRIVE_SPEED, steering towards a heading
 *        by running the wheels at different speeds (both forward).
 *
 * @param[in] heading_deg The heading wanted.
 */
static void
steer (float heading_deg)
{
    float   error = heading_deg - g_heading;
    float   diff  = clampf((error * (float) BYPASS_STEER_KP_X10) / 10.0f,
                           (float) BYPASS_STEER_MAX);
    float command=diff+(gb_fixed_test ? g_steer_residual : 0);
    int32_t adjustment=(int32_t)lroundf(clampf(command,BYPASS_STEER_MAX));
    if(gb_fixed_test) g_steer_residual=command-adjustment;
    int32_t left  = BYPASS_DRIVE_SPEED + adjustment;
    int32_t right = BYPASS_DRIVE_SPEED - adjustment;

    // Both forward, always, and the inner wheel never stopped: a stopped
    // wheel is dragged back, which the encoders (no direction) would
    // count as forward, so the heading would go wrong.
    motor_set(((left < BYPASS_WHEEL_MIN) ? BYPASS_WHEEL_MIN : left),
              ((right < BYPASS_WHEEL_MIN) ? BYPASS_WHEEL_MIN : right));
}   /* steer() */


/*!
 * @brief Pivot on the spot.
 *
 * @param[in] b_left True to turn left.
 */
static void
pivot (bool b_left)
{
    if (b_left)
    {
        motor_set(-BYPASS_PIVOT_SPEED, BYPASS_PIVOT_SPEED);
    }
    else
    {
        motor_set(BYPASS_PIVOT_SPEED, -BYPASS_PIVOT_SPEED);
    }
}   /* pivot() */


/*!
 * @brief Reverse at BYPASS_DRIVE_SPEED, holding a heading.
 *
 * @par
 * Reversing, the nose swings right when the RIGHT wheel goes back
 * faster, so the wheel difference is the other way round from steer().
 *
 * @param[in] heading_deg The heading to hold.
 */
static void
reverse (float heading_deg)
{
    float   error = heading_deg - g_heading;
    float   diff  = clampf((error * (float) BYPASS_STEER_KP_X10) / 10.0f,
                           (float) BYPASS_STEER_MAX);
    int32_t left  = BYPASS_DRIVE_SPEED - (int32_t) lroundf(diff);
    int32_t right = BYPASS_DRIVE_SPEED + (int32_t) lroundf(diff);

    motor_set(-((left < BYPASS_WHEEL_MIN) ? BYPASS_WHEEL_MIN : left),
              -((right < BYPASS_WHEEL_MIN) ? BYPASS_WHEEL_MIN : right));
}   /* reverse() */


/*!
 * @brief Report whether a valid echo straight ahead is within a limit.
 *
 * @param[in] range_mm The latest range.
 * @param[in] limit_mm The limit.
 *
 * @return True if the servo has settled and the echo is within the limit.
 */
static bool
front_within (uint32_t range_mm, uint32_t limit_mm)
{
    return (servo_is_settled() && (ULTRASONIC_NO_RANGE != range_mm) &&
            (range_mm > 0u) && (range_mm <= limit_mm));
}   /* front_within() */


/*!
 * @brief Limit a value to +/- limit.
 *
 * @param[in] value The value.
 * @param[in] limit The limit (positive).
 *
 * @return The limited value.
 */
static float
clampf (float value, float limit)
{
    return ((value > limit) ? limit : ((value < -limit) ? -limit : value));
}   /* clampf() */

/*** end of file ***/

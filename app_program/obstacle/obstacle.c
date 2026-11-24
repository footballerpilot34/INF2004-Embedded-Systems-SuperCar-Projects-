/** @file obstacle.c
 * @brief Project-owned SuperCar implementation.
 *
 * NOTE: Remaining standards deviations are tracked in BARR_C_REVIEW.md.
 */
#include "pico/time.h"
#include <math.h>
#include <string.h>
#include "config.h"
#include "log.h"
#include "obstacle.h"
#include "servo.h"
#include "ultrasonic.h"

#define SCAN_CAPACITY 64u
#define RAD_PER_DEG 0.017453292519943295f

_Static_assert(((SERVO_CENTER_DEG - SERVO_RANGE_DEG) >= 0) &&
                   ((SERVO_CENTER_DEG + SERVO_RANGE_DEG) <= 180),
               "Scan range must fit the servo travel");
_Static_assert(OBSTACLE_SCAN_SAMPLES == 3u, "Median filter uses three pings");
_Static_assert((OBSTACLE_SCAN_COARSE_DEG > 0) && (OBSTACLE_SCAN_FINE_DEG > 0),
               "Scan steps must advance");
_Static_assert(((SERVO_RANGE_DEG % OBSTACLE_SCAN_COARSE_DEG) == 0) &&
                   ((OBSTACLE_SCAN_COARSE_DEG % OBSTACLE_SCAN_FINE_DEG) == 0),
               "Scan grid must include centre, limits and fine subdivisions");
_Static_assert((((2 * SERVO_RANGE_DEG) / OBSTACLE_SCAN_FINE_DEG) + 1) <=
                   SCAN_CAPACITY,
               "Scan grid exceeds storage");
static obstacle_point_t g_points[SCAN_CAPACITY];
static uint32_t         g_count;
static int8_t           g_queue[SCAN_CAPACITY]; // Angles, +/-30 deg.
static int32_t          g_scan_direction;
static uint32_t         g_queue_count;
static uint32_t         g_queue_pos;
static uint32_t         g_values[OBSTACLE_SCAN_SAMPLES];
static uint32_t         g_samples;
static uint32_t         g_valid;
static uint32_t         g_sequence;
static uint32_t         g_started;

// Flags and small state values, packed into bit-fields: each takes
// only the bits its range needs (change requirement 1). Only the
// main loop reads or writes them, so no locking is needed.
static struct
{
    bool b_searching_right : 1;
    bool b_active : 1;
} g_ob;

static obstacle_profile_t g_profile;

/** @brief Clear retired classification fields; geometry remains continuous. */
void
obstacle_width_estimate_update(obstacle_profile_t * p_value_p)
{
    p_value_p->corrected_width_mm     = 0u;
    p_value_p->width_fallback         = false;
    p_value_p->calibrated_width_mm    = 0u;
    p_value_p->calibrated_width_valid = false;
}

/** @brief Project a genuine radial echo onto the forward axis. */
static float
forward_depth(obstacle_point_t const * point)
{
    return (point->range_mm) * (cosf((point->angle_deg) * RAD_PER_DEG));
}

/** @brief Check whether a scan point contains a usable measured distance. */
static bool
has_range(obstacle_point_t const * p_value_p)
{
    return ((((p_value_p->kind) == OBSTACLE_RETURN_RANGE) ||
             ((p_value_p->kind) == OBSTACLE_RETURN_WEAK_RANGE)) &&
            ((p_value_p->range_mm) > 0u)) &&
           ((p_value_p->range_mm) != OBSTACLE_POINT_NO_RANGE);
}

// Detect the FIRST change immediately. Additional rays only determine whether
// the resulting geometry also satisfies the existing clearance safety gate.
/** @brief Find a foreground boundary and classify its evidence quality. */
static bool
edge_bracket(obstacle_point_t const * points, uint32_t count, uint32_t centre,
             int32_t step, uint32_t * inner, uint32_t * outer,
             obstacle_edge_status_t * status)
{
    *inner = *outer   = centre;
    *status           = OBSTACLE_EDGE_UNOBSERVED;
    uint32_t near     = centre;
    bool     reliable = (points[centre].kind) == OBSTACLE_RETURN_RANGE;
    for (int32_t index_i = ((int32_t)centre) + step;
         (index_i >= 0) && (index_i < ((int32_t)count)); index_i += step)
    {
        obstacle_point_t const * p_value_p = &points[index_i];
        bool adjacent = ((p_value_p->angle_deg) - (points[near].angle_deg)) ==
                        (step * OBSTACLE_SCAN_FINE_DEG);
        if (!has_range(p_value_p))
        {
            *inner  = near;
            *outer  = (uint32_t)index_i;
            *status = (p_value_p->kind) == OBSTACLE_RETURN_NO_ECHO
                          ? OBSTACLE_EDGE_NO_ECHO
                          : OBSTACLE_EDGE_UNRELIABLE;
            return true; // Missing readings bound an estimate, never clearance.
        }
        float depth      = forward_depth(p_value_p);
        float near_depth = forward_depth(&points[near]);
        if ((depth - near_depth) >= OBSTACLE_SCAN_EDGE_JUMP_MM)
        {
            *inner     = near;
            *outer     = (uint32_t)index_i;
            *status    = OBSTACLE_EDGE_ESTIMATED;
            float low  = depth;
            float high = depth;
            bool  safe = reliable && adjacent;
            for (uint32_t count_n = 0;
                 count_n < OBSTACLE_SCAN_EDGE_CONFIRM_RAYS; ++count_n)
            {
                int32_t index_j = index_i + (step * ((int32_t)count_n));
                if ((((index_j < 0) || (index_j >= ((int32_t)count))) ||
                     ((points[index_j].kind) != OBSTACLE_RETURN_RANGE)) ||
                    ((points[index_j].angle_deg) !=
                     ((p_value_p->angle_deg) +
                      ((step * ((int32_t)count_n)) * OBSTACLE_SCAN_FINE_DEG))))
                {
                    safe = false;
                    break;
                }
                float value_d = forward_depth(&points[index_j]);
                low           = fminf(low, value_d);
                high          = fmaxf(high, value_d);
                if (((value_d - near_depth) < OBSTACLE_SCAN_EDGE_JUMP_MM) ||
                    ((high - low) > OBSTACLE_SCAN_EDGE_JUMP_MM))
                {
                    safe = false;
                }
            }
            if (safe)
            {
                *status = OBSTACLE_EDGE_CONFIRMED;
            }
            return true;
        }
        reliable = ((reliable && adjacent) &&
                    ((p_value_p->kind) == OBSTACLE_RETURN_RANGE)) &&
                   ((near_depth - depth) < OBSTACLE_SCAN_EDGE_JUMP_MM);
        near     = (uint32_t)index_i;
    }
    *inner = near;
    *outer = near;
    // If the obstacle fills the field of view, its edge lies at or beyond
    // the mechanical limit. Show that limit explicitly as an estimate.
    if ((points[near].angle_deg) == (step * SERVO_RANGE_DEG))
    {
        *status = OBSTACLE_EDGE_SCAN_LIMIT;
        return true;
    }
    return false;
}

/** @brief Project radial range onto the lateral axis. */
static float
lateral(float depth, float angle)
{
    return depth * (tanf(angle * RAD_PER_DEG));
}

/** @brief Retain the inner and outer rays that bracket an obstacle edge. */
static int32_t
edge_geometry(obstacle_point_t const * near, obstacle_point_t const * far,
              obstacle_edge_geometry_t * edge, int32_t * angle_x10)
{
    edge->near_angle_deg = near->angle_deg;
    edge->far_angle_deg  = far->angle_deg;
    *angle_x10           = ((near->angle_deg) + (far->angle_deg)) * 5;
    float depth          = forward_depth(near);
    float value_x        = lateral(depth, (*angle_x10) / 10.0f);
    edge->forward_mm     = (uint32_t)lroundf(depth);
    edge->distance_mm    = (uint32_t)lroundf(hypotf(value_x, depth));
    return (int32_t)lroundf(value_x);
}

/** @brief Build provisional corridor metadata without commanding motion. */
static void
bypass_candidate(obstacle_profile_t * p_value_p, float left_outer,
                 float right_outer)
{
    uint32_t clearance =
        ((uint32_t)ceilf(hypotf(CAR_WIDTH_MM / 2.0f, CAR_LENGTH_MM / 2.0f))) +
        OBSTACLE_BYPASS_MARGIN_MM;
    if ((((p_value_p->front_mm) <= clearance) ||
         ((p_value_p->left_edge.forward_mm) <= clearance)) ||
        ((p_value_p->right_edge.forward_mm) <= clearance))
    {
        return;
    }

    // Reserve the full turning footprint at each outer edge of the scan
    // bracket. The rear of the obstacle has to be measured before returning.
    p_value_p->bypass.left_x_mm =
        ((int32_t)floorf(left_outer)) - ((int32_t)clearance);
    p_value_p->bypass.right_x_mm =
        ((int32_t)ceilf(right_outer)) + ((int32_t)clearance);
    p_value_p->bypass.preferred_x_mm =
        ((-p_value_p->bypass.left_x_mm) <= (p_value_p->bypass.right_x_mm))
            ? p_value_p->bypass.left_x_mm
            : p_value_p->bypass.right_x_mm;
    uint32_t farthest_front =
        (p_value_p->left_edge.forward_mm) > (p_value_p->right_edge.forward_mm)
            ? p_value_p->left_edge.forward_mm
            : p_value_p->right_edge.forward_mm;
    p_value_p->bypass.alongside_y_mm    = farthest_front + clearance;
    p_value_p->bypass.turn_clearance_mm = clearance;
    p_value_p->bypass.available         = true;
}

/** @brief Derive obstacle geometry from collected ultrasonic scan points. */
void
obstacle_profile_build(obstacle_point_t const * points, uint32_t count,
                       uint32_t front, obstacle_profile_t * p_value_p)
{
    memset(p_value_p, 0, sizeof(*p_value_p));
    obstacle_width_estimate_update(p_value_p);
    p_value_p->front_mm    = front;
    p_value_p->point_count = count;
    p_value_p->left_ray_mm = p_value_p->right_ray_mm = ULTRASONIC_NO_RANGE;
    if ((((front == ULTRASONIC_NO_RANGE) || (front == 0u)) || (count == 0u)) ||
        (count > SCAN_CAPACITY))
    {
        return;
    }
    uint32_t centre = count;
    for (uint32_t index_i = 0; index_i < count; ++index_i)
    {
        if (((points[index_i].angle_deg) == (-SERVO_RANGE_DEG)) &&
            (has_range(&points[index_i])))
        {
            p_value_p->left_ray_mm = points[index_i].range_mm;
        }
        if (((points[index_i].angle_deg) == SERVO_RANGE_DEG) &&
            (has_range(&points[index_i])))
        {
            p_value_p->right_ray_mm = points[index_i].range_mm;
        }
        if ((points[index_i].angle_deg) == 0)
        {
            centre = index_i;
        }
    }
    if (centre == count)
    {
        return;
    }
    p_value_p->centre_seen = (has_range(&points[centre])) &&
                             ((points[centre].kind) == OBSTACLE_RETURN_RANGE);
    if (!p_value_p->centre_seen)
    {
        p_value_p->front_mm = ULTRASONIC_NO_RANGE;
        return;
    }
    if ((points[centre].range_mm) > OBSTACLE_CLEAR_MM)
    {
        return;
    }
    p_value_p->front_mm = points[centre].range_mm;
    uint32_t left;
    uint32_t right;
    uint32_t left_outer     = centre;
    uint32_t right_outer    = centre;
    p_value_p->left_bounded = edge_bracket(
        points, count, centre, -1, &left, &left_outer, &p_value_p->left_status);
    p_value_p->right_bounded =
        edge_bracket(points, count, centre, 1, &right, &right_outer,
                     &p_value_p->right_status);
    float left_depth  = forward_depth(&points[left]);
    float right_depth = forward_depth(&points[right]);
    p_value_p->left_observed_mm =
        (int32_t)lroundf(lateral(left_depth, points[left].angle_deg));
    p_value_p->right_observed_mm =
        (int32_t)lroundf(lateral(right_depth, points[right].angle_deg));
    if (p_value_p->left_bounded)
    {
        p_value_p->left_mm =
            edge_geometry(&points[left], &points[left_outer],
                          &p_value_p->left_edge, &p_value_p->left_angle_x10);
    }
    if (p_value_p->right_bounded)
    {
        p_value_p->right_mm =
            edge_geometry(&points[right], &points[right_outer],
                          &p_value_p->right_edge, &p_value_p->right_angle_x10);
    }
    if ((p_value_p->left_bounded) && (p_value_p->right_bounded))
    {
        p_value_p->width_mm =
            (uint32_t)((p_value_p->right_mm) - (p_value_p->left_mm));
        p_value_p->width_min_mm = (uint32_t)((p_value_p->right_observed_mm) -
                                             (p_value_p->left_observed_mm));
        float left_bound  = lateral(left_depth, points[left_outer].angle_deg);
        float right_bound = lateral(right_depth, points[right_outer].angle_deg);
        p_value_p->width_max_mm = (uint32_t)ceilf(right_bound - left_bound);
        float middle_x = ((p_value_p->left_mm) + (p_value_p->right_mm)) / 2.0f;
        float middle_y = (left_depth + right_depth) / 2.0f;
        p_value_p->middle_x_mm = (int32_t)lroundf(middle_x);
        p_value_p->middle_y_mm = (uint32_t)lroundf(middle_y);
        p_value_p->middle_distance_mm =
            (uint32_t)lroundf(hypotf(middle_x, middle_y));
        p_value_p->edges_confirmed =
            ((p_value_p->left_status) == OBSTACLE_EDGE_CONFIRMED) &&
            ((p_value_p->right_status) == OBSTACLE_EDGE_CONFIRMED);
        if (p_value_p->edges_confirmed)
        {
            bypass_candidate(p_value_p, left_bound, right_bound);
        }
    }
    obstacle_width_estimate_update(p_value_p);
}

/** @brief Describe the evidence supporting an edge estimate. */
char const *
obstacle_edge_status_name(obstacle_edge_status_t status)
{
    switch (status)
    {
        case OBSTACLE_EDGE_NO_ECHO:
            return "missing echoes";
        case OBSTACLE_EDGE_UNRELIABLE:
            return "unreliable observations";
        case OBSTACLE_EDGE_CONFIRMED:
            return "echo-bracketed";
        case OBSTACLE_EDGE_SCAN_LIMIT:
            return "estimated at scan limit";
        case OBSTACLE_EDGE_ESTIMATED:
            return "estimated distance jump";
        case OBSTACLE_EDGE_UNOBSERVED:
            return "edge not observed";

        default:
            // No action for values without a matching case.
            break;
    }
    return "unknown";
}

/** @brief Queue a scan angle without duplicating an existing observation. */
static void
queue_angle(int32_t angle)
{
    // Skip already measured and queued angles, including the centre.
    for (uint32_t index_i = 0u; index_i < g_count; ++index_i)
    {
        if ((g_points[index_i].angle_deg) == angle)
        {
            return;
        }
    }
    for (uint32_t index_i = 0u; index_i < g_queue_count; ++index_i)
    {
        if ((g_queue[index_i]) == angle)
        {
            return;
        }
    }
    if (g_queue_count < SCAN_CAPACITY)
    {
        g_queue[g_queue_count++] = (int8_t)angle; // +/-SERVO_RANGE_DEG.
    }
}

/** @brief Command the next scan angle and require fresh ultrasonic samples. */
static void
aim(void)
{
    servo_set_angle(SERVO_CENTER_DEG + (g_queue[g_queue_pos]));
    // Centre may already be selected: force a fresh first observation too.
    ultrasonic_invalidate();
    g_sequence = ultrasonic_sample_sequence();
    g_samples = g_valid = 0u;
}

/** @brief Start the complete stationary sweep using measured front range. */
void
obstacle_scan_start(uint32_t forward_mm, uint32_t now_msec)
{
    memset(&g_profile, 0, sizeof(g_profile));
    obstacle_width_estimate_update(&g_profile);
    g_profile.front_mm = forward_mm;
    g_count = g_queue_count = g_queue_pos = 0u;
    g_started                             = now_msec;
    g_ob.b_active                         = true;
    g_scan_direction                      = -1;
    g_ob.b_searching_right                = false;

    queue_angle(0);
    queue_angle(-OBSTACLE_SCAN_FINE_DEG);

    log_obstacle("SCAN: centre-outward search, left then right.");
    log_obstacle("SCAN start: front=%lu mm", (unsigned long)forward_mm);
    aim();
}

/** @brief Store the actual echo median and its quality in angle order. */
static void
store_point(void)
{
    obstacle_point_t p_value_p = {.angle_deg = g_queue[g_queue_pos],
                                  .range_mm  = OBSTACLE_POINT_NO_RANGE,
                                  .kind      = OBSTACLE_RETURN_UNKNOWN};
    if (g_valid == 0u)
    {
        p_value_p.kind = OBSTACLE_RETURN_NO_ECHO;
    }
    else
    {
        for (uint32_t index_i = 0u; index_i < g_valid; ++index_i)
        {
            for (uint32_t index_j = index_i + 1u; index_j < g_valid; ++index_j)
            {
                if ((g_values[index_j]) < (g_values[index_i]))
                {
                    uint32_t value_v  = g_values[index_i];
                    g_values[index_i] = g_values[index_j];
                    g_values[index_j] = value_v;
                }
            }
        }
        uint32_t median =
            (g_valid & 1u)
                ? g_values[g_valid / 2u]
                : ((g_values[(g_valid / 2u) - 1u]) + (g_values[g_valid / 2u])) /
                      2u;
        p_value_p.range_mm = (median > OBSTACLE_POINT_RANGE_MAX)
                                 ? OBSTACLE_POINT_RANGE_MAX
                                 : median;
        // Keep EVERY available distance, including a single valid echo.
        // Quality affects clearance safety only, never graph visibility.
        bool stable = (g_valid >= 2u) && (((g_values[1]) - (g_values[0])) <=
                                          OBSTACLE_SCAN_STABLE_SPREAD_MM);
        if (g_valid == 3u)
        {
            stable = stable || (((g_values[2]) - (g_values[1])) <=
                                OBSTACLE_SCAN_STABLE_SPREAD_MM);
        }
        p_value_p.kind =
            stable ? OBSTACLE_RETURN_RANGE : OBSTACLE_RETURN_WEAK_RANGE;
    }
    // Maintain angle ordering for geometry and exported profile points.
    uint32_t pos = g_count;
    while ((pos > 0u) &&
           ((g_points[pos - 1u].angle_deg) > (p_value_p.angle_deg)))
    {
        g_points[pos] = g_points[pos - 1u];
        --pos;
    }
    g_points[pos] = p_value_p;
    ++g_count;
    if (g_valid > 0u)
    {
        log_obstacle("SCAN echoes: angle=%ld valid=%lu/3 min=%lu max=%lu mm",
                     (long)p_value_p.angle_deg, (unsigned long)g_valid,
                     (unsigned long)g_values[0],
                     (unsigned long)g_values[g_valid - 1u]);
        log_obstacle("SCAN point: angle=%ld range=%lu mm quality=%u",
                     (long)p_value_p.angle_deg,
                     (unsigned long)p_value_p.range_mm,
                     (unsigned)p_value_p.kind);
    }
    else
    {
        log_obstacle("SCAN point: angle=%ld distance UNKNOWN (no echo)",
                     (long)p_value_p.angle_deg);
    }
}

/** @brief Finalize edge estimates, report uncertainty and recenter the servo.
 */
static void
finish(bool timeout)
{
    uint32_t front = g_profile.front_mm;
    obstacle_profile_build(g_points, g_count, front, &g_profile);
    g_profile.finished  = true;
    g_profile.timed_out = timeout;
    g_ob.b_active       = false;
    servo_set_angle(SERVO_CENTER_DEG);
    if ((g_profile.left_status) == OBSTACLE_EDGE_CONFIRMED)
    {
        log_obstacle(
            "SCAN left edge: x=%ld y=%lu range=%lu mm; %ld to %ld deg (%s)",
            (long)g_profile.left_mm,
            (unsigned long)g_profile.left_edge.forward_mm,
            (unsigned long)g_profile.left_edge.distance_mm,
            (long)g_profile.left_edge.near_angle_deg,
            (long)g_profile.left_edge.far_angle_deg,
            obstacle_edge_status_name(g_profile.left_status));
    }
    else
    {
        log_obstacle("SCAN left edge UNKNOWN (%s).",
                     obstacle_edge_status_name(g_profile.left_status));
    }
    if ((g_profile.right_status) == OBSTACLE_EDGE_CONFIRMED)
    {
        log_obstacle(
            "SCAN right edge: x=%ld y=%lu range=%lu mm; %ld to %ld deg (%s)",
            (long)g_profile.right_mm,
            (unsigned long)g_profile.right_edge.forward_mm,
            (unsigned long)g_profile.right_edge.distance_mm,
            (long)g_profile.right_edge.near_angle_deg,
            (long)g_profile.right_edge.far_angle_deg,
            obstacle_edge_status_name(g_profile.right_status));
    }
    else
    {
        log_obstacle("SCAN right edge UNKNOWN (%s).",
                     obstacle_edge_status_name(g_profile.right_status));
    }
    if ((g_profile.edges_confirmed) && (!timeout))
    {
        log_obstacle("SCAN %swidth=%lu mm; sampling interval=%lu..%lu mm",
                     g_profile.edges_confirmed ? "" : "apparent ",
                     (unsigned long)g_profile.width_mm,
                     (unsigned long)g_profile.width_min_mm,
                     (unsigned long)g_profile.width_max_mm);
        log_obstacle("SCAN middle: x=%ld y=%lu range=%lu mm; centre ray=%lu mm",
                     (long)g_profile.middle_x_mm,
                     (unsigned long)g_profile.middle_y_mm,
                     (unsigned long)g_profile.middle_distance_mm,
                     (unsigned long)g_profile.front_mm);
    }
    else
    {
        log_obstacle("SCAN width UNKNOWN: one or both edges are outside "
                     "reliable observations.");
    }
    if (g_profile.centre_seen)
    {
        log_obstacle("SCAN measured centre distance=%lu mm",
                     (unsigned long)g_profile.front_mm);
    }
    else
    {
        log_obstacle("SCAN centre distance UNKNOWN");
    }
    if (timeout)
    {
        g_profile.bypass.available = false;
        log_obstacle("SCAN timed out; measurements incomplete.");
    }
    obstacle_bypass_candidate_t const * path = &g_profile.bypass;
    if (path->available)
    {
        log_obstacle("SCAN bypass candidate: left x=%ld, right x=%ld, "
                     "preferred x=%ld mm.",
                     (long)path->left_x_mm, (long)path->right_x_mm,
                     (long)path->preferred_x_mm);
        log_obstacle(
            "SCAN alongside y=%lu mm; return needs measured rear y + %lu mm.",
            (unsigned long)path->alongside_y_mm,
            (unsigned long)path->turn_clearance_mm);
        log_obstacle(
            "SCAN corridor unverified; vehicle controller decides motion.");
    }
    else
    {
        log_obstacle(
            "SCAN no verified corridor; measurement profile available to "
            "vehicle controller.");
    }
}

/** @brief Advance the sweep after settling and a fresh echo sequence. */
void
obstacle_scan_task(uint32_t now_msec)
{
    if (!g_ob.b_active)
    {
        return;
    }
    if ((now_msec - g_started) >= OBSTACLE_SCAN_TIMEOUT_MSEC)
    {
        finish(true);
        return;
    }
    if ((!servo_is_settled()) || (!ultrasonic_measurement_ready()))
    {
        return;
    }
    uint32_t sequence = ultrasonic_sample_sequence();
    if (sequence == g_sequence)
    {
        return;
    }
    g_sequence     = sequence;
    uint32_t range = ultrasonic_range_mm();
    if ((range > 0u) && (range != ULTRASONIC_NO_RANGE))
    {
        g_values[g_valid++] = range;
    }
    if ((++g_samples) < OBSTACLE_SCAN_SAMPLES)
    {
        return;
    }
    store_point();
    // Rebuild live geometry from the accumulated points; finish() marks the
    // final profile. A missing centre must not prevent the full angular scan.
    uint32_t front = g_profile.front_mm;
    obstacle_profile_build(g_points, g_count, front, &g_profile);
    int32_t measured_angle = g_queue[g_queue_pos];

    bool side_finished = measured_angle == (g_scan_direction * SERVO_RANGE_DEG);

    ++g_queue_pos;

    if (side_finished && (!g_ob.b_searching_right))
    {
        g_ob.b_searching_right = true;
        g_scan_direction       = 1;
        queue_angle(OBSTACLE_SCAN_FINE_DEG);
        log_obstacle("SCAN: left search complete; searching right.");
    }
    else if (side_finished && g_ob.b_searching_right)
    {
        finish(false);
        return;
    }
    else if (g_queue_pos == g_queue_count)
    {
        int32_t next =
            measured_angle + (g_scan_direction * OBSTACLE_SCAN_FINE_DEG);

        if ((next >= (-SERVO_RANGE_DEG)) && (next <= SERVO_RANGE_DEG))
        {
            queue_angle(next);
        }
        else
        {
            finish(false);
            return;
        }
    }

    if (g_queue_pos == g_queue_count)
    {
        finish(false);
    }
    else
    {
        aim();
    }
}

/** @brief Cancel scanning and invalidate its motion candidate. */
void
obstacle_scan_cancel(void)
{
    if (g_ob.b_active)
    {
        g_profile.cancelled = true;
    }
    g_ob.b_active = false;
    obstacle_width_estimate_update(&g_profile);
    g_profile.bypass.available = false;
    servo_set_angle(SERVO_CENTER_DEG);
}
/** @brief Report whether a stationary ultrasonic sweep is active. */
bool
obstacle_scan_active(void)
{
    return g_ob.b_active;
}
/** @brief Return the current measured scan profile. */
obstacle_profile_t const *
obstacle_profile(void)
{
    return &g_profile;
}
/** @brief Return collected points and their count without copying them. */
obstacle_point_t const *
obstacle_scan_points(uint32_t * count)
{
    *count = g_count;
    return g_points;
}

/*** end of file ***/

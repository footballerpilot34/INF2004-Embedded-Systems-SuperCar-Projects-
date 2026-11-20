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

_Static_assert(SERVO_CENTER_DEG - SERVO_RANGE_DEG >= 0 &&
               SERVO_CENTER_DEG + SERVO_RANGE_DEG <= 180,
               "Scan range must fit the servo travel");
_Static_assert(OBSTACLE_SCAN_SAMPLES == 3u, "Median filter uses three pings");
_Static_assert(OBSTACLE_SCAN_COARSE_DEG > 0 && OBSTACLE_SCAN_FINE_DEG > 0,
               "Scan steps must advance");
_Static_assert(SERVO_RANGE_DEG % OBSTACLE_SCAN_COARSE_DEG == 0 &&
               OBSTACLE_SCAN_COARSE_DEG % OBSTACLE_SCAN_FINE_DEG == 0,
               "Scan grid must include centre, limits and fine subdivisions");
_Static_assert(2 * SERVO_RANGE_DEG / OBSTACLE_SCAN_FINE_DEG + 1 <= SCAN_CAPACITY,
               "Scan grid exceeds storage");
_Static_assert(OBSTACLE_WIDTH_BEAM_HALF_DEG_X10 >= 0 &&
               OBSTACLE_WIDTH_BEAM_HALF_DEG_X10 < 900,
               "Effective beam half-angle must be within [0, 90) degrees");
_Static_assert(OBSTACLE_WIDTH_CAL_MIN_MM > 0u &&
               OBSTACLE_WIDTH_CAL_MIN_MM <= OBSTACLE_WIDTH_CAL_MAX_MM,
               "Width calibration requires a valid distance interval");

static obstacle_point_t g_points[SCAN_CAPACITY];
static uint32_t g_count;
static int32_t g_queue[SCAN_CAPACITY];
static int32_t g_scan_direction;
static bool g_searching_right;
static uint32_t g_queue_count, g_queue_pos;
static uint32_t g_values[OBSTACLE_SCAN_SAMPLES], g_samples, g_valid;
static uint32_t g_sequence, g_started;
static bool g_active;
static bool g_manual_scan;
static obstacle_profile_t g_profile;

static float forward_depth(obstacle_point_t const * point)
{
    return point->range_mm * cosf(point->angle_deg * RAD_PER_DEG);
}

static bool has_range(obstacle_point_t const * p)
{
    return (p->kind == OBSTACLE_RETURN_RANGE ||
            p->kind == OBSTACLE_RETURN_WEAK_RANGE) &&
           p->range_mm > 0u && p->range_mm != ULTRASONIC_NO_RANGE;
}

// Detect the FIRST change immediately. Additional rays only determine whether
// the resulting geometry also satisfies the existing clearance safety gate.
static bool edge_bracket(obstacle_point_t const * points, uint32_t count,
                         uint32_t centre, int32_t step,
                         uint32_t * inner, uint32_t * outer,
                         obstacle_edge_status_t * status)
{
    *inner = *outer = centre;
    *status = OBSTACLE_EDGE_UNOBSERVED;
    uint32_t near = centre;
    bool reliable = points[centre].kind == OBSTACLE_RETURN_RANGE;
    for (int32_t i = (int32_t)centre + step;
         i >= 0 && i < (int32_t)count; i += step) {
        obstacle_point_t const * p = &points[i];
        bool adjacent = p->angle_deg - points[near].angle_deg ==
                        step * OBSTACLE_SCAN_FINE_DEG;
        if (!has_range(p)) {
            *inner = near;
            *outer = (uint32_t)i;
            *status = p->kind == OBSTACLE_RETURN_NO_ECHO ?
                      OBSTACLE_EDGE_NO_ECHO : OBSTACLE_EDGE_UNRELIABLE;
            return true; // Missing readings bound an estimate, never clearance.
        }
        float depth = forward_depth(p);
        float near_depth = forward_depth(&points[near]);
        if (depth - near_depth >= OBSTACLE_SCAN_EDGE_JUMP_MM) {
            *inner = near;
            *outer = (uint32_t)i;
            *status = OBSTACLE_EDGE_ESTIMATED;
            float low = depth, high = depth;
            bool safe = reliable && adjacent;
            for (uint32_t n = 0; n < OBSTACLE_SCAN_EDGE_CONFIRM_RAYS; ++n) {
                int32_t j = i + step * (int32_t)n;
                if (j < 0 || j >= (int32_t)count ||
                    points[j].kind != OBSTACLE_RETURN_RANGE ||
                    points[j].angle_deg != p->angle_deg +
                        step * (int32_t)n * OBSTACLE_SCAN_FINE_DEG) {
                    safe = false;
                    break;
                }
                float d = forward_depth(&points[j]);
                low = fminf(low, d); high = fmaxf(high, d);
                if (d - near_depth < OBSTACLE_SCAN_EDGE_JUMP_MM ||
                    high - low > OBSTACLE_SCAN_EDGE_JUMP_MM) { safe = false; }
            }
            if (safe) { *status = OBSTACLE_EDGE_CONFIRMED; }
            return true;
        }
        reliable = reliable && adjacent && p->kind == OBSTACLE_RETURN_RANGE &&
                   near_depth - depth < OBSTACLE_SCAN_EDGE_JUMP_MM;
        near = (uint32_t)i;
    }
    *inner = near;
    *outer = near;
    // If the obstacle fills the field of view, its edge lies at or beyond
    // the mechanical limit. Show that limit explicitly as an estimate.
    if (points[near].angle_deg == step * SERVO_RANGE_DEG) {
        *status = OBSTACLE_EDGE_SCAN_LIMIT;
        return true;
    }
    return false;
}

static float lateral(float depth, float angle)
{
    return depth * tanf(angle * RAD_PER_DEG);
}

static int32_t edge_geometry(obstacle_point_t const * near,
                             obstacle_point_t const * far,
                             obstacle_edge_geometry_t * edge,
                             int32_t * angle_x10)
{
    edge->near_angle_deg = near->angle_deg;
    edge->far_angle_deg = far->angle_deg;
    *angle_x10 = (near->angle_deg + far->angle_deg) * 5;
    float depth = forward_depth(near);
    float x = lateral(depth, *angle_x10 / 10.0f);
    edge->forward_mm = (uint32_t)lroundf(depth);
    edge->distance_mm = (uint32_t)lroundf(hypotf(x, depth));
    return (int32_t)lroundf(x);
}

static void bypass_candidate(obstacle_profile_t * p, float left_outer,
                             float right_outer)
{
    uint32_t clearance = (uint32_t)ceilf(hypotf(CAR_WIDTH_MM / 2.0f,
                                               CAR_LENGTH_MM / 2.0f)) +
                         OBSTACLE_BYPASS_MARGIN_MM;
    if (p->front_mm <= clearance || p->left_edge.forward_mm <= clearance ||
        p->right_edge.forward_mm <= clearance) {
        return;
    }

    // Reserve the full turning footprint at each outer edge of the scan
    // bracket. The rear of the obstacle has to be measured before returning.
    p->bypass.left_x_mm = (int32_t)floorf(left_outer) - (int32_t)clearance;
    p->bypass.right_x_mm = (int32_t)ceilf(right_outer) + (int32_t)clearance;
    p->bypass.preferred_x_mm = (-p->bypass.left_x_mm <= p->bypass.right_x_mm) ?
        p->bypass.left_x_mm : p->bypass.right_x_mm;
    uint32_t farthest_front = p->left_edge.forward_mm > p->right_edge.forward_mm ?
        p->left_edge.forward_mm : p->right_edge.forward_mm;
    p->bypass.alongside_y_mm = farthest_front + clearance;
    p->bypass.turn_clearance_mm = clearance;
    p->bypass.available = true;
}

// Detection uses the beam axis; the actual target boundary may lie inside
// that axis by beta. A single measured central depth avoids mixing the two
// off-axis echoes in this planar approximation. Raw geometry stays intact.
static void calibrated_width(obstacle_profile_t * p)
{
    if (!OBSTACLE_WIDTH_CALIBRATION_ENABLED || !p->centre_seen ||
        !p->left_bounded || !p->right_bounded ||
        p->front_mm < OBSTACLE_WIDTH_CAL_MIN_MM ||
        p->front_mm > OBSTACLE_WIDTH_CAL_MAX_MM ||
        p->left_status == OBSTACLE_EDGE_SCAN_LIMIT ||
        p->right_status == OBSTACLE_EDGE_SCAN_LIMIT ||
        p->left_status == OBSTACLE_EDGE_UNRELIABLE ||
        p->right_status == OBSTACLE_EDGE_UNRELIABLE) {
        return;
    }
    float beta = OBSTACLE_WIDTH_BEAM_HALF_DEG_X10 / 10.0f;
    float left = p->left_angle_x10 / 10.0f + beta;
    float right = p->right_angle_x10 / 10.0f - beta;
    // Overlapping corrected edges are unresolved, not a zero-width object.
    if (left >= right || left <= -90.0f || right >= 90.0f) { return; }
    float width = lateral((float)p->front_mm, right) -
                  lateral((float)p->front_mm, left);
    if (!isfinite(width) || width <= 0.0f) { return; }
    p->calibrated_width_mm = (uint32_t)lroundf(width);
    p->calibrated_width_valid = true;
}

void
obstacle_profile_build(obstacle_point_t const * points, uint32_t count,
                       uint32_t front, obstacle_profile_t * p)
{
    memset(p, 0, sizeof(*p));
    p->front_mm = front;
    p->point_count = count;
    p->left_ray_mm = p->right_ray_mm = ULTRASONIC_NO_RANGE;
    if (front == ULTRASONIC_NO_RANGE || front == 0u || count == 0u ||
        count > SCAN_CAPACITY) { return; }
    obstacle_point_t resolved[SCAN_CAPACITY];
    uint32_t centre = count;
    for (uint32_t i = 0; i < count; ++i) {
        if (points[i].angle_deg == -SERVO_RANGE_DEG &&
            has_range(&points[i])) {
            p->left_ray_mm = points[i].range_mm;
        }
        if (points[i].angle_deg == SERVO_RANGE_DEG &&
            has_range(&points[i])) {
            p->right_ray_mm = points[i].range_mm;
        }
        if (points[i].angle_deg == 0) { centre = i; }
    }
    if (centre == count) { return; }
    p->centre_seen = has_range(&points[centre]);
    if (!p->centre_seen) {
        uint32_t nearest = SERVO_RANGE_DEG + 1u;
        uint32_t reference = front;
        for (uint32_t i = 0u; i < count; ++i) {
            int32_t angle = points[i].angle_deg;
            uint32_t distance = (uint32_t)(angle < 0 ? -angle : angle);
            if (has_range(&points[i]) && distance < nearest &&
                forward_depth(&points[i]) <= OBSTACLE_CLEAR_MM) {
                nearest = distance;
                reference = (uint32_t)lroundf(forward_depth(&points[i]));
            }
        }
        memcpy(resolved, points, count * sizeof(*points));
        resolved[centre].range_mm = reference;
        resolved[centre].kind = OBSTACLE_RETURN_WEAK_RANGE;
        points = resolved;
    }
    if (points[centre].range_mm > OBSTACLE_CLEAR_MM) { return; }
    p->front_mm = points[centre].range_mm;
    uint32_t left, right, left_outer = centre, right_outer = centre;
    p->left_bounded = edge_bracket(points, count, centre, -1,
                                  &left, &left_outer, &p->left_status);
    p->right_bounded = edge_bracket(points, count, centre, 1,
                                   &right, &right_outer, &p->right_status);
    float left_depth = forward_depth(&points[left]);
    float right_depth = forward_depth(&points[right]);
    p->left_observed_mm = (int32_t)lroundf(lateral(left_depth, points[left].angle_deg));
    p->right_observed_mm = (int32_t)lroundf(lateral(right_depth, points[right].angle_deg));
    if (p->left_bounded) {
        p->left_mm = edge_geometry(&points[left], &points[left_outer],
                                   &p->left_edge, &p->left_angle_x10);
    }
    if (p->right_bounded) {
        p->right_mm = edge_geometry(&points[right], &points[right_outer],
                                    &p->right_edge, &p->right_angle_x10);
    }
    if (p->left_bounded && p->right_bounded) {
        p->width_mm = (uint32_t)(p->right_mm - p->left_mm);
        p->width_min_mm = (uint32_t)(p->right_observed_mm - p->left_observed_mm);
        float left_bound = lateral(left_depth, points[left_outer].angle_deg);
        float right_bound = lateral(right_depth, points[right_outer].angle_deg);
        p->width_max_mm = (uint32_t)ceilf(right_bound - left_bound);
        float middle_x = (p->left_mm + p->right_mm) / 2.0f;
        float middle_y = (left_depth + right_depth) / 2.0f;
        p->middle_x_mm = (int32_t)lroundf(middle_x);
        p->middle_y_mm = (uint32_t)lroundf(middle_y);
        p->middle_distance_mm = (uint32_t)lroundf(hypotf(middle_x, middle_y));
        p->edges_confirmed = p->left_status == OBSTACLE_EDGE_CONFIRMED &&
                             p->right_status == OBSTACLE_EDGE_CONFIRMED;
        p->exceeds_expected_width = p->width_min_mm > OBSTACLE_EXPECTED_WIDTH_MM;
        if (p->edges_confirmed) {
            bypass_candidate(p, left_bound, right_bound);
        }
        calibrated_width(p); // Separate from all conservative safety geometry.
    }
}

char const * obstacle_edge_status_name(obstacle_edge_status_t status)
{
    switch (status) {
    case OBSTACLE_EDGE_NO_ECHO: return "missing echoes";
    case OBSTACLE_EDGE_UNRELIABLE: return "unreliable observations";
    case OBSTACLE_EDGE_CONFIRMED: return "echo-bracketed";
    case OBSTACLE_EDGE_SCAN_LIMIT: return "estimated at scan limit";
    case OBSTACLE_EDGE_ESTIMATED: return "estimated distance jump";
    case OBSTACLE_EDGE_UNOBSERVED: return "edge not observed";
    }
    return "unknown";
}

static void scan_profile_build(void)
{
    uint32_t front = g_profile.front_mm;
    if (g_manual_scan) {
        // Empty diagnostics must not create a virtual centre from an old
        // distance or a stray side echo. Plot every ray, but identify geometry
        // only from an actual, current centre reading in the foreground band.
        front = ULTRASONIC_NO_RANGE;
        for (uint32_t i = 0u; i < g_count; ++i) {
            if (g_points[i].angle_deg == 0 && has_range(&g_points[i]) &&
                g_points[i].range_mm <= OBSTACLE_CLEAR_MM) {
                front = g_points[i].range_mm;
                break;
            }
        }
    }
    // Automatic scans retain their original foreground/fallback behaviour.
    obstacle_profile_build(g_points, g_count, front, &g_profile);
}

static void queue_angle(int32_t angle)
{
    // Skip already measured and queued angles, including the centre.
    for (uint32_t i = 0u; i < g_count; ++i) {
        if (g_points[i].angle_deg == angle) { return; }
    }
    for (uint32_t i = 0u; i < g_queue_count; ++i) {
        if (g_queue[i] == angle) { return; }
    }
    if (g_queue_count < SCAN_CAPACITY) { g_queue[g_queue_count++] = angle; }
}

static void aim(void)
{
    servo_set_angle(SERVO_CENTER_DEG + g_queue[g_queue_pos]);
    // Centre may already be selected: force a fresh first observation too.
    ultrasonic_invalidate();
    g_sequence = ultrasonic_sample_sequence();
    g_samples = g_valid = 0u;
}

void
obstacle_scan_start(uint32_t forward_mm, uint32_t now_msec)
{
    g_manual_scan = false;
    memset(&g_profile, 0, sizeof(g_profile));
    g_profile.front_mm = forward_mm;
    g_count = g_queue_count = g_queue_pos = 0u;
    g_started = now_msec;
    g_active = true;
    g_scan_direction = -1;
    g_searching_right = false;

    queue_angle(0);
    queue_angle(-OBSTACLE_SCAN_FINE_DEG);

    log_obstacle("SCAN: centre-outward search, left then right.");
    log_obstacle("SCAN start: front=%lu mm",
                 (unsigned long)forward_mm);
    aim();
}

void
obstacle_scan_start_manual(uint32_t now_msec)
{
    obstacle_scan_start(ULTRASONIC_NO_RANGE, now_msec);
    g_manual_scan = true;
    log_obstacle("SCAN manual: full-range diagnostics; no initial distance gate.");
}

static void store_point(void)
{
    obstacle_point_t p = { .angle_deg = g_queue[g_queue_pos],
        .range_mm = ULTRASONIC_NO_RANGE, .kind = OBSTACLE_RETURN_UNKNOWN };
    if (g_valid == 0u) {
        p.kind = OBSTACLE_RETURN_NO_ECHO;
    } else {
        for (uint32_t i = 0u; i < g_valid; ++i) {
            for (uint32_t j = i + 1u; j < g_valid; ++j) {
                if (g_values[j] < g_values[i]) {
                    uint32_t v = g_values[i]; g_values[i] = g_values[j]; g_values[j] = v;
                }
            }
        }
        p.range_mm = (g_valid & 1u) ? g_values[g_valid / 2u] :
            (g_values[g_valid / 2u - 1u] + g_values[g_valid / 2u]) / 2u;
        // Keep EVERY available distance, including a single valid echo.
        // Quality affects clearance safety only, never graph visibility.
        bool stable = g_valid >= 2u &&
            g_values[1] - g_values[0] <= OBSTACLE_SCAN_STABLE_SPREAD_MM;
        if (g_valid == 3u) {
            stable = stable || g_values[2] - g_values[1] <=
                               OBSTACLE_SCAN_STABLE_SPREAD_MM;
        }
        p.kind = stable ? OBSTACLE_RETURN_RANGE : OBSTACLE_RETURN_WEAK_RANGE;
    }
    // Maintain angle ordering for geometry and exported profile points.
    uint32_t pos = g_count;
    while (pos > 0u && g_points[pos - 1u].angle_deg > p.angle_deg) {
        g_points[pos] = g_points[pos - 1u]; --pos;
    }
    g_points[pos] = p;
    ++g_count;
    log_obstacle("SCAN echoes: angle=%ld valid=%lu/3 min=%ld max=%ld mm",
        (long)p.angle_deg, (unsigned long)g_valid,
        g_valid ? (long)g_values[0] : -1L,
        g_valid ? (long)g_values[g_valid - 1u] : -1L);
    log_obstacle("SCAN point: t=%lu ms angle=%ld range=%ld mm return=%u",
        (unsigned long)to_ms_since_boot(get_absolute_time()),
        (long)p.angle_deg,
        has_range(&p) ? (long)p.range_mm : -1L,
        (unsigned)p.kind);
}

static void finish(bool timeout)
{
    scan_profile_build();
    g_profile.finished = true;
    g_profile.timed_out = timeout;
    g_active = false;
    servo_set_angle(SERVO_CENTER_DEG);
    if (g_profile.left_bounded) {
        log_obstacle("SCAN left edge: x=%ld y=%lu range=%lu mm; %ld to %ld deg (%s)",
            (long)g_profile.left_mm, (unsigned long)g_profile.left_edge.forward_mm,
            (unsigned long)g_profile.left_edge.distance_mm,
            (long)g_profile.left_edge.near_angle_deg, (long)g_profile.left_edge.far_angle_deg,
            obstacle_edge_status_name(g_profile.left_status));
    } else {
        log_obstacle("SCAN left edge UNKNOWN (%s); observed foreground reaches %ld mm.",
            obstacle_edge_status_name(g_profile.left_status), (long)g_profile.left_observed_mm);
    }
    if (g_profile.right_bounded) {
        log_obstacle("SCAN right edge: x=%ld y=%lu range=%lu mm; %ld to %ld deg (%s)",
            (long)g_profile.right_mm, (unsigned long)g_profile.right_edge.forward_mm,
            (unsigned long)g_profile.right_edge.distance_mm,
            (long)g_profile.right_edge.near_angle_deg, (long)g_profile.right_edge.far_angle_deg,
            obstacle_edge_status_name(g_profile.right_status));
    } else {
        log_obstacle("SCAN right edge UNKNOWN (%s); observed foreground reaches %ld mm.",
            obstacle_edge_status_name(g_profile.right_status), (long)g_profile.right_observed_mm);
    }
    if (g_profile.left_bounded && g_profile.right_bounded) {
        log_obstacle("SCAN %swidth=%lu mm; sampling interval=%lu..%lu mm",
            g_profile.edges_confirmed ? "" : "apparent ",
            (unsigned long)g_profile.width_mm, (unsigned long)g_profile.width_min_mm,
            (unsigned long)g_profile.width_max_mm);
        log_obstacle("SCAN middle: x=%ld y=%lu range=%lu mm; centre ray=%lu mm",
            (long)g_profile.middle_x_mm, (unsigned long)g_profile.middle_y_mm,
            (unsigned long)g_profile.middle_distance_mm, (unsigned long)g_profile.front_mm);
    } else {
        log_obstacle("SCAN width UNKNOWN: one or both edges are outside reliable observations.");
    }
    if (timeout) {
        g_profile.bypass.available = false;
        log_obstacle("SCAN timed out; profile is incomplete.");
    }
    obstacle_bypass_candidate_t const * path = &g_profile.bypass;
    if (path->available) {
        log_obstacle("SCAN bypass candidate: left x=%ld, right x=%ld, preferred x=%ld mm.",
            (long)path->left_x_mm, (long)path->right_x_mm, (long)path->preferred_x_mm);
        log_obstacle("SCAN alongside y=%lu mm; return needs measured rear y + %lu mm.",
            (unsigned long)path->alongside_y_mm, (unsigned long)path->turn_clearance_mm);
        log_obstacle("SCAN bypass corridor unverified. Staying braked.");
    } else {
        log_obstacle("SCAN no bypass candidate. Staying braked.");
    }
}

void
obstacle_scan_task(uint32_t now_msec)
{
    if (!g_active) { return; }
    if (now_msec - g_started >= OBSTACLE_SCAN_TIMEOUT_MSEC) { finish(true); return; }
    if (!servo_is_settled() || !ultrasonic_measurement_ready()) { return; }
    uint32_t sequence = ultrasonic_sample_sequence();
    if (sequence == g_sequence) { return; }
    g_sequence = sequence;
    uint32_t range = ultrasonic_range_mm();
    if (range > 0u && range != ULTRASONIC_NO_RANGE) { g_values[g_valid++] = range; }
    if (++g_samples < OBSTACLE_SCAN_SAMPLES) { return; }
    store_point();
    // Rebuild live geometry from the accumulated points; finish() marks the
    // final profile. A missing centre must not prevent the full angular scan.
    scan_profile_build();
    int32_t measured_angle = g_queue[g_queue_pos];

    bool side_finished =
    measured_angle == g_scan_direction * SERVO_RANGE_DEG;

    ++g_queue_pos;

    if (side_finished && !g_searching_right) {
        g_searching_right = true;
        g_scan_direction = 1;
        queue_angle(OBSTACLE_SCAN_FINE_DEG);
        log_obstacle("SCAN: left search complete; searching right.");
    } else if (side_finished && g_searching_right) {
        finish(false);
        return;
    } else if (g_queue_pos == g_queue_count) {
        int32_t next = measured_angle +
                       g_scan_direction * OBSTACLE_SCAN_FINE_DEG;

        if (next >= -SERVO_RANGE_DEG &&
            next <= SERVO_RANGE_DEG) {
            queue_angle(next);
        } else {
            finish(false);
            return;
        }
    }

    if (g_queue_pos == g_queue_count) {
        finish(false);
    } else {
        aim();
    }
}

void obstacle_scan_cancel(void)
{
    if (g_active) { g_profile.cancelled = true; }
    g_active = false;
    g_profile.bypass.available = false;
    servo_set_angle(SERVO_CENTER_DEG);
}
bool obstacle_scan_active(void) { return g_active; }
obstacle_profile_t const * obstacle_profile(void) { return &g_profile; }
obstacle_point_t const * obstacle_scan_points(uint32_t * count)
{
    *count = g_count;
    return g_points;
}

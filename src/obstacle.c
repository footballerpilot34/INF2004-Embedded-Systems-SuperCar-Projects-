#include <math.h>
#include <string.h>
#include "config.h"
#include "log.h"
#include "obstacle.h"
#include "servo.h"
#include "ultrasonic.h"

#define SCAN_CAPACITY 64u
#define RAD_PER_DEG 0.017453292519943295f
#define SCAN_SPREAD_MM 40u

_Static_assert(SERVO_RANGE_DEG == 30, "Review scan capacity if limits change");
_Static_assert(OBSTACLE_SCAN_SAMPLES == 3u, "Median filter uses three pings");
_Static_assert(OBSTACLE_SCAN_COARSE_DEG > 0 && OBSTACLE_SCAN_FINE_DEG > 0,
               "Scan steps must advance");
_Static_assert(SERVO_RANGE_DEG % OBSTACLE_SCAN_COARSE_DEG == 0 &&
               OBSTACLE_SCAN_COARSE_DEG % OBSTACLE_SCAN_FINE_DEG == 0,
               "Scan grid must include centre, limits and fine subdivisions");
_Static_assert(2 * SERVO_RANGE_DEG / OBSTACLE_SCAN_FINE_DEG + 1 <= SCAN_CAPACITY,
               "Scan grid exceeds storage");

static obstacle_point_t g_points[SCAN_CAPACITY];
static uint32_t g_count;
static int32_t g_queue[SCAN_CAPACITY];
static uint32_t g_queue_count, g_queue_pos;
static uint32_t g_values[OBSTACLE_SCAN_SAMPLES], g_samples, g_valid;
static uint32_t g_sequence, g_started;
static bool g_active, g_fine;
static obstacle_profile_t g_profile;

static bool foreground(obstacle_point_t const * p, uint32_t front)
{
    return p->kind == OBSTACLE_RETURN_RANGE &&
        p->range_mm * cosf(p->angle_deg * RAD_PER_DEG) <=
            front + OBSTACLE_SCAN_PLANE_MM;
}

// An inconsistent fine reading cannot erase a previously observed coarse
// bracket. Walk past unknown rays to the next observation, retaining the
// full angular interval instead of pretending the missing ray is clear.
static bool edge_bracket(obstacle_point_t const * points, uint32_t count,
                         uint32_t centre, uint32_t front, int32_t step,
                         uint32_t * inner, uint32_t * outer)
{
    *inner = centre;
    for (int32_t i = (int32_t)centre + step;
         i >= 0 && i < (int32_t)count; i += step) {
        if (points[i].kind == OBSTACLE_RETURN_UNKNOWN) { continue; }
        if (!foreground(&points[i], front)) {
            *outer = (uint32_t)i;
            return true;
        }
        *inner = (uint32_t)i;
    }
    return false;
}

static int32_t lateral(uint32_t front, float angle)
{
    return (int32_t)lroundf(front * tanf(angle * RAD_PER_DEG));
}

void
obstacle_profile_build(obstacle_point_t const * points, uint32_t count,
                       uint32_t front, obstacle_profile_t * p)
{
    memset(p, 0, sizeof(*p));
    p->front_mm = front;
    p->point_count = count;
    p->left_ray_mm = p->right_ray_mm = ULTRASONIC_NO_RANGE;
    if (front == ULTRASONIC_NO_RANGE || count == 0u) { return; }
    uint32_t centre = count;
    for (uint32_t i = 0; i < count; ++i) {
        if (points[i].angle_deg == -SERVO_RANGE_DEG &&
            points[i].kind == OBSTACLE_RETURN_RANGE) {
            p->left_ray_mm = points[i].range_mm;
        }
        if (points[i].angle_deg == SERVO_RANGE_DEG &&
            points[i].kind == OBSTACLE_RETURN_RANGE) {
            p->right_ray_mm = points[i].range_mm;
        }
        if (points[i].angle_deg == 0) { centre = i; }
    }
    if (centre == count || !foreground(&points[centre], front)) { return; }
    p->centre_seen = true;
    uint32_t left, right, left_outer = centre, right_outer = centre;
    // No-echo brackets permit a tentative estimate, never confirmed clearance.
    p->left_bounded = edge_bracket(points, count, centre, front, -1, &left, &left_outer);
    p->right_bounded = edge_bracket(points, count, centre, front, 1, &right, &right_outer);
    p->left_observed_mm = lateral(front, points[left].angle_deg);
    p->right_observed_mm = lateral(front, points[right].angle_deg);
    if (p->left_bounded) {
        p->left_angle_x10 = (points[left].angle_deg + points[left_outer].angle_deg) * 5;
        p->left_mm = lateral(front, p->left_angle_x10 / 10.0f);
    }
    if (p->right_bounded) {
        p->right_angle_x10 = (points[right].angle_deg + points[right_outer].angle_deg) * 5;
        p->right_mm = lateral(front, p->right_angle_x10 / 10.0f);
    }
    if (p->left_bounded && p->right_bounded) {
        p->width_mm = (uint32_t)(p->right_mm - p->left_mm);
        p->width_min_mm = (uint32_t)(lateral(front, points[right].angle_deg) -
                                     lateral(front, points[left].angle_deg));
        p->width_max_mm = (uint32_t)(lateral(front, points[right_outer].angle_deg) -
                                     lateral(front, points[left_outer].angle_deg));
        p->edges_confirmed = points[left_outer].kind == OBSTACLE_RETURN_RANGE &&
                             points[right_outer].kind == OBSTACLE_RETURN_RANGE;
        p->exceeds_expected_width = p->width_min_mm > OBSTACLE_EXPECTED_WIDTH_MM;
    }
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
    memset(&g_profile, 0, sizeof(g_profile));
    g_profile.front_mm = forward_mm;
    g_count = g_queue_count = g_queue_pos = 0u;
    g_started = now_msec;
    g_active = true;
    g_fine = false;
    queue_angle(0);
    for (int32_t a = -SERVO_RANGE_DEG; a <= SERVO_RANGE_DEG;
         a += OBSTACLE_SCAN_COARSE_DEG) { queue_angle(a); }
    log_obstacle("SCAN: wheels braked; coarse sweep 60-120 deg, then fine edges.");
    log_obstacle("SCAN start: front=%lu mm, sweep span at this depth=%ld mm (expected width <=300).",
        (unsigned long)forward_mm, (long)(2 * lateral(forward_mm, SERVO_RANGE_DEG)));
    aim();
}

static void store_point(void)
{
    obstacle_point_t p = { .angle_deg = g_queue[g_queue_pos],
        .range_mm = ULTRASONIC_NO_RANGE, .kind = OBSTACLE_RETURN_UNKNOWN };
    if (g_valid == 0u) {
        p.kind = OBSTACLE_RETURN_NO_ECHO;
    } else if (g_valid >= 2u) {
        for (uint32_t i = 0u; i < g_valid; ++i) {
            for (uint32_t j = i + 1u; j < g_valid; ++j) {
                if (g_values[j] < g_values[i]) {
                    uint32_t v = g_values[i]; g_values[i] = g_values[j]; g_values[j] = v;
                }
            }
        }
        // Two agreeing echoes out of three survive a single multipath outlier.
        bool stable = g_values[1] - g_values[0] <= SCAN_SPREAD_MM;
        if (g_valid == 3u) {
            stable = stable || g_values[2] - g_values[1] <= SCAN_SPREAD_MM;
        }
        if (stable) {
            p.kind = OBSTACLE_RETURN_RANGE;
            p.range_mm = g_valid == 3u ? g_values[1] :
                (g_values[0] + g_values[1]) / 2u;
        }
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
    log_obstacle("SCAN point: angle=%ld range=%ld mm return=%u (0 unknown, 1 echo, 2 no echo)",
        (long)p.angle_deg, p.kind == OBSTACLE_RETURN_RANGE ? (long)p.range_mm : -1L,
        (unsigned)p.kind);
}

static void finish(bool timeout)
{
    uint32_t front = g_profile.front_mm;
    obstacle_profile_build(g_points, g_count, front, &g_profile);
    g_profile.finished = true;
    g_profile.timed_out = timeout;
    g_active = false;
    servo_set_angle(SERVO_CENTER_DEG);
    if (g_profile.left_bounded) {
        log_obstacle("SCAN left edge: %ld mm, angle=%ld/10 deg", (long)g_profile.left_mm,
            (long)g_profile.left_angle_x10);
    } else {
        log_obstacle("SCAN left edge UNKNOWN; observed foreground reaches %ld mm; scan limit/gap.",
            (long)g_profile.left_observed_mm);
    }
    if (g_profile.right_bounded) {
        log_obstacle("SCAN right edge: %ld mm, angle=%ld/10 deg", (long)g_profile.right_mm,
            (long)g_profile.right_angle_x10);
    } else {
        log_obstacle("SCAN right edge UNKNOWN; observed foreground reaches %ld mm; scan limit/gap.",
            (long)g_profile.right_observed_mm);
    }
    if (g_profile.left_bounded && g_profile.right_bounded) {
        log_obstacle("SCAN width=%lu mm; sampling interval=%lu..%lu mm; echo brackets=%u",
            (unsigned long)g_profile.width_mm, (unsigned long)g_profile.width_min_mm,
            (unsigned long)g_profile.width_max_mm, (unsigned)g_profile.edges_confirmed);
    } else {
        log_obstacle("SCAN width UNKNOWN: one or both edges are outside reliable observations.");
    }
    if (timeout) { log_obstacle("SCAN timed out; profile is incomplete."); }
    log_obstacle("SCAN: depth unknown; bypass awaits encoder calibration. Staying braked.");
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
    if (range != ULTRASONIC_NO_RANGE) { g_values[g_valid++] = range; }
    if (++g_samples < OBSTACLE_SCAN_SAMPLES) { return; }
    store_point();
    // The centre is measured first. Do not scan a vanished or unstable target.
    if (g_queue_pos == 0u) {
        if (g_points[0].kind != OBSTACLE_RETURN_RANGE ||
            g_points[0].range_mm > OBSTACLE_CLEAR_MM) { finish(false); return; }
        g_profile.front_mm = g_points[0].range_mm;
    }
    ++g_queue_pos;
    if (g_queue_pos == g_queue_count && !g_fine) {
        g_fine = true;
        // Refine coarse foreground-to-background/unknown transitions.
        // The profile uses only the contiguous foreground around centre.
        for (uint32_t i = 0u; i + 1u < g_count; ++i) {
            if (foreground(&g_points[i], g_profile.front_mm) !=
                foreground(&g_points[i + 1u], g_profile.front_mm)) {
                for (int32_t a = g_points[i].angle_deg + OBSTACLE_SCAN_FINE_DEG;
                     a < g_points[i + 1u].angle_deg; a += OBSTACLE_SCAN_FINE_DEG) {
                    queue_angle(a);
                }
            }
        }
    }
    if (g_queue_pos == g_queue_count) { finish(false); }
    else { aim(); }
}

void obstacle_scan_cancel(void)
{
    g_active = false;
    g_count = 0u;
    memset(&g_profile, 0, sizeof(g_profile));
    servo_set_angle(SERVO_CENTER_DEG);
}
bool obstacle_scan_active(void) { return g_active; }
obstacle_profile_t const * obstacle_profile(void) { return &g_profile; }
obstacle_point_t const * obstacle_scan_points(uint32_t * count)
{
    *count = g_count;
    return g_points;
}

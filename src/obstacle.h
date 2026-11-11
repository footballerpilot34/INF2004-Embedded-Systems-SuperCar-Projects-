/** Stopped coarse/fine ultrasonic scan. Coordinates: right +x, forward +y. */
#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    OBSTACLE_RETURN_UNKNOWN,       // Inconsistent/insufficient echoes.
    OBSTACLE_RETURN_RANGE,         // Majority of stable, valid echoes.
    OBSTACLE_RETURN_NO_ECHO        // All pings timed out; NOT proven clear.
} obstacle_return_t;

typedef struct {
    int32_t angle_deg;             // Relative to forward; positive = right.
    uint32_t range_mm;
    obstacle_return_t kind;
} obstacle_point_t;

typedef struct {
    bool finished;
    bool timed_out;
    bool centre_seen;
    bool left_bounded;
    bool right_bounded;
    bool edges_confirmed;          // Both outside brackets have valid echoes.
    bool exceeds_expected_width;
    uint32_t front_mm;
    int32_t left_mm;               // Estimated edge at angular bracket midpoint.
    int32_t right_mm;
    int32_t left_observed_mm;      // Farthest observed foreground, not an edge.
    int32_t right_observed_mm;
    int32_t left_angle_x10;
    int32_t right_angle_x10;
    uint32_t width_mm;             // Only meaningful with BOTH bounded flags.
    uint32_t width_min_mm;         // Angular sampling interval, not accuracy.
    uint32_t width_max_mm;
    uint32_t left_ray_mm;          // Endpoint ray ranges; NOT corridor clearance.
    uint32_t right_ray_mm;
    uint32_t point_count;
} obstacle_profile_t;

void obstacle_scan_start(uint32_t forward_mm, uint32_t now_msec);
void obstacle_scan_task(uint32_t now_msec);
void obstacle_scan_cancel(void);
bool obstacle_scan_active(void);
obstacle_profile_t const * obstacle_profile(void);
obstacle_point_t const * obstacle_scan_points(uint32_t * count);
// Pure geometry helper; input points must be sorted by relative angle.
void obstacle_profile_build(obstacle_point_t const * points, uint32_t count,
                            uint32_t forward_mm, obstacle_profile_t * profile);

#endif

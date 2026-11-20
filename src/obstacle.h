/** Stopped ultrasonic scan. Coordinates: right +x, forward +y. */
#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    OBSTACLE_RETURN_UNKNOWN,       // Inconsistent/insufficient echoes.
    OBSTACLE_RETURN_RANGE,         // Majority of stable, valid echoes.
    OBSTACLE_RETURN_NO_ECHO,       // All pings timed out; NOT proven clear.
    OBSTACLE_RETURN_WEAK_RANGE     // Actual median, but sparse/unstable echoes.
} obstacle_return_t;

typedef struct {
    int32_t angle_deg;             // Relative to forward; positive = right.
    uint32_t range_mm;
    obstacle_return_t kind;
} obstacle_point_t;

typedef enum {
    OBSTACLE_EDGE_UNOBSERVED,
    OBSTACLE_EDGE_NO_ECHO,         // Apparent edge only; no proven clearance.
    OBSTACLE_EDGE_UNRELIABLE,      // Gaps, disagreement or insufficient evidence.
    OBSTACLE_EDGE_CONFIRMED,       // Adjacent, consistent far rays; no bracket gaps.
    OBSTACLE_EDGE_SCAN_LIMIT,      // Range ended without an observed transition.
    OBSTACLE_EDGE_ESTIMATED        // First jump; clearance not independently verified.
} obstacle_edge_status_t;

typedef struct {
    int32_t near_angle_deg;
    int32_t far_angle_deg;
    uint32_t forward_mm;
    uint32_t distance_mm;          // Estimated straight-line range to the edge.
} obstacle_edge_geometry_t;

typedef struct {
    bool available;               // Geometry only; requires corridor checks.
    int32_t left_x_mm;
    int32_t right_x_mm;
    int32_t preferred_x_mm;       // Side with the smaller lateral shift.
    uint32_t alongside_y_mm;      // Station beyond the observed front.
    uint32_t turn_clearance_mm;   // Add to measured rear depth before return.
} obstacle_bypass_candidate_t;

typedef struct {
    bool finished;
    bool timed_out;
    bool cancelled;               // Points retained for the dashboard.
    bool centre_seen;
    bool left_bounded;             // Angular bracket may end in missing echoes.
    bool right_bounded;
    bool edges_confirmed;          // Both edge transitions have reliable echoes.
    obstacle_edge_status_t left_status;
    obstacle_edge_status_t right_status;
    bool exceeds_expected_width;
    uint32_t front_mm;
    int32_t left_mm;               // Estimated edge at angular bracket midpoint.
    int32_t right_mm;
    int32_t left_observed_mm;      // Farthest observed foreground, not an edge.
    int32_t right_observed_mm;
    int32_t left_angle_x10;
    int32_t right_angle_x10;
    obstacle_edge_geometry_t left_edge;
    obstacle_edge_geometry_t right_edge;
    int32_t middle_x_mm;
    uint32_t middle_y_mm;
    uint32_t middle_distance_mm;  // Range to the midpoint of the visible edges.
    uint32_t width_mm;             // Only meaningful with BOTH bounded flags.
    uint32_t calibrated_width_mm;  // 150 or 250 mm from measured reference scans.
    bool calibrated_width_valid;   // False means unknown; raw geometry is retained.
    uint32_t width_min_mm;         // Angular sampling interval, not accuracy.
    uint32_t width_max_mm;
    uint32_t left_ray_mm;          // Endpoint ray ranges; NOT corridor clearance.
    uint32_t right_ray_mm;
    uint32_t point_count;
    obstacle_bypass_candidate_t bypass;
} obstacle_profile_t;

void obstacle_scan_start(uint32_t forward_mm, uint32_t now_msec);
void obstacle_scan_task(uint32_t now_msec);
void obstacle_scan_cancel(void);
bool obstacle_scan_active(void);
obstacle_profile_t const * obstacle_profile(void);
char const * obstacle_edge_status_name(obstacle_edge_status_t status);
obstacle_point_t const * obstacle_scan_points(uint32_t * count);
// Pure geometry helper; input points must be sorted by relative angle.
void obstacle_profile_build(obstacle_point_t const * points, uint32_t count,
                            uint32_t forward_mm, obstacle_profile_t * profile);

#endif

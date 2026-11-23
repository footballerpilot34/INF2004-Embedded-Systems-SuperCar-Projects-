/* Pure geometry for the existing pivot/straight bypass. x right, y forward;
 * heading is relative to the initial forward direction. No hardware calls. */
#ifndef BYPASS_GEOMETRY_H
#define BYPASS_GEOMETRY_H
#include <math.h>
#include <stdbool.h>
#include "config.h"
#define BG_RAD 0.017453292519943295f

typedef struct {
    float left, right, front, depth; /* Obstacle, front relative to start bumper. */
    float offset, departure, return_deg, departure_mm, departure_y;
    float clear_y, return_mm, translation_mm, wheel_mm;
} bypass_geometry_t;

/* Largest separating projection gap is a LOWER bound on Euclidean clearance.
 * Axes of both rectangles suffice for collision; requiring a positive margin
 * is conservative near corners. Axle-to-front is measured, not half-length. */
static inline float
bypass_geometry_gap_envelope(bypass_geometry_t const *p, float x, float y, float heading, float half_y)
{
    float s = sinf(heading * BG_RAD), c = cosf(heading * BG_RAD);
    float k = BYPASS_AXLE_TO_FRONT_MM - CAR_LENGTH_MM / 2.0f;
    float cx = x + k * s;
    float cy = y + k * c - BYPASS_AXLE_TO_FRONT_MM;
    float axes[4][2] = {{1,0}, {0,1}, {c,-s}, {s,c}};
    float gap = -1e9f;
    for (unsigned i = 0; i < 4; ++i) {
        float ux = axes[i][0], uy = axes[i][1];
        float ce = CAR_WIDTH_MM / 2.0f * fabsf(ux*c-uy*s) +
                   CAR_LENGTH_MM / 2.0f * fabsf(ux*s+uy*c) + half_y*fabsf(uy);
        float cp = cx*ux + cy*uy;
        float op = (p->left+p->right)/2.0f*ux + (p->front+p->depth/2.0f)*uy;
        float oe = (p->right-p->left)/2.0f*fabsf(ux) + p->depth/2.0f*fabsf(uy);
        gap = fmaxf(gap, fabsf(cp-op)-ce-oe);
    }
    return gap;
}

static inline float
bypass_geometry_gap(bypass_geometry_t const *p, float x, float y, float heading)
{
    return bypass_geometry_gap_envelope(p,x,y,heading,0);
}

static inline float bypass_geometry_radius(void)
{
    float longitudinal = fmaxf(BYPASS_AXLE_TO_FRONT_MM,
                              CAR_LENGTH_MM - BYPASS_AXLE_TO_FRONT_MM);
    return hypotf(longitudinal, CAR_WIDTH_MM / 2.0f);
}

static inline float bypass_geometry_margin(void)
{
    /* Unknown slip is not claimed bounded by this allowance: calibrate on car.
     * This reserves configured position and +/-5 degree heading uncertainty. */
    return OBSTACLE_BYPASS_MARGIN_MM + BYPASS_POSITION_UNCERTAINTY_MM +
        2.0f*bypass_geometry_radius()*sinf(BYPASS_ALIGN_TOL_DEG*BG_RAD/2.0f);
}

/* y intervals bound the complete family of straight legs tracked within
 * +/-DRIVE_HEADING_TOL. Project this vertical corridor onto each SAT axis,
 * rather than inflating equally in every direction and wasting side room. */
static inline bool
bypass_geometry_corridor(bypass_geometry_t const *p,
                         float x0, float low0, float high0, float h0,
                         float x1, float low1, float high1, float h1)
{
    float travel=fmaxf(hypotf(x1-x0,low1-low0),hypotf(x1-x0,high1-high0));
    float turn=fabsf(h1-h0);
    unsigned n=(unsigned)ceilf(fmaxf(travel/BYPASS_GEOMETRY_STEP_MM,
                                    turn/BYPASS_GEOMETRY_STEP_DEG));
    if (!n) { n=1; }
    // Every chassis point is within this bound of a sampled envelope.
    float between=(travel+bypass_geometry_radius()*turn*BG_RAD)/(2.0f*n);
    float tracking_rotation=2*bypass_geometry_radius()*
        sinf(BYPASS_DRIVE_HEADING_TOL_DEG*BG_RAD/2);
    float margin=bypass_geometry_margin()+between+tracking_rotation;
    for (unsigned i=0;i<=n;++i) {
        float u=(float)i/n;
        float low=low0+(low1-low0)*u, high=high0+(high1-high0)*u;
        if (bypass_geometry_gap_envelope(p,x0+(x1-x0)*u,(low+high)/2,
                                        h0+(h1-h0)*u,(high-low)/2)<margin) {
            return false;
        }
    }
    return true;
}

static inline void
bypass_geometry_init(bypass_geometry_t *p, float front, float left, float right)
{
    *p = (bypass_geometry_t){.left=left, .right=right, .front=front,
                             .depth=BYPASS_ASSUMED_DEPTH_MM};
    p->offset = BYPASS_COMPACT_OFFSET_MM;
    /* At zero heading: rear bumper is clear after front gap + depth + length.
     * Preserve a positive whole-car clearance rather than trim a magic distance. */
    p->clear_y = front + p->depth + CAR_LENGTH_MM + OBSTACLE_BYPASS_MARGIN_MM;
    // Fixed compact return; the complete route is validated before movement.
    p->return_deg=BYPASS_COMPACT_RETURN_DEG;
    p->return_mm=p->offset/sinf(p->return_deg*BG_RAD);
}

/* Return feasibility depends only on the chosen pass offset/station, not on
 * departure angle. Search this separately, then departure costs are monotone. */
static inline bool
bypass_geometry_return_safe(bypass_geometry_t const *p, float b)
{
    float d=p->offset, y=p->clear_y;
    float low=y+d/tanf((b+BYPASS_DRIVE_HEADING_TOL_DEG)*BG_RAD);
    float high=y+d/tanf((b-BYPASS_DRIVE_HEADING_TOL_DEG)*BG_RAD);
    return bypass_geometry_corridor(p,-d,y,y,0,-d,y,y,b) &&
           bypass_geometry_corridor(p,-d,y,y,b,0,low,high,b) &&
           bypass_geometry_corridor(p,0,low,high,b,0,low,high,0);
}

/* Validate the fixed pivot/straight route, including uncertainty corridors.
 * Total axle translation = clear_y + offset*tan(a/2) + offset/sin(b). */
static inline bool
bypass_geometry_candidate(bypass_geometry_t *p, float a)
{
    p->departure = a;
    p->departure_mm = p->offset/sinf(a*BG_RAD);
    p->departure_y = p->offset/tanf(a*BG_RAD);
    float d=p->offset, end=p->clear_y, b=p->return_deg;
    float low=d/tanf((a+BYPASS_DRIVE_HEADING_TOL_DEG)*BG_RAD);
    float high=d/tanf((a-BYPASS_DRIVE_HEADING_TOL_DEG)*BG_RAD);
    if (high >= end || end-low > BYPASS_PASS_MAX_MM) { return false; }
    float final_low=end+d/tanf((b+BYPASS_DRIVE_HEADING_TOL_DEG)*BG_RAD);
    float final_high=end+d/tanf((b-BYPASS_DRIVE_HEADING_TOL_DEG)*BG_RAD);
    // Each row: x, low_y, high_y, heading. A common y interval also
    // covers the departure's uncertain arrival station during its next pivot.
    float poses[8][4]={{0,0,0,0},{0,0,0,-a},{-d,low,high,-a},{-d,low,high,0},
                       {-d,end,end,0},{-d,end,end,b},
                       {0,final_low,final_high,b},{0,final_low,final_high,0}};
    for (unsigned i=0;i<7;++i) {
        if (!bypass_geometry_corridor(p,poses[i][0],poses[i][1],poses[i][2],poses[i][3],
                                      poses[i+1][0],poses[i+1][1],poses[i+1][2],poses[i+1][3])) {
            return false;
        }
    }
    p->translation_mm = p->clear_y + d*tanf(a*BG_RAD/2.0f) + p->return_mm;
    p->wheel_mm = p->translation_mm + WHEEL_TRACK_MM/2.0f *
        (BYPASS_PIVOT_LEFT_X100+BYPASS_PIVOT_RIGHT_X100)/100.0f*(a+b)*BG_RAD;
    return true;
}
#endif

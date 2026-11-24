/** @file bypass_geometry.h
 * @brief Shared declarations and configuration for the SuperCar project.
 *
 * NOTE: Remaining standards deviations are tracked in BARR_C_REVIEW.md.
 */
/* Pure geometry for the existing pivot/straight bypass. x right, y forward;
 * heading is relative to the initial forward direction. No hardware calls. */
#ifndef BYPASS_GEOMETRY_H
#define BYPASS_GEOMETRY_H
#include <math.h>
#include <stdbool.h>
#include "config.h"
#define BG_RAD 0.017453292519943295f

typedef struct
{
    float left;
    float right;
    float front;
    float depth; /* Obstacle, front relative to start bumper. */
    float offset;
    float departure;
    float return_deg;
    float departure_mm;
    float departure_y;
    float clear_y;
    float return_mm;
    float translation_mm;
    float wheel_mm;
    float reverse_mm;
} bypass_geometry_t;

/* Largest separating projection gap is a LOWER bound on Euclidean clearance.
 * Axes of both rectangles suffice for collision; requiring a positive margin
 * is conservative near corners. Axle-to-front is measured, not half-length. */
/** @brief Compute a conservative separating-axis footprint clearance. */
static inline float
bypass_geometry_gap_envelope(bypass_geometry_t const * p_value_p, float value_x,
                             float value_y, float heading, float half_y)
{
    float value_s  = sinf(heading * BG_RAD);
    float symbol_c = cosf(heading * BG_RAD);
    float index_k  = BYPASS_AXLE_TO_FRONT_MM - (CAR_LENGTH_MM / 2.0f);
    float value_cx = value_x + (index_k * value_s);
    float value_cy = (value_y + (index_k * symbol_c)) - BYPASS_AXLE_TO_FRONT_MM;
    float axes[4][2] = {
        {1, 0}, {0, 1}, {symbol_c, -value_s}, {value_s, symbol_c}};
    float gap = -1e9f;
    for (unsigned index_i = 0; index_i < 4; ++index_i)
    {
        float value_ux = axes[index_i][0];
        float value_uy = axes[index_i][1];
        float value_ce =
            (((CAR_WIDTH_MM / 2.0f) *
              (fabsf((value_ux * symbol_c) - (value_uy * value_s)))) +
             ((CAR_LENGTH_MM / 2.0f) *
              (fabsf((value_ux * value_s) + (value_uy * symbol_c))))) +
            (half_y * (fabsf(value_uy)));
        float value_cp = (value_cx * value_ux) + (value_cy * value_uy);
        float value_op =
            ((((p_value_p->left) + (p_value_p->right)) / 2.0f) * value_ux) +
            (((p_value_p->front) + ((p_value_p->depth) / 2.0f)) * value_uy);
        float value_oe = ((((p_value_p->right) - (p_value_p->left)) / 2.0f) *
                          (fabsf(value_ux))) +
                         (((p_value_p->depth) / 2.0f) * (fabsf(value_uy)));
        gap = fmaxf(gap, ((fabsf(value_cp - value_op)) - value_ce) - value_oe);
    }
    return gap;
}

/** @brief Evaluate clearance for the configured car footprint. */
static inline float
bypass_geometry_gap(bypass_geometry_t const * p_value_p, float value_x,
                    float value_y, float heading)
{
    return bypass_geometry_gap_envelope(p_value_p, value_x, value_y, heading,
                                        0);
}

/** @brief Return the furthest axle-to-corner footprint radius. */
static inline float
bypass_geometry_radius(void)
{
    float longitudinal =
        fmaxf(BYPASS_AXLE_TO_FRONT_MM, CAR_LENGTH_MM - BYPASS_AXLE_TO_FRONT_MM);
    return hypotf(longitudinal, CAR_WIDTH_MM / 2.0f);
}

/** @brief Return the reserved collision uncertainty margin. */
static inline float
bypass_geometry_margin(void)
{
    /* Unknown slip is not claimed bounded by this allowance: calibrate on car.
     * This reserves configured position and +/-5 degree heading uncertainty. */
    return (OBSTACLE_BYPASS_MARGIN_MM + BYPASS_POSITION_UNCERTAINTY_MM) +
           ((2.0f * (bypass_geometry_radius())) *
            (sinf((BYPASS_ALIGN_TOL_DEG * BG_RAD) / 2.0f)));
}

/* y intervals bound the complete family of straight legs tracked within
 * +/-DRIVE_HEADING_TOL. Project this vertical corridor onto each SAT axis,
 * rather than inflating equally in every direction and wasting side room. */
/** @brief Check a sampled movement corridor against the full footprint. */
static inline bool
bypass_geometry_corridor(bypass_geometry_t const * p_value_p, float value_x0,
                         float low0, float high0, float value_h0,
                         float value_x1, float low1, float high1,
                         float value_h1)
{
    float    travel  = fmaxf(hypotf(value_x1 - value_x0, low1 - low0),
                             hypotf(value_x1 - value_x0, high1 - high0));
    float    turn    = fabsf(value_h1 - value_h0);
    unsigned count_n = (unsigned)ceilf(fmaxf(travel / BYPASS_GEOMETRY_STEP_MM,
                                             turn / BYPASS_GEOMETRY_STEP_DEG));
    if (!count_n)
    {
        count_n = 1;
    }
    // Every chassis point is within this bound of a sampled envelope.
    float between = (travel + (((bypass_geometry_radius()) * turn) * BG_RAD)) /
                    (2.0f * count_n);
    float tracking_rotation =
        (2 * (bypass_geometry_radius())) *
        (sinf((BYPASS_DRIVE_HEADING_TOL_DEG * BG_RAD) / 2));
    float margin = ((bypass_geometry_margin()) + between) + tracking_rotation;
    for (unsigned index_i = 0; index_i <= count_n; ++index_i)
    {
        float value_u = ((float)index_i) / count_n;
        float low     = low0 + ((low1 - low0) * value_u);
        float high    = high0 + ((high1 - high0) * value_u);
        if ((bypass_geometry_gap_envelope(
                p_value_p, value_x0 + ((value_x1 - value_x0) * value_u),
                (low + high) / 2, value_h0 + ((value_h1 - value_h0) * value_u),
                (high - low) / 2)) < margin)
        {
            return false;
        }
    }
    return true;
}

/** @brief Initialize geometry from measured front distance and obstacle edges.
 */
static inline void
bypass_geometry_init(bypass_geometry_t * p_value_p, float front, float left,
                     float right)
{
    *p_value_p = (bypass_geometry_t){.left  = left,
                                     .right = right,
                                     .front = front,
                                     .depth = BYPASS_ASSUMED_DEPTH_MM};
    float error =
        (BYPASS_DRIVE_HEADING_TOL_DEG + BYPASS_ALIGN_TOL_DEG) * BG_RAD;
    float side = ((CAR_WIDTH_MM / 2.0f) * (cosf(error))) +
                 ((CAR_LENGTH_MM - BYPASS_AXLE_TO_FRONT_MM) * (sinf(error)));
    p_value_p->offset =
        fmaxf(BYPASS_COMPACT_OFFSET_MM,
              ceilf((((-left) + side) + BYPASS_POSITION_UNCERTAINTY_MM) +
                    BYPASS_MIN_SIDE_GAP_MM));
    /* At zero heading: rear bumper is clear after front gap + depth + length.
     * Preserve a positive whole-car clearance rather than trim a magic
     * distance. */
    p_value_p->clear_y = ((((front + (p_value_p->depth)) + CAR_LENGTH_MM) +
                           OBSTACLE_BYPASS_MARGIN_MM) +
                          BYPASS_PASS_EXTRA_MM) -
                         BYPASS_PASS_TRIM_MM;
    // Fixed compact return; the complete route is validated before movement.
    p_value_p->return_deg = BYPASS_COMPACT_RETURN_DEG;
    p_value_p->return_mm =
        (p_value_p->offset) / (sinf((p_value_p->return_deg) * BG_RAD));
}

/* Return feasibility depends only on the chosen pass offset/station, not on
 * departure angle. Search this separately, then departure costs are monotone.
 */
/** @brief Validate the return, post-contact advance and final rotation. */
static inline bool
bypass_geometry_return_safe(bypass_geometry_t const * p_value_p,
                            float                     operand_b)
{
    float value_d = p_value_p->offset;
    float value_y = p_value_p->clear_y;
    float low =
        value_y +
        (value_d / (tanf((operand_b + BYPASS_DRIVE_HEADING_TOL_DEG) * BG_RAD)));
    float high =
        value_y +
        (value_d / (tanf((operand_b - BYPASS_DRIVE_HEADING_TOL_DEG) * BG_RAD)));
    return ((bypass_geometry_corridor(p_value_p, -value_d, value_y, value_y, 0,
                                      -value_d, value_y, value_y, operand_b)) &&
            (bypass_geometry_corridor(p_value_p, -value_d, value_y, value_y,
                                      operand_b, 0, low, high, operand_b))) &&
           (bypass_geometry_corridor(p_value_p, 0, low, high, operand_b, 0, low,
                                     high, 0));
}

/* Validate the fixed pivot/straight route, including uncertainty corridors.
 * Total axle translation = clear_y + offset*tan(a/2) + offset/sin(b). */
/** @brief Validate a candidate departure and its complete return route. */
static inline bool
bypass_geometry_candidate(bypass_geometry_t * p_value_p, float operand_a)
{
    float error =
        (BYPASS_DRIVE_HEADING_TOL_DEG + BYPASS_ALIGN_TOL_DEG) * BG_RAD;
    float worst_side =
        ((CAR_WIDTH_MM / 2.0f) * (cosf(error))) +
        ((CAR_LENGTH_MM - BYPASS_AXLE_TO_FRONT_MM) * (sinf(error)));
    if (((((p_value_p->offset) + (p_value_p->left)) - worst_side) -
         BYPASS_POSITION_UNCERTAINTY_MM) < BYPASS_MIN_SIDE_GAP_MM)
    {
        return false;
    }
    p_value_p->departure    = operand_a;
    p_value_p->departure_mm = (p_value_p->offset) / (sinf(operand_a * BG_RAD));
    p_value_p->departure_y  = (p_value_p->offset) / (tanf(operand_a * BG_RAD));
    float value_d           = p_value_p->offset;
    float end               = p_value_p->clear_y;
    float operand_b         = p_value_p->return_deg;
    float initial           = BYPASS_INITIAL_HEADING_DEG;
    float value_x  = (-p_value_p->reverse_mm) * (sinf(initial * BG_RAD));
    float value_y  = (-p_value_p->reverse_mm) * (cosf(initial * BG_RAD));
    float value_dx = value_d + value_x;
    p_value_p->departure_mm = value_dx / (sinf(operand_a * BG_RAD));
    p_value_p->departure_y  = value_y + (value_dx / (tanf(operand_a * BG_RAD)));
    float low =
        value_y + (value_dx /
                   (tanf((operand_a + BYPASS_DRIVE_HEADING_TOL_DEG) * BG_RAD)));
    float high =
        value_y + (value_dx /
                   (tanf((operand_a - BYPASS_DRIVE_HEADING_TOL_DEG) * BG_RAD)));
    if ((high >= end) || ((end - low) > BYPASS_PASS_MAX_MM))
    {
        return false;
    }
    float final_low =
        end +
        (value_d / (tanf((operand_b + BYPASS_DRIVE_HEADING_TOL_DEG) * BG_RAD)));
    float final_high =
        end +
        (value_d / (tanf((operand_b - BYPASS_DRIVE_HEADING_TOL_DEG) * BG_RAD)));
    float poses[9][4] = {{0, 0, 0, initial},
                         {value_x, value_y, value_y, initial},
                         {value_x, value_y, value_y, -operand_a},
                         {-value_d, low, high, -operand_a},
                         {-value_d, low, high, 0},
                         {-value_d, end, end, 0},
                         {-value_d, end, end, operand_b},
                         {0, final_low, final_high, operand_b},
                         {0, final_low, final_high, 0}};
    for (unsigned index_i = 0; index_i < 8; ++index_i)
    {
        if (!bypass_geometry_corridor(
                p_value_p, poses[index_i][0], poses[index_i][1],
                poses[index_i][2], poses[index_i][3], poses[index_i + 1][0],
                poses[index_i + 1][1], poses[index_i + 1][2],
                poses[index_i + 1][3]))
        {
            return false;
        }
    }
    // Normal recovery may deliberately advance farther than the mounting
    // offset. Validate that extra diagonal and final pivot as well as dry x=0.
    float extra = fmaxf(0.0f, ((float)BYPASS_REACQUIRE_ADVANCE_MM) -
                                  BYPASS_LINE_SENSOR_FORWARD_MM);
    if (extra > 0)
    {
        float value_ex = extra * (sinf(operand_b * BG_RAD));
        float value_lo =
            final_low +
            (extra *
             (cosf((operand_b + BYPASS_DRIVE_HEADING_TOL_DEG) * BG_RAD)));
        float value_hi =
            final_high +
            (extra *
             (cosf((operand_b - BYPASS_DRIVE_HEADING_TOL_DEG) * BG_RAD)));
        if ((!bypass_geometry_corridor(p_value_p, 0, final_low, final_high,
                                       operand_b, value_ex, value_lo, value_hi,
                                       operand_b)) ||
            (!bypass_geometry_corridor(p_value_p, value_ex, value_lo, value_hi,
                                       operand_b, value_ex, value_lo, value_hi,
                                       0)))
        {
            return false;
        }
    }
    p_value_p->translation_mm =
        ((((p_value_p->reverse_mm) + (p_value_p->departure_mm)) + end) -
         (p_value_p->departure_y)) +
        (p_value_p->return_mm);
    p_value_p->wheel_mm =
        (p_value_p->translation_mm) +
        ((((WHEEL_TRACK_MM / 2.0f) * BG_RAD) / 100.0f) *
         ((BYPASS_PIVOT_LEFT_X100 *
           ((operand_a + operand_b) + BYPASS_INITIAL_HEADING_DEG)) +
          (BYPASS_PIVOT_RIGHT_X100 * (operand_a + operand_b))));
    return true;
}
#endif

/*** end of file ***/

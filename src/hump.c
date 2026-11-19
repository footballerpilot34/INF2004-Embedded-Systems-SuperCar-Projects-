/** @file hump.c
 *
 * @brief Hump height: how high the car climbs, from pitch and distance.
 *
 * @par Method
 * Two estimates; the larger is used.
 * - Tipping: a hump shorter than the car only tips it. With one axle on
 *   top and the other on the floor, sin(pitch) = height / wheelbase, so
 *   the raised end is HUMP_WHEELBASE_MM x sin(pitch) above the floor. The
 *   course hump (4-5 mm) is of this kind.
 * - Climbing: each control tick, the distance driven (encoders, the two
 *   wheels averaged) times sin(pitch) is the height gained or lost. Added
 *   up, this follows a hump longer than the car. The sum restarts at 0
 *   whenever the car is level (within HUMP_LEVEL_PITCH_X10) and not on a
 *   hump, so noise and calibration error cannot build up; it only adds up
 *   while the car drives forwards (vehicle.c says when).
 * Both are worked out all the time: hump_height_x10() is the height the
 * car is on now, and a hump's height is the highest of it while the car
 * was on that hump.
 *
 * @par Accuracy
 * - The IMU's reading of pitch includes speeding up and braking, and
 *   jolts as the wheels meet the hump: those read as extra tilt.
 * - Wheel slip on a climb counts as distance: the climb reads high.
 * - Without IMU calibration the pitch reads 0, and so does the height.
 *
 * Heights are in tenths of a mm ("_x10"), as the course hump is small.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "encoder.h"
#include "hump.h"
#include "imu.h"

#define RAD_PER_TENTH_DEG   (0.1f * 0.0174532925f)

// Encoder position units per mm: both wheels' edges added, times the
// position scale. The edges per m are per wheel.
#define POS_PER_MM          (((float) ENCODER_POS_SCALE *                 \
                              (float) (ENC_LEFT_EDGES_PER_M +              \
                                       ENC_RIGHT_EDGES_PER_M)) / 1000.0f)

static uint32_t g_last_pos     = 0u;
static bool     gb_pos_valid   = false;
static float    g_sum_mm       = 0.0f;  // Climbing sum (see above).
static float    g_now_mm       = 0.0f;  // Height the car is on now.
static float    g_peak_mm      = 0.0f;  // Highest of it on this hump.
static float    g_on_mm        = 0.0f;  // Driven since HUMP began.
static int32_t  g_last_x10     = 0;     // The last hump cleared.
static int32_t  g_last_tilt    = 0;
static int32_t  g_run_x10      = 0;     // The highest hump this run.
static int32_t  g_run_tilt     = 0;
static uint32_t g_count        = 0u;    // Humps cleared this run.


/*!
 * @brief Work out the height the car is on. Call every control tick.
 *
 * @param[in] b_forward True while the car drives forwards (line following,
 *                      probing, climbing, feature test).
 * @param[in] b_on_hump True while the car is in HUMP: the climbing sum and
 *                      the peak then keep going even when it is level.
 */
void
hump_task (bool b_forward, bool b_on_hump)
{
    uint32_t pos    = encoder_position();
    int32_t  pitch  = imu_pitch_x10();
    int32_t  size   = ((pitch < 0) ? -pitch : pitch);
    float    mm     = 0.0f;
    float    tipped = 0.0f;
    float    climb  = 0.0f;

    if (gb_pos_valid)
    {
        mm = (float) (pos - g_last_pos) / POS_PER_MM;  // Wraps correctly.
    }
    g_last_pos   = pos;
    gb_pos_valid = true;

    if (b_on_hump)
    {
        g_on_mm += mm;
    }
    else
    {
        g_on_mm = 0.0f;
    }

    if (!b_on_hump && (size <= HUMP_LEVEL_PITCH_X10))
    {
        g_sum_mm  = 0.0f;               // On the flat: start again.
        g_peak_mm = 0.0f;
    }
    else if (b_forward)
    {
        g_sum_mm += mm * sinf((float) pitch * RAD_PER_TENTH_DEG);
    }
    else
    {
        // Tilted but not driving forwards (pivoting, stopped): no change.
    }

    // Either end raised: the size of the tilt says by how much.
    tipped   = (float) HUMP_WHEELBASE_MM *
               sinf((float) size * RAD_PER_TENTH_DEG);
    climb    = ((g_sum_mm < 0.0f) ? -g_sum_mm : g_sum_mm);
    g_now_mm = ((tipped > climb) ? tipped : climb);
    if ((b_on_hump || (size > HUMP_LEVEL_PITCH_X10)) &&
        (g_now_mm > g_peak_mm))
    {
        g_peak_mm = g_now_mm;
    }
}   /* hump_task() */


/*!
 * @brief Record a hump the car has just cleared.
 *
 * @param[in] peak_tilt_x10 Its highest tilt, in tenths of a degree.
 */
void
hump_finish (int32_t peak_tilt_x10)
{
    g_last_x10  = (int32_t) ((g_peak_mm * 10.0f) + 0.5f);
    g_last_tilt = peak_tilt_x10;
    if (g_last_x10 > g_run_x10)
    {
        g_run_x10 = g_last_x10;
    }
    if (peak_tilt_x10 > g_run_tilt)
    {
        g_run_tilt = peak_tilt_x10;
    }
    g_count++;
}   /* hump_finish() */


/*!
 * @brief Forget the humps of the last run (a new run is starting).
 */
void
hump_new_run (void)
{
    g_last_x10  = 0;
    g_last_tilt = 0;
    g_run_x10   = 0;
    g_run_tilt  = 0;
    g_count     = 0u;
}   /* hump_new_run() */


/*!
 * @brief Report the height the car is on now, for live display.
 *
 * @return Tenths of a mm above the floor (the raised end); about 0 on the
 *         flat.
 */
int32_t
hump_height_x10 (void)
{
    return ((int32_t) ((g_now_mm * 10.0f) + 0.5f));
}   /* hump_height_x10() */


/*!
 * @brief Report how far the car has driven since this hump began.
 *
 * @return mm; 0 when not on a hump.
 */
int32_t
hump_on_mm (void)
{
    return ((int32_t) g_on_mm);
}   /* hump_on_mm() */


/*!
 * @brief Report the peak height of the last hump cleared.
 *
 * @return Tenths of a mm; 0 before the first hump.
 */
int32_t
hump_last_height_x10 (void)
{
    return (g_last_x10);
}   /* hump_last_height_x10() */


/*!
 * @brief Report the highest tilt on the last hump cleared.
 *
 * @return Tenths of a degree; 0 before the first hump.
 */
int32_t
hump_last_tilt_x10 (void)
{
    return (g_last_tilt);
}   /* hump_last_tilt_x10() */


/*!
 * @brief Report the highest hump peak of this run.
 *
 * @return Tenths of a mm; 0 before the first hump.
 */
int32_t
hump_run_height_x10 (void)
{
    return (g_run_x10);
}   /* hump_run_height_x10() */


/*!
 * @brief Report the highest tilt of this run.
 *
 * @return Tenths of a degree; 0 before the first hump.
 */
int32_t
hump_run_tilt_x10 (void)
{
    return (g_run_tilt);
}   /* hump_run_tilt_x10() */


/*!
 * @brief Report how many humps were cleared this run.
 *
 * @return The count.
 */
uint32_t
hump_count (void)
{
    return (g_count);
}   /* hump_count() */

/*** end of file ***/

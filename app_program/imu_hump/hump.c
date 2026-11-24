/** @file hump.c
 *
 * @brief Hump height: how high the car is lifted, from pitch and roll.
 *
 * @par Method
 * The car stands on three points: two drive wheels (WHEEL_TRACK_MM apart)
 * and a caster HUMP_WHEELBASE_MM in front of them. A hump shorter than
 * the car lifts one of them at a time:
 * - the caster lifted by h tips the nose up:  h = wheelbase x sin(pitch);
 * - one drive wheel lifted by h rolls the car: h = track x sin(roll)
 *   (and tips the nose down by half as much, measured -2 deg at +4.5 deg
 *   of roll on the course hump).
 * The larger of the two is the height the car is on now (either sign).
 * A hump's height is the highest height HELD for HUMP_STEADY_MSEC while
 * the car was on it (the lowest of the last HUMP_STEADY_MSEC of readings,
 * at its highest): one bounce does not count.
 * Measured on the course hump with the car still: 10.9 mm from the
 * caster, 9.8 mm from one drive wheel (2026-11-19).
 *
 * @par Accuracy
 * - The IMU's reading includes speeding up, braking and jolts as the
 *   wheels meet the hump: those read as extra tilt (filtered in imu.c).
 * - Without IMU calibration pitch and roll read 0, and so does the height.
 *
 * Heights are in tenths of a mm ("_x10").
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "encoder.h"
#include "hump.h"
#include "imu.h"

#define RAD_PER_TENTH_DEG (0.1f * 0.0174532925f)

// Encoder position units per mm: both wheels' edges added, times the
// position scale. The edges per m are per wheel.
#define POS_PER_MM                                                             \
    (((float)ENCODER_POS_SCALE *                                               \
      (float)(ENC_LEFT_EDGES_PER_M + ENC_RIGHT_EDGES_PER_M)) /                 \
     1000.0f)

// Readings kept for the steady height: one per control tick.
#define STEADY_TICKS ((HUMP_STEADY_MSEC * 1000u) / TICK_USEC)

static float    g_recent[STEADY_TICKS]; // The last readings.
static uint32_t g_recent_next = 0u;
static uint32_t g_last_pos    = 0u;
static bool     gb_pos_valid  = false;
static float    g_now_mm      = 0.0f; // Height the car is on now.
static float    g_peak_mm     = 0.0f; // Highest of it on this hump.
static float    g_on_mm       = 0.0f; // Driven since HUMP began.
static int32_t  g_last_x10    = 0;    // The last hump cleared.
static int32_t  g_last_tilt   = 0;
static int32_t  g_run_x10     = 0; // The highest hump this run.
static int32_t  g_run_tilt    = 0;
static uint32_t g_count       = 0u; // Humps cleared this run.

/*!
 * @brief Lifted by this much at this tilt.
 *
 * @param[in] base_mm  The distance to the point that stays down.
 * @param[in] tilt_x10 The tilt, in tenths of a degree (either sign).
 *
 * @return The height, in mm.
 */
static float
lift_mm(uint32_t base_mm, int32_t tilt_x10)
{
    int32_t size = ((tilt_x10 < 0) ? -tilt_x10 : tilt_x10);

    return (((float)base_mm) * (sinf(((float)size) * RAD_PER_TENTH_DEG)));
} /* lift_mm() */

/*!
 * @brief Work out the height the car is on. Call every control tick.
 *
 * @param[in] b_forward Unused (kept for the caller); the height does not
 *                      depend on how the car moves.
 * @param[in] b_on_hump True while the car is in HUMP: its peak is kept.
 */
void
hump_task(bool b_forward, bool b_on_hump)
{
    uint32_t pos      = encoder_position();
    float    caster   = lift_mm(HUMP_WHEELBASE_MM, imu_pitch_x10());
    float    wheel    = lift_mm(WHEEL_TRACK_MM, imu_roll_x10());
    float    value_mm = 0.0f;
    float    steady   = 0.0f;

    (void)b_forward;

    if (gb_pos_valid)
    {
        value_mm = ((float)(pos - g_last_pos)) / POS_PER_MM; // Wraps correctly.
    }
    g_last_pos   = pos;
    gb_pos_valid = true;

    g_now_mm = ((caster > wheel) ? caster : wheel);

    // Held for HUMP_STEADY_MSEC: the lowest of the recent readings.
    g_recent[g_recent_next] = g_now_mm;
    g_recent_next           = ((g_recent_next + 1u) % STEADY_TICKS);
    steady                  = g_recent[0];
    for (uint32_t idx = 1u; idx < STEADY_TICKS; idx++)
    {
        if ((g_recent[idx]) < steady)
        {
            steady = g_recent[idx];
        }
    }

    if (b_on_hump)
    {
        g_on_mm += value_mm;
        if (steady > g_peak_mm)
        {
            g_peak_mm = steady;
        }
    }
    else
    {
        g_on_mm   = 0.0f;
        g_peak_mm = steady; // Includes the climb before it was seen.
    }
} /* hump_task() */

/*!
 * @brief Record a hump the car has just cleared.
 *
 * @param[in] peak_tilt_x10 Its highest tilt, in tenths of a degree.
 */
void
hump_finish(int32_t peak_tilt_x10)
{
    g_last_x10  = (int32_t)((g_peak_mm * 10.0f) + 0.5f);
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
} /* hump_finish() */

/*!
 * @brief Forget the humps of the last run (a new run is starting).
 */
void
hump_new_run(void)
{
    g_last_x10  = 0;
    g_last_tilt = 0;
    g_run_x10   = 0;
    g_run_tilt  = 0;
    g_count     = 0u;
} /* hump_new_run() */

/*!
 * @brief Report the height the car is on now, for live display.
 *
 * @return Tenths of a mm: how far the raised point is above the floor;
 *         about 0 on the flat.
 */
int32_t
hump_height_x10(void)
{
    return ((int32_t)((g_now_mm * 10.0f) + 0.5f));
} /* hump_height_x10() */

/*!
 * @brief Report how far the car has driven since this hump began.
 *
 * @return mm; 0 when not on a hump.
 */
int32_t
hump_on_mm(void)
{
    return ((int32_t)g_on_mm);
} /* hump_on_mm() */

/*!
 * @brief Report the peak height of the last hump cleared.
 *
 * @return Tenths of a mm; 0 before the first hump.
 */
int32_t
hump_last_height_x10(void)
{
    return (g_last_x10);
} /* hump_last_height_x10() */

/*!
 * @brief Report the highest tilt on the last hump cleared.
 *
 * @return Tenths of a degree; 0 before the first hump.
 */
int32_t
hump_last_tilt_x10(void)
{
    return (g_last_tilt);
} /* hump_last_tilt_x10() */

/*!
 * @brief Report the highest hump peak of this run.
 *
 * @return Tenths of a mm; 0 before the first hump.
 */
int32_t
hump_run_height_x10(void)
{
    return (g_run_x10);
} /* hump_run_height_x10() */

/*!
 * @brief Report the highest tilt of this run.
 *
 * @return Tenths of a degree; 0 before the first hump.
 */
int32_t
hump_run_tilt_x10(void)
{
    return (g_run_tilt);
} /* hump_run_tilt_x10() */

/*!
 * @brief Report how many humps were cleared this run.
 *
 * @return The count.
 */
uint32_t
hump_count(void)
{
    return (g_count);
} /* hump_count() */

/*** end of file ***/

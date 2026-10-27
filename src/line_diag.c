/** @file line_diag.c
 *
 * @brief Line IR sensor diagnostic mode (motors off).
 *
 * @par What it shows
 * - On entry: whether each sensor is driving its pin or floating (wiring
 *   or power fault), and the current raw levels.
 * - Every raw level change, as it happens, with a running change count.
 * - A status line every 2 s, so a sensor that never changes is obvious.
 *
 * @par Finding the polarity by testing
 * Place BOTH sensors over black and send "learn black"; then over white
 * and send "learn white". Each sensor's black level is then taken from
 * the measurement (not assumed). If a sensor reads the same on black and
 * white, it is not switching: its potentiometer needs adjusting.
 *
 * @par Analogue values
 * Only the modules' digital DOUT pins are wired to the Pico (GP28 left,
 * GP0 right), so there are no analogue readings to show. The analogue
 * outputs (AO) would have to be wired to a free ADC pin.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"

#include "config.h"
#include "line_diag.h"
#include "line_sensor.h"
#include "log.h"
#include "motor.h"


#define STATUS_EVERY_TICKS  200u    // 2 s at 10 ms per tick.


static bool       gb_active      = false;
static line_raw_t g_last_raw;
static uint32_t   g_left_changes  = 0u;
static uint32_t   g_right_changes = 0u;
static uint32_t   g_ticks         = 0u;

// Learnt levels, per sensor. "Known" flags say whether each was recorded.
static bool       gb_black_known  = false;
static bool       gb_white_known  = false;
static line_raw_t g_black_raw;
static line_raw_t g_white_raw;


static char const * level_text(bool b_high);
static char const * colour_text(bool b_left, bool b_high);
static void         polarity_apply(bool b_left, char const * p_name,
                                   bool b_black_high, bool b_white_high);


/*!
 * @brief Enter diagnostic mode: brake, check wiring, show the levels.
 *
 * @par
 * Call only while the car is parked (READY or STOPPED).
 */
void
line_diag_enter (void)
{
    bool b_left_ok  = line_sensor_is_driven(true);
    bool b_right_ok = line_sensor_is_driven(false);

    motor_set(0, 0);
    gb_active       = true;
    g_last_raw      = line_sensor_raw();
    g_left_changes  = 0u;
    g_right_changes = 0u;
    g_ticks         = 0u;

    log_printf("DIAG ON: motors disabled. Move each IR sensor over black "
               "and white by hand.");
    log_printf("DIAG wiring: LEFT GP28 %s, RIGHT GP0 %s.",
               (b_left_ok ? "driven (OK)" : "FLOATING: check DOUT/VCC/GND"),
               (b_right_ok ? "driven (OK)" : "FLOATING: check DOUT/VCC/GND"));
    log_printf("DIAG now: LEFT=%s RIGHT=%s. Digital DOUT only: no analogue "
               "values are wired.",
               level_text(g_last_raw.b_left_high),
               level_text(g_last_raw.b_right_high));
    log_printf("DIAG: both over black -> 'b' / BLACK; both over white -> "
               "'n' / WHITE; 'd' / DIAG to exit.");
}   /* line_diag_enter() */


/*!
 * @brief Leave diagnostic mode and report the change counts.
 */
void
line_diag_exit (void)
{
    if (gb_active)
    {
        log_printf("DIAG OFF. Changes seen: LEFT %" PRIu32 ", RIGHT %"
                   PRIu32 ".", g_left_changes, g_right_changes);
    }

    gb_active = false;
}   /* line_diag_exit() */


/*!
 * @brief Report whether diagnostic mode is on.
 *
 * @return True while the motors must stay off for sensor testing.
 */
bool
line_diag_is_active (void)
{
    return (gb_active);
}   /* line_diag_is_active() */


/*!
 * @brief Log every raw level change, plus a status line every 2 s.
 *
 * @par
 * Call every control tick. Does nothing unless diagnostic mode is on.
 */
void
line_diag_task (void)
{
    line_raw_t raw = line_sensor_raw();

    if (gb_active)
    {
        motor_set(0, 0);    // Belt and braces: no motion while testing.

        if (raw.b_left_high != g_last_raw.b_left_high)
        {
            g_left_changes++;
            log_printf("DIAG LEFT  GP28: %s -> %s (%s)",
                       level_text(g_last_raw.b_left_high),
                       level_text(raw.b_left_high),
                       colour_text(true, raw.b_left_high));
        }

        if (raw.b_right_high != g_last_raw.b_right_high)
        {
            g_right_changes++;
            log_printf("DIAG RIGHT GP0:  %s -> %s (%s)",
                       level_text(g_last_raw.b_right_high),
                       level_text(raw.b_right_high),
                       colour_text(false, raw.b_right_high));
        }

        g_ticks++;
        if (0u == (g_ticks % STATUS_EVERY_TICKS))
        {
            log_printf("DIAG status: LEFT=%s (%s) changes %" PRIu32
                       " | RIGHT=%s (%s) changes %" PRIu32,
                       level_text(raw.b_left_high),
                       colour_text(true, raw.b_left_high), g_left_changes,
                       level_text(raw.b_right_high),
                       colour_text(false, raw.b_right_high),
                       g_right_changes);
        }

        g_last_raw = raw;
    }
}   /* line_diag_task() */


/*!
 * @brief Record the current levels as black or white, and set each
 *        sensor's polarity once both have been recorded.
 *
 * @param[in] b_black True if both sensors are over black now; false if
 *                    both are over white.
 */
void
line_diag_learn (bool b_black)
{
    line_raw_t raw = line_sensor_raw();

    if (!gb_active)
    {
        log_printf("Enter diagnostic mode ('d' / DIAG) before learning "
                   "black or white.");
    }
    else if (b_black)
    {
        g_black_raw    = raw;
        gb_black_known = true;
        log_printf("DIAG learnt BLACK: LEFT=%s RIGHT=%s.",
                   level_text(raw.b_left_high),
                   level_text(raw.b_right_high));
    }
    else
    {
        g_white_raw    = raw;
        gb_white_known = true;
        log_printf("DIAG learnt WHITE: LEFT=%s RIGHT=%s.",
                   level_text(raw.b_left_high),
                   level_text(raw.b_right_high));
    }

    if (gb_active && gb_black_known && gb_white_known)
    {
        polarity_apply(true, "LEFT", g_black_raw.b_left_high,
                       g_white_raw.b_left_high);
        polarity_apply(false, "RIGHT", g_black_raw.b_right_high,
                       g_white_raw.b_right_high);
    }
}   /* line_diag_learn() */


/*!
 * @brief Set one sensor's polarity from its black and white readings, or
 *        report that it is not switching.
 *
 * @param[in] b_left       True for the left sensor.
 * @param[in] p_name       "LEFT" or "RIGHT", for the log.
 * @param[in] b_black_high The level read over black.
 * @param[in] b_white_high The level read over white.
 */
static void
polarity_apply (bool b_left, char const * p_name, bool b_black_high,
                bool b_white_high)
{
    if (b_black_high == b_white_high)
    {
        log_printf("DIAG %s: %s on BOTH black and white -> NOT "
                   "switching. Adjust its pot.",
                   p_name, level_text(b_black_high));
    }
    else
    {
        line_sensor_set_black_level(b_left, b_black_high);
        log_printf("DIAG %s OK: black=%s white=%s. Set config.h "
                   "LINE_BLACK_IS_HIGH %s.",
                   p_name, level_text(b_black_high),
                   level_text(b_white_high),
                   (b_black_high ? "true" : "false"));
    }
}   /* polarity_apply() */


/*!
 * @brief Name a raw level.
 *
 * @param[in] b_high The level.
 *
 * @return "HIGH" or "LOW".
 */
static char const *
level_text (bool b_high)
{
    return (b_high ? "HIGH" : "LOW");
}   /* level_text() */


/*!
 * @brief Name the colour a raw level means with the current polarity.
 *
 * @param[in] b_left True for the left sensor.
 * @param[in] b_high The raw level.
 *
 * @return "black" or "white".
 */
static char const *
colour_text (bool b_left, bool b_high)
{
    return ((line_sensor_black_level(b_left) == b_high) ? "black"
                                                        : "white");
}   /* colour_text() */

/*** end of file ***/

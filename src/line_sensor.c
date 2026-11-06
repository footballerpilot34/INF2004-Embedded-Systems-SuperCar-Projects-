/** @file line_sensor.c
 *
 * @brief Left and right line IR sensors (digital DOUT), and crossroad
 *        detection.
 *
 * @par
 * Each sensor's "black" level starts as LINE_BLACK_IS_HIGH and can be
 * changed at runtime from a measured black/white test (see line_diag.c),
 * so the polarity comes from testing, not assumption. The value lasts
 * until reset; copy the measured value into config.h to keep it.
 *
 * @par Crossroads
 * The two line sensors sit side by side on the line, so a line crossing
 * it keeps them both black: they cannot see a crossroad. The LEFT side
 * sensor (the barcode sensor, beside the line) can: a crossroad's left
 * branch is one black mark of bounded length under it.
 * line_junction_detect() is given that sensor's black/white reading every
 * tick and reports a crossroad when a mark that lasted
 * LINE_JUNCTION_MIN_MSEC to LINE_JUNCTION_MAX_MSEC has ended.
 */

#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "config.h"
#include "line_sensor.h"


#define PULL_SETTLE_USEC    50u


static bool gb_left_black_high  = LINE_BLACK_IS_HIGH;
static bool gb_right_black_high = LINE_BLACK_IS_HIGH;

// Crossroad detection, from the side sensor.
static bool     gb_jn_armed     = false;    // White seen: a mark can start.
static bool     gb_jn_on_mark   = false;    // Timing a black mark.
static bool     gb_jn_white     = false;    // On white (in a mark: a gap).
static uint32_t g_jn_mark_msec  = 0u;       // The mark began at.
static uint32_t g_jn_white_msec = 0u;       // The white began at.


/*!
 * @brief Set up both line sensor pins as plain inputs.
 *
 * @par
 * The IR modules drive their outputs, so no internal pull is needed.
 */
void
line_sensor_init (void)
{
    gpio_init(PIN_LINE_LEFT);
    gpio_set_dir(PIN_LINE_LEFT, GPIO_IN);
    gpio_disable_pulls(PIN_LINE_LEFT);

    gpio_init(PIN_LINE_RIGHT);
    gpio_set_dir(PIN_LINE_RIGHT, GPIO_IN);
    gpio_disable_pulls(PIN_LINE_RIGHT);
}   /* line_sensor_init() */


/*!
 * @brief Read which sensors are over the line, using each sensor's black
 *        level.
 *
 * @return The current line state.
 */
line_state_t
line_sensor_read (void)
{
    line_raw_t   raw = line_sensor_raw();
    line_state_t state;

    state.b_left_black  = (gb_left_black_high == raw.b_left_high);
    state.b_right_black = (gb_right_black_high == raw.b_right_high);

    return (state);
}   /* line_sensor_read() */


/*!
 * @brief Read the raw DOUT levels, with no polarity applied.
 *
 * @return The raw levels.
 */
line_raw_t
line_sensor_raw (void)
{
    line_raw_t raw;

    raw.b_left_high  = gpio_get(PIN_LINE_LEFT);
    raw.b_right_high = gpio_get(PIN_LINE_RIGHT);

    return (raw);
}   /* line_sensor_raw() */


/*!
 * @brief Check whether a sensor is actually driving its pin.
 *
 * @par
 * The pin is read once with the internal pull-up and once with the
 * pull-down. A connected, powered module drives the pin, so both reads
 * match; a floating pin follows the pull. Takes about 100 us.
 *
 * @param[in] b_left True for the left sensor, false for the right.
 *
 * @return True if the pin is driven; false if it is floating.
 */
bool
line_sensor_is_driven (bool b_left)
{
    uint32_t pin           = (b_left ? PIN_LINE_LEFT : PIN_LINE_RIGHT);
    bool     b_pulled_up   = false;
    bool     b_pulled_down = false;

    gpio_pull_up(pin);
    sleep_us(PULL_SETTLE_USEC);
    b_pulled_up = gpio_get(pin);

    gpio_pull_down(pin);
    sleep_us(PULL_SETTLE_USEC);
    b_pulled_down = gpio_get(pin);

    gpio_disable_pulls(pin);

    return (b_pulled_up == b_pulled_down);
}   /* line_sensor_is_driven() */


/*!
 * @brief Set which raw level means "black" for one sensor.
 *
 * @param[in] b_left          True for the left sensor, false for right.
 * @param[in] b_black_is_high True if the sensor reads HIGH over black.
 */
void
line_sensor_set_black_level (bool b_left, bool b_black_is_high)
{
    if (b_left)
    {
        gb_left_black_high = b_black_is_high;
    }
    else
    {
        gb_right_black_high = b_black_is_high;
    }
}   /* line_sensor_set_black_level() */


/*!
 * @brief Report which raw level currently means "black" for one sensor.
 *
 * @param[in] b_left True for the left sensor, false for the right.
 *
 * @return True if black is HIGH.
 */
bool
line_sensor_black_level (bool b_left)
{
    return (b_left ? gb_left_black_high : gb_right_black_high);
}   /* line_sensor_black_level() */


/*!
 * @brief Watch the side sensor for a crossroad's branch. Call every tick.
 *
 * @par
 * A branch is 18 mm wide, so it is a black mark of bounded length: it is
 * judged once it has ended, and is a crossroad only if it lasted
 * LINE_JUNCTION_MIN_MSEC to LINE_JUNCTION_MAX_MSEC. Longer marks (the
 * dark mat beyond a paper edge, a long pause on black) are reported as
 * too long instead, so they can be logged. A mark only counts if it starts
 * (white to black) while enabled, so a mark the sensor was already on
 * when detection was switched on (after a turn, say) is never mistaken
 * for a new crossroad. White shorter than LINE_JUNCTION_GAP_MSEC inside a
 * mark (glare, a scuff) does not end it.
 *
 * @param[in]  b_side_black True if the side sensor sees black now.
 * @param[in]  b_enabled    False while crossroads must be ignored (not
 *                          following, or reading a barcode's bars).
 * @param[in]  now_msec     The current time.
 * @param[out] p_mark_msec  The mark's length, when one is reported.
 *
 * @return LINE_JUNCTION_FOUND or LINE_JUNCTION_TOO_LONG once, when a mark
 *         has ended; otherwise LINE_JUNCTION_NONE.
 */
line_junction_t
line_junction_detect (bool b_side_black, bool b_enabled, uint32_t now_msec,
                      uint32_t * p_mark_msec)
{
    line_junction_t result = LINE_JUNCTION_NONE;

    if (!b_enabled)
    {
        gb_jn_armed   = false;  // Wait for white before the next mark.
        gb_jn_on_mark = false;
        gb_jn_white   = false;
    }
    else if (!b_side_black)
    {
        if (!gb_jn_white)
        {
            gb_jn_white     = true;
            g_jn_white_msec = now_msec;
        }

        if ((now_msec - g_jn_white_msec) >= LINE_JUNCTION_GAP_MSEC)
        {
            if (gb_jn_on_mark)
            {
                // Real white: the mark ended when this white began.
                uint32_t length = g_jn_white_msec - g_jn_mark_msec;

                *p_mark_msec = length;
                if (length > LINE_JUNCTION_MAX_MSEC)
                {
                    result = LINE_JUNCTION_TOO_LONG;
                }
                else if (length >= LINE_JUNCTION_MIN_MSEC)
                {
                    result = LINE_JUNCTION_FOUND;
                }
                else
                {
                    // Too short: a bar or a smudge.
                }
            }

            gb_jn_armed   = true;
            gb_jn_on_mark = false;
        }
    }
    else
    {
        gb_jn_white = false;

        if ((!gb_jn_on_mark) && gb_jn_armed)
        {
            gb_jn_on_mark  = true;
            g_jn_mark_msec = now_msec;
        }
    }

    return (result);
}   /* line_junction_detect() */

/*** end of file ***/

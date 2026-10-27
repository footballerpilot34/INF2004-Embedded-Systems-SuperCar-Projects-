/** @file line_sensor.c
 *
 * @brief Left and right line IR sensors (digital DOUT).
 *
 * @par
 * Each sensor's "black" level starts as LINE_BLACK_IS_HIGH and can be
 * changed at runtime from a measured black/white test (see line_diag.c),
 * so the polarity comes from testing, not assumption. The value lasts
 * until reset; copy the measured value into config.h to keep it.
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

/*** end of file ***/

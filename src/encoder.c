/** @file encoder.c
 *
 * @brief Wheel encoders: distance travelled, for measuring barcode bars.
 *
 * @par How it works
 * Every edge on both channels of both encoders is one tick (4 per slot on
 * a quadrature encoder, the finest it allows). Direction is not tracked:
 * distance is only needed while driving forward over a barcode. The two
 * wheels are added, so the position follows the middle of the car.
 *
 * @par Between ticks
 * A coarse encoder gives few ticks per bar, so encoder_position() adds a
 * fraction of a tick from time: the time since the wheel's last tick
 * divided by its last tick interval, capped just below one tick. While
 * the car moves steadily this follows it smoothly; when the car stops,
 * the fraction stops growing at the cap, so a pause adds less than one
 * tick instead of the whole pause time.
 *
 * @par Calibration
 * The tick counts are shown on the web page ("Encoders L/R"). Push the
 * car 10 cm by hand to see how many ticks that is. Barcode decoding only
 * compares widths with each other, so it needs no calibration.
 */

#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "config.h"
#include "encoder.h"


#define NUM_WHEELS          2u
#define WHEEL_LEFT          0u
#define WHEEL_RIGHT         1u
#define FRACTION_MAX        (ENCODER_POS_SCALE - 1u)   // Just under 1 tick.


// Written by the GPIO interrupt only; read anywhere.
static volatile uint32_t g_count[NUM_WHEELS]      = { 0u, 0u };
static volatile uint32_t g_tick_usec[NUM_WHEELS]  = { 0u, 0u };
static volatile uint32_t g_interval[NUM_WHEELS]   = { 0u, 0u };


static void     pin_init(uint32_t pin);
static uint32_t wheel_position(uint32_t wheel, uint32_t now_usec);


/*!
 * @brief Set up the four encoder pins as inputs with pull-ups.
 *
 * @par
 * Pull-ups suit open-collector encoder outputs and do no harm to driven
 * ones. Edge interrupts are enabled by gpio_isr_init().
 */
void
encoder_init (void)
{
    pin_init(PIN_ENC_LEFT_A);
    pin_init(PIN_ENC_LEFT_B);
    pin_init(PIN_ENC_RIGHT_A);
    pin_init(PIN_ENC_RIGHT_B);
}   /* encoder_init() */


/*!
 * @brief Count one encoder edge. Interrupt context (gpio_isr.c).
 *
 * @param[in] gpio The encoder pin that changed.
 */
void
encoder_edge_isr (uint32_t gpio)
{
    uint32_t now_usec = time_us_32();
    uint32_t wheel    = (((PIN_ENC_LEFT_A == gpio) ||
                          (PIN_ENC_LEFT_B == gpio)) ? WHEEL_LEFT
                                                     : WHEEL_RIGHT);

    g_interval[wheel]  = now_usec - g_tick_usec[wheel];
    g_tick_usec[wheel] = now_usec;
    g_count[wheel]++;
}   /* encoder_edge_isr() */


/*!
 * @brief Report how far the car has moved: both wheels' ticks added, in
 *        ticks x ENCODER_POS_SCALE, with a timed fraction between ticks.
 *
 * @par
 * Safe to call from an interrupt (the barcode edge handlers do).
 *
 * @return The position. It only ever increases (wrapping at 2^32).
 */
uint32_t
encoder_position (void)
{
    uint32_t now_usec = time_us_32();

    return (wheel_position(WHEEL_LEFT, now_usec) +
            wheel_position(WHEEL_RIGHT, now_usec));
}   /* encoder_position() */


/*!
 * @brief Report the raw tick count of one wheel, for the web page.
 *
 * @param[in] gpio_a The wheel's A pin (PIN_ENC_LEFT_A or PIN_ENC_RIGHT_A).
 *
 * @return The ticks counted since power-up.
 */
uint32_t
encoder_count (uint32_t gpio_a)
{
    return (g_count[(PIN_ENC_LEFT_A == gpio_a) ? WHEEL_LEFT : WHEEL_RIGHT]);
}   /* encoder_count() */


/*!
 * @brief Set up one encoder pin.
 *
 * @param[in] pin The pin.
 */
static void
pin_init (uint32_t pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}   /* pin_init() */


/*!
 * @brief One wheel's position with a timed fraction of a tick.
 *
 * @param[in] wheel    WHEEL_LEFT or WHEEL_RIGHT.
 * @param[in] now_usec The current time.
 *
 * @return Ticks x ENCODER_POS_SCALE.
 */
static uint32_t
wheel_position (uint32_t wheel, uint32_t now_usec)
{
    uint32_t count     = 0u;
    uint32_t tick_usec = 0u;
    uint32_t interval  = 0u;
    uint32_t fraction  = 0u;

    // The interrupt may change these meanwhile: read until a count is seen
    // that did not change while the others were read.
    do
    {
        count     = g_count[wheel];
        tick_usec = g_tick_usec[wheel];
        interval  = g_interval[wheel];
    } while (count != g_count[wheel]);

    if ((count > 0u) && (interval > 0u))
    {
        uint32_t since = now_usec - tick_usec;

        // 64-bit: after a long stop the interval can exceed 2^32 / 256 us.
        since    = ((since < interval) ? since : interval);
        fraction = (uint32_t) (((uint64_t) since * ENCODER_POS_SCALE) /
                               interval);
        fraction = ((fraction < FRACTION_MAX) ? fraction : FRACTION_MAX);
    }

    return ((count * ENCODER_POS_SCALE) + fraction);
}   /* wheel_position() */

/*** end of file ***/

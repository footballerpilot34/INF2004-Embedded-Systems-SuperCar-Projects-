/** @file ultrasonic.c
 *
 * @brief HC-SR04 (3.3 V) forward range sensor, timed by edge interrupt.
 *
 * @par
 * ultrasonic_task() sends a ping when one is due and never waits for the
 * echo; ultrasonic_echo_isr() timestamps the ECHO edges. Distance in mm is
 * echo microseconds x 10 / 58 (sound takes about 58 us per cm, there and
 * back).
 */

#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "config.h"
#include "ultrasonic.h"


#define USEC_PER_CM_ROUND_TRIP  58u
#define MM_PER_CM               10u
#define USEC_PER_MSEC           1000u


// ECHO edge times, written by the ISR. Zero means "not seen yet".
// NOTE: 32-bit so each read and write is atomic on the RP2040.
static volatile uint32_t g_rise_usec = 0u;
static volatile uint32_t g_fall_usec = 0u;

static bool     gb_waiting       = false;
static uint64_t g_next_ping_usec = 0u;
static uint64_t g_deadline_usec  = 0u;
static uint32_t g_range_mm       = ULTRASONIC_NO_RANGE;
static uint64_t g_range_usec     = 0u;


/*!
 * @brief Set up the TRIG output and ECHO input.
 *
 * @par
 * The ECHO edge interrupt itself is enabled by gpio_isr_init().
 */
void
ultrasonic_init (void)
{
    gpio_init(PIN_US_TRIG);
    gpio_set_dir(PIN_US_TRIG, GPIO_OUT);
    gpio_put(PIN_US_TRIG, false);

    gpio_init(PIN_US_ECHO);
    gpio_set_dir(PIN_US_ECHO, GPIO_IN);
    gpio_pull_down(PIN_US_ECHO);
}   /* ultrasonic_init() */


/*!
 * @brief Timestamp an ECHO edge. Called from the GPIO interrupt.
 *
 * @param[in] events The edge events on the ECHO pin.
 */
void
ultrasonic_echo_isr (uint32_t events)
{
    uint32_t now_usec = time_us_32();

    if (0u != (events & (uint32_t) GPIO_IRQ_EDGE_RISE))
    {
        g_rise_usec = now_usec;
    }

    if ((0u != (events & (uint32_t) GPIO_IRQ_EDGE_FALL)) &&
        (0u != g_rise_usec))
    {
        g_fall_usec = now_usec;
    }
}   /* ultrasonic_echo_isr() */


/*!
 * @brief Send a ping when due, and record the range when its echo ends.
 *
 * @par
 * Call every control tick. It never blocks for longer than the 10 us
 * trigger pulse.
 */
void
ultrasonic_task (void)
{
    uint64_t now_usec = time_us_64();

    if (!gb_waiting)
    {
        if (now_usec >= g_next_ping_usec)
        {
            g_rise_usec = 0u;
            g_fall_usec = 0u;

            gpio_put(PIN_US_TRIG, true);
            sleep_us(US_TRIGGER_USEC);
            gpio_put(PIN_US_TRIG, false);

            g_deadline_usec  = time_us_64() + US_ECHO_TIMEOUT_USEC;
            g_next_ping_usec = time_us_64() + US_PING_PERIOD_USEC;
            gb_waiting       = true;
        }
    }
    else if (0u != g_fall_usec)
    {
        // NOTE: Unsigned subtraction is correct even if the 32-bit timer
        // wrapped between the two edges.
        uint32_t echo_usec = g_fall_usec - g_rise_usec;

        g_range_mm   = (echo_usec * MM_PER_CM) / USEC_PER_CM_ROUND_TRIP;
        g_range_usec = now_usec;
        gb_waiting   = false;
    }
    else if (now_usec >= g_deadline_usec)
    {
        // No echo, or out of range: nothing is close, so report clear.
        g_range_mm   = ULTRASONIC_NO_RANGE;
        g_range_usec = now_usec;
        gb_waiting   = false;
    }
    else
    {
        // The echo has not finished yet.
    }
}   /* ultrasonic_task() */


/*!
 * @brief Report the latest forward range.
 *
 * @return The range in millimetres, or ULTRASONIC_NO_RANGE if there is no
 *         echo or the reading is older than US_STALE_MSEC.
 */
uint32_t
ultrasonic_range_mm (void)
{
    uint32_t range_mm = g_range_mm;

    if ((time_us_64() - g_range_usec) >
        ((uint64_t) US_STALE_MSEC * USEC_PER_MSEC))
    {
        range_mm = ULTRASONIC_NO_RANGE;
    }

    return (range_mm);
}   /* ultrasonic_range_mm() */

/*** end of file ***/

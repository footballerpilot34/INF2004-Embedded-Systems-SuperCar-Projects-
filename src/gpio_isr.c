/** @file gpio_isr.c
 *
 * @brief Single GPIO interrupt dispatcher for all edge-timed inputs.
 *
 * @par
 * The Pico SDK allows one GPIO callback per core, so this module owns it
 * and routes each pin's edges to its driver: ultrasonic ECHO and barcode.
 * Handlers only timestamp edges; all processing happens in the tasks.
 *
 * @par Deviation
 * Rule 6.5.a: the SDK calls gpio_dispatch_isr() from its own interrupt
 * handler, so no compiler ISR keyword is needed.
 */

#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "barcode.h"
#include "config.h"
#include "gpio_isr.h"
#include "ultrasonic.h"


// NOTE: The SDK's edge flags are small positive enum values, so they can
// be cast to an unsigned mask safely.
#define BOTH_EDGES  ((uint32_t) GPIO_IRQ_EDGE_RISE | \
                     (uint32_t) GPIO_IRQ_EDGE_FALL)


static void gpio_dispatch_isr(uint gpio, uint32_t events);


/*!
 * @brief Enable edge interrupts on the ECHO and barcode pins.
 *
 * @par
 * Call after ultrasonic_init() and barcode_init() have set up the pins.
 */
void
gpio_isr_init (void)
{
    gpio_set_irq_enabled_with_callback(PIN_US_ECHO, BOTH_EDGES, true,
                                       gpio_dispatch_isr);
    gpio_set_irq_enabled(PIN_BARCODE, BOTH_EDGES, true);
}   /* gpio_isr_init() */


/*!
 * @brief Route a GPIO edge to the driver that owns the pin.
 *
 * @param[in] gpio   The pin that interrupted.
 * @param[in] events The edge events that occurred.
 */
static void
gpio_dispatch_isr (uint gpio, uint32_t events)
{
    if (PIN_US_ECHO == gpio)
    {
        ultrasonic_echo_isr(events);
    }
    else if (PIN_BARCODE == gpio)
    {
        barcode_edge_isr(events);
    }
    else
    {
        // No other pin has edge interrupts enabled.
    }
}   /* gpio_dispatch_isr() */

/*** end of file ***/

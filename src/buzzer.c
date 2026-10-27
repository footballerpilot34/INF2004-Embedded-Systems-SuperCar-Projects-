/** @file buzzer.c
 *
 * @brief Robo Pico piezo buzzer (GP22): short non-blocking beeps.
 *
 * @par
 * The buzzer is passive, so it needs a tone: a 2 kHz, 50% duty PWM signal.
 * buzzer_beep() starts the tone and buzzer_task() stops it later, so
 * nothing waits. The Robo Pico's mute switch silences it in hardware.
 */

#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "buzzer.h"
#include "config.h"


#define BUZZER_HZ       2000u
#define HZ_PER_MHZ      1000000u    // 1 PWM tick per microsecond.
#define USEC_PER_MSEC   1000u


static uint64_t g_off_usec = 0u;    // When the beep ends; 0 = silent.


/*!
 * @brief Set up the buzzer pin for a 2 kHz tone, silent to start.
 */
void
buzzer_init (void)
{
    uint32_t slice = pwm_gpio_to_slice_num(PIN_BUZZER);

    gpio_set_function(PIN_BUZZER, GPIO_FUNC_PWM);

    // NOTE: clk_sys is 125 or 150 MHz, so the divider fits in 8 bits.
    pwm_set_clkdiv_int_frac(slice,
                            (uint8_t) (clock_get_hz(clk_sys) / HZ_PER_MHZ),
                            0u);
    pwm_set_wrap(slice, (HZ_PER_MHZ / BUZZER_HZ) - 1u);
    pwm_set_gpio_level(PIN_BUZZER, 0u);
    pwm_set_enabled(slice, true);
}   /* buzzer_init() */


/*!
 * @brief Start a beep; buzzer_task() ends it.
 *
 * @param[in] duration_msec How long to beep.
 */
void
buzzer_beep (uint32_t duration_msec)
{
    pwm_set_gpio_level(PIN_BUZZER, (HZ_PER_MHZ / BUZZER_HZ) / 2u);
    g_off_usec = time_us_64() + ((uint64_t) duration_msec * USEC_PER_MSEC);
}   /* buzzer_beep() */


/*!
 * @brief End the beep when its time is up. Call every control tick.
 */
void
buzzer_task (void)
{
    if ((0u != g_off_usec) && (time_us_64() >= g_off_usec))
    {
        pwm_set_gpio_level(PIN_BUZZER, 0u);
        g_off_usec = 0u;
    }
}   /* buzzer_task() */

/*** end of file ***/

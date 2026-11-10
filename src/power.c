/** @file power.c
 *
 * @brief Supply voltage (VSYS) watch, to spot a sagging battery.
 *
 * @par How it works
 * The Pico W measures VSYS / 3 on GP29 (ADC3). VSYS is what the Robo Pico
 * feeds the Pico from the battery (or USB), so it follows the battery,
 * a little lower. Every POWER_SAMPLE_MSEC the voltage is read and the
 * lowest since power-up is kept: a deep dip while the motors run means
 * the battery cannot keep up (and is what makes the Pico restart).
 *
 * @par Sharing GP29
 * On the Pico W, GP29 is also the Wi-Fi chip's SPI clock, so a reading is
 * taken inside cyw43_thread_enter() and the chip is woken first, as in the
 * Raspberry Pi example (adc/read_vsys). The ADC is otherwise unused while
 * the barcode reader uses DOUT (BARCODE_USE_ANALOG false); with analog
 * barcode reading on, the ADC is busy and nothing is measured.
 *
 * @par
 * Only the main loop calls these functions (never an ISR).
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"

#include "config.h"
#include "log.h"
#include "power.h"


#define ADC_FIRST_PIN       26u     // ADC0 is GP26.
#define ADC_FULL_SCALE      4096u   // 12-bit ADC.
#define ADC_REF_MV          3300u   // ADC reference (3.3 V).
#define VSYS_DIVIDER        3u      // GP29 sees VSYS / 3.
#define SAMPLES             8u      // Averaged per reading.
#define SAMPLES_SKIPPED     3u      // The first readings come out low.


static bool     gb_measured  = false;
static uint32_t g_vsys_mv    = 0u;
static uint32_t g_min_mv     = UINT32_MAX;
static uint32_t g_next_msec  = 0u;
static bool     gb_low_said  = false;   // "Battery low" logged already.
static bool     gb_bad_said  = false;   // "Cannot be right" logged.


static uint32_t vsys_read_mv(void);


/*!
 * @brief Prepare the supply measurement (the ADC itself).
 */
void
power_init (void)
{
    if (!BARCODE_USE_ANALOG)
    {
        adc_init();
    }
}   /* power_init() */


/*!
 * @brief Read the supply voltage every POWER_SAMPLE_MSEC. Call every tick
 *        (after Wi-Fi has started).
 */
void
power_task (void)
{
    uint32_t now_msec = to_ms_since_boot(get_absolute_time());

    if ((!BARCODE_USE_ANALOG) && ((int32_t) (now_msec - g_next_msec) >= 0))
    {
        uint32_t mv = vsys_read_mv();

        g_next_msec = now_msec + POWER_SAMPLE_MSEC;

        if (mv < POWER_VALID_MIN_MV)
        {
            // The Pico could not be running on this: the pin was not read
            // properly (GP29 is shared with the Wi-Fi chip). Show nothing
            // rather than a false alarm.
            if (!gb_bad_said)
            {
                log_printf("Power: VSYS read as %" PRIu32 " mV, which "
                           "cannot be right; battery voltage unavailable.",
                           mv);
                gb_bad_said = true;
            }
            g_vsys_mv = 0u;
        }
        else
        {
            g_vsys_mv   = mv;
            gb_measured = true;
            g_min_mv    = ((mv < g_min_mv) ? mv : g_min_mv);
        }

        if ((0u == g_vsys_mv) || (!gb_measured))
        {
            // Nothing valid to judge.
        }
        else if ((g_vsys_mv < POWER_LOW_MV) && !gb_low_said)
        {
            log_printf("Power: supply only %" PRIu32 ".%02" PRIu32 " V: "
                       "battery low or sagging; charge it.",
                       (g_vsys_mv / 1000u), ((g_vsys_mv % 1000u) / 10u));
            gb_low_said = true;
        }
        else if (g_vsys_mv >= (POWER_LOW_MV + POWER_LOW_HYST_MV))
        {
            gb_low_said = false;            // Recovered: warn again later.
        }
        else
        {
            // Unchanged.
        }
    }
}   /* power_task() */


/*!
 * @brief Report whether the supply has been measured yet.
 *
 * @return True once a reading exists.
 */
bool
power_is_measured (void)
{
    return (gb_measured);
}   /* power_is_measured() */


/*!
 * @brief Report the latest supply voltage.
 *
 * @return VSYS in millivolts (0 until measured).
 */
uint32_t
power_vsys_mv (void)
{
    return (g_vsys_mv);
}   /* power_vsys_mv() */


/*!
 * @brief Report the lowest supply voltage since power-up.
 *
 * @return VSYS in millivolts (0 until measured).
 */
uint32_t
power_vsys_min_mv (void)
{
    return (gb_measured ? g_min_mv : 0u);
}   /* power_vsys_min_mv() */


/*!
 * @brief Measure VSYS once (averaged), sharing GP29 with the Wi-Fi chip.
 *
 * @return VSYS in millivolts.
 */
static uint32_t
vsys_read_mv (void)
{
    static bool b_raw_logged = false;
    uint32_t    sum          = 0u;
    uint32_t    skipped      = 0u;
    uint32_t    raw          = 0u;

    // As Raspberry Pi's adc/read_vsys example: wake the Wi-Fi chip (which
    // also leaves its chip select, GPIO25, high: needed for the reading),
    // then sample back to back with the ADC free-running.
    cyw43_thread_enter();
    (void) cyw43_arch_gpio_get(CYW43_WL_GPIO_VBUS_PIN);

    adc_gpio_init(PICO_VSYS_PIN);
    adc_select_input(PICO_VSYS_PIN - ADC_FIRST_PIN);
    adc_fifo_setup(true, false, 0u, false, false);
    adc_run(true);

    // The first readings come out low: skip them (and anything stale).
    while ((!adc_fifo_is_empty()) || (skipped < SAMPLES_SKIPPED))
    {
        (void) adc_fifo_get_blocking();
        skipped++;
    }

    for (uint32_t idx = 0u; idx < SAMPLES; idx++)
    {
        sum += adc_fifo_get_blocking();
    }

    adc_run(false);
    adc_fifo_drain();
    cyw43_thread_exit();

    raw = sum / SAMPLES;
    if (!b_raw_logged)
    {
        log_printf("Power: first VSYS reading: ADC %" PRIu32 " of 4095.",
                   raw);
        b_raw_logged = true;
    }

    return ((raw * VSYS_DIVIDER * ADC_REF_MV) / ADC_FULL_SCALE);
}   /* vsys_read_mv() */

/*** end of file ***/

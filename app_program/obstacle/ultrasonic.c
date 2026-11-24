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
#include "hardware/sync.h"

#include "config.h"
#include "ultrasonic.h"

#define USEC_PER_CM_ROUND_TRIP 58u
#define MM_PER_CM 10u
#define USEC_PER_MSEC 1000u

// ECHO edge times, written by the ISR. Zero means "not seen yet".
// NOTE: 32-bit so each read and write is atomic on the RP2040.
static volatile uint32_t g_rise_usec = 0u;
static volatile uint32_t g_fall_usec = 0u;

// Flags, one bit each in one byte (change requirement 1):
//   US_ARMED   - false while TRIG is pulsed, so crosstalk from TRIG onto
//                ECHO is ignored. Read by the echo ISR.
//   US_WAITING - a ping is out, waiting for its echo.
//   US_READY   - a fresh range is available for the current direction.
//   US_NO_ECHO - the last ping timed out with no echo.
// Only the main loop writes the byte, and it does so with interrupts off
// (flags_write()), so the ISR never sees a half-written update. The ISR
// only reads it: a byte load is atomic on the RP2040.
#define US_ARMED (1u << 0)
#define US_WAITING (1u << 1)
#define US_READY (1u << 2)
#define US_NO_ECHO (1u << 3)

static volatile uint8_t g_us_flags = 0u;

static uint64_t g_next_ping_usec = 0u;
static uint64_t g_deadline_usec  = 0u;
static uint32_t g_range_mm       = ULTRASONIC_NO_RANGE;
static uint64_t g_range_usec     = 0u;
static uint32_t g_sample_sequence;

static void flags_write(uint32_t set, uint32_t clear);
static bool flag_is(uint32_t mask);

/** @brief Return the sequence number used to reject repeated echo samples. */
uint32_t
ultrasonic_sample_sequence(void)
{
    return g_sample_sequence;
}

/** @brief Invalidate cached range data after a servo or measurement change. */
void
ultrasonic_invalidate(void)
{
    flags_write(0u, (US_ARMED | US_WAITING) | US_READY);
    g_range_mm = ULTRASONIC_NO_RANGE;
    // Preserve the minimum interval even when abandoning an active ping.
    g_next_ping_usec = (time_us_64()) + US_PING_PERIOD_USEC;
}

/** @brief Check whether a fresh range is available for the current direction.
 */
bool
ultrasonic_measurement_ready(void)
{
    return (flag_is(US_READY));
}

/*!
 * @brief Set up the TRIG output and ECHO input.
 *
 * @par
 * The ECHO edge interrupt itself is enabled by gpio_isr_init().
 */
void
ultrasonic_init(void)
{
    gpio_init(PIN_US_TRIG);
    gpio_set_dir(PIN_US_TRIG, GPIO_OUT);
    gpio_put(PIN_US_TRIG, false);

    gpio_init(PIN_US_ECHO);
    gpio_set_dir(PIN_US_ECHO, GPIO_IN);
    gpio_pull_down(PIN_US_ECHO);
} /* ultrasonic_init() */

/*!
 * @brief Timestamp an ECHO edge. Called from the GPIO interrupt.
 *
 * @param[in] events The edge events on the ECHO pin.
 */
void
ultrasonic_echo_isr(uint32_t events)
{
    uint32_t now_usec = time_us_32();

    if (!flag_is(US_ARMED))
    {
        // Edge during the trigger pulse: crosstalk, not an echo.
    }
    else if (0u != (events & ((uint32_t)GPIO_IRQ_EDGE_RISE)))
    {
        g_rise_usec = now_usec;
    }
    else if (((0u != (events & ((uint32_t)GPIO_IRQ_EDGE_FALL))) &&
              (0u != g_rise_usec)) &&
             (0u == g_fall_usec))
    {
        if ((now_usec - g_rise_usec) < US_ECHO_MIN_USEC)
        {
            // Too short to be an echo: a glitch. Wait for the real one.
            g_rise_usec = 0u;
        }
        else
        {
            g_fall_usec = now_usec;
        }
    }
    else
    {
        // A fall with no rise, or after the echo was already taken.
    }
} /* ultrasonic_echo_isr() */

/*!
 * @brief Send a ping when due, and record the range when its echo ends.
 *
 * @par
 * Call every control tick. It never blocks for longer than the 10 us
 * trigger pulse.
 */
void
ultrasonic_task(void)
{
    uint64_t now_usec = time_us_64();

    if (!flag_is(US_WAITING))
    {
        if (now_usec >= g_next_ping_usec)
        {
            flags_write(0u, US_ARMED);
            g_rise_usec = 0u;
            g_fall_usec = 0u;

            gpio_put(PIN_US_TRIG, true);
            sleep_us(US_TRIGGER_USEC);
            gpio_put(PIN_US_TRIG, false);
            flags_write(US_ARMED, 0u);

            g_deadline_usec  = (time_us_64()) + US_ECHO_TIMEOUT_USEC;
            g_next_ping_usec = (time_us_64()) + US_PING_PERIOD_USEC;
            flags_write(US_WAITING, 0u);
        }
    }
    else if (0u != g_fall_usec)
    {
        // NOTE: Unsigned subtraction is correct even if the 32-bit timer
        // wrapped between the two edges.
        uint32_t echo_usec = g_fall_usec - g_rise_usec;

        g_range_mm   = (echo_usec * MM_PER_CM) / USEC_PER_CM_ROUND_TRIP;
        g_range_usec = now_usec;
        g_sample_sequence++;
        flags_write(US_READY, (US_ARMED | US_WAITING) | US_NO_ECHO);
    }
    else if (now_usec >= g_deadline_usec)
    {
        // No echo, or out of range: nothing is close, so report clear.
        g_range_mm   = ULTRASONIC_NO_RANGE;
        g_range_usec = now_usec;
        g_sample_sequence++;
        flags_write(US_READY | US_NO_ECHO, US_ARMED | US_WAITING);
    }
    else
    {
        // The echo has not finished yet.
    }
} /* ultrasonic_task() */

/*!
 * @brief Report the latest forward range.
 *
 * @return The range in millimetres, or ULTRASONIC_NO_RANGE if there is no
 *         echo or the reading is older than US_STALE_MSEC.
 */
uint32_t
ultrasonic_range_mm(void)
{
    uint32_t range_mm = g_range_mm;

    if (((time_us_64()) - g_range_usec) >
        (((uint64_t)US_STALE_MSEC) * USEC_PER_MSEC))
    {
        range_mm = ULTRASONIC_NO_RANGE;
    }

    return (range_mm);
} /* ultrasonic_range_mm() */

/*!
 * @brief Report whether the last completed ping had no echo (nothing in
 *        range, or the sensor is not answering).
 *
 * @return True if it timed out.
 */
bool
ultrasonic_last_no_echo(void)
{
    return (flag_is(US_NO_ECHO));
} /* ultrasonic_last_no_echo() */

/*!
 * @brief Set and clear flag bits together, with interrupts off so the
 *        read-modify-write of the shared byte cannot be interrupted.
 *
 * @param[in] set   US_... bits to set.
 * @param[in] clear US_... bits to clear.
 */
static void
flags_write(uint32_t set, uint32_t clear)
{
    uint32_t saved = save_and_disable_interrupts();

    g_us_flags = (uint8_t)(((g_us_flags & (~clear)) | set) & 0xFFu);
    restore_interrupts(saved);
} /* flags_write() */

/*!
 * @brief Read flag bits (one byte load: atomic, also from the ISR).
 *
 * @param[in] mask US_... bits.
 *
 * @return True if any of them is set.
 */
static bool
flag_is(uint32_t mask)
{
    return (0u != (g_us_flags & mask));
} /* flag_is() */

/*** end of file ***/

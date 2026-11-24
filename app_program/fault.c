/** @file fault.c
 *
 * @brief One shared fault bitmask, and the control loop's pass time.
 *
 * @par The fault byte
 * Every fault condition is one bit of g_faults (FAULT_... in fault.h)
 * instead of a separate bool or int. fault_task() works most of them out
 * once per tick from the other modules; fault_set() / fault_clear() let
 * any module (including an interrupt handler) raise or clear one itself.
 *
 * @par Safe access
 * Setting or clearing one bit is a read-modify-write of the whole byte. If
 * an interrupt changed another bit between the read and the write, its
 * change would be lost, so every read-modify-write runs with interrupts
 * off (save_and_disable_interrupts(), a few cycles). Core 1 is not used,
 * so this is enough. A plain read of the byte is a single load, which is
 * atomic on the RP2040, so fault_get() needs no lock.
 *
 * @par Reporting only
 * The faults are reported (MQTT telemetry and heartbeat); nothing here
 * changes how the car drives.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/sync.h"

#include "config.h"
#include "encoder.h"
#include "fault.h"
#include "imu.h"
#include "motor.h"
#include "mqtt_comm.h"
#include "power.h"
#include "ultrasonic.h"
#include "web.h"

#define STALL_WINDOW_MSEC 500u // Wheels are checked over this long.
#define STALL_MIN_POWER 15     // Below this the wheels may not turn (%).
#define STALL_MIN_EDGES 3u     // A turning wheel gives more than this.
#define LOOP_USEC_MAX 65535u   // Pass time is kept in 16 bits.
#define OVERRUNS_MAX 65535u

// The fault byte. Written in a critical section (see above).
static volatile uint8_t g_faults = 0u;

// Wheel check window. Main loop only.
static uint32_t g_window_msec  = 0u;
static uint32_t g_window_left  = 0u; // Encoder counts at its start.
static uint32_t g_window_right = 0u;

// Loop pass time. Main loop only. 16 bits: a pass over 65 ms is shown as
// 65535 us (it is an overrun either way).
static uint16_t g_loop_max_usec = 0u;
static uint16_t g_loop_overruns = 0u;

static bool driven(int32_t power);

/*!
 * @brief Clear every fault. Call once at start-up.
 */
void
fault_init(void)
{
    g_faults       = 0u;
    g_window_msec  = 0u;
    g_window_left  = encoder_count(PIN_ENC_LEFT_A);
    g_window_right = encoder_count(PIN_ENC_RIGHT_A);
} /* fault_init() */

/*!
 * @brief Raise faults. Safe from the main loop and from interrupts.
 *
 * @param[in] mask FAULT_... bits to set.
 */
void
fault_set(uint32_t mask)
{
    uint32_t saved = save_and_disable_interrupts();

    g_faults = (uint8_t)(g_faults | (mask & 0xFFu));
    restore_interrupts(saved);
} /* fault_set() */

/*!
 * @brief Clear faults. Safe from the main loop and from interrupts.
 *
 * @param[in] mask FAULT_... bits to clear.
 */
void
fault_clear(uint32_t mask)
{
    uint32_t saved = save_and_disable_interrupts();

    g_faults = (uint8_t)(g_faults & (~mask) & 0xFFu);
    restore_interrupts(saved);
} /* fault_clear() */

/*!
 * @brief Set or clear faults.
 *
 * @param[in] mask     FAULT_... bits.
 * @param[in] b_active True to set them, false to clear them.
 */
void
fault_update(uint32_t mask, bool b_active)
{
    if (b_active)
    {
        fault_set(mask);
    }
    else
    {
        fault_clear(mask);
    }
} /* fault_update() */

/*!
 * @brief Read the fault byte.
 *
 * @return FAULT_... bits now set.
 */
uint32_t
fault_get(void)
{
    return ((uint32_t)g_faults); // One byte load: atomic.
} /* fault_get() */

/*!
 * @brief Work out the faults the other modules report. Call every tick.
 *
 * @par Wheels
 * Every STALL_WINDOW_MSEC, if a motor was driven (at least
 * STALL_MIN_POWER) at both ends of the window: neither wheel turning is a
 * stall; one wheel turning while the other's encoder gave no edges is an
 * encoder fault. Braked or slow, both are cleared.
 *
 * @param[in] now_msec The time now.
 */
void
fault_task(uint32_t now_msec)
{
    static bool b_was_driven = false;
    bool        b_driven =
        (driven(motor_left_speed())) || (driven(motor_right_speed()));

    if ((now_msec - g_window_msec) >= STALL_WINDOW_MSEC)
    {
        uint32_t left    = encoder_count(PIN_ENC_LEFT_A);
        uint32_t right   = encoder_count(PIN_ENC_RIGHT_A);
        uint32_t d_left  = left - g_window_left;
        uint32_t d_right = right - g_window_right;

        if (b_driven && b_was_driven)
        {
            bool b_left_still  = (d_left <= STALL_MIN_EDGES);
            bool b_right_still = (d_right <= STALL_MIN_EDGES);

            fault_update(FAULT_MOTOR_STALL, b_left_still && b_right_still);
            fault_update(FAULT_ENCODER, (b_left_still != b_right_still));
        }
        else
        {
            fault_clear(FAULT_MOTOR_STALL | FAULT_ENCODER);
        }
        g_window_msec  = now_msec;
        g_window_left  = left;
        g_window_right = right;
        b_was_driven   = b_driven;
    }

    fault_update(FAULT_IMU, (!imu_is_ok()) || (!imu_reading_is_valid()));
    fault_update(FAULT_NO_ECHO, ultrasonic_last_no_echo());
    fault_update(FAULT_WIFI_LOSS, !web_is_online());
    fault_update(FAULT_MQTT_DOWN, !mqtt_comm_is_connected());
    fault_update(FAULT_BATTERY_LOW,
                 (power_is_measured()) && ((power_vsys_mv()) < POWER_LOW_MV));
} /* fault_task() */

/*!
 * @brief Record how long one loop pass took (before the wait for the next
 *        tick). A pass longer than TICK_USEC raises FAULT_LOOP_OVERRUN
 *        until a pass fits again.
 *
 * @param[in] work_usec The pass time.
 */
void
fault_loop_record(uint32_t work_usec)
{
    uint32_t held = (work_usec > LOOP_USEC_MAX) ? LOOP_USEC_MAX : work_usec;

    if (held > g_loop_max_usec)
    {
        g_loop_max_usec = (uint16_t)held;
    }
    if (work_usec > TICK_USEC)
    {
        if (g_loop_overruns < OVERRUNS_MAX)
        {
            g_loop_overruns++;
        }
        fault_set(FAULT_LOOP_OVERRUN);
    }
    else
    {
        fault_clear(FAULT_LOOP_OVERRUN);
    }
} /* fault_loop_record() */

/*!
 * @brief Report the longest loop pass since the last call, and start
 *        again (for the heartbeat).
 *
 * @return Microseconds (65535 = that long or longer).
 */
uint32_t
fault_loop_max_usec_take(void)
{
    uint32_t max_usec = g_loop_max_usec;

    g_loop_max_usec = 0u;

    return (max_usec);
} /* fault_loop_max_usec_take() */

/*!
 * @brief Report how many loop passes took longer than TICK_USEC.
 *
 * @return The count since power-up (held at 65535).
 */
uint32_t
fault_loop_overruns(void)
{
    return (g_loop_overruns);
} /* fault_loop_overruns() */

/*!
 * @brief Say whether a motor power is enough to expect the wheel to turn.
 *
 * @param[in] power Motor power, -100 to 100 %.
 *
 * @return True if driven.
 */
static bool
driven(int32_t power)
{
    return ((abs(power)) >= STALL_MIN_POWER);
} /* driven() */

/*** end of file ***/

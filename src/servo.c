/** Ultrasonic pan servo. Increasing angle faces right (confirmed wiring). */
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "config.h"
#include "log.h"
#include "servo.h"
#include "ultrasonic.h"

static int32_t g_angle = -1;
static uint64_t g_ready_usec;
static bool gb_released = false;    // Pulse off: idle, not holding.

static uint16_t
angle_level(int32_t degrees)
{
    return (uint16_t)(SERVO_MIN_USEC +
        degrees * (SERVO_MAX_USEC - SERVO_MIN_USEC) / 180);
}

void
servo_init(void)
{
    uint32_t slice = pwm_gpio_to_slice_num(PIN_SERVO);
    gpio_set_function(PIN_SERVO, GPIO_FUNC_PWM);
    pwm_set_clkdiv_int_frac(slice,
        (uint8_t)(clock_get_hz(clk_sys) / 1000000u), 0u);
    pwm_set_wrap(slice, 19999u);
    servo_set_angle(SERVO_CENTER_DEG);
    pwm_set_enabled(slice, true);
}

void
servo_set_angle(int32_t degrees)
{
    int32_t low = SERVO_CENTER_DEG - SERVO_RANGE_DEG;
    int32_t high = SERVO_CENTER_DEG + SERVO_RANGE_DEG;
    degrees = degrees < low ? low : (degrees > high ? high : degrees);
    if (degrees == g_angle)
    {
        if (gb_released)
        {
            // Hold again where it was left: it has not moved, so no
            // settling wait and no new ultrasonic reading are needed.
            pwm_set_gpio_level(PIN_SERVO, angle_level(degrees));
            gb_released = false;
        }
        return;
    }
    g_angle = degrees;
    gb_released = false;
    g_ready_usec = time_us_64() + (uint64_t)SERVO_SETTLE_MSEC * 1000u;
    ultrasonic_invalidate();
    pwm_set_gpio_level(PIN_SERVO, angle_level(degrees));
    log_servo("Servo: %ld degrees.", (long)degrees);
}

/*!
 * @brief Stop sending the servo's position pulse while the car is idle,
 *        so it stops holding (and drawing current). Call every tick.
 *
 * @par
 * Released only once it has been settled for SERVO_RELEASE_MSEC and the
 * caller allows it (car idle, no scan). The next servo_set_angle() holds
 * it again at once.
 *
 * @param[in] b_allow True while nothing needs the servo held.
 */
void
servo_idle_release(bool b_allow)
{
    uint64_t release_usec = g_ready_usec +
                            ((uint64_t)SERVO_RELEASE_MSEC * 1000u);

    if (b_allow && !gb_released && (time_us_64() >= release_usec))
    {
        pwm_set_gpio_level(PIN_SERVO, 0u);
        gb_released = true;
        log_servo("Servo: released while idle (saves power).");
    }
}

int32_t servo_angle(void) { return g_angle; }
bool servo_is_settled(void) { return time_us_64() >= g_ready_usec; }

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
        return;
    }
    g_angle = degrees;
    g_ready_usec = time_us_64() + (uint64_t)SERVO_SETTLE_MSEC * 1000u;
    ultrasonic_invalidate();
    pwm_set_gpio_level(PIN_SERVO, (uint16_t)(SERVO_MIN_USEC +
        degrees * (SERVO_MAX_USEC - SERVO_MIN_USEC) / 180));
    log_servo("Servo: %ld degrees.", (long)degrees);
}

int32_t servo_angle(void) { return g_angle; }
bool servo_is_settled(void) { return time_us_64() >= g_ready_usec; }

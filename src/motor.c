/** @file motor.c
 *
 * @brief Left and right wheel motor driver (Robo Pico, 20 kHz PWM).
 *
 * @par
 * Speed 0 drives both inputs LOW, which the Robo Pico truth table defines
 * as BRAKE (both HIGH would coast).
 */

#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "config.h"
#include "motor.h"


#define MOTOR_PWM_HZ    20000u  // Silent, and the Robo Pico's maximum.
#define MAX_SPEED       100
#define NUM_MOTOR_PINS  4u


static uint16_t g_pwm_top   = 0u;
static int32_t  g_left      = 0;
static int32_t  g_right     = 0;


static void motor_drive(uint32_t pin_a, uint32_t pin_b, int32_t speed,
                        bool b_reversed);


/*!
 * @brief Set up PWM on all four motor pins with both wheels braked.
 */
void
motor_init (void)
{
    static uint32_t const pins[NUM_MOTOR_PINS] =
    {
        PIN_LEFT_MOTOR_A, PIN_LEFT_MOTOR_B,
        PIN_RIGHT_MOTOR_A, PIN_RIGHT_MOTOR_B
    };

    // NOTE: clk_sys / 20 kHz - 1 is 6249 at 125 MHz and 7499 at 150 MHz,
    // so it always fits in 16 bits.
    g_pwm_top = (uint16_t) ((clock_get_hz(clk_sys) / MOTOR_PWM_HZ) - 1u);

    for (uint32_t idx = 0u; idx < NUM_MOTOR_PINS; idx++)
    {
        uint32_t slice = pwm_gpio_to_slice_num(pins[idx]);

        gpio_set_function(pins[idx], GPIO_FUNC_PWM);
        pwm_set_wrap(slice, g_pwm_top);
        pwm_set_gpio_level(pins[idx], 0u);
        pwm_set_enabled(slice, true);
    }

    motor_set(0, 0);
}   /* motor_init() */


/*!
 * @brief Set both wheel speeds.
 *
 * @param[in] left  Left wheel, -100 (full reverse) to +100 (full forward).
 * @param[in] right Right wheel, -100 to +100. 0 brakes the wheel.
 */
void
motor_set (int32_t left, int32_t right)
{
    g_left  = left;
    g_right = right;
    motor_drive(PIN_LEFT_MOTOR_A, PIN_LEFT_MOTOR_B, left,
                LEFT_MOTOR_REVERSED);
    motor_drive(PIN_RIGHT_MOTOR_A, PIN_RIGHT_MOTOR_B, right,
                RIGHT_MOTOR_REVERSED);
}   /* motor_set() */


/*!
 * @brief Report the last left wheel speed requested.
 *
 * @return The speed, -100 to +100.
 */
int32_t
motor_left_speed (void)
{
    return (g_left);
}   /* motor_left_speed() */


/*!
 * @brief Report the last right wheel speed requested.
 *
 * @return The speed, -100 to +100.
 */
int32_t
motor_right_speed (void)
{
    return (g_right);
}   /* motor_right_speed() */


/*!
 * @brief Drive one motor.
 *
 * @param[in] pin_a      The motor's A input.
 * @param[in] pin_b      The motor's B input.
 * @param[in] speed      -100 to +100.
 * @param[in] b_reversed True if this motor is wired backwards.
 */
static void
motor_drive (uint32_t pin_a, uint32_t pin_b, int32_t speed, bool b_reversed)
{
    int32_t  limited   = (b_reversed ? -speed : speed);
    uint32_t magnitude = 0u;
    uint16_t level     = 0u;

    if (limited > MAX_SPEED)
    {
        limited = MAX_SPEED;
    }
    else if (limited < -MAX_SPEED)
    {
        limited = -MAX_SPEED;
    }
    else
    {
        // Already within range.
    }

    // NOTE: limited is within +/-100, so its magnitude fits unsigned and
    // magnitude * g_pwm_top / 100 never exceeds g_pwm_top (16 bits).
    magnitude = (uint32_t) ((limited < 0) ? -limited : limited);
    level     = (uint16_t) ((magnitude * g_pwm_top) / (uint32_t) MAX_SPEED);

    if (limited >= 0)
    {
        pwm_set_gpio_level(pin_a, level);
        pwm_set_gpio_level(pin_b, 0u);
    }
    else
    {
        pwm_set_gpio_level(pin_a, 0u);
        pwm_set_gpio_level(pin_b, level);
    }
}   /* motor_drive() */

/*** end of file ***/

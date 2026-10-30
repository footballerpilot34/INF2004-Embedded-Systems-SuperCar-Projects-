/** @file supercar_main.c
 *
 * @brief SuperCar firmware: follows a black line, reads Code 39 barcodes,
 *        climbs humps and stops at walls, with Teleplot telemetry and a
 *        wireless web UI.
 *
 * @par Control loop
 * Every 10 ms (the design's task period) main() runs, in order: Wi-Fi,
 * commands (buttons, serial, web), ultrasonic, IMU, barcode, the vehicle
 * state machine and the buzzer; Teleplot points go out every 50 ms. Each
 * module's *_task() function is written so it can become its own RTOS
 * task later.
 *
 * @par Controls (buttons, serial key, or web page)
 * - START: short press GP20, 's', or START on the page. Place the car
 *   on the line first: it sweeps left/right for 4 s to check the line
 *   sensors, shifts until both are on the line, then follows the line.
 * - STOP: press GP21, 'x', or STOP on the page. Works in any state.
 * - IMU CALIBRATE (while stopped and level): hold GP20 for 2 s, press 'c',
 *   or use IMU CALIBRATE on the page. Keep the car still for 5 s.
 * - WHEEL TEST (while stopped, car lifted): 'w' or WHEEL TEST on the page.
 * - IR DIAG (while stopped): 'd' or IR DIAG on the page turns the motors
 *   off and logs every line sensor change. Then 'b' / LEARN BLACK with
 *   both sensors over black, and 'n' / LEARN WHITE over white, to measure
 *   each sensor's polarity.
 *
 * @par Coding standard
 * BARR-C:2018. Deviation: Rule 6.5.a, see gpio_isr.c.
 *
 * TODO: Record the name of the person who approved this deviation.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "barcode.h"
#include "buzzer.h"
#include "config.h"
#include "gpio_isr.h"
#include "imu.h"
#include "line_diag.h"
#include "line_sensor.h"
#include "log.h"
#include "motor.h"
#include "telemetry.h"
#include "ultrasonic.h"
#include "vehicle.h"
#include "web.h"


#define USB_WAIT_MSEC           3000u
#define USB_POLL_MSEC           100u
#define WHEEL_TEST_MSEC         1000u
#define WHEEL_TEST_PAUSE_MSEC   500u
#define WHEEL_TEST_SPEED        20

#define SERVO_CENTER_USEC       1500u   // 90 degrees: facing forward.
#define SERVO_PERIOD_TICKS      19999u  // 20 ms at 1 tick per us.
#define HZ_PER_MHZ              1000000u

#define KEY_START               's'
#define KEY_STOP                'x'
#define KEY_WHEEL_TEST          'w'
#define KEY_IMU_CALIBRATE       'c'
#define KEY_DIAG                'd'
#define KEY_LEARN_BLACK         'b'
#define KEY_LEARN_WHITE         'n'


static void          usb_wait(void);
static void          servo_center(void);
static void          buttons_init(void);
static web_command_t buttons_poll(void);
static web_command_t serial_poll(void);
static void          command_run(web_command_t command);
static void          imu_calibrate_run(void);
static void          wheel_test(void);
static bool          vehicle_is_idle(void);
static void          barcode_report(barcode_result_t result);


/*!
 * @brief Set up every module, then run the 10 ms control loop forever.
 *
 * @return Never returns.
 */
int
main (void)
{
    uint64_t next_tick_usec = 0u;
    uint32_t tick_count     = 0u;

    // Brake first, before anything else can move the wheels.
    motor_init();

    stdio_init_all();
    usb_wait();
    log_printf("=== SuperCar AAI14: line + barcode + hump/wall ===");

    buzzer_init();
    servo_center();
    line_sensor_init();
    barcode_init();
    ultrasonic_init();
    gpio_isr_init();
    buttons_init();

    log_printf("Checking LSM303DLHC...");
    (void) imu_init();

    (void) web_init();  // Logs its own result; the car runs either way.

    vehicle_init();
    log_printf("READY. START: GP20 / 's' / web. STOP: GP21 / 'x' / web. "
               "Hold GP20 2 s for IMU calibration.");

    next_tick_usec = time_us_64() + TICK_USEC;

    for (;;)
    {
        web_task();
        command_run(buttons_poll());
        command_run(serial_poll());
        command_run(web_command_take());

        ultrasonic_task();
        imu_task();
        barcode_report(barcode_task());
        vehicle_task();
        line_diag_task();
        buzzer_task();

        tick_count++;
        if (0u == (tick_count % TELEMETRY_EVERY_TICKS))
        {
            telemetry_task();
        }

        // Wait for the next 10 ms tick.
        while (time_us_64() < next_tick_usec)
        {
            tight_loop_contents();
        }
        next_tick_usec += TICK_USEC;

        // After a long blocking action (wheel test), skip the
        // missed ticks instead of running them all at once.
        if (time_us_64() > next_tick_usec)
        {
            next_tick_usec = time_us_64() + TICK_USEC;
        }
    }

    return (0);
}   /* main() */


/*!
 * @brief Run one command, from whichever source it came.
 *
 * @param[in] command The command (WEB_CMD_NONE does nothing).
 */
static void
command_run (web_command_t command)
{
    switch (command)
    {
    case WEB_CMD_STOP:
        vehicle_stop();
    break;

    case WEB_CMD_START:
        if (line_diag_is_active())
        {
            log_printf("IR DIAG is on (motors off): turn it off first.");
        }
        else
        {
            vehicle_start();
        }
    break;

    case WEB_CMD_DIAG:
        if (line_diag_is_active())
        {
            line_diag_exit();
        }
        else if (vehicle_is_idle())
        {
            line_diag_enter();
        }
        else
        {
            log_printf("Stop the car before IR DIAG.");
        }
    break;

    case WEB_CMD_LEARN_BLACK:
        line_diag_learn(true);
    break;

    case WEB_CMD_LEARN_WHITE:
        line_diag_learn(false);
    break;

    case WEB_CMD_IMU_CALIBRATE:
        if (vehicle_is_idle())
        {
            imu_calibrate_run();
        }
        else
        {
            log_printf("Stop the car before calibrating the IMU.");
        }
    break;

    case WEB_CMD_WHEEL_TEST:
        if (line_diag_is_active())
        {
            log_printf("IR DIAG is on (motors off): turn it off first.");
        }
        else if (vehicle_is_idle())
        {
            wheel_test();
        }
        else
        {
            log_printf("Stop the car before the wheel test.");
        }
    break;

    case WEB_CMD_NONE:
    default:
        // Nothing to do.
    break;
    }
}   /* command_run() */


/*!
 * @brief Calibrate the LSM303DLHC pitch reference while the car is still.
 */
static void
imu_calibrate_run (void)
{
    if (!imu_is_ok())
    {
        log_printf("LSM303DLHC is offline; check I2C wiring and power.");
    }
    else if (imu_calibrate())
    {
        log_printf("IMU calibrated. Pitch/roll baseline set; hump detection enabled.");
        buzzer_beep(CAL_DONE_BEEP_MSEC);
    }
    else
    {
        log_printf("IMU calibration failed. Keep it still, level and try again.");
    }
}   /* imu_calibrate_run() */


/*!
 * @brief Report a finished barcode scan to the log, web page and Teleplot.
 *
 * @param[in] result The result from barcode_task().
 */
static void
barcode_report (barcode_result_t result)
{
    if (BARCODE_DECODED == result)
    {
        log_printf("Barcode decoded: \"%s\"", barcode_message());
        telemetry_text("barcode", barcode_message());
    }
    else if (BARCODE_INVALID == result)
    {
        log_printf("Barcode scan could not be decoded.");
        telemetry_text("barcode", "(invalid scan)");
    }
    else
    {
        // No scan finished this tick.
    }
}   /* barcode_report() */


/*!
 * @brief Wait up to 3 s for a serial monitor, without blocking on battery.
 */
static void
usb_wait (void)
{
    uint32_t waited_ms = 0u;

    while ((!stdio_usb_connected()) && (waited_ms < USB_WAIT_MSEC))
    {
        sleep_ms(USB_POLL_MSEC);
        waited_ms += USB_POLL_MSEC;
    }
}   /* usb_wait() */


/*!
 * @brief Point the ultrasonic servo straight ahead (90 degrees).
 */
static void
servo_center (void)
{
    uint32_t slice = pwm_gpio_to_slice_num(PIN_SERVO);

    gpio_set_function(PIN_SERVO, GPIO_FUNC_PWM);

    // NOTE: clk_sys is 125 or 150 MHz, so the divider fits in 8 bits.
    pwm_set_clkdiv_int_frac(slice,
                            (uint8_t) (clock_get_hz(clk_sys) / HZ_PER_MHZ),
                            0u);
    pwm_set_wrap(slice, SERVO_PERIOD_TICKS);
    pwm_set_gpio_level(PIN_SERVO, SERVO_CENTER_USEC);
    pwm_set_enabled(slice, true);
}   /* servo_center() */


/*!
 * @brief Set up the START and STOP buttons (active low).
 */
static void
buttons_init (void)
{
    gpio_init(PIN_BUTTON_START);
    gpio_set_dir(PIN_BUTTON_START, GPIO_IN);
    gpio_pull_up(PIN_BUTTON_START);

    gpio_init(PIN_BUTTON_STOP);
    gpio_set_dir(PIN_BUTTON_STOP, GPIO_IN);
    gpio_pull_up(PIN_BUTTON_STOP);
}   /* buttons_init() */


/*!
 * @brief Turn button presses into commands.
 *
 * @par
 * STOP acts at once. START acts when released after a short press; held
 * for 2 s it requests a still/level IMU calibration instead.
 *
 * @return The command to run, or WEB_CMD_NONE.
 */
static web_command_t
buttons_poll (void)
{
    static uint32_t held_ticks = 0u;
    web_command_t   command    = WEB_CMD_NONE;

    if (!gpio_get(PIN_BUTTON_STOP))
    {
        if (!vehicle_is_idle())
        {
            command = WEB_CMD_STOP;
        }
    }
    else if (!gpio_get(PIN_BUTTON_START))
    {
        held_ticks++;
        if (BUTTON_LONG_PRESS_TICKS == held_ticks)
        {
            command = WEB_CMD_IMU_CALIBRATE;
        }
    }
    else
    {
        // Released: a short press (past the debounce) means START.
        if ((held_ticks >= BUTTON_DEBOUNCE_TICKS) &&
            (held_ticks < BUTTON_LONG_PRESS_TICKS))
        {
            command = WEB_CMD_START;
        }
        held_ticks = 0u;
    }

    return (command);
}   /* buttons_poll() */


/*!
 * @brief Turn a serial key press into a command.
 *
 * @return The command to run, or WEB_CMD_NONE.
 */
static web_command_t
serial_poll (void)
{
    int           key     = getchar_timeout_us(0u);
    web_command_t command = WEB_CMD_NONE;

    if (KEY_STOP == key)
    {
        command = WEB_CMD_STOP;
    }
    else if (KEY_START == key)
    {
        command = WEB_CMD_START;
    }
    else if (KEY_IMU_CALIBRATE == key)
    {
        command = WEB_CMD_IMU_CALIBRATE;
    }
    else if (KEY_WHEEL_TEST == key)
    {
        command = WEB_CMD_WHEEL_TEST;
    }
    else if (KEY_DIAG == key)
    {
        command = WEB_CMD_DIAG;
    }
    else if (KEY_LEARN_BLACK == key)
    {
        command = WEB_CMD_LEARN_BLACK;
    }
    else if (KEY_LEARN_WHITE == key)
    {
        command = WEB_CMD_LEARN_WHITE;
    }
    else
    {
        // No key, or one with no meaning.
    }

    return (command);
}   /* serial_poll() */


/*!
 * @brief Spin each wheel forward in turn, to check the motor wiring.
 *
 * @par
 * Lift the car first. If the wrong wheel spins, swap the LEFT and RIGHT
 * motor pins in config.h; if a wheel spins backwards, flip its REVERSED
 * flag.
 */
static void
wheel_test (void)
{
    log_printf("Wheel test: LEFT wheel should spin FORWARD...");
    motor_set(WHEEL_TEST_SPEED, 0);
    sleep_ms(WHEEL_TEST_MSEC);
    motor_set(0, 0);
    sleep_ms(WHEEL_TEST_PAUSE_MSEC);

    log_printf("Wheel test: RIGHT wheel should spin FORWARD...");
    motor_set(0, WHEEL_TEST_SPEED);
    sleep_ms(WHEEL_TEST_MSEC);
    motor_set(0, 0);

    log_printf("Wheel test done. Fix config.h if either wheel was wrong.");
}   /* wheel_test() */


/*!
 * @brief Report whether the car is parked (READY or STOPPED).
 *
 * @return True if the motors are not being driven.
 */
static bool
vehicle_is_idle (void)
{
    vehicle_state_t state = vehicle_state();

    return ((VEHICLE_READY == state) || (VEHICLE_STOPPED == state));
}   /* vehicle_is_idle() */

/*** end of file ***/

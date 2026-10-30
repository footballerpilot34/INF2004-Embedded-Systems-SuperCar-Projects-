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
 *   on or near the line: it searches for the line, checks both line
 *   sensors detect black, lines up on it, then follows the line.
 * - STOP: press GP21, 'x', or STOP on the page. Works in any state.
 * - CALIBRATE (while stopped, car still and level): hold GP20 for 2 s,
 *   'c', or CALIBRATE on the page. One beep when it succeeds.
 * - WHEEL TEST (while stopped, car lifted): 'w' or WHEEL TEST on the page.
 * - IR DIAG (while stopped): 'd' or IR DIAG on the page turns the motors
 *   off and logs every line sensor change. Then 'b' / LEARN BLACK with
 *   both sensors over black, and 'n' / LEARN WHITE over white, to measure
 *   each sensor's polarity.
 * - TEST LEFT / U-TURN / RIGHT: 'l', 'u', 'r' or the page buttons act
 *   exactly like a barcode command. While stopped, the car turns and
 *   stops again (to tune TURN_90_MSEC); while following, it turns and
 *   follows on.
 *
 * @par Barcodes
 * Every decoded barcode is logged and shown on the web page. A command
 * (see BARCODE_TEXT_... in config.h) also turns the car at once.
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
#define KEY_CALIBRATE           'c'
#define KEY_DIAG                'd'
#define KEY_LEARN_BLACK         'b'
#define KEY_LEARN_WHITE         'n'
#define KEY_TURN_LEFT           'l'
#define KEY_TURN_RIGHT          'r'
#define KEY_UTURN               'u'


static void          usb_wait(void);
static void          servo_center(void);
static void          buttons_init(void);
static web_command_t buttons_poll(void);
static web_command_t serial_poll(void);
static void          command_run(web_command_t command);
static void          calibrate_run(void);
static void          wheel_test(void);
static bool          vehicle_is_idle(void);
static void          barcode_report(barcode_result_t result);
static void          turn_test(barcode_command_t command);


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

    log_printf("Keep the car still and level: calibrating the IMU...");
    if (imu_init())
    {
        buzzer_beep(CAL_DONE_BEEP_MSEC);
    }

    (void) web_init();  // Logs its own result; the car runs either way.

    vehicle_init();
    log_printf("READY. START: GP20 / 's' / web. STOP: GP21 / 'x' / web. "
               "Hold GP20 2 s to calibrate.");

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

        // After a long blocking action (calibration, wheel test), skip the
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

    case WEB_CMD_CALIBRATE:
        if (vehicle_is_idle())
        {
            calibrate_run();
        }
        else
        {
            log_printf("Stop the car before calibrating.");
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

    case WEB_CMD_TURN_LEFT:
        turn_test(BARCODE_CMD_LEFT);
    break;

    case WEB_CMD_TURN_RIGHT:
        turn_test(BARCODE_CMD_RIGHT);
    break;

    case WEB_CMD_UTURN:
        turn_test(BARCODE_CMD_UTURN);
    break;

    case WEB_CMD_NONE:
    default:
        // Nothing to do.
    break;
    }
}   /* command_run() */


/*!
 * @brief Run a barcode command without a barcode, to test the turns.
 *
 * @param[in] command The command to act on.
 */
static void
turn_test (barcode_command_t command)
{
    if (line_diag_is_active())
    {
        log_printf("IR DIAG is on (motors off): turn it off first.");
    }
    else
    {
        log_printf("TEST: as if a barcode said %s.",
                   barcode_command_name(command));
        vehicle_barcode_command(command);
    }
}   /* turn_test() */


/*!
 * @brief Calibrate the IMU and beep once if it worked.
 *
 * @par
 * If the IMU was not found at power-up, this tries to find it again, so a
 * loose wire can be fixed without a reset.
 */
static void
calibrate_run (void)
{
    bool b_ok = false;

    log_printf("Calibrating the IMU: keep the car still and level...");

    if (imu_is_ok())
    {
        b_ok = imu_calibrate();
    }
    else
    {
        b_ok = imu_init();
    }

    if (b_ok)
    {
        log_printf("IMU calibration done.");
        buzzer_beep(CAL_DONE_BEEP_MSEC);
    }
    else
    {
        log_printf("IMU calibration FAILED: check the gyroscope wiring.");
    }
}   /* calibrate_run() */


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
        barcode_command_t command = barcode_command();

        if (BARCODE_CMD_NONE == command)
        {
            log_printf("Barcode MESSAGE: \"%s\"", barcode_message());
        }
        else
        {
            log_printf("Barcode COMMAND: \"%s\" = %s", barcode_message(),
                       barcode_command_name(command));
            vehicle_barcode_command(command);
        }
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
 * for 2 s it calibrates instead (as soon as the 2 s is reached).
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
            command = WEB_CMD_CALIBRATE;
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
    else if (KEY_CALIBRATE == key)
    {
        command = WEB_CMD_CALIBRATE;
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
    else if (KEY_TURN_LEFT == key)
    {
        command = WEB_CMD_TURN_LEFT;
    }
    else if (KEY_TURN_RIGHT == key)
    {
        command = WEB_CMD_TURN_RIGHT;
    }
    else if (KEY_UTURN == key)
    {
        command = WEB_CMD_UTURN;
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

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
 * - IMU CALIBRATE (while stopped and level): hold GP20 for 2 s, press 'c',
 *   or use IMU CALIBRATE on the page. Keep the car still for 5 s.
 * - WHEEL TEST (while stopped, car lifted): 'w' or WHEEL TEST on the page.
 * - IR DIAG (while stopped): 'd' or IR DIAG on the page turns the motors
 *   off and logs every line sensor change. Then 'b' / LEARN BLACK with
 *   both sensors over black, and 'n' / LEARN WHITE over white, to measure
 *   each sensor's polarity.
 * - BARCODE TEST (while stopped): 't' or BARCODE TEST on the page drives
 *   straight, ignoring the line sensors, and reads barcodes. STOP ends it.
 * - Test turns (serial only): 'l', 'u', 'r' act exactly like a barcode
 *   command. While stopped, the car turns and
 *   stops again (to tune TURN_90_MSEC); while following, 'l'/'r' become
 *   the pending turn for the next crossroad and 'u' turns at once.
 *
 * @par Barcodes
 * Every decoded barcode is logged and shown on the web page. A command
 * (see BARCODE_TEXT_... in config.h: 'A' left, 'Z' right) is carried out
 * at the next crossroad (detected in line_sensor.c from the side sensor).
 *
 * @par Coding standard
 * BARR-C:2018. Deviation: Rule 6.5.a, see gpio_isr.c.
 *
 * TODO: Record the name of the person who approved this deviation.
 */

#include "obstacle.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/structs/vreg_and_chip_reset.h"
#include "hardware/watchdog.h"

#include "barcode.h"
#include "encoder.h"
#include "buzzer.h"
#include "config.h"
#include "gpio_isr.h"
#include "imu.h"
#include "line_diag.h"
#include "line_sensor.h"
#include "log.h"
#include "motor.h"
#include "power.h"
#include "servo.h"
#include "telemetry.h"
#include "ultrasonic.h"
#include "vehicle.h"
#include "web.h"


#define USB_WAIT_MSEC           3000u
#define USB_POLL_MSEC           100u
#define WHEEL_TEST_MSEC         1000u
#define WHEEL_TEST_PAUSE_MSEC   500u
#define WHEEL_TEST_SPEED        20


#define KEY_START               's'
#define KEY_STOP                'x'
#define KEY_WHEEL_TEST          'w'
#define KEY_IMU_CALIBRATE       'c'
#define KEY_DIAG                'd'
#define KEY_LEARN_BLACK         'b'
#define KEY_LEARN_WHITE         'n'
#define KEY_TURN_LEFT           'l'
#define KEY_TURN_RIGHT          'r'
#define KEY_UTURN               'u'
#define KEY_BARCODE_TEST        't'
#define KEY_ENCODER_MARK        'e'

static bool g_encoder_marked;

static void          usb_wait(void);
static void          buttons_init(void);
static web_command_t buttons_poll(void);
static bool g_stationary_scan_test = false;

static web_command_t serial_poll(void);
static void          command_run(web_command_t command);
static void          imu_calibrate_run(void);
static void          wheel_test(void);
static bool          vehicle_is_idle(void);
static char const *  restart_reason(void);
static void          barcode_report(barcode_result_t result);
static void          turn_test(barcode_command_t command);
static void          encoder_calibration_mark(void);


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
    web_set_restart_reason(restart_reason());
    log_printf("Last restart: %s.", restart_reason());

    buzzer_init();
    servo_init();
    line_sensor_init();
    barcode_init();
    ultrasonic_init();
    encoder_init();
    gpio_isr_init();
    buttons_init();

    log_printf("Checking LSM303DLHC...");
    (void) imu_init();

    (void) web_init();  // Logs its own result; the car runs either way.
    power_init();       // After Wi-Fi: shares GP29 with the Wi-Fi chip.

    vehicle_init();
    log_printf("READY. START: GP20 / 's' / web. STOP: GP21 / 'x' / web. "
               "Hold GP20 2 s for IMU calibration.");

    next_tick_usec = time_us_64() + TICK_USEC;

    for (;;)
    {
        int32_t servo_deg = 0;

        web_task();
        power_task();
        command_run(buttons_poll());
        command_run(serial_poll());
        command_run(web_command_take());
        if (web_servo_take(&servo_deg))
        {
            if (vehicle_is_moving())
            {
                log_servo("Servo: locked while the car is moving.");
            }
            else
            {
                servo_set_angle(servo_deg);
            }
        }

        if (vehicle_is_moving() &&
            (VEHICLE_OBSTACLE_SCAN != vehicle_state()))
        {
            servo_set_angle(SERVO_CENTER_DEG);
        }
        // Keep STOP and the UI responsive while the sensor turns. A range
        // from a previous angle must never be used as a forward reading.
        if (servo_is_settled())
        {
            ultrasonic_task();
        }
        imu_task();
        barcode_report(barcode_task());
        if (vehicle_is_moving() &&
            (VEHICLE_OBSTACLE_SCAN != vehicle_state()) &&
            ((!servo_is_settled()) ||
             !ultrasonic_measurement_ready()))
        {
            motor_set(0, 0);
        }
        else
        {
            vehicle_task();
        }
        if (g_stationary_scan_test)
        {
            if (!vehicle_is_idle())
            {
                obstacle_scan_cancel();
                g_stationary_scan_test = false;
                motor_set(0, 0);
                log_printf("STATIONARY SCAN TEST CANCELLED");
            }
            else
            {
                motor_set(0, 0);
                obstacle_scan_task(to_ms_since_boot(get_absolute_time()));

                if (!obstacle_scan_active())
                {
                    g_stationary_scan_test = false;
                    log_printf("STATIONARY SCAN TEST COMPLETE");
                }
            }
        }

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
 * @brief Say why the Pico last started, from the chip's reset flags.
 *
 * @par
 * "Power-on or brown-out" covers both a real power-up and the supply
 * dipping too low (the chip cannot tell them apart). Seen while the car
 * stayed switched on, it means the battery sagged: the web page's uptime
 * going back to zero shows the same.
 *
 * @return A short description.
 */
static char const *
restart_reason (void)
{
    uint32_t     flags    = vreg_and_chip_reset_hw->chip_reset;
    char const * p_reason = "unknown";

    if (watchdog_caused_reboot())
    {
        p_reason = "software or watchdog reset (e.g. after flashing)";
    }
    else if (0u != (flags & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_RUN_BITS))
    {
        p_reason = "reset button (RUN pin)";
    }
    else if (0u != (flags &
                    VREG_AND_CHIP_RESET_CHIP_RESET_HAD_PSM_RESTART_BITS))
    {
        p_reason = "debugger restart";
    }
    else if (0u != (flags & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_POR_BITS))
    {
        p_reason = "power-on or brown-out (supply dipped)";
    }
    else
    {
        // No flag set: leave "unknown".
    }

    return (p_reason);
}   /* restart_reason() */


/*!
 * @brief Run one command, from whichever source it came.
 *
 * @param[in] command The command (WEB_CMD_NONE does nothing).
 */
static void
command_run (web_command_t command)
{
    if (WEB_CMD_NONE != command && g_encoder_marked)
    {
        g_encoder_marked = false;
        log_printf("Encoder roll mark cancelled by a vehicle command; press 'e' to restart.");
    }
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

    case WEB_CMD_TURN_LEFT:
        turn_test(BARCODE_CMD_LEFT);
    break;

    case WEB_CMD_TURN_RIGHT:
        turn_test(BARCODE_CMD_RIGHT);
    break;

    case WEB_CMD_UTURN:
        turn_test(BARCODE_CMD_UTURN);
    break;

    case WEB_CMD_BARCODE_TEST:
        if (line_diag_is_active())
        {
            log_printf("IR DIAG is on (motors off): turn it off first.");
        }
        else
        {
            vehicle_barcode_test();
        }
    break;

    case WEB_CMD_BARCODE_STATIC:
        if (line_diag_is_active())
        {
            log_printf("IR DIAG is on (motors off): turn it off first.");
        }
        else
        {
            vehicle_barcode_static();
        }
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
        log_line("TEST: as if a barcode said %s.",
                 barcode_command_name(command));
        vehicle_barcode_command(command, true);
    }
}   /* turn_test() */


/*!
 * @brief Calibrate the IMU and beep once if it worked.
 *
 * @par
 * If the IMU was not found at power-up, this tries to find it again, so a
 * loose wire can be fixed without a reset.
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
        log_obstacle("IMU calibrated. Pitch/roll baseline set; hump "
                     "detection enabled.");
        buzzer_beep(CAL_DONE_BEEP_MSEC);
    }
    else
    {
        log_obstacle("IMU calibration failed. Keep it still, level and "
                     "try again.");
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
        barcode_command_t command = barcode_command();

        buzzer_beep_soft(BARCODE_BEEP_MSEC);   // A barcode was read.
        web_barcode_record(barcode_message(), command);

        if (BARCODE_CMD_NONE == command)
        {
            log_line("Barcode MESSAGE: \"%s\"", barcode_message());
        }
        else
        {
            log_line("Barcode COMMAND: \"%s\" = %s", barcode_message(),
                     barcode_command_name(command));
            vehicle_barcode_command(command, false);
        }
        telemetry_text("barcode", barcode_message());
    }
    else if (BARCODE_INVALID == result)
    {
        log_line("Barcode NOT decoded: %" PRIu32 " bars+spaces seen "
                 "(valid: 10 per character - 1).", barcode_scan_size());
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
    static bool     b_stop_was = false;     // STOP down last tick.
    web_command_t   command    = WEB_CMD_NONE;
    bool            b_stop     = !gpio_get(PIN_BUTTON_STOP);

    // Every press is logged, so a button that does nothing can be told
    // apart from one the Pico never sees (wiring, or a pin in use).
    if (b_stop && !b_stop_was)
    {
        log_printf("Button: STOP (GP%u) pressed%s.", PIN_BUTTON_STOP,
                   (vehicle_is_idle() ? " (already stopped)" : ""));
    }
    b_stop_was = b_stop;

    if (b_stop)
    {
        if (!vehicle_is_idle())
        {
            command = WEB_CMD_STOP;
        }
    }
    else if (!gpio_get(PIN_BUTTON_START))
    {
        held_ticks++;
        if (BUTTON_DEBOUNCE_TICKS == held_ticks)
        {
            log_printf("Button: START (GP%u) pressed; starts on release, "
                       "hold 2 s to calibrate the IMU.", PIN_BUTTON_START);
        }
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

    if ('p' == key)
    {
        log_printf("Encoder pins: GP7=%lu GP28=%lu GP0=%lu GP1=%lu",
                (unsigned long)encoder_watch_edges(0u),
                (unsigned long)encoder_watch_edges(1u),
                (unsigned long)encoder_watch_edges(2u),
                (unsigned long)encoder_watch_edges(3u));
    }
    else if ('t' == key)
    {
        static uint32_t step = 0u;
        static const int32_t angles[] = {60, 90, 120, 90};

        if (vehicle_is_idle())
        {
            int32_t angle = angles[step];
            servo_set_angle(angle);
            log_printf("SERVO TEST: commanded %ld degrees",
                       (long)angle);
            step = (step + 1u) % 4u;
        }
        else
        {
            log_printf("SERVO TEST: stop the car first.");
        }
    }
    else if ('g' == key)
    {
        if (vehicle_is_idle() && !g_stationary_scan_test)
        {
            uint32_t range = ultrasonic_range_mm();

            if (range > 0u && range <= OBSTACLE_STOP_MM)
            {
                motor_set(0, 0);
                g_stationary_scan_test = true;
                obstacle_scan_start(range, to_ms_since_boot(get_absolute_time()));
                log_printf("STATIONARY SCAN TEST START");
            }
            else
            {
                log_printf("SCAN TEST: obstacle must be within stop range.");
            }
        }
        else
        {
            log_printf("SCAN TEST: stop the car first.");
        }
    }
    else if (KEY_ENCODER_MARK == key)
    {
        encoder_calibration_mark();
    }
    else if (KEY_STOP == key)
    {
        if (g_stationary_scan_test)
        {
            obstacle_scan_cancel();
            g_stationary_scan_test = false;
            log_printf("STATIONARY SCAN TEST CANCELLED");
        }
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
    else if (KEY_BARCODE_TEST == key)
    {
        command = WEB_CMD_BARCODE_TEST;
    }
    else
    {
        // No key, or one with no meaning.
    }

    return (command);
}   /* serial_poll() */


// Measure raw per-wheel edges over a user-measured straight roll. No
// wheel diameter or encoder counts/revolution assumption is required.
static void
encoder_calibration_mark(void)
{
    static uint32_t left_start, right_start;
    if (!vehicle_is_idle())
    {
        g_encoder_marked = false;
        log_printf("Encoder mark: STOP first, then press 'e'.");
        return;
    }
    if (!g_encoder_marked)
    {
        left_start = encoder_count(PIN_ENC_LEFT_A);
        right_start = encoder_count(PIN_ENC_RIGHT_A);
        g_encoder_marked = true;
        log_printf("Encoder mark set. Roll straight exactly 300 mm, then 'e' again. "
                   "Keep the car STOPPED throughout; do not lift/spin wheels.");
    }
    else
    {
        uint32_t left = encoder_count(PIN_ENC_LEFT_A) - left_start;
        uint32_t right = encoder_count(PIN_ENC_RIGHT_A) - right_start;
        g_encoder_marked = false;
        log_printf("Encoder 300 mm roll: LEFT=%lu RIGHT=%lu edges. "
                   "Repeat 3 times and record results; calibration not applied.",
                   (unsigned long)left, (unsigned long)right);
        if (left == 0u || right == 0u)
        {
            log_printf("Encoder calibration invalid: at least one wheel counted no edges.");
        }
    }
}


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

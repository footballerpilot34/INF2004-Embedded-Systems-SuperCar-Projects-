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
#include "stage.h"
#include "telemetry.h"
#include "ultrasonic.h"
#include "vehicle.h"
#include "web.h"


#define USB_WAIT_MSEC           3000u
#define USB_POLL_MSEC           100u
#define RESTART_TEXT_BYTES      96u     // "WATCHDOG: ... froze in ...".
#define FLASH_BYTES_MAX         0x01000000u // XIP window: 16 MB.
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
#define KEY_STATIONARY_SCAN     'g'
#define SCAN_READY_TIMEOUT_MSEC 2000u // Nonblocking wait for a fresh forward ping.

static bool g_encoder_marked;

static void          usb_wait(void);
static void          buttons_init(void);
static web_command_t buttons_poll(void);
static bool g_stationary_scan_test = false;
static bool g_stationary_scan_starting = false;
static uint32_t g_stationary_scan_since = 0u;

static web_command_t serial_poll(void);
static void          command_run(web_command_t command);
static void          imu_calibrate_run(void);
static void          wheel_test(void);
static bool          vehicle_is_idle(void);
static char const *  restart_reason(void);
void                 lwip_assert_record(char const * p_message);
void                 isr_hardfault(void);
static void          barcode_report(barcode_result_t result);
static void          turn_test(barcode_command_t command);
static void          encoder_calibration_mark(void);
static void          stationary_scan_test_start(void);
static void          stationary_scan_test_cancel(void);
static void          stationary_scan_test_task(void);


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
    {
        char const * p_reason = restart_reason();   // Reads it once.

        web_set_restart_reason(p_reason);
        log_printf("Last restart: %s.", p_reason);
    }

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

    // Restart by itself if the loop ever stops (a freeze), instead of
    // sitting dead; the page then reports a watchdog restart.
    watchdog_enable(WATCHDOG_MSEC, true);   // Paused while debugging.
    log_printf("Watchdog on: the car restarts itself if it freezes for "
               "%u ms.", WATCHDOG_MSEC);

    next_tick_usec = time_us_64() + TICK_USEC;

    for (;;)
    {
        int32_t servo_deg = 0;

        watchdog_update();
        watchdog_hw->scratch[STAGE_TIME_SCRATCH] =
            to_ms_since_boot(get_absolute_time());
        web_task();                         // Sets its own stages.
        stage_set(STAGE_POWER);
        power_task();
        stage_set(STAGE_BUTTONS);
        command_run(buttons_poll());
        stage_set(STAGE_SERIAL);
        command_run(serial_poll());
        stage_set(STAGE_COMMANDS);
        command_run(web_command_take());
        stage_set(STAGE_SERVO);
        if (web_servo_take(&servo_deg))
        {
            if (vehicle_is_moving() || g_stationary_scan_test)
            {
                log_servo("Servo: locked while moving or scanning.");
            }
            else
            {
                servo_set_angle(servo_deg);
            }
        }

        // The scan and the bypass aim the servo themselves.
        if (vehicle_is_moving() &&
            (VEHICLE_OBSTACLE_SCAN != vehicle_state()) &&
            (VEHICLE_BYPASS != vehicle_state()))
        {
            servo_set_angle(SERVO_CENTER_DEG);
        }
        // Keep STOP and the UI responsive while the sensor turns. A range
        // from a previous angle must never be used as a forward reading.
        stage_set(STAGE_ULTRASONIC);
        if (servo_is_settled())
        {
            ultrasonic_task();
        }
        stage_set(STAGE_IMU);
        imu_task();
        stage_set(STAGE_BARCODE);
        barcode_report(barcode_task());
        stage_set(STAGE_VEHICLE);
        if (vehicle_is_moving() &&
            (VEHICLE_OBSTACLE_SCAN != vehicle_state()) &&
            (VEHICLE_BYPASS != vehicle_state()) &&
            ((!servo_is_settled()) ||
             !ultrasonic_measurement_ready()))
        {
            motor_set(0, 0);
        }
        else
        {
            vehicle_task();
        }
        stage_set(STAGE_SCAN_TEST);
        stationary_scan_test_task();
        servo_idle_release(IDLE_POWER_SAVING && vehicle_is_idle() &&
                           !g_stationary_scan_test &&
                           !obstacle_scan_active());

        stage_set(STAGE_LINE_DIAG);
        line_diag_task();
        stage_set(STAGE_BUZZER);
        buzzer_task();

        tick_count++;
        stage_set(STAGE_TELEMETRY);
        if (0u == (tick_count % TELEMETRY_EVERY_TICKS))
        {
            telemetry_task();
        }

        // Wait for the next 10 ms tick.
        stage_set(STAGE_TICK_WAIT);
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
 * @brief lwIP's failed-check hook (LWIP_PLATFORM_ASSERT, lwipopts.h):
 *        record the message for the next boot, then stop. The watchdog
 *        restarts the Pico, and restart_reason() reports the message.
 *
 * @param[in] p_message lwIP's message (a string literal, in flash).
 */
void
lwip_assert_record (char const * p_message)
{
    watchdog_hw->scratch[STAGE_FAULT_SCRATCH] = (uint32_t) p_message;
    watchdog_hw->scratch[STAGE_KIND_SCRATCH]  = STAGE_KIND_ASSERT;
    panic("lwIP: %s", p_message);   // Prints, then waits for the watchdog.
}   /* lwip_assert_record() */


/*!
 * @brief CPU hard fault (a bad pointer, for example): record it for the
 *        next boot, then wait for the watchdog to restart the Pico.
 *
 * @par
 * Replaces the SDK's default handler, which would just stop for good.
 */
void
isr_hardfault (void)
{
    watchdog_hw->scratch[STAGE_KIND_SCRATCH] = STAGE_KIND_HARDFAULT;

    for (;;)
    {
        // Wait for the watchdog (WATCHDOG_MSEC).
    }
}   /* isr_hardfault() */


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
    static char const * const stage_names[STAGE_COUNT] =
    {
        "unknown", "Wi-Fi chip/network polling", "web server",
        "hotspot joining", "battery reading", "buttons", "serial keys",
        "running a command", "servo", "ultrasonic", "IMU", "barcode",
        "vehicle control", "obstacle scan test", "IR diag", "buzzer",
        "telemetry", "waiting for the next tick"
    };
    static char  text[RESTART_TEXT_BYTES];
    uint32_t     flags    = vreg_and_chip_reset_hw->chip_reset;
    uint32_t     marker   = watchdog_hw->scratch[STAGE_SCRATCH];
    uint32_t     stage    = (marker & ~STAGE_MAGIC_MASK);
    char const * p_reason = "unknown";

    uint32_t     kind     = watchdog_hw->scratch[STAGE_KIND_SCRATCH];
    uint32_t     message  = watchdog_hw->scratch[STAGE_FAULT_SCRATCH];

    // The freeze finder's record is only valid if we wrote it; clear it so
    // a later, different restart is not mistaken for this one.
    watchdog_hw->scratch[STAGE_SCRATCH]      = 0u;
    watchdog_hw->scratch[STAGE_KIND_SCRATCH] = 0u;

    if (watchdog_enable_caused_reboot() && (STAGE_KIND_ASSERT == kind) &&
        (message >= XIP_BASE) && (message < (XIP_BASE + FLASH_BYTES_MAX)))
    {
        // lwIP stopped on a failed check; its message is a string in flash
        // (the same firmware is still loaded), so it can be shown.
        (void) snprintf(text, sizeof(text), "CRASH: network stack check "
                        "failed: %.60s", (char const *) message);
        p_reason = text;
    }
    else if (watchdog_enable_caused_reboot() &&
             (STAGE_KIND_HARDFAULT == kind))
    {
        (void) snprintf(text, sizeof(text), "CRASH: CPU hard fault in %s",
                        ((((marker & STAGE_MAGIC_MASK) == STAGE_MAGIC) &&
                          (stage < (uint32_t) STAGE_COUNT)) ?
                         stage_names[stage] : "unknown"));
        p_reason = text;
    }
    else if (watchdog_enable_caused_reboot())
    {
        // Our watchdog fired: the loop stopped for WATCHDOG_MSEC.
        (void) snprintf(text, sizeof(text), "WATCHDOG: the program froze in "
                        "%s, %" PRIu32 " s after power-up",
                        ((((marker & STAGE_MAGIC_MASK) == STAGE_MAGIC) &&
                          (stage < (uint32_t) STAGE_COUNT)) ?
                         stage_names[stage] : "unknown"),
                        (watchdog_hw->scratch[STAGE_TIME_SCRATCH] / 1000u));
        p_reason = text;
    }
    else if (watchdog_caused_reboot())
    {
        p_reason = "software reset (e.g. after flashing)";
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
    if (g_stationary_scan_test &&
        (command == WEB_CMD_START || command == WEB_CMD_WHEEL_TEST ||
         command == WEB_CMD_TURN_LEFT || command == WEB_CMD_TURN_RIGHT ||
         command == WEB_CMD_UTURN || command == WEB_CMD_BARCODE_TEST ||
         command == WEB_CMD_FEATURE_TEST ||
         command == WEB_CMD_BARCODE_STATIC || command == WEB_CMD_IMU_CALIBRATE))
    {
        web_set_scan_test_status("REJECTED: scan running; STOP before driving or other tests.");
        return;
    }
    // Any other command cancels an unfinished roll measurement; the mark
    // command itself (the second press) must not, or it never finishes.
    if ((WEB_CMD_NONE != command) && (WEB_CMD_ENCODER_MARK != command) &&
        g_encoder_marked)
    {
        g_encoder_marked = false;
        log_printf("Encoder roll mark cancelled by a vehicle command; "
                   "press 'e' to restart.");
    }
    switch (command)
    {
    case WEB_CMD_STOP:
        stationary_scan_test_cancel();
        vehicle_stop();
    break;

    case WEB_CMD_STATIONARY_SCAN:
        stationary_scan_test_start();
    break;

    case WEB_CMD_ENCODER_MARK:
        encoder_calibration_mark();     // Same as serial 'e'.
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

    case WEB_CMD_FEATURE_TEST:
        if (line_diag_is_active())
        {
            log_printf("IR DIAG is on (motors off): turn it off first.");
        }
        else
        {
            bool b_line    = true;
            bool b_avoid   = true;
            bool b_barcode = true;

            web_features(&b_line, &b_avoid, &b_barcode);
            vehicle_feature_test(b_line, b_avoid, b_barcode);
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


static void
stationary_scan_test_status(char const * p_status)
{
    web_set_scan_test_status(p_status);
    log_printf("STATIONARY SCAN TEST %s", p_status);
}


static void
stationary_scan_test_start(void)
{
    if (g_stationary_scan_test || obstacle_scan_active())
    {
        stationary_scan_test_status("REJECTED: scan already running.");
    }
    else if (!vehicle_is_idle())
    {
        stationary_scan_test_status("REJECTED: stop the vehicle first.");
    }
    else
    {
        motor_set(0, 0);
        g_stationary_scan_test = true;
        g_stationary_scan_starting = true;
        g_stationary_scan_since = to_ms_since_boot(get_absolute_time());
        web_set_scan_test_busy(true);
        // Completion recentres the servo and invalidates the previous ray.
        // Never reject a repeat click just because that fresh ping is pending,
        // and never use the last side-facing echo as a forward safety check.
        servo_set_angle(SERVO_CENTER_DEG);
        ultrasonic_invalidate();
        stationary_scan_test_status("STARTED: waiting for forward reading; wheels braked.");
    }
}


static void
stationary_scan_test_cancel(void)
{
    if (g_stationary_scan_test)
    {
        obstacle_scan_cancel();
        g_stationary_scan_test = false;
        g_stationary_scan_starting = false;
        web_set_scan_test_busy(false);
        motor_set(0, 0);
        stationary_scan_test_status("CANCELLED: wheels braked.");
    }
}


static void
stationary_scan_test_task(void)
{
    if (!g_stationary_scan_test) { return; }
    if (!vehicle_is_idle())
    {
        stationary_scan_test_cancel();
        return;
    }
    motor_set(0, 0);
    if (g_stationary_scan_starting)
    {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (!servo_is_settled() || !ultrasonic_measurement_ready())
        {
            if (now - g_stationary_scan_since >= SCAN_READY_TIMEOUT_MSEC)
            {
                g_stationary_scan_test = g_stationary_scan_starting = false;
                web_set_scan_test_busy(false);
                stationary_scan_test_status("FAILED: no fresh forward reading; wheels braked.");
            }
            return;
        }
        uint32_t range = ultrasonic_range_mm();
        g_stationary_scan_starting = false;
        if (range == 0u || range > OBSTACLE_STOP_MM)
        {
            g_stationary_scan_test = false;
            web_set_scan_test_busy(false);
            stationary_scan_test_status("REJECTED: obstacle must be within stop range.");
            return;
        }
        obstacle_scan_start(range, now);
        stationary_scan_test_status("STARTED: scanning; wheels braked.");
        return;
    }
    obstacle_scan_task(to_ms_since_boot(get_absolute_time()));
    if (!obstacle_scan_active())
    {
        g_stationary_scan_test = false;
        web_set_scan_test_busy(false);
        stationary_scan_test_status(obstacle_profile()->timed_out ?
            "FAILED: timed out; results incomplete. Wheels braked." :
            "COMPLETE: results below; wheels braked.");
    }
}


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

        if (vehicle_is_idle() && !obstacle_scan_active())
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
    else if (KEY_STATIONARY_SCAN == key)
    {
        command = WEB_CMD_STATIONARY_SCAN;
    }
    else if (KEY_ENCODER_MARK == key)
    {
        encoder_calibration_mark();
    }
    else if (KEY_STOP == key)
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
        log_printf("Encoder mark: STOP first, then press 'e' / ENCODER MARK.");
        return;
    }
    if (!g_encoder_marked)
    {
        left_start = encoder_count(PIN_ENC_LEFT_A);
        right_start = encoder_count(PIN_ENC_RIGHT_A);
        g_encoder_marked = true;
        // Two lines: a log line holds at most 96 characters.
        log_printf("Encoder mark set. Roll straight exactly 300 mm, then "
                   "press 'e' / ENCODER MARK again.");
        log_printf("Keep the car STOPPED throughout; do not lift or spin the "
                   "wheels.");
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
    watchdog_update();                  // Each wait is under WATCHDOG_MSEC.
    motor_set(0, 0);
    sleep_ms(WHEEL_TEST_PAUSE_MSEC);
    watchdog_update();

    log_printf("Wheel test: RIGHT wheel should spin FORWARD...");
    motor_set(0, WHEEL_TEST_SPEED);
    sleep_ms(WHEEL_TEST_MSEC);
    watchdog_update();
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

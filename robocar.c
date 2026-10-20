/** @file robocar.c
 *
 * @brief Wi-Fi remote-control car for the Cytron Robo Pico with a
 *        Raspberry Pi Pico W (or Pico 2 W).
 *
 * @par How it works
 * 1. The Pico W makes its own Wi-Fi network called "RoboCar".
 * 2. A phone joins that network and opens http://192.168.4.1.
 * 3. Tapping the buttons on that page drives the car.
 *
 * @par Safety
 * If no phone is connected to the car's Wi-Fi, the motors stop. The on-board
 * LED blinks while waiting for a phone and stays on once one is connected.
 * While the car is stopped for an obstacle, the LED flashes every 3 seconds.
 *
 * @par Wiring
 * The Robo Pico fixes these pins: left motor (M1) on GP8 and GP9, right
 * motor (M2) on GP10 and GP11, and servo port 1 on GP12. The 3.3 V HC-SR04
 * ultrasonic sensor is powered from 3V3, with TRIG on GP16 and ECHO on GP17.
 * ECHO is a 3.3 V signal, so no voltage divider is needed.
 *
 * @par Coding standard
 * This module follows BARR-C:2018, with these module-level deviations:
 * - Rules 4.1.d, 4.2.a and 4.3.a: the whole program is one source file
 *   without a header file, to keep this single-board project simple.
 * - Rule 6.5.a: echo_edge_isr() is a GPIO callback that the Pico SDK calls
 *   from its own interrupt handler, so it needs no compiler ISR keyword.
 * - Rule 5.2.a: Pico SDK and lwIP functions use their own types (uint,
 *   u16_t, int, err_t); those types are kept where an API requires them.
 *
 * TODO: Record the name of the person who approved these deviations.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "lwip/tcp.h"
#include "dhcpserver.h"


// Settings you can change.
//
#define WIFI_NAME                   "RoboCar"
#define WIFI_PASSWORD               "robocar123"  // At least 8 characters.

#define DRIVE_SPEED                 15    // Percent power for forward/back.
#define TURN_SPEED                  15    // Percent power for spinning.

// Set a flag to true if that wheel spins backwards on "forward".
//
#define LEFT_REVERSED               true
#define RIGHT_REVERSED              true

#define SERVO_MIN_USEC              500   // Pulse width at 0 degrees.
#define SERVO_MAX_USEC              2500  // Pulse width at 180 degrees.
#define SERVO_CENTER                90    // Starting angle in degrees.
#define SERVO_RANGE                 30    // Maximum degrees either side.

#define ULTRASONIC_STOP_DISTANCE_CM 25u   // Stop when closer than this.
#define ULTRASONIC_CLEAR_READINGS   3u    // Clear pings needed to resume.

#define OBSTACLE_FLASH_PERIOD_MSEC  3000u // LED flash rate while stopped.
#define OBSTACLE_FLASH_ON_MSEC      200u  // Length of each flash.

// Robo Pico pin assignments, fixed by the board.
//
#define M1A_PIN                     8u
#define M1B_PIN                     9u
#define M2A_PIN                     10u
#define M2B_PIN                     11u
#define NUM_MOTOR_PINS              4u
#define SERVO_PIN                   12u
#define ULTRASONIC_TRIG_PIN         16u
#define ULTRASONIC_ECHO_PIN         17u

// Motor PWM settings. 20 kHz is silent and is the Robo Pico's maximum.
//
#define MOTOR_PWM_HZ                20000u
#define MAX_SPEED                   100

// Servo PWM settings: 1 tick per microsecond, 20 ms period (50 Hz).
//
#define HZ_PER_MHZ                  1000000u
#define SERVO_PERIOD_TICKS          19999u
#define SERVO_FULL_SCALE_DEGREES    180

// Ultrasonic timing. Sound takes about 58 us to travel 1 cm and back.
//
#define ULTRASONIC_TRIGGER_USEC     10u
#define ULTRASONIC_TIMEOUT_USEC     30000u
#define ULTRASONIC_PING_PERIOD_USEC 60000u
#define USEC_PER_CM                 58u
#define TENTHS_PER_UNIT             10u

// GPIO interrupt edges for the ECHO pin.
// NOTE: The SDK's edge flags are small positive enum values, so they can be
// cast to an unsigned mask safely.
//
#define ECHO_EDGE_RISE              ((uint32_t) GPIO_IRQ_EDGE_RISE)
#define ECHO_EDGE_FALL              ((uint32_t) GPIO_IRQ_EDGE_FALL)

// Main loop timing.
//
#define USEC_PER_MSEC               1000u
#define STATUS_CHECK_MSEC           500u
#define LOOP_WAIT_MSEC              1u

// Wi-Fi station query limits.
//
#define MAX_STATIONS                8
#define MAC_ADDRESS_BYTES           6

// Web server settings.
//
#define HTTP_PORT                   80u
#define LISTEN_BACKLOG              4u
#define REQUEST_BUFFER_BYTES        128u
#define HEADER_BUFFER_BYTES         160u
#define HTTP_GET_PREFIX             "GET "
#define HTTP_GET_PREFIX_LEN         (sizeof(HTTP_GET_PREFIX) - 1u)
#define SERVO_COMMAND_PREFIX        "/servo?a="
#define SERVO_COMMAND_PREFIX_LEN    (sizeof(SERVO_COMMAND_PREFIX) - 1u)


// PWM counter value that means 100% motor power.
//
static uint16_t g_motor_top = 0u;

// Obstacle state, shared by the ultrasonic, motor and LED code.
//
static bool     gb_is_obstacle_close = false;
static uint64_t g_obstacle_since_usec = 0u;

// Last speeds asked for, so driving resumes after an obstacle clears.
//
static int32_t  g_drive_left  = 0;
static int32_t  g_drive_right = 0;

// ECHO edge times, written by echo_edge_isr(). Zero means "not seen yet".
// NOTE: These are 32-bit so that each read and write is atomic.
//
static volatile uint32_t g_echo_rise_usec = 0u;
static volatile uint32_t g_echo_fall_usec = 0u;

// The control page served to the phone.
//
static char const g_page_html[] =
    "<!DOCTYPE html><html><head>"
    "<meta name=viewport content=\"width=device-width,initial-scale=1\">"
    "<title>RoboCar</title><style>"
    "body{font-family:sans-serif;text-align:center;background:#111;"
    "color:#eee;margin:0;padding:16px}"
    ".g{display:grid;grid-template-columns:repeat(3,90px);gap:10px;"
    "justify-content:center;margin:20px 0}"
    "button{height:90px;font-size:30px;border:0;border-radius:14px;"
    "background:#2a7;color:#fff}"
    "#s{background:#c33}input{width:260px}"
    "</style></head><body><h2>RoboCar</h2><div class=g>"
    "<span></span><button onclick=\"c('fwd')\">&#9650;</button>"
    "<span></span>"
    "<button onclick=\"c('left')\">&#9664;</button>"
    "<button id=s onclick=\"c('stop')\">&#9632;</button>"
    "<button onclick=\"c('right')\">&#9654;</button>"
    "<span></span><button onclick=\"c('back')\">&#9660;</button>"
    "<span></span>"
    "</div><p>Servo: <b id=v>90</b>&deg;</p>"
    "<input type=range min=60 max=120 value=90 "
    "oninput=\"document.getElementById('v').textContent=this.value\" "
    "onchange=\"c('servo?a='+this.value)\">"
    "<script>function c(x){fetch('/'+x).catch(function(){})}</script>"
    "</body></html>";


static void  motors_init(void);
static void  motor_set(uint32_t pin_a, uint32_t pin_b, int32_t speed,
                       bool b_reversed);
static void  motors_apply(void);
static void  drive(int32_t left, int32_t right);
static void  echo_edge_isr(uint gpio, uint32_t events);
static void  ultrasonic_init(void);
static void  ultrasonic_update_obstacle(bool b_detected);
static void  ultrasonic_reading(bool b_is_close);
static void  ultrasonic_poll(void);
static void  servo_set_angle(int32_t degrees);
static void  servo_init(void);
static bool  handle_command(char const * p_path);
static void  send_reply(struct tcp_pcb * p_pcb, char const * p_status,
                        char const * p_type, char const * p_body,
                        size_t body_len);
static void  request_handle(struct tcp_pcb * p_pcb, struct pbuf * p_buf);
static err_t web_receive_callback(void * p_arg, struct tcp_pcb * p_pcb,
                                  struct pbuf * p_buf, err_t err);
static err_t web_accept_callback(void * p_arg, struct tcp_pcb * p_client,
                                 err_t err);
static bool  web_server_start(void);
static bool  network_start(void);
static bool  wifi_status_update(bool b_led_on);
static void  led_update(bool b_status_on);
static void  car_control_task(void);


/*!
 * @brief Start the hardware and network, then run the car forever.
 *
 * @return 1 if the Wi-Fi chip or web server failed to start. Otherwise
 *         this function never returns.
 */
int
main (void)
{
    int result = 0;

    stdio_init_all();
    motors_init();
    drive(0, 0);
    ultrasonic_init();
    servo_init();

    if (!network_start())
    {
        result = 1;
    }
    else
    {
        car_control_task();
    }

    return (result);
}   /* main() */


/*!
 * @brief Set up PWM on all four motor pins, with every motor stopped.
 */
static void
motors_init (void)
{
    static uint32_t const motor_pins[NUM_MOTOR_PINS] =
    {
        M1A_PIN, M1B_PIN, M2A_PIN, M2B_PIN
    };

    // NOTE: clk_sys / 20 kHz - 1 is 6249 at 125 MHz and 7499 at 150 MHz,
    // so the result always fits in 16 bits.
    g_motor_top = (uint16_t) ((clock_get_hz(clk_sys) / MOTOR_PWM_HZ) - 1u);

    for (uint32_t pin_idx = 0u; pin_idx < NUM_MOTOR_PINS; pin_idx++)
    {
        uint32_t slice = pwm_gpio_to_slice_num(motor_pins[pin_idx]);

        gpio_set_function(motor_pins[pin_idx], GPIO_FUNC_PWM);
        pwm_set_wrap(slice, g_motor_top);
        pwm_set_gpio_level(motor_pins[pin_idx], 0u);
        pwm_set_enabled(slice, true);
    }
}   /* motors_init() */


/*!
 * @brief Drive one motor at a given speed and direction.
 *
 * @param[in] pin_a      The motor's A pin.
 * @param[in] pin_b      The motor's B pin.
 * @param[in] speed      -100 (full reverse) to +100 (full forward).
 * @param[in] b_reversed True if this motor is wired backwards.
 */
static void
motor_set (uint32_t pin_a, uint32_t pin_b, int32_t speed, bool b_reversed)
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
        // The speed is already within range.
    }

    // NOTE: limited is within +/-100 here, so its magnitude fits unsigned,
    // and magnitude * g_motor_top / 100 is at most g_motor_top (16 bits).
    magnitude = (uint32_t) ((limited < 0) ? -limited : limited);
    level     = (uint16_t) ((magnitude * g_motor_top) / (uint32_t) MAX_SPEED);

    if (limited >= 0)
    {
        // A pulses and B stays low: one direction.
        pwm_set_gpio_level(pin_a, level);
        pwm_set_gpio_level(pin_b, 0u);
    }
    else
    {
        // A stays low and B pulses: the other direction.
        pwm_set_gpio_level(pin_a, 0u);
        pwm_set_gpio_level(pin_b, level);
    }
}   /* motor_set() */


/*!
 * @brief Send the last requested speeds to the motors, or stop them while
 *        an obstacle is close.
 */
static void
motors_apply (void)
{
    int32_t left  = (gb_is_obstacle_close ? 0 : g_drive_left);
    int32_t right = (gb_is_obstacle_close ? 0 : g_drive_right);

    motor_set(M1A_PIN, M1B_PIN, left, LEFT_REVERSED);
    motor_set(M2A_PIN, M2B_PIN, right, RIGHT_REVERSED);
}   /* motors_apply() */


/*!
 * @brief Remember and apply new speeds for both wheels.
 *
 * @param[in] left  Left wheel speed, -100 to +100.
 * @param[in] right Right wheel speed, -100 to +100.
 */
static void
drive (int32_t left, int32_t right)
{
    g_drive_left  = left;
    g_drive_right = right;
    motors_apply();
}   /* drive() */


/*!
 * @brief Timestamp the ECHO pin's edges.
 *
 * @par
 * Timing the echo in an interrupt keeps the measurement independent of how
 * often the main loop runs (only about once per millisecond).
 *
 * @param[in] gpio   The GPIO pin that caused the interrupt.
 * @param[in] events The edge events that occurred.
 */
static void
echo_edge_isr (uint gpio, uint32_t events)
{
    uint32_t now_usec = time_us_32();

    if (ULTRASONIC_ECHO_PIN == gpio)
    {
        if (0u != (events & ECHO_EDGE_RISE))
        {
            g_echo_rise_usec = now_usec;
        }

        if ((0u != (events & ECHO_EDGE_FALL)) && (0u != g_echo_rise_usec))
        {
            g_echo_fall_usec = now_usec;
        }
    }
}   /* echo_edge_isr() */


/*!
 * @brief Set up the TRIG and ECHO pins and the ECHO edge interrupt.
 */
static void
ultrasonic_init (void)
{
    gpio_init(ULTRASONIC_TRIG_PIN);
    gpio_set_dir(ULTRASONIC_TRIG_PIN, GPIO_OUT);
    gpio_put(ULTRASONIC_TRIG_PIN, false);

    gpio_init(ULTRASONIC_ECHO_PIN);
    gpio_set_dir(ULTRASONIC_ECHO_PIN, GPIO_IN);
    gpio_pull_down(ULTRASONIC_ECHO_PIN);
    gpio_set_irq_enabled_with_callback(ULTRASONIC_ECHO_PIN,
                                       (ECHO_EDGE_RISE | ECHO_EDGE_FALL),
                                       true, echo_edge_isr);
}   /* ultrasonic_init() */


/*!
 * @brief Record whether an obstacle is close, and stop or resume driving.
 *
 * @param[in] b_detected True if an obstacle is within the stop distance.
 */
static void
ultrasonic_update_obstacle (bool b_detected)
{
    if (b_detected != gb_is_obstacle_close)
    {
        if (b_detected)
        {
            printf("Obstacle detected within %u cm; motors stopped.\n",
                   ULTRASONIC_STOP_DISTANCE_CM);
            g_obstacle_since_usec = time_us_64();
        }
        else
        {
            printf("Obstacle cleared; resuming.\n");
        }
    }

    gb_is_obstacle_close = b_detected;
    motors_apply();
}   /* ultrasonic_update_obstacle() */


/*!
 * @brief Filter one ultrasonic reading before acting on it.
 *
 * @par
 * The car stops on the first close reading but resumes only after several
 * clear readings in a row, so a single bad ping (for example, motor noise
 * on battery power) cannot restart it.
 *
 * @param[in] b_is_close True if this reading was within the stop distance.
 */
static void
ultrasonic_reading (bool b_is_close)
{
    static uint32_t clear_count = 0u;

    if (b_is_close)
    {
        clear_count = 0u;
        ultrasonic_update_obstacle(true);
    }
    else if (clear_count < (ULTRASONIC_CLEAR_READINGS - 1u))
    {
        clear_count++;
    }
    else
    {
        clear_count = ULTRASONIC_CLEAR_READINGS;
        ultrasonic_update_obstacle(false);
    }
}   /* ultrasonic_reading() */


/*!
 * @brief Send a ping when one is due, and handle the echo once it ends.
 *
 * @par
 * Call this from the main loop. It never waits for an echo, so Wi-Fi and
 * the web server keep running.
 */
static void
ultrasonic_poll (void)
{
    static bool     b_waiting_for_echo = false;
    static uint64_t next_ping_usec     = 0u;
    static uint64_t echo_deadline_usec = 0u;
    uint64_t        now_usec           = time_us_64();

    if (!b_waiting_for_echo)
    {
        if (now_usec >= next_ping_usec)
        {
            g_echo_rise_usec = 0u;
            g_echo_fall_usec = 0u;

            gpio_put(ULTRASONIC_TRIG_PIN, true);
            sleep_us(ULTRASONIC_TRIGGER_USEC);
            gpio_put(ULTRASONIC_TRIG_PIN, false);

            echo_deadline_usec = time_us_64() + ULTRASONIC_TIMEOUT_USEC;
            next_ping_usec     = time_us_64() + ULTRASONIC_PING_PERIOD_USEC;
            b_waiting_for_echo = true;
        }
    }
    else if (0u != g_echo_fall_usec)
    {
        // NOTE: Unsigned subtraction gives the right pulse width even if
        // the 32-bit timer wrapped between the two edges.
        uint32_t echo_usec = g_echo_fall_usec - g_echo_rise_usec;
        uint32_t tenths_cm = (echo_usec * TENTHS_PER_UNIT) / USEC_PER_CM;

        printf("Ultrasonic: %" PRIu32 ".%" PRIu32 " cm\n",
               tenths_cm / TENTHS_PER_UNIT, tenths_cm % TENTHS_PER_UNIT);
        ultrasonic_reading(echo_usec <
                           (ULTRASONIC_STOP_DISTANCE_CM * USEC_PER_CM));
        b_waiting_for_echo = false;
    }
    else if (now_usec >= echo_deadline_usec)
    {
        // An echo that started but never ended is out of range. No echo at
        // all usually means the sensor has no power or is not wired.
        printf((0u != g_echo_rise_usec) ? "Ultrasonic: out of range\n"
                                        : "Ultrasonic: no echo\n");
        ultrasonic_reading(false);
        b_waiting_for_echo = false;
    }
    else
    {
        // The echo has not finished yet; check again next time.
    }
}   /* ultrasonic_poll() */


/*!
 * @brief Turn the servo, limited to its allowed range around the centre.
 *
 * @param[in] degrees The requested angle in degrees.
 */
static void
servo_set_angle (int32_t degrees)
{
    int32_t limited = degrees;

    if (limited < (SERVO_CENTER - SERVO_RANGE))
    {
        limited = SERVO_CENTER - SERVO_RANGE;
    }
    else if (limited > (SERVO_CENTER + SERVO_RANGE))
    {
        limited = SERVO_CENTER + SERVO_RANGE;
    }
    else
    {
        // The angle is already within range.
    }

    // NOTE: limited is 60 to 120 degrees here, so the pulse width is
    // positive and well under 16 bits.
    pwm_set_gpio_level(SERVO_PIN,
                       (uint16_t) (SERVO_MIN_USEC +
                                   ((limited * (SERVO_MAX_USEC -
                                                SERVO_MIN_USEC)) /
                                    SERVO_FULL_SCALE_DEGREES)));
}   /* servo_set_angle() */


/*!
 * @brief Set up a 50 Hz servo PWM signal and centre the servo.
 */
static void
servo_init (void)
{
    uint32_t slice = pwm_gpio_to_slice_num(SERVO_PIN);

    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);

    // Divide the system clock so that 1 tick is 1 microsecond.
    // NOTE: clk_sys is 125 or 150 MHz, so the divider fits in 8 bits.
    pwm_set_clkdiv_int_frac(slice,
                            (uint8_t) (clock_get_hz(clk_sys) / HZ_PER_MHZ),
                            0u);
    pwm_set_wrap(slice, SERVO_PERIOD_TICKS);
    pwm_set_enabled(slice, true);
    servo_set_angle(SERVO_CENTER);
}   /* servo_init() */


/*!
 * @brief Run the drive or servo command named in a URL path.
 *
 * @param[in] p_path The URL path, such as "/fwd" or "/servo?a=90".
 *
 * @return True if the path was a known command.
 */
static bool
handle_command (char const * p_path)
{
    bool b_handled = true;

    if (0 == strcmp(p_path, "/fwd"))
    {
        drive(DRIVE_SPEED, DRIVE_SPEED);
    }
    else if (0 == strcmp(p_path, "/back"))
    {
        drive(-DRIVE_SPEED, -DRIVE_SPEED);
    }
    else if (0 == strcmp(p_path, "/left"))
    {
        drive(-TURN_SPEED, TURN_SPEED);
    }
    else if (0 == strcmp(p_path, "/right"))
    {
        drive(TURN_SPEED, -TURN_SPEED);
    }
    else if (0 == strcmp(p_path, "/stop"))
    {
        drive(0, 0);
    }
    else if (0 == strncmp(p_path, SERVO_COMMAND_PREFIX,
                          SERVO_COMMAND_PREFIX_LEN))
    {
        servo_set_angle(atoi(p_path + SERVO_COMMAND_PREFIX_LEN));
    }
    else
    {
        b_handled = false;
    }

    return (b_handled);
}   /* handle_command() */


/*!
 * @brief Send an HTTP reply on a connection.
 *
 * @param[in] p_pcb    The connection to reply on.
 * @param[in] p_status The HTTP status, such as "200 OK".
 * @param[in] p_type   The content type, such as "text/html".
 * @param[in] p_body   The reply body, which must be static.
 * @param[in] body_len The number of bytes in the body.
 */
static void
send_reply (struct tcp_pcb * p_pcb, char const * p_status,
            char const * p_type, char const * p_body, size_t body_len)
{
    char header[HEADER_BUFFER_BYTES] = { 0 };

    // NOTE: The largest body is the web page (about 1 KB), so body_len
    // fits in both uint32_t and lwIP's 16-bit length.
    int  header_len = snprintf(header, sizeof(header),
                               "HTTP/1.1 %s\r\n"
                               "Content-Type: %s\r\n"
                               "Content-Length: %" PRIu32 "\r\n"
                               "Cache-Control: no-store\r\n"
                               "Connection: close\r\n\r\n",
                               p_status, p_type, (uint32_t) body_len);

    // NOTE: The header always fits its buffer, so header_len is positive
    // and under 160.
    tcp_write(p_pcb, header, (u16_t) header_len, TCP_WRITE_FLAG_COPY);

    if (body_len > 0u)
    {
        // The body is static, so lwIP can send it without copying.
        tcp_write(p_pcb, p_body, (u16_t) body_len, 0u);
    }

    tcp_output(p_pcb);
}   /* send_reply() */


/*!
 * @brief Read one HTTP request and send the matching reply.
 *
 * @param[in] p_pcb The connection the request arrived on.
 * @param[in] p_buf The received data, which this function frees.
 */
static void
request_handle (struct tcp_pcb * p_pcb, struct pbuf * p_buf)
{
    char  request[REQUEST_BUFFER_BYTES] = { 0 };
    u16_t length = pbuf_copy_partial(p_buf, request, sizeof(request) - 1u,
                                     0u);

    request[length] = '\0';
    tcp_recved(p_pcb, p_buf->tot_len);
    pbuf_free(p_buf);

    // The first line looks like "GET /fwd HTTP/1.1"; cut out "/fwd".
    if (0 == strncmp(request, HTTP_GET_PREFIX, HTTP_GET_PREFIX_LEN))
    {
        char * p_path  = request + HTTP_GET_PREFIX_LEN;
        char * p_space = strchr(p_path, ' ');

        if (NULL != p_space)
        {
            *p_space = '\0';
        }

        if (0 == strcmp(p_path, "/"))
        {
            send_reply(p_pcb, "200 OK", "text/html", g_page_html,
                       sizeof(g_page_html) - 1u);
        }
        else if (handle_command(p_path))
        {
            send_reply(p_pcb, "200 OK", "text/plain", "OK", 2u);
        }
        else
        {
            send_reply(p_pcb, "404 Not Found", "text/plain", "", 0u);
        }
    }
}   /* request_handle() */


/*!
 * @brief lwIP callback for data received on a web connection.
 *
 * @param[in] p_arg Unused.
 * @param[in] p_pcb The connection.
 * @param[in] p_buf The received data, or NULL if the phone hung up.
 * @param[in] err   Unused.
 *
 * @return ERR_ABRT if the connection had to be aborted, otherwise ERR_OK.
 */
static err_t
web_receive_callback (void * p_arg, struct tcp_pcb * p_pcb,
                      struct pbuf * p_buf, err_t err)
{
    err_t result = ERR_OK;

    // Both parameters are required by lwIP's callback signature only.
    (void) p_arg;
    (void) err;

    if (NULL == p_buf)
    {
        // The phone hung up.
        tcp_close(p_pcb);
    }
    else
    {
        request_handle(p_pcb, p_buf);

        // Handle one request per connection; closing sends the reply.
        tcp_recv(p_pcb, NULL);
        if (ERR_OK != tcp_close(p_pcb))
        {
            tcp_abort(p_pcb);
            result = ERR_ABRT;
        }
    }

    return (result);
}   /* web_receive_callback() */


/*!
 * @brief lwIP callback for a new web connection.
 *
 * @param[in] p_arg    Unused.
 * @param[in] p_client The new connection.
 * @param[in] err      The accept status from lwIP.
 *
 * @return ERR_VAL if the connection is unusable, otherwise ERR_OK.
 */
static err_t
web_accept_callback (void * p_arg, struct tcp_pcb * p_client, err_t err)
{
    err_t result = ERR_OK;

    // This parameter is required by lwIP's callback signature only.
    (void) p_arg;

    if ((ERR_OK != err) || (NULL == p_client))
    {
        result = ERR_VAL;
    }
    else
    {
        tcp_recv(p_client, web_receive_callback);
    }

    return (result);
}   /* web_accept_callback() */


/*!
 * @brief Start listening for web connections on port 80.
 *
 * @return True if the web server started.
 */
static bool
web_server_start (void)
{
    struct tcp_pcb * p_pcb      = tcp_new_ip_type(IPADDR_TYPE_ANY);
    struct tcp_pcb * p_listener = NULL;
    bool             b_started  = false;

    if ((NULL != p_pcb) && (ERR_OK == tcp_bind(p_pcb, IP_ANY_TYPE, HTTP_PORT)))
    {
        p_listener = tcp_listen_with_backlog(p_pcb, LISTEN_BACKLOG);
    }

    if (NULL != p_listener)
    {
        tcp_accept(p_listener, web_accept_callback);
        b_started = true;
    }

    return (b_started);
}   /* web_server_start() */


/*!
 * @brief Start the Wi-Fi hotspot, DHCP server and web server.
 *
 * @return True if everything started.
 */
static bool
network_start (void)
{
    static dhcp_server_t dhcp_server;
    ip_addr_t            gateway;
    ip_addr_t            netmask;
    bool                 b_started = false;

    if (0 != cyw43_arch_init())
    {
        printf("Wi-Fi chip failed to start\n");
    }
    else
    {
        // Turn the Pico W into a Wi-Fi hotspot.
        cyw43_arch_enable_ap_mode(WIFI_NAME, WIFI_PASSWORD,
                                  CYW43_AUTH_WPA2_AES_PSK);

        // Hand out IP addresses to phones that join, like a home router.
        // The gateway is 192.168.4.1 and the netmask is 255.255.255.0.
        ip_addr_set_ip4_u32(&gateway, PP_HTONL(CYW43_DEFAULT_IP_AP_ADDRESS));
        ip_addr_set_ip4_u32(&netmask, PP_HTONL(CYW43_DEFAULT_IP_MASK));
        dhcp_server_init(&dhcp_server, &cyw43_state.netif[CYW43_ITF_AP],
                         &gateway, &netmask);

        if (!web_server_start())
        {
            printf("Web server failed to start\n");
        }
        else
        {
            printf("Join Wi-Fi \"%s\" and open http://192.168.4.1\n",
                   WIFI_NAME);
            b_started = true;
        }
    }

    return (b_started);
}   /* network_start() */


/*!
 * @brief Check whether a phone is connected, and stop the car if not.
 *
 * @param[in] b_led_on The current Wi-Fi status LED state.
 *
 * @return The new status LED state: blinking while waiting for a phone,
 *         on while one is connected.
 */
static bool
wifi_status_update (bool b_led_on)
{
    uint8_t macs[MAX_STATIONS * MAC_ADDRESS_BYTES] = { 0 };
    int     num_phones = MAX_STATIONS;  // The SDK takes an int count.
    bool    b_new_led  = true;

    cyw43_wifi_ap_get_stas(&cyw43_state, &num_phones, macs);

    if (0 == num_phones)
    {
        // Failsafe: nobody is connected, so stop and blink while waiting.
        drive(0, 0);
        b_new_led = !b_led_on;
    }

    return (b_new_led);
}   /* wifi_status_update() */


/*!
 * @brief Show the Wi-Fi status on the on-board LED, or flash it while the
 *        car is stopped for an obstacle.
 *
 * @param[in] b_status_on The Wi-Fi status LED state.
 */
static void
led_update (bool b_status_on)
{
    static bool b_led_shown = false;
    bool        b_led_now   = b_status_on;

    if (gb_is_obstacle_close)
    {
        uint64_t elapsed_msec = (time_us_64() - g_obstacle_since_usec) /
                                USEC_PER_MSEC;

        b_led_now = ((elapsed_msec % OBSTACLE_FLASH_PERIOD_MSEC) <
                     OBSTACLE_FLASH_ON_MSEC);
    }

    // Only talk to the Wi-Fi chip when the LED needs to change.
    if (b_led_now != b_led_shown)
    {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, b_led_now);
        b_led_shown = b_led_now;
    }
}   /* led_update() */


/*!
 * @brief Run the car's main loop forever.
 */
static void
car_control_task (void)
{
    bool            b_led_on   = false;
    absolute_time_t next_check = get_absolute_time();

    for (;;)
    {
        // Let the Wi-Fi chip and web server do their work.
        cyw43_arch_poll();
        ultrasonic_poll();

        if (time_reached(next_check))
        {
            b_led_on   = wifi_status_update(b_led_on);
            next_check = make_timeout_time_ms(STATUS_CHECK_MSEC);
        }

        led_update(b_led_on);
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(LOOP_WAIT_MSEC));
    }
}   /* car_control_task() */

/*** end of file ***/

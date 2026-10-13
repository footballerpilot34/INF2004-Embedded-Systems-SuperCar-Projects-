/*
 * robocar.c - Wi-Fi remote-control car
 * Board: Cytron Robo Pico + Raspberry Pi Pico W (or Pico 2 W)
 *
 * How it works:
 *   1. The Pico W makes its own Wi-Fi network called "RoboCar".
 *   2. Your phone joins that network and opens http://192.168.4.1
 *   3. Tapping the buttons on that page drives the car.
 *
 * Safety: if no phone is connected to the car's Wi-Fi, the motors stop.
 * On-board LED: blinking = waiting for a phone, solid = phone connected.
 *
 * Robo Pico pins (fixed by the board, no wiring needed):
 *   Left motor  (M1): GP8 = M1A, GP9  = M1B
 *   Right motor (M2): GP10 = M2A, GP11 = M2B
 *   Servo port  1   : GP12
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "lwip/tcp.h"
#include "dhcpserver.h"

/* ---------------- Settings you can change ---------------- */
#define WIFI_NAME       "RoboCar"
#define WIFI_PASSWORD   "robocar123"   /* must be at least 8 characters */

#define DRIVE_SPEED     60             /* % power for forward/back (0-100) */
#define TURN_SPEED      45             /* % power when spinning left/right */

#define LEFT_REVERSED   false          /* set true if left wheel spins backwards on "forward" */
#define RIGHT_REVERSED  false          /* set true if right wheel spins backwards on "forward" */

#define SERVO_MIN_US    500            /* pulse width at 0 degrees   */
#define SERVO_MAX_US    2500           /* pulse width at 180 degrees */
/* ---------------------------------------------------------- */

#define M1A 8
#define M1B 9
#define M2A 10
#define M2B 11
#define SERVO_PIN 12
#define MOTOR_PWM_HZ 20000             /* 20 kHz = silent, and the Robo Pico's max */

static uint16_t motor_top;             /* PWM counter value that means "100%" */

/* ===================== Motors ===================== */

static void motors_init(void) {
    motor_top = clock_get_hz(clk_sys) / MOTOR_PWM_HZ - 1;
    const uint pins[] = {M1A, M1B, M2A, M2B};
    for (int i = 0; i < 4; i++) {
        gpio_set_function(pins[i], GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(pins[i]);
        pwm_set_wrap(slice, motor_top);
        pwm_set_gpio_level(pins[i], 0);
        pwm_set_enabled(slice, true);
    }
}

/* speed: -100 (full reverse) ... 0 (stop) ... +100 (full forward) */
static void motor_set(uint pin_a, uint pin_b, int speed, bool reversed) {
    if (reversed) speed = -speed;
    if (speed > 100)  speed = 100;
    if (speed < -100) speed = -100;

    uint16_t level = (uint32_t)abs(speed) * motor_top / 100;
    if (speed >= 0) {                       /* A pulses, B low  -> one direction   */
        pwm_set_gpio_level(pin_a, level);
        pwm_set_gpio_level(pin_b, 0);
    } else {                                /* A low,  B pulses -> other direction */
        pwm_set_gpio_level(pin_a, 0);
        pwm_set_gpio_level(pin_b, level);
    }
}

static void drive(int left, int right) {
    motor_set(M1A, M1B, left,  LEFT_REVERSED);
    motor_set(M2A, M2B, right, RIGHT_REVERSED);
}

/* ===================== Servo ===================== */

static void servo_set_angle(int deg) {
    if (deg < 0)   deg = 0;
    if (deg > 180) deg = 180;
    pwm_set_gpio_level(SERVO_PIN, SERVO_MIN_US + deg * (SERVO_MAX_US - SERVO_MIN_US) / 180);
}

static void servo_init(void) {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_set_clkdiv(slice, clock_get_hz(clk_sys) / 1000000.0f);  /* 1 tick = 1 microsecond */
    pwm_set_wrap(slice, 19999);                                  /* 20 ms period = 50 Hz   */
    pwm_set_enabled(slice, true);
    servo_set_angle(90);                                         /* start centred */
}

/* ===================== Web page ===================== */

static const char PAGE[] =
    "<!DOCTYPE html><html><head>"
    "<meta name=viewport content=\"width=device-width,initial-scale=1\">"
    "<title>RoboCar</title><style>"
    "body{font-family:sans-serif;text-align:center;background:#111;color:#eee;margin:0;padding:16px}"
    ".g{display:grid;grid-template-columns:repeat(3,90px);gap:10px;justify-content:center;margin:20px 0}"
    "button{height:90px;font-size:30px;border:0;border-radius:14px;background:#2a7;color:#fff}"
    "#s{background:#c33}input{width:260px}"
    "</style></head><body><h2>RoboCar</h2><div class=g>"
    "<span></span><button onclick=\"c('fwd')\">&#9650;</button><span></span>"
    "<button onclick=\"c('left')\">&#9664;</button>"
    "<button id=s onclick=\"c('stop')\">&#9632;</button>"
    "<button onclick=\"c('right')\">&#9654;</button>"
    "<span></span><button onclick=\"c('back')\">&#9660;</button><span></span>"
    "</div><p>Servo: <b id=v>90</b>&deg;</p>"
    "<input type=range min=0 max=180 value=90 "
    "oninput=\"document.getElementById('v').textContent=this.value\" "
    "onchange=\"c('servo?a='+this.value)\">"
    "<script>function c(x){fetch('/'+x).catch(function(){})}</script>"
    "</body></html>";

/* Runs the command in the URL. Returns false if the URL isn't a known command. */
static bool handle_command(const char *path) {
    if      (strcmp(path, "/fwd") == 0)   drive(DRIVE_SPEED, DRIVE_SPEED);
    else if (strcmp(path, "/back") == 0)  drive(-DRIVE_SPEED, -DRIVE_SPEED);
    else if (strcmp(path, "/left") == 0)  drive(-TURN_SPEED, TURN_SPEED);
    else if (strcmp(path, "/right") == 0) drive(TURN_SPEED, -TURN_SPEED);
    else if (strcmp(path, "/stop") == 0)  drive(0, 0);
    else if (strncmp(path, "/servo?a=", 9) == 0) servo_set_angle(atoi(path + 9));
    else return false;
    return true;
}

/* ===================== Tiny web server ===================== */

static void send_reply(struct tcp_pcb *pcb, const char *status, const char *type,
                       const char *body, size_t body_len) {
    char header[160];
    int n = snprintf(header, sizeof header,
                     "HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %u\r\n"
                     "Cache-Control: no-store\r\nConnection: close\r\n\r\n",
                     status, type, (unsigned)body_len);
    tcp_write(pcb, header, n, TCP_WRITE_FLAG_COPY);
    if (body_len) tcp_write(pcb, body, body_len, 0);   /* body is static, no copy needed */
    tcp_output(pcb);
}

static err_t on_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    (void)arg; (void)err;
    if (p == NULL) {                       /* the phone hung up */
        tcp_close(pcb);
        return ERR_OK;
    }

    char req[128];
    u16_t n = pbuf_copy_partial(p, req, sizeof req - 1, 0);
    req[n] = '\0';
    tcp_recved(pcb, p->tot_len);
    pbuf_free(p);

    /* First line looks like: "GET /fwd HTTP/1.1" -> cut out "/fwd" */
    if (strncmp(req, "GET ", 4) == 0) {
        char *path = req + 4;
        char *space = strchr(path, ' ');
        if (space) *space = '\0';

        if (strcmp(path, "/") == 0)
            send_reply(pcb, "200 OK", "text/html", PAGE, sizeof PAGE - 1);
        else if (handle_command(path))
            send_reply(pcb, "200 OK", "text/plain", "OK", 2);
        else
            send_reply(pcb, "404 Not Found", "text/plain", "", 0);
    }

    tcp_recv(pcb, NULL);                   /* one request per connection */
    if (tcp_close(pcb) != ERR_OK) {        /* sends the reply, then closes */
        tcp_abort(pcb);
        return ERR_ABRT;
    }
    return ERR_OK;
}

static err_t on_accept(void *arg, struct tcp_pcb *client, err_t err) {
    (void)arg;
    if (err != ERR_OK || client == NULL) return ERR_VAL;
    tcp_recv(client, on_recv);
    return ERR_OK;
}

static bool web_server_start(void) {
    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb || tcp_bind(pcb, IP_ANY_TYPE, 80) != ERR_OK) return false;
    struct tcp_pcb *listener = tcp_listen_with_backlog(pcb, 4);
    if (!listener) return false;
    tcp_accept(listener, on_accept);
    return true;
}

/* ===================== Main ===================== */

int main(void) {
    stdio_init_all();
    motors_init();
    drive(0, 0);
    servo_init();

    if (cyw43_arch_init()) {
        printf("Wi-Fi chip failed to start\n");
        return 1;
    }

    /* Turn the Pico W into a Wi-Fi hotspot */
    cyw43_arch_enable_ap_mode(WIFI_NAME, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);

    /* Hand out IP addresses to phones that join (like a home router does) */
    ip_addr_t gw, mask;
    ip_addr_set_ip4_u32(&gw,   PP_HTONL(CYW43_DEFAULT_IP_AP_ADDRESS));   /* 192.168.4.1   */
    ip_addr_set_ip4_u32(&mask, PP_HTONL(CYW43_DEFAULT_IP_MASK));         /* 255.255.255.0 */
    static dhcp_server_t dhcp;
    dhcp_server_init(&dhcp, &cyw43_state.netif[CYW43_ITF_AP], &gw, &mask);

    if (!web_server_start()) {
        printf("Web server failed to start\n");
        return 1;
    }
    printf("Join Wi-Fi \"%s\" and open http://192.168.4.1\n", WIFI_NAME);

    bool led = false;
    absolute_time_t next_check = get_absolute_time();
    while (true) {
        cyw43_arch_poll();                                 /* let Wi-Fi + web server do their work */

        if (time_reached(next_check)) {
            uint8_t macs[8 * 6];
            int phones = 8;
            cyw43_wifi_ap_get_stas(&cyw43_state, &phones, macs);

            if (phones == 0) {
                drive(0, 0);                               /* failsafe: nobody connected -> stop */
                led = !led;                                /* blink while waiting */
            } else {
                led = true;                                /* solid when connected */
            }
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led);
            next_check = make_timeout_time_ms(500);
        }

        cyw43_arch_wait_for_work_until(next_check);        /* nap until there's work */
    }
}

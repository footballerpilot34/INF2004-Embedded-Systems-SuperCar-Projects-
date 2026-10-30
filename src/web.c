/** @file web.c
 *
 * @brief Wireless UI: Wi-Fi hotspot plus a small web page with controls,
 *        live readings, the barcode message and the message log.
 *
 * @par How it works
 * The Pico W makes its own Wi-Fi network (WIFI_NAME in config.h) and
 * serves a page at http://192.168.4.1. The page:
 * - sends GET /cmd?c=start|stop|cal|wheel when a button is pressed;
 * - polls GET /status?s=N every 0.5 s, which returns the live readings,
 *   the last barcode message and every log message from number N on.
 * Commands are only queued here; main() runs them, exactly like a button
 * press, so the web UI never drives the motors directly.
 *
 * @par
 * lwIP runs in polled mode (NO_SYS): web_task() must be called every
 * control tick, and all callbacks run inside it, on the main loop.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "dhcpserver.h"

#include "barcode.h"
#include "config.h"
#include "imu.h"
#include "line_sensor.h"
#include "log.h"
#include "motor.h"
#include "ultrasonic.h"
#include "vehicle.h"
#include "web.h"


#define HTTP_PORT               80u
#define LISTEN_BACKLOG          4u
#define REQUEST_BYTES           128u
#define HEADER_BYTES            160u
#define STATUS_BYTES            1400u   // Fits lwIP's 4 KB copy heap.
#define LINE_RESERVE            200u    // Room for one escaped log line.
#define LOG_KEPT                32u     // Must match LOG_LINES in log.c.
#define NO_RANGE_JSON           (-1)

#define GET_PREFIX              "GET "
#define GET_PREFIX_LEN          (sizeof(GET_PREFIX) - 1u)
#define CMD_PREFIX              "/cmd?c="
#define CMD_PREFIX_LEN          (sizeof(CMD_PREFIX) - 1u)
#define STATUS_PREFIX           "/status?s="
#define STATUS_PREFIX_LEN       (sizeof(STATUS_PREFIX) - 1u)
#define DECIMAL_BASE            10


static char const g_page_html[] =
    "<!DOCTYPE html><html><head>"
    "<meta name=viewport content=\"width=device-width,initial-scale=1\">"
    "<title>SuperCar</title><style>"
    "body{font-family:sans-serif;background:#111;color:#eee;margin:0;"
    "padding:12px}"
    ".g{display:grid;grid-template-columns:1fr 1fr;gap:8px}"
    "button{font-size:20px;padding:16px 0;border:0;border-radius:10px;"
    "color:#fff}"
    "#a{background:#2a7}#b{background:#c33}#k{background:#36c}"
    "#w{background:#555}#d{background:#a60}#n{background:#888}"
    "#v{background:#333}.t{background:#538;font-size:16px}"
    ".c{background:#222;border-radius:10px;padding:10px;margin:10px 0;"
    "line-height:1.6}"
    "#bc{font-size:30px;font-weight:bold;color:#fc3;word-break:break-all}"
    "#log{font-family:monospace;font-size:12px;height:45vh;overflow:auto;"
    "white-space:pre-wrap;background:#000;padding:8px;border-radius:8px}"
    "</style></head><body><h2>SuperCar AAI14</h2><div class=g>"
    "<button id=a onclick=\"c('start')\">START</button>"
    "<button id=b onclick=\"c('stop')\">STOP</button>"
    "<button id=k onclick=\"c('cal')\">CALIBRATE</button>"
    "<button id=w onclick=\"c('wheel')\">WHEEL TEST</button>"
    "<button id=d onclick=\"c('diag')\">IR DIAG on/off</button>"
    "<button id=v onclick=\"c('black')\">LEARN BLACK</button>"
    "<button id=n onclick=\"c('white')\">LEARN WHITE</button>"
    "<button class=t onclick=\"c('tl')\">TEST LEFT</button>"
    "<button class=t onclick=\"c('tr')\">TEST RIGHT</button>"
    "<button class=t onclick=\"c('tu')\">TEST U-TURN</button></div>"
    "<div class=c>State: <b id=s>-</b><br>"
    "Range: <b id=r>-</b> cm &nbsp; Pitch: <b id=p>-</b>&deg;<br>"
    "IR L/R/Barcode: <b id=i>-</b> &nbsp; Motors L/R: <b id=m>-</b><br>"
    "IR raw pins L(GP28)/R(GP0): <b id=x>-</b><br>"
    "IMU: <b id=u>-</b></div>"
    "<div class=c>Barcode (<b id=bs>-</b>)<div id=bc>-</div>"
    "<b id=bk></b></div>"
    "<div id=log></div>"
    "<script>var q=0;function $(i){return document.getElementById(i)}"
    "function c(x){fetch('/cmd?c='+x).catch(function(){})}"
    "function t(v){return (v/10).toFixed(1)}"
    "function u(){fetch('/status?s='+q).then(function(r){return r.json()})"
    ".then(function(d){var L=$('log');"
    "if(d.nx<q){L.textContent=''}"
    "$('s').textContent=d.st;$('r').textContent=d.rg<0?'none':t(d.rg);"
    "$('p').textContent=t(d.pt);$('i').textContent=d.l+' / '+d.r+' / '+d.b;"
    "$('m').textContent=d.ml+' / '+d.mr;"
    "$('x').textContent=(d.lh?'HIGH':'LOW')+' / '+(d.rh?'HIGH':'LOW');"
    "$('u').textContent=d.imu?'OK':'NOT FOUND';"
    "$('bc').textContent=d.bc||'-';$('bk').textContent=d.bc?d.bk:'';"
    "$('bs').textContent=d.b?'over black':'over white';"
    "for(var k=0;k<d.log.length;k++){L.textContent+=d.log[k]+'\\n'}"
    "if(d.log.length){L.scrollTop=L.scrollHeight}q=d.nx;"
    "setTimeout(u,d.more?50:500)})"
    ".catch(function(){setTimeout(u,1000)})}u();</script>"
    "</body></html>";

static web_command_t g_pending = WEB_CMD_NONE;
static bool          gb_ready  = false;


static err_t  accept_callback(void * p_arg, struct tcp_pcb * p_client,
                              err_t err);
static err_t  receive_callback(void * p_arg, struct tcp_pcb * p_pcb,
                               struct pbuf * p_buf, err_t err);
static void   request_handle(struct tcp_pcb * p_pcb, char * p_path);
static void   command_parse(char const * p_name);
static size_t status_build(char * p_out, size_t capacity, uint32_t since);
static size_t json_string(char * p_out, size_t capacity, char const * p_text);
static void   reply_send(struct tcp_pcb * p_pcb, char const * p_type,
                         char const * p_body, size_t body_len,
                         bool b_static);


/*!
 * @brief Start the Wi-Fi hotspot, DHCP server and web server.
 *
 * @return True if the wireless UI is running.
 */
bool
web_init (void)
{
    static dhcp_server_t dhcp_server;
    ip_addr_t            gateway;
    ip_addr_t            netmask;
    struct tcp_pcb *     p_pcb      = NULL;
    struct tcp_pcb *     p_listener = NULL;

    gb_ready = false;

    if (0 != cyw43_arch_init())
    {
        log_printf("Wi-Fi: chip failed to start; wireless UI is OFF.");
    }
    else
    {
        cyw43_arch_enable_ap_mode(WIFI_NAME, WIFI_PASSWORD,
                                  CYW43_AUTH_WPA2_AES_PSK);

        // Hand out addresses like a home router: 192.168.4.x.
        ip_addr_set_ip4_u32(&gateway, PP_HTONL(CYW43_DEFAULT_IP_AP_ADDRESS));
        ip_addr_set_ip4_u32(&netmask, PP_HTONL(CYW43_DEFAULT_IP_MASK));
        dhcp_server_init(&dhcp_server, &cyw43_state.netif[CYW43_ITF_AP],
                         &gateway, &netmask);

        p_pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
        if ((NULL != p_pcb) &&
            (ERR_OK == tcp_bind(p_pcb, IP_ANY_TYPE, HTTP_PORT)))
        {
            p_listener = tcp_listen_with_backlog(p_pcb, LISTEN_BACKLOG);
        }

        if (NULL != p_listener)
        {
            tcp_accept(p_listener, accept_callback);
            gb_ready = true;
            log_printf("Wi-Fi: join \"%s\" (password \"%s\"), then open "
                       "http://192.168.4.1", WIFI_NAME, WIFI_PASSWORD);
        }
        else
        {
            log_printf("Wi-Fi: web server failed to start.");
        }
    }

    return (gb_ready);
}   /* web_init() */


/*!
 * @brief Let the Wi-Fi chip and web server do their work. Call every tick.
 */
void
web_task (void)
{
    if (gb_ready)
    {
        cyw43_arch_poll();
    }
}   /* web_task() */


/*!
 * @brief Take the command from the last web button press, if any.
 *
 * @return The pending command, or WEB_CMD_NONE. It is cleared once taken.
 */
web_command_t
web_command_take (void)
{
    web_command_t command = g_pending;

    g_pending = WEB_CMD_NONE;

    return (command);
}   /* web_command_take() */


/*!
 * @brief lwIP callback for a new connection.
 *
 * @param[in] p_arg    Unused.
 * @param[in] p_client The new connection.
 * @param[in] err      The accept status.
 *
 * @return ERR_VAL if the connection is unusable, otherwise ERR_OK.
 */
static err_t
accept_callback (void * p_arg, struct tcp_pcb * p_client, err_t err)
{
    err_t result = ERR_OK;

    (void) p_arg;   // Required by lwIP's callback signature only.

    if ((ERR_OK != err) || (NULL == p_client))
    {
        result = ERR_VAL;
    }
    else
    {
        tcp_recv(p_client, receive_callback);
    }

    return (result);
}   /* accept_callback() */


/*!
 * @brief lwIP callback for received data: one request per connection.
 *
 * @param[in] p_arg Unused.
 * @param[in] p_pcb The connection.
 * @param[in] p_buf The data, or NULL if the client closed.
 * @param[in] err   Unused.
 *
 * @return ERR_ABRT if the connection was aborted, otherwise ERR_OK.
 */
static err_t
receive_callback (void * p_arg, struct tcp_pcb * p_pcb,
                  struct pbuf * p_buf, err_t err)
{
    err_t result = ERR_OK;

    (void) p_arg;   // Both required by lwIP's callback signature only.
    (void) err;

    if (NULL == p_buf)
    {
        tcp_close(p_pcb);
    }
    else
    {
        char  request[REQUEST_BYTES] = { 0 };
        u16_t length = pbuf_copy_partial(p_buf, request,
                                         sizeof(request) - 1u, 0u);

        request[length] = '\0';
        tcp_recved(p_pcb, p_buf->tot_len);
        pbuf_free(p_buf);

        // The first line looks like "GET /path HTTP/1.1"; cut out the path.
        if (0 == strncmp(request, GET_PREFIX, GET_PREFIX_LEN))
        {
            char * p_path  = request + GET_PREFIX_LEN;
            char * p_space = strchr(p_path, ' ');

            if (NULL != p_space)
            {
                *p_space = '\0';
            }
            request_handle(p_pcb, p_path);
        }

        tcp_recv(p_pcb, NULL);
        if (ERR_OK != tcp_close(p_pcb))
        {
            tcp_abort(p_pcb);
            result = ERR_ABRT;
        }
    }

    return (result);
}   /* receive_callback() */


/*!
 * @brief Answer one GET request.
 *
 * @param[in] p_pcb  The connection.
 * @param[in] p_path The requested path, such as "/status?s=12".
 */
static void
request_handle (struct tcp_pcb * p_pcb, char * p_path)
{
    static char status[STATUS_BYTES];

    if (0 == strcmp(p_path, "/"))
    {
        reply_send(p_pcb, "text/html", g_page_html, sizeof(g_page_html) - 1u,
                   true);
    }
    else if (0 == strncmp(p_path, STATUS_PREFIX, STATUS_PREFIX_LEN))
    {
        // NOTE: The page only ever sends sequence numbers it was given,
        // which fit uint32_t.
        uint32_t since  = (uint32_t) strtoul(p_path + STATUS_PREFIX_LEN, NULL,
                                             DECIMAL_BASE);
        size_t   length = status_build(status, sizeof(status), since);

        reply_send(p_pcb, "application/json", status, length, false);
    }
    else if (0 == strncmp(p_path, CMD_PREFIX, CMD_PREFIX_LEN))
    {
        command_parse(p_path + CMD_PREFIX_LEN);
        reply_send(p_pcb, "text/plain", "OK", 2u, true);
    }
    else
    {
        reply_send(p_pcb, "text/plain", "", 0u, true);
    }
}   /* request_handle() */


/*!
 * @brief Queue the command named in a /cmd request.
 *
 * @param[in] p_name "start", "stop", "cal", "wheel", "diag", "black",
 *                   "white", or "tl" / "tr" / "tu" (test turns).
 */
static void
command_parse (char const * p_name)
{
    if (0 == strcmp(p_name, "stop"))
    {
        g_pending = WEB_CMD_STOP;
    }
    else if (0 == strcmp(p_name, "start"))
    {
        g_pending = WEB_CMD_START;
    }
    else if (0 == strcmp(p_name, "cal"))
    {
        g_pending = WEB_CMD_CALIBRATE;
    }
    else if (0 == strcmp(p_name, "wheel"))
    {
        g_pending = WEB_CMD_WHEEL_TEST;
    }
    else if (0 == strcmp(p_name, "diag"))
    {
        g_pending = WEB_CMD_DIAG;
    }
    else if (0 == strcmp(p_name, "black"))
    {
        g_pending = WEB_CMD_LEARN_BLACK;
    }
    else if (0 == strcmp(p_name, "white"))
    {
        g_pending = WEB_CMD_LEARN_WHITE;
    }
    else if (0 == strcmp(p_name, "tl"))
    {
        g_pending = WEB_CMD_TURN_LEFT;
    }
    else if (0 == strcmp(p_name, "tr"))
    {
        g_pending = WEB_CMD_TURN_RIGHT;
    }
    else if (0 == strcmp(p_name, "tu"))
    {
        g_pending = WEB_CMD_UTURN;
    }
    else
    {
        log_printf("Web: unknown command \"%s\" ignored.", p_name);
    }
}   /* command_parse() */


/*!
 * @brief Build the /status JSON reply.
 *
 * @par
 * Readings are whole numbers (range in mm, pitch in tenths of a degree);
 * the page divides by 10. As many log lines as fit are included; "more"
 * tells the page to ask again at once.
 *
 * @param[out] p_out    The buffer to fill.
 * @param[in]  capacity The buffer size.
 * @param[in]  since    The first log sequence number the page wants.
 *
 * @return The number of bytes written.
 */
static size_t
status_build (char * p_out, size_t capacity, uint32_t since)
{
    line_state_t line     = line_sensor_read();
    line_raw_t   raw      = line_sensor_raw();
    uint32_t     range_mm = ultrasonic_range_mm();
    uint32_t     next_seq = log_next_seq();
    uint32_t     seq      = since;
    size_t       length   = 0u;
    char const * p_line   = NULL;
    bool         b_first  = true;

    // A sequence number from before a reboot starts the log again.
    if (seq > next_seq)
    {
        seq = 0u;
    }
    if ((next_seq - seq) > LOG_KEPT)
    {
        seq = next_seq - LOG_KEPT;
    }

    // NOTE: The fixed part of the reply is under 300 bytes, far below the
    // 1400-byte buffer, so these snprintf calls never truncate.
    length += (size_t) snprintf(p_out + length, capacity - length,
                                "{\"st\":\"%s\",\"rg\":%" PRId32 ","
                                "\"pt\":%" PRId32 ",\"l\":%d,\"r\":%d,"
                                "\"b\":%d,\"ml\":%" PRId32 ",\"mr\":%" PRId32
                                ",\"imu\":%d,\"lh\":%d,\"rh\":%d,"
                                "\"bc\":",
                                vehicle_state_name(vehicle_state()),
                                ((ULTRASONIC_NO_RANGE == range_mm) ?
                                 NO_RANGE_JSON : (int32_t) range_mm),
                                imu_pitch_x10(),
                                (line.b_left_black ? 1 : 0),
                                (line.b_right_black ? 1 : 0),
                                (barcode_is_black() ? 1 : 0),
                                motor_left_speed(), motor_right_speed(),
                                (imu_is_ok() ? 1 : 0),
                                (raw.b_left_high ? 1 : 0),
                                (raw.b_right_high ? 1 : 0));
    length += json_string(p_out + length, capacity - length,
                          barcode_message());
    length += (size_t) snprintf(p_out + length, capacity - length,
                                ",\"bk\":\"%s\",\"log\":[",
                                barcode_command_name(barcode_command()));

    while ((seq < next_seq) && ((capacity - length) > LINE_RESERVE) &&
           log_get(seq, &p_line))
    {
        if (!b_first)
        {
            p_out[length] = ',';
            length++;
        }
        length += json_string(p_out + length, capacity - length, p_line);
        b_first = false;
        seq++;
    }

    length += (size_t) snprintf(p_out + length, capacity - length,
                                "],\"nx\":%" PRIu32 ",\"more\":%d}", seq,
                                ((seq < next_seq) ? 1 : 0));

    return (length);
}   /* status_build() */


/*!
 * @brief Write a JSON string literal, escaping quotes and backslashes.
 *
 * @param[out] p_out    Where to write.
 * @param[in]  capacity Room left in the buffer (at least 3 bytes).
 * @param[in]  p_text   The text.
 *
 * @return The number of bytes written (not counting the terminator).
 */
static size_t
json_string (char * p_out, size_t capacity, char const * p_text)
{
    size_t length = 0u;

    p_out[length] = '"';
    length++;

    // Leave room for a 2-byte escape, the closing quote and a terminator.
    for (size_t idx = 0u; ('\0' != p_text[idx]) && ((length + 4u) < capacity);
         idx++)
    {
        char symbol = p_text[idx];

        if (('"' == symbol) || ('\\' == symbol))
        {
            p_out[length] = '\\';
            length++;
        }

        // Control characters would break the JSON, so show them as spaces.
        p_out[length] = ((symbol < ' ') ? ' ' : symbol);
        length++;
    }

    p_out[length] = '"';
    length++;
    p_out[length] = '\0';

    return (length);
}   /* json_string() */


/*!
 * @brief Send an HTTP reply.
 *
 * @param[in] p_pcb    The connection.
 * @param[in] p_type   The content type.
 * @param[in] p_body   The body.
 * @param[in] body_len The body length (under 4 KB).
 * @param[in] b_static True if the body is constant, so lwIP need not copy.
 */
static void
reply_send (struct tcp_pcb * p_pcb, char const * p_type,
            char const * p_body, size_t body_len, bool b_static)
{
    char header[HEADER_BYTES] = { 0 };

    // NOTE: Every body is under 4 KB, so it fits uint32_t and u16_t.
    int  header_len = snprintf(header, sizeof(header),
                               "HTTP/1.1 200 OK\r\n"
                               "Content-Type: %s\r\n"
                               "Content-Length: %" PRIu32 "\r\n"
                               "Cache-Control: no-store\r\n"
                               "Connection: close\r\n\r\n",
                               p_type, (uint32_t) body_len);

    // NOTE: The header always fits its buffer, so header_len is positive
    // and under 160.
    tcp_write(p_pcb, header, (u16_t) header_len, TCP_WRITE_FLAG_COPY);

    if (body_len > 0u)
    {
        tcp_write(p_pcb, p_body, (u16_t) body_len,
                  (b_static ? 0u : TCP_WRITE_FLAG_COPY));
    }

    tcp_output(p_pcb);
}   /* reply_send() */

/*** end of file ***/

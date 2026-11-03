/** @file web.c
 *
 * @brief Wireless UI with vehicle controls, live readings, IMU graphs and
 *        the message log (the same messages as the USB serial monitor).
 *
 * @par How it works
 * The Pico W makes its own Wi-Fi network (WIFI_NAME in config.h) and
 * serves a page at http://192.168.4.1. The page:
 * - sends GET /cmd?c=... when a button is pressed;
 * - sends GET /servo?a=<degrees> when the servo slider is released;
 * - polls GET /status?s=N every 0.2 s for live vehicle and sensor readings
 *   and every log message from number N on.
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
#define SERVO_PREFIX            "/servo?a="
#define SERVO_PREFIX_LEN        (sizeof(SERVO_PREFIX) - 1u)


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
    "canvas{display:block;width:100%;height:180px;background:#151a22;"
    "border-radius:8px}"
    ".vals{font-family:monospace;color:#ddd;font-size:14px}"
    ".hint{color:#aaa;font-size:12px}"
    ".leg{font-size:12px;color:#bbb}"
    "#log{font-family:monospace;font-size:12px;height:45vh;overflow:auto;"
    "white-space:pre-wrap;background:#000;padding:8px;border-radius:8px}"
    "</style></head><body><h2>SuperCar AAI14</h2><div class=g>"
    "<button id=a onclick=\"c('start')\">START</button>"
    "<button id=b onclick=\"c('stop')\">STOP</button>"
    "<button id=k onclick=\"c('calibrate')\">CALIBRATE IMU</button>"
    "<button id=w onclick=\"c('wheel')\">WHEEL TEST</button>"
    "<button id=d onclick=\"c('diag')\">IR DIAG on/off</button>"
    "<button id=v onclick=\"c('black')\">LEARN BLACK</button>"
    "<button id=n onclick=\"c('white')\">LEARN WHITE</button>"
    "<button class=t onclick=\"c('bt')\">BARCODE TEST</button></div>"
    "<div class=c>Servo: <b id=sv>90</b>&deg;<br>"
    "<input type=range min=60 max=120 value=90 style=width:100% "
    "oninput=\"$('sv').textContent=this.value\" "
    "onchange=\"fetch('/servo?a='+this.value).catch(function(){})\">"
    "</div>"
    "<div class=c>State: <b id=s>-</b><br>"
    "Range: <b id=r>-</b> cm &nbsp; LSM303DLHC: <b id=imu>-</b><br>"
    "Obstacle profile: <b id=op>-</b><br>"
    "IR L/R/Barcode: <b id=i>-</b> &nbsp; Motors L/R: <b id=m>-</b><br>"
    "IR raw pins L(GP28)/R(GP0): <b id=x>-</b></div>"
    "<div class=c><b>Accelerometer</b> <span class=leg>"
    "<span style=color:#ff6464>X</span> &nbsp;"
    "<span style=color:#58d8ff>Y</span> &nbsp;"
    "<span style=color:#9be564>Z</span></span>"
    "<div class=vals id=av>AX - &nbsp; AY - &nbsp; AZ -</div>"
    "<canvas id=ac></canvas><div class=hint>Fixed scale: -1.2 to +1.2 g. "
    "Three-sample display average.</div></div>"
    "<div class=c><b>Calibrated pitch / roll (one-wheel tilt)</b>"
    "<span class=leg><span style=color:#ff6464>Pitch</span> &nbsp;"
    "<span style=color:#58d8ff>Roll</span></span>"
    "<div class=vals id=pv>Pitch - &nbsp; Peak tilt -</div>"
    "<canvas id=pc></canvas><div class=hint>Fixed scale: -30 to +30 degrees. "
    "Pitch is low-pass filtered; calibrate while still and level.</div></div>"
    "<div class=c><b>Magnetometer</b> <span class=leg>"
    "<span style=color:#ff6464>X</span> &nbsp;"
    "<span style=color:#58d8ff>Y</span> &nbsp;"
    "<span style=color:#9be564>Z</span></span>"
    "<div class=vals id=mv>MX - &nbsp; MY - &nbsp; MZ -</div>"
    "<canvas id=mc></canvas><div class=hint>Fixed scale: -0.8 to +0.8 gauss. "
    "Three-sample display average.</div></div>"
    "<div class=c>Barcode message<div id=bc>-</div></div>"
    "<div class=c><b>Serial monitor</b><div id=log></div></div>"
    "<script>var hist=[],q=0;function $(i){return document.getElementById(i)}"
    "function c(x){fetch('/cmd?c='+x).catch(function(){})}"
    "function f(v){return (v<0?'-':'+')+Math.abs(v).toFixed(3)}"
    "function avg(i,k){var a=0,n=0;for(var j=Math.max(0,i-2);j<=i;j++)"
    "{a+=hist[j][k];n++}return n?a/n:0}"
    "function draw(id,keys,lim){var e=$(id),w=e.clientWidth,h=180;"
    "if(!w)return;var d=window.devicePixelRatio||1;e.width=w*d;e.height=h*d;"
    "var x=e.getContext('2d');x.scale(d,d);x.fillStyle='#151a22';"
    "x.fillRect(0,0,w,h);var L=36,R=8,T=10,B=22,P=w-L-R;"
    "x.font='11px sans-serif';x.textAlign='right';"
    "for(var z=-1;z<=1;z++){var y=T+(1-(z+1)/2)*(h-T-B);"
    "x.strokeStyle='#34404a';x.beginPath();x.moveTo(L,y);"
    "x.lineTo(w-R,y);x.stroke();x.fillStyle='#aab4bd';"
    "x.fillText((z*lim).toFixed(1),L-5,y+4)}"
    "var col=['#ff6464','#58d8ff','#9be564'];"
    "for(var a=0;a<keys.length;a++){x.strokeStyle=col[a];x.lineWidth=2;"
    "x.beginPath();for(var i=0;i<hist.length;i++){var v=avg(i,keys[a])/1000;"
    "v=Math.max(-lim,Math.min(lim,v));var px=L+(hist.length<2?0:"
    "i*P/(hist.length-1));var py=T+(1-(v/lim+1)/2)*(h-T-B);"
    "if(i===0)x.moveTo(px,py);else x.lineTo(px,py)}x.stroke()}"
    "x.fillStyle='#aab4bd';x.textAlign='left';x.fillText('old',L,h-5);"
    "x.textAlign='right';x.fillText('12 s',w-R,h-5)}"
    "function u(){fetch('/status?s='+q).then(function(r){return r.json()})"
    ".then(function(d){var L=$('log');if(d.nx<q){L.textContent=''}"
    "for(var k=0;k<d.log.length;k++){L.textContent+=d.log[k]+'\\n'}"
    "if(d.log.length){L.scrollTop=L.scrollHeight}q=d.nx;"
    "$('s').textContent=d.st;"
    "$('r').textContent=d.rg<0?'none':(d.rg/10).toFixed(1);"
    "var side=d.os<0?'LEFT':(d.os>0?'RIGHT':'-');"
    "$('op').textContent='width ~'+d.ow+' mm, clear L/R '+d.ol+'/'+d.or+"
    "' mm, choice '+side+(d.st==='OBSTACLE_SCAN'?' (scan '+d.oa+' deg: '+"
    "(d.od<0?'none':d.od+' mm')+')':'');"
    "$('i').textContent=d.l+' / '+d.r+' / '+d.b;"
    "$('m').textContent=d.ml+' / '+d.mr;"
    "$('x').textContent=(d.lh?'HIGH':'LOW')+' / '+(d.rh?'HIGH':'LOW');"
    "$('imu').textContent=!d.imu?'NOT FOUND':(!d.iv?'READ ERROR':"
    "(d.ic?'CALIBRATED':'NEEDS CALIBRATION'));"
    "$('bc').textContent=d.bc||'-';"
    "if(d.iv){hist.push({ax:d.ax,ay:d.ay,az:d.az,mx:d.mx,my:d.my,mz:d.mz,"
    "pt:d.pt,rl:d.rl});"
    "if(hist.length>60)hist.shift();"
    "function val(k){return f(avg(hist.length-1,k)/1000)}"
    "$('av').textContent='AX '+val('ax')+' g   AY '+val('ay')+"
    "' g   AZ '+val('az')+' g';"
    "$('mv').textContent='MX '+val('mx')+' G   MY '+val('my')+"
    "' G   MZ '+val('mz')+' G'}"
    "$('pv').textContent='Pitch '+(d.ic?(d.pt/1000).toFixed(1)+' deg':'-')+"
    "'   Peak tilt '+(d.pk/10).toFixed(1)+' deg';"
    "draw('ac',['ax','ay','az'],1.2);draw('pc',['pt','rl'],30.0);"
    "draw('mc',['mx','my','mz'],0.8);"
    "setTimeout(u,d.more?50:200)})"
    ".catch(function(){setTimeout(u,1000)})}u();</script>"
    "</body></html>";

static web_command_t g_pending = WEB_CMD_NONE;
static bool          gb_ready  = false;
static bool          gb_servo_pending = false;
static int32_t       g_servo_deg      = 0;


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
 * @brief Take the angle from the last servo slider change, if any.
 *
 * @param[out] p_degrees The requested angle in degrees (not yet limited).
 *
 * @return True if a new angle was requested since the last call.
 */
bool
web_servo_take (int32_t * p_degrees)
{
    bool b_new = gb_servo_pending;

    if (b_new)
    {
        *p_degrees       = g_servo_deg;
        gb_servo_pending = false;
    }

    return (b_new);
}   /* web_servo_take() */


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
 * @param[in] p_path The requested path.
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
    else if (0 == strncmp(p_path, SERVO_PREFIX, SERVO_PREFIX_LEN))
    {
        // main() limits the angle when it turns the servo.
        g_servo_deg      = (int32_t) atoi(p_path + SERVO_PREFIX_LEN);
        gb_servo_pending = true;
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
 *                   "white" or "bt" (barcode test).
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
    else if (0 == strcmp(p_name, "calibrate"))
    {
        g_pending = WEB_CMD_IMU_CALIBRATE;
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
    else if (0 == strcmp(p_name, "bt"))
    {
        g_pending = WEB_CMD_BARCODE_TEST;
    }
    else
    {
        log_printf("Web: unknown command \"%s\" ignored.", p_name);
    }
}   /* command_parse() */


/*!
 * @brief Build live vehicle, six-axis and calibrated pitch status, plus
 *        the log messages the page has not seen yet.
 *
 * @par
 * As many log lines as fit are included; "more" tells the page to ask
 * again at once.
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
    imu_axes_t   axes;
    size_t       length   = 0u;
    char const * p_line   = NULL;
    bool         b_first  = true;

    imu_get_axes(&axes);

    // A sequence number from before a reboot starts the log again.
    if (seq > next_seq)
    {
        seq = 0u;
    }
    if ((next_seq - seq) > LOG_KEPT)
    {
        seq = next_seq - LOG_KEPT;
    }

    // NOTE: The fixed part of the reply is under 500 bytes, well below the
    // 1400-byte buffer, so these snprintf calls never truncate.
    length += (size_t) snprintf(p_out + length, capacity - length,
                                "{\"st\":\"%s\",\"rg\":%" PRId32 ","
                                "\"l\":%d,\"r\":%d,\"b\":%d,"
                                "\"ml\":%" PRId32 ",\"mr\":%" PRId32 ","
                                "\"imu\":%d,\"iv\":%d,\"ic\":%d,"
                                "\"pt\":%" PRId32 ",\"rl\":%" PRId32 ","
                                "\"pk\":%" PRId32 ","
                                "\"ow\":%" PRId32 ",\"ol\":%" PRId32 ","
                                "\"or\":%" PRId32 ",\"os\":%" PRId32 ","
                                "\"oa\":%" PRId32 ",\"od\":%" PRId32 ","
                                "\"lh\":%d,\"rh\":%d,"
                                "\"ax\":%" PRId32 ",\"ay\":%" PRId32 ","
                                "\"az\":%" PRId32 ",\"mx\":%" PRId32 ","
                                "\"my\":%" PRId32 ",\"mz\":%" PRId32 ","
                                "\"bc\":",
                                vehicle_state_name(vehicle_state()),
                                ((ULTRASONIC_NO_RANGE == range_mm) ?
                                 NO_RANGE_JSON : (int32_t) range_mm),
                                (line.b_left_black ? 1 : 0),
                                (line.b_right_black ? 1 : 0),
                                (barcode_is_black() ? 1 : 0),
                                motor_left_speed(), motor_right_speed(),
                                (imu_is_ok() ? 1 : 0),
                                (imu_reading_is_valid() ? 1 : 0),
                                (imu_is_calibrated() ? 1 : 0),
                                (imu_pitch_x10() * 10),
                                (imu_roll_x10() * 10),
                                vehicle_hump_peak_tilt_x10(),
                                vehicle_obstacle_width_mm(),
                                vehicle_obstacle_left_clearance_mm(),
                                vehicle_obstacle_right_clearance_mm(),
                                vehicle_obstacle_side(),
                                vehicle_obstacle_scan_angle_deg(),
                                vehicle_obstacle_scan_range_mm(),
                                (raw.b_left_high ? 1 : 0),
                                (raw.b_right_high ? 1 : 0),
                                axes.ax_mg, axes.ay_mg, axes.az_mg,
                                axes.mx_mgauss, axes.my_mgauss,
                                axes.mz_mgauss);
    length += json_string(p_out + length, capacity - length,
                          barcode_message());
    length += (size_t) snprintf(p_out + length, capacity - length,
                                ",\"log\":[");

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

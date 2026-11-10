/** @file web.c
 *
 * @brief Wireless UI with vehicle controls, live readings, IMU graphs and
 *        the message log (the same messages as the USB serial monitor).
 *
 * @par How it works
 * If src/wifi_secrets.h lists hotspots, the Pico W tries each in turn
 * (or only the one chosen on the page) and serves the page at
 * http://WIFI_HOST_NAME.local (or the IP it logs). It keeps trying until
 * one answers and never makes a network of its own. Only if no hotspot is
 * listed at all does it make its own network (WIFI_NAME), with the page
 * at http://192.168.4.1. Joining never holds up the car. The page:
 * - sends GET /cmd?c=... when a button is pressed;
 * - sends GET /servo?a=<degrees> when the servo slider is released;
 * - gets GET /hotspots once for its hotspot chooser, and sends
 *   GET /wifi?h=<index, or -1 for each in turn> when USE is pressed;
 * - gets GET /history when the barcode history has changed;
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
#include "lwip/raw.h"
#include "lwip/icmp.h"
#include "lwip/inet_chksum.h"
#include "lwip/prot/ip4.h"
#include "lwip/apps/mdns.h"
#include "dhcpserver.h"

#include "barcode.h"
#include "encoder.h"
#include "config.h"
#include "imu.h"
#include "line_sensor.h"
#include "log.h"
#include "motor.h"
#include "power.h"
#include "ultrasonic.h"
#include "vehicle.h"
#include "web.h"


#define HTTP_PORT               80u
#define LISTEN_BACKLOG          4u
#define HTTP_POLL_INTERVAL      10u     // x 0.5 s: a connection open 5 s is
                                        // stuck and is freed.
#define PING_ID                 0xAF14u // Marks our pings' replies.
#define PING_DATA_BYTES         8u
#define REQUEST_BYTES           128u
#define HEADER_BYTES            160u
#define STATUS_BYTES            1400u   // Fits lwIP's 4 KB copy heap.
#define LINE_RESERVE            240u    // Room for one escaped log line
                                        // (under 200) and the reply's end.
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
#define WIFI_TEXT_BYTES         80u     // "Hotspot \"name\" at a.b.c.d".
#define HISTORY_PATH            "/history"
#define HISTORY_MAX             20u     // Barcodes kept for the page.
#define HISTORY_TEXT_BYTES      16u     // Longest message kept, plus '\0'.
#define MSEC_PER_SEC            1000u

typedef struct
{
    char              text[HISTORY_TEXT_BYTES];
    barcode_command_t command;
    uint32_t          time_sec;     // When it was read, since power-up.
} history_entry_t;

typedef enum
{
    WIFI_OFF,           // Chip failed to start.
    WIFI_JOINING,       // Joining the hotspot after power-up.
    WIFI_JOINED,        // On the hotspot.
    WIFI_REJOINING,     // Was on the hotspot; link lost, rejoining.
    WIFI_OWN_NETWORK    // Making our own network (192.168.4.1).
} wifi_mode_t;

typedef struct
{
    char const * p_ssid;
    char const * p_password;
} hotspot_t;

#define HOTSPOT_AUTO            (-1)    // Page choice: try each in turn.
#define HOTSPOT_NO_REQUEST      (-2)    // No new choice from the page.
#define HOTSPOTS_PATH           "/hotspots"
#define WIFI_PREFIX             "/wifi?h="
#define WIFI_PREFIX_LEN         (sizeof(WIFI_PREFIX) - 1u)
#define NUM_HOTSPOTS            ((uint32_t) (sizeof(g_hotspots) / \
                                             sizeof(g_hotspots[0])))


static char const g_page_html[] =
    "<!DOCTYPE html><html><head>"
    "<meta name=viewport content=\"width=device-width,initial-scale=1\">"
    "<title>SuperCar</title><style>"
    "body{font-family:sans-serif;background:#111;color:#eee;margin:0;"
    "padding:12px}"
    ".g{display:grid;grid-template-columns:1fr 1fr;gap:8px}"
    "button{font-size:20px;padding:16px 0;border:0;border-radius:10px;"
    "color:#fff}"
    "#a{background:#2a7}#b{background:#c33}#w{background:#555}"
    "#k{background:#36c}#d{background:#a60}#n{background:#888}"
    "#v{background:#333}summary{cursor:pointer;font-weight:bold}"
    ".t{background:#538;font-size:16px}"
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
    ".tabs{display:flex;flex-wrap:wrap;gap:6px;margin:8px 0}"
    ".tb{font-size:13px;padding:8px 10px;background:#333;border-radius:8px}"
    ".tb.on{background:#36c}"
    "#cn{padding:8px 10px;border-radius:8px;margin-bottom:8px;"
    "font-weight:bold;background:#555}"
    "#bh{font-family:monospace;font-size:13px;max-height:150px;"
    "overflow:auto;white-space:pre;background:#000;padding:6px 8px;"
    "border-radius:8px;margin-top:4px}"
    "</style></head><body><h2>SuperCar AAI14</h2>"
    "<div id=cn>Connecting to the car...</div><div class=g>"
    "<button id=a onclick=\"c('start')\">START</button>"
    "<button id=b onclick=\"c('stop')\">STOP/RESET</button>"
    "<button id=w onclick=\"c('wheel')\">WHEEL TEST</button>"
    "<button class=t onclick=\"c('bt')\">BARCODE TEST</button>"
    "<button class=t style=grid-column:1/3;background:#746 "
    "onclick=\"c('bs')\">BARCODE TEST (STATIC)</button></div>"
    "<details class=c><summary>More controls</summary><div class=g "
    "style=margin-top:8px>"
    "<button id=k onclick=\"c('calibrate')\">CALIBRATE IMU</button>"
    "<button id=d onclick=\"c('diag')\">IR DIAG on/off</button>"
    "<button id=v onclick=\"c('black')\">LEARN BLACK</button>"
    "<button id=n onclick=\"c('white')\">LEARN WHITE</button>"
    "</div></details>"
    // Servo: only while unlocked, and never while the car moves (the car
    // also ignores it then).
    "<div class=c>Servo: <b id=sv>90</b>&deg; "
    "<label style=float:right><input type=checkbox id=lk checked "
    "onchange=lock()> Locked</label><br>"
    "<input type=range id=sl min=60 max=120 value=90 disabled "
    "style=width:100% oninput=\"$('sv').textContent=this.value\" "
    "onchange=\"fetch('/servo?a='+this.value).catch(function(){})\">"
    "<div class=hint id=lh>Untick Locked to move the servo.</div>"
    "</div>"
    "<div class=c>Car State: <b id=s>-</b><br>"
    "Wi-Fi: <b id=wf>-</b><br>"
    "Uptime: <b id=up>-</b> &nbsp; Last restart: <b id=rr>-</b><br>"
    "Battery (VSYS): <b id=vb>-</b><br>"
    "<span id=hw style=display:none>Hotspot: <select id=hs "
    "style=font-size:14px;max-width:55%></select> "
    "<button class=tb onclick=hset()>USE</button><br>"
    "<span class=hint>The car may change network: rejoin your device to "
    "the same hotspot.</span><br></span>"
    "Obstacle Detection Range: <b id=r>-</b> cm<br>"
    "IR L/R/Barcode: <b id=i>-</b> &nbsp; Motors L/R: <b id=m>-</b><br>"
    "IR raw pins L(GP28)/R(GP0): <b id=x>-</b><br>"
    "Encoder ticks L/R: <b id=en>-</b><br>"
    "Encoder pins (edges): <b id=ep>-</b></div>"
    "<div class=c>Barcode message<div id=bc>-</div>"
    "Next crossroad: <b id=tn>-</b>"
    "<div class=hint style=margin-top:6px>History (newest first)</div>"
    "<div id=bh>No barcodes read yet.</div></div>"
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
    "Pitch is low-pass filtered; calibrate (hold GP20 2 s) while still "
    "and level.</div></div>"
    "<div class=c><b>Magnetometer</b> <span class=leg>"
    "<span style=color:#ff6464>X</span> &nbsp;"
    "<span style=color:#58d8ff>Y</span> &nbsp;"
    "<span style=color:#9be564>Z</span></span>"
    "<div class=vals id=mv>MX - &nbsp; MY - &nbsp; MZ -</div>"
    "<canvas id=mc></canvas><div class=hint>Fixed scale: -0.8 to +0.8 gauss. "
    "Three-sample display average.</div></div>"
    "<div class=c><b>Serial monitor</b><div class=tabs>"
    "<button class=\"tb on\" id=t0 onclick=tab(0)>All</button>"
    "<button class=tb id=t1 onclick=tab(1)>Obstacle / Hump</button>"
    "<button class=tb id=t2 onclick=tab(2)>Line / Barcode</button>"
    "<button class=tb id=t3 onclick=tab(3)>Vehicle state</button>"
    "<button class=tb id=t4 onclick=tab(4)>Servo</button></div>"
    "<div id=log></div>"
    "<button class=t style=width:100%;margin-top:8px "
    "onclick=\"c('clear');lg=[];nw=[0,0,0,0,0];show();"
    "$('bc').textContent='-'\">CLEAR OUTPUT</button></div>"
    "<script>var hist=[],q=0;function $(i){return document.getElementById(i)}"
    // Serial monitor: lg holds {c:category,t:text}; tab ft 0 shows all,
    // 1-4 one category (log_cat_t). nw counts lines unseen in each tab.
    "var lg=[],ft=0,nw=[0,0,0,0,0],"
    "tn=['All','Obstacle / Hump','Line / Barcode','Vehicle state',"
    "'Servo'];"
    "function show(){var L=$('log'),"
    "b=L.scrollTop+L.clientHeight>=L.scrollHeight-30,s='';"
    "for(var i=0;i<lg.length;i++){if(!ft||lg[i].c==ft)s+=lg[i].t+'\\n'}"
    "L.textContent=s;if(b)L.scrollTop=L.scrollHeight;"
    "for(var k=0;k<5;k++){var e=$('t'+k);"
    "e.className=k==ft?'tb on':'tb';"
    "e.textContent=tn[k]+(nw[k]?' ('+nw[k]+')':'')}}"
    "function tab(k){ft=k;if(k)nw[k]=0;else nw=[0,0,0,0,0];show();"
    "var L=$('log');L.scrollTop=L.scrollHeight}"
    // Barcode history: refetched whenever its number hq changes.
    // Servo lock: moving forces it locked and greys out the checkbox.
    "var mv=false;function lock(){var k=$('lk');if(mv)k.checked=true;"
    "k.disabled=mv;$('sl').disabled=k.checked;"
    "$('lh').textContent=mv?'Locked while the car is moving.':"
    "(k.checked?'Untick Locked to move the servo.':'')}"
    // Hotspot chooser: filled once; hidden when no hotspots are listed.
    "function hlist(){fetch('/hotspots').then(function(r){return r.json()})"
    ".then(function(j){if(!j.n.length)return;var h=$('hs'),"
    "o='<option value=-1>Auto (try each in turn)</option>';"
    "for(var i=0;i<j.n.length;i++){o+='<option value='+i+'></option>'}"
    "h.innerHTML=o;for(var i=0;i<j.n.length;i++){"
    "h.options[i+1].textContent=j.n[i]}h.value=j.sel;"
    "$('hw').style.display=''}).catch(function(){setTimeout(hlist,2000)})}"
    "function hset(){fetch('/wifi?h='+$('hs').value).catch(function(){})}"
    "hlist();"
    "var hq=-1;function two(n){return (n<10?'0':'')+n}"
    "function hget(){fetch('/history').then(function(r){return r.json()})"
    ".then(function(j){var s='';for(var i=0;i<j.h.length;i++){var e=j.h[i];"
    "s+=two(Math.floor(e[2]/60))+':'+two(e[2]%60)+'  \"'+e[0]+'\"  -> '+"
    "(e[1]?e[1]+' at next crossroad':'no command')+'\\n'}"
    "$('bh').textContent=s||'No barcodes read yet.';hq=j.hq})"
    ".catch(function(){})}"
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
    // Connection banner: green while the car answers, red with the time
    // since its last answer otherwise. A request gets 4 s at most, so a
    // stalled one never stops the updates.
    "var ok=0,up=0,rs=0;setInterval(function(){var e=$('cn'),a=Date.now()-ok;"
    "if(ok&&a<2500){e.style.background='#2a7';"
    "e.textContent='Connected to the car'}else{e.style.background='#c33';"
    "e.textContent=ok?'No answer from the car for '+Math.round(a/1000)+"
    "' s: retrying...':'Connecting to the car...'}},1000);"
    "function tget(x){var c=window.AbortController?new AbortController()"
    ":null,t=c?setTimeout(function(){c.abort()},4000):0;"
    "return fetch(x,c?{signal:c.signal}:{}).then(function(r){"
    "clearTimeout(t);return r.json()})}"
    "function u(){tget('/status?s='+q)"
    ".then(function(d){if(d.nx<q){lg=[]}"
    "for(var k=0;k<d.log.length;k++){var m=d.log[k],g=+m.charAt(0);"
    "lg.push({c:g,t:m.slice(1)});"
    "if(ft&&g!=ft){nw[0]++;if(g)nw[g]++}}"
    "if(lg.length>500)lg.splice(0,lg.length-500);"
    "if(d.log.length||d.nx<q)show();q=d.nx;"
    "$('s').textContent=d.st;$('wf').textContent=d.wf;"
    // Uptime going back to zero while powered = the Pico restarted.
    "if(up>d.up)rs++;up=d.up;$('rr').textContent=d.rr;"
    // Battery: red below 3.5 V (low or sagging under load).
    "var vb=$('vb');vb.textContent=d.vb?(d.vb/1000).toFixed(2)+' V, "
    "lowest '+(d.vm/1000).toFixed(2)+' V since power-up':'not measured';"
    "vb.style.color=d.vb&&d.vm<3500?'#f66':'';"
    "$('up').textContent=Math.floor(d.up/60)+' min '+(d.up%60)+' s'+"
    "(rs?' (restarted '+rs+'x since this page opened!)':'');"
    "mv=['READY','STOPPED','BARCODE_STATIC'].indexOf(d.st)<0;lock();"
    "$('r').textContent=d.rg<0?'none':(d.rg/10).toFixed(1);"
    "$('i').textContent=d.l+' / '+d.r+' / '+d.b;"
    "$('m').textContent=d.ml+' / '+d.mr;"
    "$('x').textContent=(d.lh?'HIGH':'LOW')+' / '+(d.rh?'HIGH':'LOW');"
    "$('en').textContent=d.el+' / '+d.er;$('ep').textContent=d.ep;"
    "$('bc').textContent=d.bc||'-';$('tn').textContent=d.tn||'straight on';"
    "if(d.hq!=hq)hget();"
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
    "ok=Date.now();setTimeout(u,d.more?50:200)})"
    ".catch(function(){setTimeout(u,1000)})}u();</script>"
    "</body></html>";

static web_command_t g_pending = WEB_CMD_NONE;
static bool          gb_ready  = false;
static char const *  gp_restart_reason = "unknown";  // Why the Pico started.
static uint8_t       g_answered = 0u;           // Its address marks a
                                                // connection as answered.
static struct raw_pcb * gp_ping   = NULL;       // Pings to the hotspot.
static uint16_t      g_ping_seq   = 0u;
static uint32_t      g_ping_next  = 0u;         // Next ping due (ms).
static uint32_t      g_ping_reply = 0u;         // Last reply (ms).
static uint32_t      g_ping_rtt   = 0u;         // Its round trip (ms).
static uint32_t      g_ping_sent  = 0u;         // When the last was sent.
static bool          gb_ping_answers = false;   // Hotspot answers pings.
static int32_t       g_rssi       = 0;          // Signal (dBm), 0 = unknown.
static uint32_t      g_drops      = 0u;         // Hotspot links lost.
static bool          gb_servo_pending = false;
static uint32_t      g_log_floor      = 0u;     // CLEAR OUTPUT: first log
                                                // line still sent.
static int32_t       g_servo_deg      = 0;
static wifi_mode_t   g_wifi_mode      = WIFI_OFF;
static dhcp_server_t g_dhcp_server;             // Own network only.
#ifdef HOTSPOT_LIST
static hotspot_t const g_hotspots[] = { HOTSPOT_LIST };
static uint32_t      g_hot_idx        = 0u;     // Hotspot being tried/used.
static int32_t       g_hot_only       = HOTSPOT_AUTO; // Chosen on the page.
static int32_t       g_hot_request    = HOTSPOT_NO_REQUEST; // New choice.
static uint32_t      g_hot_since      = 0u;     // This attempt began (ms).
static uint32_t      g_hot_tries      = 0u;     // Attempts that failed.
static uint32_t      g_join_since     = 0u;     // Last "still looking" log.
static bool          gb_mdns_started  = false;
#endif
static uint32_t      g_wifi_check     = 0u;     // Next link check (ms).
static char          g_wifi_text[WIFI_TEXT_BYTES] = "Starting";
static history_entry_t g_history[HISTORY_MAX];  // Ring, oldest overwritten.
static uint32_t      g_history_count  = 0u;     // Ever recorded (since CLEAR).
static uint32_t      g_history_seq    = 0u;     // Changes on every record or
                                                // clear: the page refetches.


static bool   server_start(void);
static void   own_network_start(void);
#ifdef HOTSPOT_LIST
static void   hotspot_start(uint32_t now_msec);
static void   hotspot_try(uint32_t now_msec);
static void   hotspot_next(uint32_t now_msec, int status);
static bool   hotspot_first_round(void);
#endif
static size_t hotspots_build(char * p_out, size_t capacity);
static void   hotspot_check(uint32_t now_msec);
static err_t  connection_close(struct tcp_pcb * p_pcb);
static err_t  sent_callback(void * p_arg, struct tcp_pcb * p_pcb, u16_t len);
static err_t  poll_callback(void * p_arg, struct tcp_pcb * p_pcb);
static void   ping_send(uint32_t now_msec);
static u8_t   ping_recv(void * p_arg, struct raw_pcb * p_pcb,
                        struct pbuf * p_buf, ip_addr_t const * p_addr);
static err_t  accept_callback(void * p_arg, struct tcp_pcb * p_client,
                              err_t err);
static err_t  receive_callback(void * p_arg, struct tcp_pcb * p_pcb,
                               struct pbuf * p_buf, err_t err);
static void   request_handle(struct tcp_pcb * p_pcb, char * p_path);
static void   command_parse(char const * p_name);
static size_t status_build(char * p_out, size_t capacity, uint32_t since);
static size_t history_build(char * p_out, size_t capacity);
static size_t json_string(char * p_out, size_t capacity, char const * p_text);
static void   reply_send(struct tcp_pcb * p_pcb, char const * p_type,
                         char const * p_body, size_t body_len,
                         bool b_static);


/*!
 * @brief Start Wi-Fi and the web server.
 *
 * @par
 * With hotspots listed, joining starts here and finishes in web_task(),
 * so power-up is never held up; otherwise the car's own network starts
 * now.
 *
 * @return True if the wireless UI is running (or joining).
 */
bool
web_init (void)
{
    gb_ready    = false;
    g_wifi_mode = WIFI_OFF;

    if (0 != cyw43_arch_init())
    {
        log_printf("Wi-Fi: chip failed to start; wireless UI is OFF.");
        (void) snprintf(g_wifi_text, sizeof(g_wifi_text), "Off");
    }
    else if (server_start())
    {
        gb_ready = true;
#ifdef HOTSPOT_LIST
        hotspot_start(to_ms_since_boot(get_absolute_time()));
#else
        own_network_start();
#endif
    }
    else
    {
        log_printf("Wi-Fi: web server failed to start.");
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
        hotspot_check(to_ms_since_boot(get_absolute_time()));
    }
}   /* web_task() */


/*!
 * @brief Report how the wireless UI can be reached and how well the link
 *        is doing, for the page.
 *
 * @return Text such as "Hotspot \"name\" at 172.20.10.3, signal -58 dBm,
 *         ping 14 ms, 0 drops".
 */
char const *
web_wifi_text (void)
{
    static char text[WIFI_TEXT_BYTES * 2u];
    uint32_t    now_msec = to_ms_since_boot(get_absolute_time());

    if (WIFI_JOINED != g_wifi_mode)
    {
        (void) snprintf(text, sizeof(text), "%s (%" PRIu32 " drops)",
                        g_wifi_text, g_drops);
    }
    else if (!gb_ping_answers)
    {
        (void) snprintf(text, sizeof(text), "%s, signal %" PRId32 " dBm, "
                        "hotspot not answering pings, %" PRIu32 " drops",
                        g_wifi_text, g_rssi, g_drops);
    }
    else
    {
        (void) snprintf(text, sizeof(text), "%s, signal %" PRId32 " dBm, "
                        "ping %" PRIu32 " ms (%" PRIu32 " s ago), %" PRIu32
                        " drops", g_wifi_text, g_rssi, g_ping_rtt,
                        (uint32_t) ((now_msec - g_ping_reply) /
                                    MSEC_PER_SEC), g_drops);
    }

    return (text);
}   /* web_wifi_text() */


/*!
 * @brief Record why the Pico last started, for the page.
 *
 * @param[in] p_reason A description that stays valid (a string literal).
 */
void
web_set_restart_reason (char const * p_reason)
{
    gp_restart_reason = p_reason;
}   /* web_set_restart_reason() */


/*!
 * @brief Ping the hotspot (the gateway), to check the link really works.
 *
 * @param[in] now_msec The time now, in ms since boot.
 */
static void
ping_send (uint32_t now_msec)
{
    struct netif * p_netif = &cyw43_state.netif[CYW43_ITF_STA];
    size_t         size    = sizeof(struct icmp_echo_hdr) + PING_DATA_BYTES;
    struct pbuf *  p_buf   = NULL;

    if (NULL == gp_ping)
    {
        gp_ping = raw_new(IP_PROTO_ICMP);
        if (NULL != gp_ping)
        {
            raw_recv(gp_ping, ping_recv, NULL);
            (void) raw_bind(gp_ping, IP_ADDR_ANY);
        }
    }

    // NOTE: size is 16 bytes, so it fits u16_t.
    p_buf = pbuf_alloc(PBUF_IP, (u16_t) size, PBUF_RAM);

    if ((NULL != gp_ping) && (NULL != p_buf) && (p_buf->len == p_buf->tot_len))
    {
        struct icmp_echo_hdr * p_echo = (struct icmp_echo_hdr *) p_buf->payload;
        ip_addr_t              gateway;

        ip_addr_copy_from_ip4(gateway, *netif_ip4_gw(p_netif));
        g_ping_seq++;
        ICMPH_TYPE_SET(p_echo, ICMP_ECHO);
        ICMPH_CODE_SET(p_echo, 0u);
        p_echo->id     = PING_ID;
        p_echo->seqno  = lwip_htons(g_ping_seq);
        (void) memset(((uint8_t *) p_echo) + sizeof(*p_echo), 0,
                      PING_DATA_BYTES);
        p_echo->chksum = 0u;
        p_echo->chksum = inet_chksum(p_echo, (u16_t) size);

        g_ping_sent = now_msec;
        (void) raw_sendto(gp_ping, p_buf, &gateway);
    }

    if (NULL != p_buf)
    {
        (void) pbuf_free(p_buf);
    }
}   /* ping_send() */


/*!
 * @brief lwIP callback for ICMP arriving: note replies to our pings.
 *
 * @param[in] p_arg  Unused.
 * @param[in] p_pcb  Unused.
 * @param[in] p_buf  The packet, IP header first.
 * @param[in] p_addr Unused.
 *
 * @return 1 if it was a reply to our ping (eaten), 0 to pass it on.
 */
static u8_t
ping_recv (void * p_arg, struct raw_pcb * p_pcb, struct pbuf * p_buf,
           ip_addr_t const * p_addr)
{
    u8_t b_eaten = 0u;

    (void) p_arg;   // All required by lwIP's callback signature only.
    (void) p_pcb;
    (void) p_addr;

    if ((p_buf->tot_len >= (PBUF_IP_HLEN + sizeof(struct icmp_echo_hdr))) &&
        (0u == pbuf_remove_header(p_buf, PBUF_IP_HLEN)))
    {
        struct icmp_echo_hdr const * p_echo = p_buf->payload;

        if ((ICMP_ER == p_echo->type) && (PING_ID == p_echo->id))
        {
            uint32_t now_msec = to_ms_since_boot(get_absolute_time());

            g_ping_reply    = now_msec;
            g_ping_rtt      = now_msec - g_ping_sent;
            gb_ping_answers = true;
            (void) pbuf_free(p_buf);
            b_eaten = 1u;
        }
        else
        {
            (void) pbuf_add_header(p_buf, PBUF_IP_HLEN);    // Not ours.
        }
    }

    return (b_eaten);
}   /* ping_recv() */


/*!
 * @brief Add a decoded barcode to the page's barcode history.
 *
 * @param[in] p_message The decoded message.
 * @param[in] command   The command it gives (BARCODE_CMD_NONE if none).
 */
void
web_barcode_record (char const * p_message, barcode_command_t command)
{
    history_entry_t * p_entry = &g_history[g_history_count % HISTORY_MAX];

    (void) snprintf(p_entry->text, sizeof(p_entry->text), "%s", p_message);
    p_entry->command  = command;
    p_entry->time_sec = to_ms_since_boot(get_absolute_time()) / MSEC_PER_SEC;
    g_history_count++;
    g_history_seq++;
}   /* web_barcode_record() */


/*!
 * @brief Listen for HTTP on every network (hotspot or own).
 *
 * @return True if listening.
 */
static bool
server_start (void)
{
    struct tcp_pcb * p_pcb      = tcp_new_ip_type(IPADDR_TYPE_ANY);
    struct tcp_pcb * p_listener = NULL;

    if ((NULL != p_pcb) &&
        (ERR_OK == tcp_bind(p_pcb, IP_ANY_TYPE, HTTP_PORT)))
    {
        p_listener = tcp_listen_with_backlog(p_pcb, LISTEN_BACKLOG);
    }

    if (NULL != p_listener)
    {
        tcp_accept(p_listener, accept_callback);
    }

    return (NULL != p_listener);
}   /* server_start() */


/*!
 * @brief Make the car's own network, handing out 192.168.4.x addresses
 *        like a home router.
 */
static void
own_network_start (void)
{
    ip_addr_t gateway;
    ip_addr_t netmask;

    cyw43_arch_enable_ap_mode(WIFI_NAME, WIFI_PASSWORD,
                              CYW43_AUTH_WPA2_AES_PSK);

    ip_addr_set_ip4_u32(&gateway, PP_HTONL(CYW43_DEFAULT_IP_AP_ADDRESS));
    ip_addr_set_ip4_u32(&netmask, PP_HTONL(CYW43_DEFAULT_IP_MASK));
    dhcp_server_init(&g_dhcp_server, &cyw43_state.netif[CYW43_ITF_AP],
                     &gateway, &netmask);

    g_wifi_mode = WIFI_OWN_NETWORK;
    (void) snprintf(g_wifi_text, sizeof(g_wifi_text),
                    "Own network \"%s\" at 192.168.4.1", WIFI_NAME);
    log_printf("Wi-Fi: join \"%s\" (password \"%s\"), then open "
               "http://192.168.4.1", WIFI_NAME, WIFI_PASSWORD);
}   /* own_network_start() */


#ifdef HOTSPOT_LIST
/*!
 * @brief Start joining hotspots: the one chosen on the page, or else each
 *        in wifi_secrets.h in turn.
 *
 * @par
 * mDNS answers WIFI_HOST_NAME on whichever is joined, so the page is at
 * http://WIFI_HOST_NAME.local whatever IP address the hotspot gives.
 *
 * @param[in] now_msec The time now, in ms since boot.
 */
static void
hotspot_start (uint32_t now_msec)
{
    cyw43_arch_enable_sta_mode();

    // No Wi-Fi power saving: the page polls every 0.2 s and must not lag.
    (void) cyw43_wifi_pm(&cyw43_state, CYW43_NONE_PM);

    if (!gb_mdns_started)
    {
        mdns_resp_init();               // Once only; netifs come and go.
        gb_mdns_started = true;
    }
    if (ERR_OK != mdns_resp_add_netif(&cyw43_state.netif[CYW43_ITF_STA],
                                      WIFI_HOST_NAME))
    {
        log_printf("Wi-Fi: mDNS failed; use the IP address instead.");
    }

    g_wifi_mode  = WIFI_JOINING;
    g_hot_tries  = 0u;
    g_join_since = now_msec;
    g_hot_idx   = ((HOTSPOT_AUTO == g_hot_only) ? 0u : (uint32_t) g_hot_only);
    hotspot_try(now_msec);
}   /* hotspot_start() */


/*!
 * @brief Start one attempt at joining hotspot g_hot_idx.
 *
 * @param[in] now_msec The time now, in ms since boot.
 */
static void
hotspot_try (uint32_t now_msec)
{
    hotspot_t const * p_hot = &g_hotspots[g_hot_idx];

    // End a join still in progress (or a link): a failed one is over.
    if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) >
        CYW43_LINK_DOWN)
    {
        (void) cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
    }
    g_hot_since = now_msec;

    (void) snprintf(g_wifi_text, sizeof(g_wifi_text), "%s \"%s\"",
                    ((WIFI_REJOINING == g_wifi_mode) ? "Rejoining"
                                                     : "Joining"),
                    p_hot->p_ssid);
    if (hotspot_first_round())
    {
        log_printf("Wi-Fi: trying hotspot %" PRIu32 " of %" PRIu32
                   ": \"%s\"...", (g_hot_idx + 1u), NUM_HOTSPOTS,
                   p_hot->p_ssid);
    }

    if (0 != cyw43_arch_wifi_connect_async(p_hot->p_ssid, p_hot->p_password,
                                           HOTSPOT_AUTH))
    {
        log_printf("Wi-Fi: could not start joining \"%s\".", p_hot->p_ssid);
    }
}   /* hotspot_try() */


/*!
 * @brief The current attempt failed: try the next hotspot. The car never
 *        gives up, so it joins whichever hotspot turns up, whenever.
 *
 * @par
 * Each failure is logged on the first time round the list only; after
 * that a "still looking" line every WIFI_STILL_LOG_MSEC, so the log is
 * not flooded.
 *
 * @param[in] now_msec The time now, in ms since boot.
 * @param[in] status   The link status the attempt ended with.
 */
static void
hotspot_next (uint32_t now_msec, int status)
{
    if (hotspot_first_round())
    {
        log_printf("Wi-Fi: \"%s\" not joined (%s).",
                   g_hotspots[g_hot_idx].p_ssid,
                   ((CYW43_LINK_BADAUTH == status) ? "wrong password" :
                    ((CYW43_LINK_NONET == status) ? "not found" :
                     "no answer")));
    }
    else if ((now_msec - g_join_since) >= WIFI_STILL_LOG_MSEC)
    {
        g_join_since = now_msec;
        log_printf("Wi-Fi: still looking for a hotspot (%" PRIu32
                   " tries so far)...", g_hot_tries);
    }
    else
    {
        // Quiet: already reported.
    }

    g_hot_tries++;

    if (HOTSPOT_AUTO == g_hot_only)
    {
        g_hot_idx = ((g_hot_idx + 1u) % NUM_HOTSPOTS);
    }
    hotspot_try(now_msec);
}   /* hotspot_next() */


/*!
 * @brief Report whether the car is still on its first time round the
 *        hotspots to try (when every attempt is logged).
 *
 * @return True on the first time round.
 */
static bool
hotspot_first_round (void)
{
    uint32_t candidates = ((HOTSPOT_AUTO == g_hot_only) ? NUM_HOTSPOTS : 1u);

    return (g_hot_tries < candidates);
}   /* hotspot_first_round() */
#endif /* HOTSPOT_LIST */


/*!
 * @brief Follow the hotspot link: act on a hotspot chosen on the page,
 *        report when one is joined, move on to the next when an attempt
 *        fails, and rejoin when the link drops.
 *
 * @par
 * Each hotspot is tried for up to WIFI_TRY_MSEC in turn, round and round,
 * for as long as it takes: at power-up, after a choice on the page, and
 * when the link drops. So the car joins by itself whenever a hotspot is
 * switched on, and never makes a network of its own.
 *
 * @param[in] now_msec The time now, in ms since boot.
 */
static void
hotspot_check (uint32_t now_msec)
{
#ifdef HOTSPOT_LIST
    bool b_hotspot = ((WIFI_JOINING == g_wifi_mode) ||
                      (WIFI_JOINED == g_wifi_mode) ||
                      (WIFI_REJOINING == g_wifi_mode));

    if (HOTSPOT_NO_REQUEST != g_hot_request)
    {
        // Chosen on the page (done here, not inside the web request,
        // because it can take down the network that request came on).
        g_hot_only    = g_hot_request;
        g_hot_request = HOTSPOT_NO_REQUEST;
        log_printf("Wi-Fi: hotspot choice: %s.",
                   ((HOTSPOT_AUTO == g_hot_only) ? "try each in turn" :
                    g_hotspots[g_hot_only].p_ssid));

        if (WIFI_OWN_NETWORK == g_wifi_mode)
        {
            dhcp_server_deinit(&g_dhcp_server);
            cyw43_arch_disable_ap_mode();
        }
        if (WIFI_OFF != g_wifi_mode)
        {
            hotspot_start(now_msec);
        }
    }
    else if (b_hotspot && ((int32_t) (now_msec - g_wifi_check) >= 0))
    {
        int      status  = cyw43_tcpip_link_status(&cyw43_state,
                                                   CYW43_ITF_STA);
        uint32_t elapsed = now_msec - g_hot_since;

        g_wifi_check = now_msec + WIFI_CHECK_MSEC;

        if ((CYW43_LINK_UP == status) && (WIFI_JOINED != g_wifi_mode))
        {
            char const * p_ip = ip4addr_ntoa(netif_ip4_addr(
                                    &cyw43_state.netif[CYW43_ITF_STA]));

            g_wifi_mode     = WIFI_JOINED;
            gb_ping_answers = false;        // Learnt afresh on each link.
            g_ping_reply    = now_msec;
            g_ping_next     = now_msec;
            (void) snprintf(g_wifi_text, sizeof(g_wifi_text),
                            "Hotspot \"%s\" at %s",
                            g_hotspots[g_hot_idx].p_ssid, p_ip);
            log_printf("Wi-Fi: on \"%s\". Open http://%s.local or "
                       "http://%s", g_hotspots[g_hot_idx].p_ssid,
                       WIFI_HOST_NAME, p_ip);
        }
        else if ((CYW43_LINK_UP == status) && gb_ping_answers &&
                 ((now_msec - g_ping_reply) >= WIFI_PING_LOST_MSEC))
        {
            // The chip still says "up", but the hotspot has stopped
            // answering: the link is dead in practice, so rejoin.
            log_printf("Wi-Fi: no answer from \"%s\" for %" PRIu32
                       " s; rejoining.", g_hotspots[g_hot_idx].p_ssid,
                       (uint32_t) ((now_msec - g_ping_reply) /
                                   MSEC_PER_SEC));
            g_drops++;
            g_wifi_mode  = WIFI_REJOINING;
            g_hot_tries  = 0u;
            g_join_since = now_msec;
            hotspot_try(now_msec);
        }
        else if (CYW43_LINK_UP == status)
        {
            // Joined: keep pinging the hotspot, which also keeps the link
            // busy so a phone does not treat the car as idle.
            if ((int32_t) (now_msec - g_ping_next) >= 0)
            {
                g_ping_next = now_msec + WIFI_PING_MSEC;
                ping_send(now_msec);
                (void) cyw43_wifi_get_rssi(&cyw43_state, &g_rssi);
            }
        }
        else if (WIFI_JOINED == g_wifi_mode)
        {
            log_printf("Wi-Fi: lost \"%s\" (status %d); rejoining.",
                       g_hotspots[g_hot_idx].p_ssid, status);
            g_drops++;
            g_wifi_mode  = WIFI_REJOINING;
            g_hot_tries  = 0u;              // Log the first round again.
            g_join_since = now_msec;
            hotspot_try(now_msec);          // The same one first.
        }
        else if (((status < CYW43_LINK_DOWN) &&
                  (elapsed >= WIFI_FAIL_SETTLE_MSEC)) ||
                 ((CYW43_LINK_DOWN == status) &&
                  (elapsed >= WIFI_TRY_MSEC)) ||
                 (elapsed >= WIFI_GET_IP_MSEC))
        {
            // Failed, or nothing for too long. Once on the hotspot (JOIN
            // or NOIP: waiting for an address) it gets far longer, since
            // a phone can be slow to hand out an address.
            hotspot_next(now_msec, status);
        }
        else
        {
            // Still joining: wait.
        }
    }
    else
    {
        // Nothing to do this tick.
    }
#else
    (void) now_msec;
#endif
}   /* hotspot_check() */


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
        tcp_arg(p_client, NULL);                // Not answered yet.
        tcp_recv(p_client, receive_callback);
        tcp_sent(p_client, sent_callback);
        tcp_poll(p_client, poll_callback, HTTP_POLL_INTERVAL);
    }

    return (result);
}   /* accept_callback() */


/*!
 * @brief Close a connection, or abort it if it cannot be closed now.
 *
 * @param[in] p_pcb The connection.
 *
 * @return ERR_ABRT if it was aborted (lwIP must be told), otherwise ERR_OK.
 */
static err_t
connection_close (struct tcp_pcb * p_pcb)
{
    err_t result = ERR_OK;

    tcp_arg(p_pcb, NULL);
    tcp_recv(p_pcb, NULL);
    tcp_sent(p_pcb, NULL);
    tcp_poll(p_pcb, NULL, 0u);

    if (ERR_OK != tcp_close(p_pcb))
    {
        tcp_abort(p_pcb);
        result = ERR_ABRT;
    }

    return (result);
}   /* connection_close() */


/*!
 * @brief lwIP callback when sent data is acknowledged: once the whole
 *        reply has arrived, close the connection.
 *
 * @par
 * Closing only now (not straight after queueing the reply) means the rest
 * of the browser's request can still arrive: lwIP answers data reaching a
 * closed connection with a reset, which can destroy the reply in flight.
 * Over a phone hotspot, which relays every packet, that happens often.
 *
 * @param[in] p_arg Non-NULL once the reply has been queued.
 * @param[in] p_pcb The connection.
 * @param[in] len   Unused.
 *
 * @return ERR_ABRT if the connection was aborted, otherwise ERR_OK.
 */
static err_t
sent_callback (void * p_arg, struct tcp_pcb * p_pcb, u16_t len)
{
    err_t result = ERR_OK;

    (void) len;     // Required by lwIP's callback signature only.

    if ((NULL != p_arg) && (0u == tcp_sndqueuelen(p_pcb)))
    {
        result = connection_close(p_pcb);
    }

    return (result);
}   /* sent_callback() */


/*!
 * @brief lwIP callback every HTTP_POLL_INTERVAL: a connection still open
 *        then is stuck (the browser went away), so free it.
 *
 * @param[in] p_arg Unused.
 * @param[in] p_pcb The connection.
 *
 * @return ERR_ABRT (the connection is always aborted).
 */
static err_t
poll_callback (void * p_arg, struct tcp_pcb * p_pcb)
{
    (void) p_arg;   // Required by lwIP's callback signature only.

    tcp_arg(p_pcb, NULL);
    tcp_abort(p_pcb);

    return (ERR_ABRT);
}   /* poll_callback() */


/*!
 * @brief lwIP callback for received data: one request per connection.
 *
 * @par
 * The first piece holds the request line and is answered; anything after
 * it (the rest of the headers) is read and dropped. The connection is
 * closed by sent_callback() once the reply has been acknowledged, or here
 * if the browser closes first.
 *
 * @param[in] p_arg Non-NULL once this connection has been answered.
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

    (void) err;     // Required by lwIP's callback signature only.

    if (NULL == p_buf)
    {
        result = connection_close(p_pcb);
    }
    else if (NULL != p_arg)
    {
        // More of a request already answered: read it and drop it.
        tcp_recved(p_pcb, p_buf->tot_len);
        pbuf_free(p_buf);
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

        // Answered (or not a GET): close once the reply has been taken.
        tcp_arg(p_pcb, &g_answered);
        if (0u == tcp_sndqueuelen(p_pcb))
        {
            result = connection_close(p_pcb);   // Nothing was sent.
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
    else if (0 == strcmp(p_path, HISTORY_PATH))
    {
        size_t length = history_build(status, sizeof(status));

        reply_send(p_pcb, "application/json", status, length, false);
    }
    else if (0 == strcmp(p_path, HOTSPOTS_PATH))
    {
        size_t length = hotspots_build(status, sizeof(status));

        reply_send(p_pcb, "application/json", status, length, false);
    }
    else if (0 == strncmp(p_path, WIFI_PREFIX, WIFI_PREFIX_LEN))
    {
#ifdef HOTSPOT_LIST
        // -1 is "try each"; acted on by hotspot_check(), after this reply.
        int32_t choice = (int32_t) atoi(p_path + WIFI_PREFIX_LEN);

        if ((HOTSPOT_AUTO <= choice) && (choice < (int32_t) NUM_HOTSPOTS))
        {
            g_hot_request = choice;
        }
#endif
        reply_send(p_pcb, "text/plain", "OK", 2u, true);
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
 * @par
 * "clear" (CLEAR OUTPUT) is done here at once: it only clears the shown
 * barcode message and the log lines sent to the page, so it never needs
 * main().
 *
 * @param[in] p_name "start", "stop", "cal", "wheel", "diag", "black",
 *                   "white", "bt" (barcode test) or "clear".
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
    else if (0 == strcmp(p_name, "bs"))
    {
        g_pending = WEB_CMD_BARCODE_STATIC;
    }
    else if (0 == strcmp(p_name, "clear"))
    {
        barcode_clear_message();
        g_history_count = 0u;           // And the barcode history.
        g_history_seq++;
        g_log_floor = log_next_seq();   // Hide every line logged so far.
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
    barcode_command_t pending = vehicle_pending_turn();
    imu_axes_t   axes;
    size_t       length   = 0u;
    char const * p_line   = NULL;
    log_cat_t    cat      = LOG_CAT_SYSTEM;
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
    if (seq < g_log_floor)
    {
        seq = g_log_floor;      // Cleared: never send those lines again.
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
                                "\"lh\":%d,\"rh\":%d,"
                                "\"el\":%" PRIu32 ",\"er\":%" PRIu32 ","
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
                                (raw.b_left_high ? 1 : 0),
                                (raw.b_right_high ? 1 : 0),
                                encoder_count(PIN_ENC_LEFT_A),
                                encoder_count(PIN_ENC_RIGHT_A),
                                axes.ax_mg, axes.ay_mg, axes.az_mg,
                                axes.mx_mgauss, axes.my_mgauss,
                                axes.mz_mgauss);
    length += json_string(p_out + length, capacity - length,
                          barcode_message());

    // Pin finder: edges seen on each watched pin, as "GP4:12 GP5:0 ...".
    // NOTE: At most 10 pins of under 18 characters each: well within the
    // buffer's spare room, so snprintf never truncates.
    length += (size_t) snprintf(p_out + length, capacity - length,
                                ",\"ep\":\"");
    for (uint32_t idx = 0u; idx < encoder_watch_count(); idx++)
    {
        length += (size_t) snprintf(p_out + length, capacity - length,
                                    "GP%" PRIu32 ":%" PRIu32 " ",
                                    encoder_watch_pin(idx),
                                    encoder_watch_edges(idx));
    }
    length += (size_t) snprintf(p_out + length, capacity - length, "\"");

    length += (size_t) snprintf(p_out + length, capacity - length,
                                ",\"vb\":%" PRIu32 ",\"vm\":%" PRIu32
                                ",\"up\":%" PRIu32 ",\"rr\":",
                                power_vsys_mv(), power_vsys_min_mv(),
                                (to_ms_since_boot(get_absolute_time()) /
                                 MSEC_PER_SEC));
    length += json_string(p_out + length, capacity - length,
                          gp_restart_reason);
    length += (size_t) snprintf(p_out + length, capacity - length,
                                ",\"wf\":");
    length += json_string(p_out + length, capacity - length,
                          web_wifi_text());

    // Each log line goes as its category digit (log_cat_t), then its text,
    // so the page can sort it into a serial monitor tab.
    length += (size_t) snprintf(p_out + length, capacity - length,
                                ",\"hq\":%" PRIu32 ",\"tn\":\"%s\",\"log\":[",
                                g_history_seq,
                                ((BARCODE_CMD_NONE == pending) ? "" :
                                 barcode_command_name(pending)));

    while ((seq < next_seq) && ((capacity - length) > LINE_RESERVE) &&
           log_get(seq, &p_line, &cat))
    {
        size_t text_len = 0u;

        if (!b_first)
        {
            p_out[length] = ',';
            length++;
        }
        text_len = json_string(p_out + length + 1u,
                               capacity - length - 1u, p_line);
        p_out[length]      = '"';                     // Opening quote...
        p_out[length + 1u] = (char) ('0' + (int) cat); // ...then category.
        length += text_len + 1u;
        b_first = false;
        seq++;
    }

    length += (size_t) snprintf(p_out + length, capacity - length,
                                "],\"nx\":%" PRIu32 ",\"more\":%d}", seq,
                                ((seq < next_seq) ? 1 : 0));

    return (length);
}   /* status_build() */


/*!
 * @brief Build the hotspot list for the page's chooser, as
 *        {"n":["name",...],"sel":-1} (sel: the choice, -1 = try each).
 *
 * @par
 * Passwords are never sent.
 *
 * @param[out] p_out    The buffer to fill.
 * @param[in]  capacity The buffer size.
 *
 * @return The number of bytes written.
 */
static size_t
hotspots_build (char * p_out, size_t capacity)
{
    size_t length = 0u;

    // NOTE: A handful of names of under 33 bytes each: well within the
    // 1400-byte buffer, so these snprintf calls never truncate.
    length += (size_t) snprintf(p_out + length, capacity - length, "{\"n\":[");

#ifdef HOTSPOT_LIST
    for (uint32_t idx = 0u; idx < NUM_HOTSPOTS; idx++)
    {
        if (idx > 0u)
        {
            p_out[length] = ',';
            length++;
        }
        length += json_string(p_out + length, capacity - length,
                              g_hotspots[idx].p_ssid);
    }
    length += (size_t) snprintf(p_out + length, capacity - length,
                                "],\"sel\":%" PRId32 "}", g_hot_only);
#else
    length += (size_t) snprintf(p_out + length, capacity - length,
                                "],\"sel\":-1}");
#endif

    return (length);
}   /* hotspots_build() */


/*!
 * @brief Build the barcode history, newest first, as
 *        {"hq":seq,"h":[["A","TURN LEFT",12],...]} (text, command, second).
 *
 * @param[out] p_out    The buffer to fill.
 * @param[in]  capacity The buffer size.
 *
 * @return The number of bytes written.
 */
static size_t
history_build (char * p_out, size_t capacity)
{
    uint32_t kept   = ((g_history_count < HISTORY_MAX) ? g_history_count
                                                       : HISTORY_MAX);
    size_t   length = 0u;

    // NOTE: At most 20 entries of under 60 bytes each: well within the
    // 1400-byte buffer, so these snprintf calls never truncate.
    length += (size_t) snprintf(p_out + length, capacity - length,
                                "{\"hq\":%" PRIu32 ",\"h\":[", g_history_seq);

    for (uint32_t idx = 0u; idx < kept; idx++)
    {
        history_entry_t const * p_entry =
            &g_history[(g_history_count - 1u - idx) % HISTORY_MAX];

        length += (size_t) snprintf(p_out + length, capacity - length,
                                    "%s[", ((0u == idx) ? "" : ","));
        length += json_string(p_out + length, capacity - length,
                              p_entry->text);
        length += (size_t) snprintf(p_out + length, capacity - length,
                                    ",\"%s\",%" PRIu32 "]",
                                    ((BARCODE_CMD_NONE == p_entry->command) ?
                                     "" : barcode_command_name(
                                              p_entry->command)),
                                    p_entry->time_sec);
    }

    length += (size_t) snprintf(p_out + length, capacity - length, "]}");

    return (length);
}   /* history_build() */


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
        // NOTE: Compared unsigned, so UTF-8 bytes (0x80 up) are kept.
        p_out[length] = ((((unsigned char) symbol) < ' ') ? ' ' : symbol);
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

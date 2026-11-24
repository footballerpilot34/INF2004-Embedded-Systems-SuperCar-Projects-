"""Run web.c's actual connection state machine and JSON builders with SDK mocks.

Only platform calls are mocked. Function bodies are extracted verbatim so
network hardware is not required and the production code needs no test hooks.
"""
from pathlib import Path
import ast
import json
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "src/web.c").read_text()


def function(name):
    start = re.search(r"^static (?:void|bool|size_t|err_t)\n" + name + r"\s*\(", SOURCE, re.M)
    assert start, name
    end = re.search(r"^}", SOURCE[start.start():], re.M)
    assert end
    return SOURCE[start.start(): start.start() + end.end()] + "\n"


def block(start, end):
    return SOURCE[SOURCE.index(start): SOURCE.index(end)]


COMMON = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "config.h"
#include "web.h"
#include "encoder.h"
#include "imu.h"
#include "line_sensor.h"
#include "log.h"
#include "motor.h"
#include "power.h"
#include "obstacle.h"
#include "servo.h"
#include "ultrasonic.h"
#include "vehicle.h"
#undef HOTSPOT_LIST
#define HOTSPOT_LIST {"test-one", "test-pass"}, {"test-two", "test-pass"}
typedef int err_t;
typedef uint16_t u16_t;
typedef uint8_t u8_t;
typedef uint32_t ip4_addr_t;
typedef uint32_t ip_addr_t;
typedef int dhcp_server_t;
struct netif { bool up, link, lease; ip4_addr_t ip; };
static struct { struct netif netif[2]; } cyw43_state;
#define ERR_OK 0
#define ERR_MEM -1
#define ERR_RTE -4
#define CYW43_ITF_STA 0
#define CYW43_LINK_DOWN 0
#define CYW43_LINK_JOIN 1
#define CYW43_LINK_NOIP 2
#define CYW43_LINK_UP 3
#define CYW43_LINK_FAIL -1
#define CYW43_LINK_NONET -2
#define CYW43_LINK_BADAUTH -3
#define CYW43_NONE_PM 0
#define CYW43_DEFAULT_PM 1
#define CYW43_AUTH_WPA2_AES_PSK 1
#define IP4_ADDR_ANY4 NULL
#define IP4ADDR_STRLEN_MAX 16
static uint32_t now;
static uint32_t get_absolute_time(void) { return now; }
static uint32_t to_ms_since_boot(uint32_t t) { return t; }
void log_printf(char const * fmt, ...) { (void)fmt; }
'''

DEFINITIONS = block("#define HTTP_PORT", "    \"<!DOCTYPE html>")
# Omit the HTML declaration; tested separately below.
DEFINITIONS = DEFINITIONS[:DEFINITIONS.index("static char const g_page_html")]
STATE = SOURCE[SOURCE.index("static web_command_t g_pending"):re.search(r"static void\s+server_check", SOURCE).start()]

NETWORK = r'''
static unsigned joins, leaves, resets, recovery, ap_stops, mdns_removes;
static int driver_status;
static bool netif_is_up(struct netif * n) { return n->up; }
static bool netif_is_link_up(struct netif * n) { return n->link; }
static bool dhcp_supplied_address(struct netif * n) { return n->lease; }
static int cyw43_tcpip_link_status(void * state, int itf) {
    (void)state; (void)itf;
    struct netif * n = &cyw43_state.netif[0];
    return n->up && n->link ? (n->ip ? CYW43_LINK_UP : CYW43_LINK_NOIP) : driver_status;
}
static void cyw43_arch_enable_sta_mode(void) { cyw43_state.netif[0].up = true; }
static void cyw43_arch_disable_sta_mode(void) {
    ++resets; memset(&cyw43_state.netif[0], 0, sizeof(struct netif)); driver_status = 0;
}
static int cyw43_wifi_pm(void * s, int pm) { (void)s; (void)pm; return 0; }
static int cyw43_wifi_leave(void * s, int itf) { (void)s; (void)itf; ++leaves; return 0; }
static void dhcp_stop(struct netif * n) { n->lease = false; }
static void netif_set_addr(struct netif * n, void * ip, void * mask, void * gw) {
    (void)ip; (void)mask; (void)gw; n->ip = 0;
}
static err_t dhcp_start(struct netif * n) { assert(n->up); return ERR_OK; }
static int cyw43_arch_wifi_connect_async(char const * ssid, char const * pw, int auth) {
    (void)ssid; (void)pw; (void)auth; ++joins; driver_status = CYW43_LINK_JOIN; return 0;
}
static void mdns_resp_init(void) {}
static err_t mdns_resp_add_netif(struct netif * n, char const * host) {
    (void)n; (void)host; return ERR_OK;
}
static void mdns_resp_remove_netif(struct netif * n) { (void)n; ++mdns_removes; }
static bool ip4_addr_isany_val(ip4_addr_t ip) { return ip == 0; }
static bool ip4_addr_cmp(ip4_addr_t const * a, ip4_addr_t const * b) { return *a == *b; }
static ip4_addr_t const * netif_ip4_addr(struct netif * n) { return &n->ip; }
static void ip4addr_ntoa_r(ip4_addr_t const * ip, char * out, size_t cap) {
    (void)ip; snprintf(out, cap, "192.168.1.4");
}
static void ping_send(uint32_t t) { (void)t; }
static int cyw43_wifi_ap_get_stas(void *s, int *n, void *a) { (void)s; (void)a; *n=0; return 0; }
static int cyw43_wifi_get_rssi(void * s, int32_t * rssi) { (void)s; *rssi = -50; return 0; }
static void dhcp_server_deinit(dhcp_server_t * s) { (void)s; }
static void cyw43_arch_disable_ap_mode(void) { ++ap_stops; }
static void own_network_start(void) { ++recovery; g_wifi_mode = WIFI_OWN_NETWORK; }
static void hotspot_try(uint32_t);
static void hotspot_next(uint32_t, char const *, int);
static bool hotspot_first_round(void);
'''

NETWORK_TEST = r'''
static void tick(uint32_t t) { now = t; hotspot_check(t); }
int main(void) {
    now = 100; hotspot_start(now); tick(350);
    assert(joins == 1 && g_hot_phase == HOTSPOT_ASSOCIATING);
    driver_status = CYW43_LINK_NONET; tick(2100);
    assert(joins == 2 && g_hot_idx == 0 && g_hot_since == 350);
    // An address left over from elsewhere is not a valid DHCP lease.
    cyw43_state.netif[0].link = true; cyw43_state.netif[0].ip = 42;
    tick(2350); assert(g_hot_phase == HOTSPOT_WAITING_DHCP);
    cyw43_state.netif[0].lease = true; tick(2600);
    assert(g_hot_phase == HOTSPOT_ONLINE && g_wifi_mode == WIFI_JOINED);
    // Sleeping/filtered ICMP must not disconnect a healthy link.
    gb_ping_answers = true; g_ping_reply = 0; g_http_last_reply = 0;
    unsigned before = leaves; tick(20000);
    assert(leaves == before && g_hot_phase == HOTSPOT_ONLINE);
    // DHCP renewal/address changes remain online.
    cyw43_state.netif[0].ip = 99; tick(20250); assert(g_hot_ip == 99);
    cyw43_state.netif[0].lease = false; tick(20500);
    assert(g_hot_phase == HOTSPOT_WAITING_DHCP && g_offline_since == 20500);
    tick(20500 + WIFI_RECOVERY_MSEC);
    assert(recovery == 1 && g_wifi_mode == WIFI_OWN_NETWORK);
    before = joins; tick(20500 + 2 * WIFI_RECOVERY_MSEC);
    assert(joins == before && recovery == 1); // Recovery UI stays reachable.
    // Retry from the recovery UI shuts down AP before enabling station.
    g_hot_request = HOTSPOT_AUTO; tick(150000);
    assert(ap_stops == 1 && g_wifi_mode == WIFI_JOINING);
    tick(150250); assert(g_hot_phase == HOTSPOT_ASSOCIATING);
    // Explicit bad credentials advance to the next saved network.
    driver_status = CYW43_LINK_BADAUTH; tick(152000);
    assert(g_hot_idx == 1 && g_hot_phase == HOTSPOT_LEAVING);
    tick(152250); assert(g_hot_phase == HOTSPOT_ASSOCIATING);
    // A stuck disconnect recycles the station and unregisters mDNS first.
    cyw43_state.netif[0].link = true; gb_mdns_registered = true;
    hotspot_try(153000); before = resets; tick(155000);
    assert(resets == before + 1 && !gb_mdns_registered && mdns_removes >= 1);
    tick(155250); assert(g_hot_phase == HOTSPOT_ASSOCIATING);
    // Timeout must cover the WHOLE sequence of attempts, including wrap.
    memset(&cyw43_state, 0, sizeof(cyw43_state)); driver_status = 0;
    hotspot_start(UINT32_MAX - 30000u); before = recovery;
    tick((UINT32_MAX - 30000u) + WIFI_RECOVERY_MSEC);
    assert(recovery == before + 1 && g_wifi_mode == WIFI_OWN_NETWORK);
    return 0;
}
'''

STATUS = r'''
static obstacle_profile_t profile;
static bool scan_active;
static vehicle_state_t scan_vehicle_state = VEHICLE_OBSTACLE_SCAN;
static uint32_t scan_point_count = 3;
static obstacle_point_t scan_points[] = {
    {-30, 510, OBSTACLE_RETURN_WEAK_RANGE},
    {-28, ULTRASONIC_NO_RANGE, OBSTACLE_RETURN_NO_ECHO},
    {0, 298, OBSTACLE_RETURN_RANGE}
};
bool obstacle_scan_active(void) { return scan_active; }
obstacle_point_t const * obstacle_scan_points(uint32_t * count) { *count = scan_point_count; return scan_points; }
static char long_text[200];
static char log_text[96];
line_state_t line_sensor_read(void) { return (line_state_t){true, true}; }
line_raw_t line_sensor_raw(void) { return (line_raw_t){true, true}; }
uint32_t ultrasonic_range_mm(void) { return UINT32_MAX - 1; }
uint32_t log_next_seq(void) { return 32; }
bool log_get(uint32_t seq, char const ** line, log_cat_t * cat) {
    (void)seq; *line = log_text; *cat = LOG_CAT_SYSTEM; return true;
}
barcode_command_t vehicle_pending_turn(void) { return BARCODE_CMD_LEFT; }
obstacle_profile_t const * obstacle_profile(void) { return &profile; }
void imu_get_axes(imu_axes_t * axes) {
    *axes = (imu_axes_t){INT32_MIN, INT32_MIN, INT32_MIN, INT32_MIN, INT32_MIN, INT32_MIN};
}
vehicle_state_t vehicle_state(void) { return scan_vehicle_state; }
char const * vehicle_state_name(vehicle_state_t s) { (void)s; return "OBSTACLE_SCAN"; }
int32_t motor_left_speed(void) { return -100; }
int32_t motor_right_speed(void) { return -100; }
bool imu_is_ok(void) { return true; }
bool imu_reading_is_valid(void) { return true; }
bool imu_is_calibrated(void) { return true; }
int32_t imu_pitch_x10(void) { return -1800; }
int32_t imu_roll_x10(void) { return -1800; }
int32_t hump_last_height_x10(void) { return 0; }
int32_t hump_run_height_x10(void) { return 0; }
int32_t hump_height_x10(void) { return 0; }
uint32_t hump_count(void) { return 0; }
char const *mqtt_comm_status_text(void) { return "test"; }
int32_t vehicle_hump_peak_tilt_x10(void) { return -1800; }
uint32_t encoder_count(uint32_t p) { (void)p; return UINT32_MAX; }
char const * barcode_message(void) { return "ABCDEFGHIJKLMNOP"; }
bool barcode_is_black(void) { return true; }
bool barcode_is_enabled(void) { return true; }
uint32_t barcode_edge_count(void) { return UINT32_MAX; }
bool barcode_is_capturing(void) { return true; }
uint32_t barcode_scan_size(void) { return 200; }
barcode_result_t barcode_last_result(void) { return BARCODE_INVALID; }
int32_t servo_angle(void) { return 120; }
char const * obstacle_edge_status_name(obstacle_edge_status_t s) {
    (void)s; return "unreliable observations";
}
uint32_t encoder_watch_count(void) { return 10; }
uint32_t encoder_watch_pin(uint32_t i) { return i; }
uint32_t encoder_watch_edges(uint32_t i) { (void)i; return UINT32_MAX; }
uint32_t power_vsys_mv(void) { return 4500; }
uint32_t power_vsys_min_mv(void) { return 3500; }
char const * web_wifi_text(void) { return long_text; }
char const * barcode_command_name(barcode_command_t c) { (void)c; return "TURN LEFT"; }
static size_t json_string(char *, size_t, char const *);
static size_t append_text(char *, size_t, char const *, ...);
'''

STATUS_TEST = r'''
int main(void) {
    memset(long_text, '\\', sizeof(long_text) - 1); now = UINT32_MAX;
    memset(log_text, '\\', sizeof(log_text) - 1);
    gp_restart_reason = "watchdog restart";
    profile.left_status = profile.right_status = OBSTACLE_EDGE_UNRELIABLE;
    profile.width_mm = profile.width_min_mm = profile.width_max_mm = UINT32_MAX;
    profile.left_mm = profile.right_mm = INT32_MIN;
    profile.left_observed_mm = profile.right_observed_mm = INT32_MIN;
    char output[STATUS_BYTES + 16];
    for (size_t cap = 0; cap <= STATUS_BYTES; ++cap) {
        memset(output, 'X', sizeof(output));
        size_t n = status_build(output, cap, 0);
        assert(n < cap || cap == 0);
        for (size_t i = cap; i < sizeof(output); ++i) { assert(output[i] == 'X'); }
        if (cap == STATUS_BYTES) { assert(n > 2 && n == strlen(output)); puts(output); }
    }
    g_history_count = HISTORY_MAX;
    for (unsigned i = 0; i < HISTORY_MAX; ++i) {
        memset(g_history[i].text, '\\', HISTORY_TEXT_BYTES - 1);
        g_history[i].command = BARCODE_CMD_LEFT; g_history[i].time_sec = UINT32_MAX;
    }
    size_t n = history_build(output, STATUS_BYTES);
    assert(n > 2 && n < STATUS_BYTES && n == strlen(output)); puts(output);
    n = hotspots_build(output, STATUS_BYTES);
    assert(n > 2 && n < STATUS_BYTES); puts(output);
    scan_active = true;
    n = scan_build(output, STATUS_BYTES); assert(n == strlen(output)); puts(output);
    scan_active = false; profile.finished = true; scan_vehicle_state = VEHICLE_READY;
    profile.left_bounded = profile.right_bounded = true;
    profile.calibrated_width_valid = true; profile.calibrated_width_mm = 150;
    n = scan_build(output, STATUS_BYTES); assert(n == strlen(output)); puts(output);
    profile.timed_out = true; scan_build(output, STATUS_BYTES); puts(output);
    profile.timed_out = false; profile.finished = false; scan_point_count = 0;
    scan_build(output, STATUS_BYTES); puts(output);
    profile.cancelled = true; scan_build(output, STATUS_BYTES); puts(output);
    for (size_t cap = 0; cap <= STATUS_BYTES; ++cap) {
        memset(output, 'X', sizeof(output));
        n = history_build(output, cap);
        assert(n < cap || cap == 0);
        for (size_t i = cap; i < sizeof(output); ++i) { assert(output[i] == 'X'); }
        memset(output, 'X', sizeof(output));
        n = hotspots_build(output, cap);
        assert(n < cap || cap == 0);
        for (size_t i = cap; i < sizeof(output); ++i) { assert(output[i] == 'X'); }
    }
    return 0;
}
'''

HTTP = r'''
#define TCP_WRITE_FLAG_COPY 1
#define TCP_WRITE_FLAG_MORE 2
struct tcp_pcb { int unused; };
static unsigned writes, outputs, fail_write;
static err_t output_error;
static char header[HEADER_BYTES];
static err_t tcp_write(struct tcp_pcb * pcb, void const * data, u16_t n, u8_t flags) {
    (void)pcb;
    ++writes;
    if (writes == fail_write) { return ERR_MEM; }
    if (writes == 1) {
        assert(flags & TCP_WRITE_FLAG_COPY); assert(n < sizeof(header));
        memcpy(header, data, n); header[n] = 0;
    }
    return ERR_OK;
}
static err_t tcp_output(struct tcp_pcb * pcb) { (void)pcb; ++outputs; return output_error; }
'''

HTTP_TEST = r'''
int main(void) {
    struct tcp_pcb pcb = {0};
    output_error = ERR_MEM;
    assert(reply_send(&pcb, "text/plain", "hello", 5, true) == ERR_OK);
    assert(writes == 2 && outputs == 1 && strstr(header, "Content-Length: 5\r\n"));
    writes = outputs = 0; output_error = ERR_RTE;
    assert(reply_send(&pcb, "text/plain", "hello", 5, true) == ERR_OK);
    writes = outputs = 0; fail_write = 2;
    assert(reply_send(&pcb, "text/plain", "hello", 5, false) == ERR_MEM);
    assert(outputs == 0); // An incomplete response is not flushed.
    return 0;
}
'''

with tempfile.TemporaryDirectory() as directory:
    work = Path(directory)
    def run(name, code):
        source = work / (name + ".c")
        binary = work / name
        source.write_text(COMMON + DEFINITIONS + STATE + code)
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-Wno-unused-variable", "-Wno-unused-function",
                        "-fsanitize=undefined", "-fno-sanitize-recover=all",
                        "-I", str(ROOT / "src"), str(source), "-o", str(binary)],
                       check=True, timeout=30)
        result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=15)
        if result.returncode:
            print(result.stderr)
        result.check_returncode()
        return result.stdout

    names = ["hotspot_first_round", "hotspot_try", "hotspot_next", "hotspot_start",
             "hotspot_join", "hotspot_mdns_check", "hotspot_online", "hotspot_rejoin",
             "hotspot_wait_for_dhcp", "hotspot_check"]
    run("network", NETWORK + "\n".join(function(n) for n in names) + NETWORK_TEST)
    names = ["append_text", "json_string", "scan_build", "status_build", "history_build", "hotspots_build"]
    output = run("status", STATUS + "\n".join(function(n) for n in names) + STATUS_TEST)
    parsed = [json.loads(line) for line in output.splitlines()]
    assert parsed[3]["state"] == "Scanning"
    assert not parsed[3]["can_scan"]
    assert parsed[3]["points"][0] == [60, 510, 3]
    assert parsed[3]["points"][1][1] is None
    assert parsed[4]["state"] == "Complete"
    assert parsed[4]["can_scan"]
    assert "calibrated_width" not in parsed[3]
    assert "corrected_width" not in parsed[4]
    assert parsed[4]["width"] is None  # Raw value is not overwritten.
    assert parsed[5]["width"] is None
    assert parsed[5]["state"] == "Failed"
    assert parsed[5]["can_scan"]
    assert parsed[6]["state"] == "Idle"
    assert parsed[7]["state"] == "Failed"
    run("http", HTTP + function("reply_send") + HTTP_TEST)

    page = block("static char const g_page_html[] =", "static char const g_dashboard_js")
    html = "".join(ast.literal_eval(s) for s in re.findall(r'"(?:[^"\\]|\\.)*"', page))
    script = work / "dashboard.js"
    assert 'Ultrasonic Distance Scan' in html and 'SCAN NOW' in html
    assert 'src=/scan.js' in html and 'src=/dashboard.js' in html
    script_code = block("static char const g_dashboard_js[] =", "\n_Static_assert")
    script.write_text("".join(ast.literal_eval(s) for s in re.findall(r'"(?:[^"\\]|\\.)*"', script_code)))
    subprocess.run(["node", "--check", str(script)], check=True, timeout=10)
    plot = (ROOT / 'src/scan_plot.h').read_text()
    plot = "".join(ast.literal_eval(s) for s in re.findall(r'"(?:[^"\\]|\\.)*"', plot))
    script.write_text(plot)
    subprocess.run(["node", "--check", str(script)], check=True, timeout=10)
    # Execute the real plot code against a small Canvas/DOM mock. A null
    # sample must break the polyline, and both edges must be drawn.
    fixture = {"active":False,"can_scan":True,"state":"Complete","min":60,"max":120,"step":2,
               "points":[[60,100,3],[62,None,2],[64,200,1],[66,210,1]],
               "left":63,"right":107,"width":249,"calibrated_width":150,"calibration_enabled":True,
               "estimated":True,"ls":"estimated","rs":"missing echoes"}
    mock = r'''
const assert=require('assert');let dots=0,links=0,edges=0;
const ctx={scale(){},fillRect(){},fillText(){},beginPath(){},moveTo(){},lineTo(){},fill(){},
 arc(){dots++},setLineDash(a){if(a.length)edges++},stroke(){if(this.strokeStyle==='#58d8ff')links++}};
const elements={scanNow:{},scanTest:{},uscan:{clientWidth:320,style:{},getContext(){return ctx}},usstate:{},usresult:{}};
global.document={getElementById(i){return elements[i]}};
global.window={devicePixelRatio:2,addEventListener(){}};
global.setTimeout=()=>1;global.clearTimeout=()=>{};
global.fetch=()=>Promise.resolve({ok:true,json:()=>Promise.resolve(FIXTURE)});
'''.replace('FIXTURE', json.dumps(fixture))
    script.write_text(mock + plot + r'''
setImmediate(()=>{assert.equal(dots,3);assert.equal(links,1);assert.equal(edges,2);
 assert.equal(elements.usstate.textContent,'Complete');assert(elements.usresult.textContent.includes('249 mm'));
 assert(elements.usresult.textContent.includes('Geometric width: 249 mm'));
 assert(elements.usresult.textContent.includes('Measured centre distance: UNKNOWN'));
 assert.equal(elements.scanNow.disabled,false);});
''')
    subprocess.run(["node", str(script)], check=True, timeout=10)
    # Keep the same script/page alive across completion, repeated scans,
    # cancellation/failure and vehicle state changes. Availability is always
    # derived from current firmware state, never a one-shot browser latch.
    frames = [dict(fixture, state=state, active=active, can_scan=allowed)
              for state, active, allowed in [
                  ('Complete',False,True), ('Scanning',True,False),
                  ('Complete',False,True), ('Scanning',True,False),
                  ('Failed',False,True), ('Idle',False,True),
                  ('Complete',False,False), ('Complete',False,True)]]
    repeat = r'''
const frames=FRAMES,queued=[];let frame=0;
global.setTimeout=(fn,ms)=>{if(ms!==4000)queued.push(fn);return 1};
global.fetch=()=>Promise.resolve({ok:true,json:()=>Promise.resolve(frames[frame])});
'''.replace('FRAMES', json.dumps(frames))
    script.write_text(mock + repeat + plot + r'''
(async()=>{
 for(let i=0;i<frames.length;i++){
  await new Promise(resolve=>setImmediate(resolve));
  const disabled=frames[i].active||!frames[i].can_scan;
  assert.equal(elements.scanNow.disabled,disabled);
  assert.equal(elements.scanTest.disabled,disabled);
  if(i+1<frames.length){frame++;assert(queued.length);queued.shift()();}
 }
})().catch(e=>{console.error(e);process.exitCode=1});
''')
    subprocess.run(["node", str(script)], check=True, timeout=10)

print("Web host checks passed (hotspot recovery, DHCP, reconnect, JSON bounds, dashboard syntax)")

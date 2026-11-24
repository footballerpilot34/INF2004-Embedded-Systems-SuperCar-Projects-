"""Exercise the real serial/web/MQTT command dispatch and stationary scan lifecycle."""
import importlib.util
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]

# Project sources live in the template layout (app_program/, config/);
# find a file by name and pass every folder as an include path.
SRC_DIRS = [ROOT / d for d in (
    'config', 'app_program', 'app_program/line_barcode',
    'app_program/motion', 'app_program/obstacle', 'app_program/comms',
    'app_program/imu_hump')]
SRC_INCLUDES = [flag for d in SRC_DIRS for flag in ('-I', str(d))]


def src_path(name):
    """Return the path of a project source file, wherever it now lives."""
    return next(d / name for d in SRC_DIRS if (d / name).exists())

MAIN = (src_path("app_main.c")).read_text()
WEB = (src_path("web.c")).read_text()
MQTT = (src_path("mqtt_comm.c")).read_text()
BARCODE = (src_path("barcode.c")).read_text()

def function(source, name):
    start = re.search(r"^(?:static )?(?:void|bool|web_command_t|char const \*)\n" + name + r"\s*\(", source, re.M)
    assert start, name
    end = re.search(r"^}", source[start.start():], re.M)
    return source[start.start():start.start() + end.end()] + "\n"

spec = importlib.util.spec_from_file_location(
    'mqtt_frame', ROOT / 'tools' / 'mqtt_frame.py')
mqtt_frame = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mqtt_frame)
FRAME_TABLE = ',\n'.join(
    '    {"%s", {%s}, %d}' % (word, ', '.join('0x%02xu' % b for b in data),
                              len(data))
    for word in ('v', 'stop', 'ft1100', 'bcar', 'bclist', 'bcreset')
    for data in [mqtt_frame.encode_command(word)])

code = r'''
#include <assert.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "config.h"
#include "mqtt_frame.h"
#include "web.h"
#include "vehicle.h"
#include "obstacle.h"
#include "ultrasonic.h"
static bool active, finish_next;
// The firmware's packed flags (change requirement 1), as app_main.c and
// web.c declare them.
static struct {
    bool b_encoder_marked : 1; bool b_stationary_scan_test : 1;
    bool b_imu_cal_due : 1; bool b_was_online : 1;
    bool b_stationary_scan_starting : 1;
} g_app;
static struct {
    web_command_t pending : 5; bool b_feat_line : 1; bool b_feat_avoid : 1;
    bool b_feat_barcode : 1; bool b_feat_hump : 1; bool b_scan_test_busy : 1;
} g_w;
void mqtt_comm_set_broker(char const *h) { (void)h; }
static bool ready = true, settled = true;
static uint32_t g_stationary_scan_since, now = 100, points;
static unsigned starts, cancellations, tasks, movement, stops;
static int32_t left, right;
static int key;
static uint32_t range = 100, g_history_count, g_history_seq, g_log_floor;
static vehicle_state_t state = VEHICLE_READY;
static obstacle_profile_t profile;
static char const * gp_scan_test_status = "";
uint32_t get_absolute_time(void) { return now; }
uint32_t to_ms_since_boot(uint32_t t) { return t; }
int getchar_timeout_us(uint32_t timeout) { (void)timeout; int k=key; key=0; return k; }
void log_printf(char const * fmt, ...) { (void)fmt; }
void log_line(char const *fmt, ...) { (void)fmt; }
static char g_letters[] = BARCODE_LETTER_COMMANDS;
uint32_t log_next_seq(void) { return 0; }
void barcode_clear_message(void) {}
uint32_t encoder_watch_edges(uint32_t i) { (void)i; return 0; }
static void encoder_calibration_mark(void) {}
static void imu_calibrate_run(void) { ++movement; }
static void wheel_test(void) { ++movement; }
static void turn_test(barcode_command_t c) { (void)c; ++movement; }
bool line_diag_is_active(void) { return false; }
void line_diag_enter(void) {}
void line_diag_exit(void) {}
void line_diag_learn(bool black) { (void)black; }
void servo_set_angle(int32_t angle) { (void)angle; }
bool servo_is_settled(void) { return settled; }
bool ultrasonic_measurement_ready(void) { return ready; }
void ultrasonic_invalidate(void) { ready = false; }
vehicle_state_t vehicle_state(void) { return state; }
void vehicle_feature_test(bool l, bool a, bool b, bool h) { (void)l; (void)a; (void)b; (void)h; ++movement; }
void web_features(bool *l, bool *a, bool *b, bool *h) { *l=*a=*b=*h=true; }
void vehicle_bypass_test(void) { ++movement; }
void vehicle_start(void) { ++movement; }
void vehicle_stop(void) { ++stops; state=VEHICLE_STOPPED; }
void vehicle_barcode_test(void) { ++movement; }
void vehicle_barcode_static(void) { ++movement; }
uint32_t ultrasonic_range_mm(void) { return ready ? range : ULTRASONIC_NO_RANGE; }
void motor_set(int32_t l, int32_t r) { left=l; right=r; assert(l==0 && r==0); }
bool obstacle_scan_active(void) { return active; }
void obstacle_scan_start(uint32_t r, uint32_t now) {
    (void)now; assert(left==0 && right==0 && r>0 && r<=OBSTACLE_STOP_MM);
    ++starts; active=true; points=0; profile.finished=false; profile.timed_out=false;
}
void obstacle_scan_cancel(void) { ++cancellations; active=false; }
void obstacle_scan_task(uint32_t now) {
    (void)now; ++tasks; ++points;
    if(finish_next){active=false;profile.finished=true;ultrasonic_invalidate();}
}
obstacle_profile_t const * obstacle_profile(void) { return &profile; }
'''
code += MAIN[MAIN.index('#define KEY_START'):MAIN.index('static void          usb_wait(void);')]
code += function(WEB, 'web_set_scan_test_status') + function(WEB, 'web_set_scan_test_busy')
for name in ['vehicle_is_idle', 'stationary_scan_test_status', 'stationary_scan_test_start',
             'stationary_scan_test_cancel', 'stationary_scan_test_task', 'command_run', 'serial_poll']:
    code += function(MAIN, name)
for name in ['barcode_set_letter', 'barcode_reset_letters', 'barcode_letters']:
    code += function(BARCODE, name)
code += function(WEB, 'letter_name') + function(WEB, 'letters_log')
code += function(WEB, 'command_parse') + function(WEB, 'web_command_take')
code += function(WEB, 'web_command_submit')
code += r'''
typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
#define CMD_BYTES 8u
#define MQTT_DATA_FLAG_LAST 1u
static struct { bool b_cmd_topic : 1; bool b_ack_pending : 1; } g_mq;
static uint32_t g_cmd_len, g_ack_len;
static uint8_t g_cmd_in[CMD_BYTES], g_ack[CMD_BYTES];
static char const *g_topic_cmd = "inf2004/aai14/supercar/cmd";
// Binary command frames, built by the reference encoder
// (tools/mqtt_frame.py), as the laptop's tools/mqtt_car.ps1 sends them.
typedef struct { char const *word; u8_t bytes[4]; u16_t len; } frame_t;
static frame_t const FRAMES[] = {
''' + FRAME_TABLE + r'''
};
static frame_t const *frame(char const *word) {
    for (unsigned i = 0; i < sizeof(FRAMES) / sizeof(FRAMES[0]); i++) {
        if (strcmp(FRAMES[i].word, word) == 0) { return &FRAMES[i]; }
    }
    assert(0); return NULL;
}
// The car acknowledges a command with the same frame, typed ACK (0xA8).
static bool ack_is(char const *word) {
    frame_t const *f = frame(word);
    return g_ack_len == f->len && g_ack[0] == 0xA8u &&
           memcmp(g_ack + 1, f->bytes + 1, f->len - 1u) == 0;
}
'''
code += function(MQTT, 'incoming_publish_cb') + function(MQTT, 'incoming_data_cb')
code += r'''
static void web(char const * name){command_parse(name);command_run(web_command_take());}
static void mqtt(char const *word) {
    frame_t const *f = frame(word);
    incoming_publish_cb(NULL,g_topic_cmd,f->len);
    incoming_data_cb(NULL,f->bytes,f->len,MQTT_DATA_FLAG_LAST);
}
static void fresh(void) { ready=true; settled=true; stationary_scan_test_task(); }
int main(void) {
    key='g'; web_command_t c=serial_poll();
    assert(c==WEB_CMD_STATIONARY_SCAN && starts==0); command_run(c);
    assert(starts==0 && g_app.b_stationary_scan_test && g_app.b_stationary_scan_starting && g_w.b_scan_test_busy);
    stationary_scan_test_task(); assert(tasks==0); // Wait asynchronously for forward ping.
    fresh(); assert(starts==1 && active && !g_app.b_stationary_scan_starting);
    stationary_scan_test_task(); assert(tasks==1 && active);
    web("scan"); assert(starts==1 && strstr(gp_scan_test_status,"already running"));
    unsigned before_dry=movement; key='v'; command_run(serial_poll());
    assert(movement==before_dry); // Rehearsal is rejected while scanning.
    mqtt("v"); command_run(web_command_take()); assert(movement==before_dry);
    web("start"); assert(movement==0);
    web("stop"); assert(!active && !g_app.b_stationary_scan_test && !g_w.b_scan_test_busy && stops==1);
    assert(strstr(gp_scan_test_status,"CANCELLED"));
    unsigned n=starts;
    uint32_t invalid[]={0,OBSTACLE_STOP_MM+1,ULTRASONIC_NO_RANGE};
    for(unsigned i=0;i<3;i++){
        range=invalid[i];web("scan");fresh();
        assert(starts==n && !g_app.b_stationary_scan_test && !g_w.b_scan_test_busy);
    }
    assert(strstr(gp_scan_test_status,"stop range"));
    range=100;state=VEHICLE_FOLLOW_LINE;web("scan");assert(starts==n);
    assert(strstr(gp_scan_test_status,"stop the vehicle"));
    state=VEHICLE_STOPPED;active=true;web("scan");assert(starts==n);active=false;
    // Repeat without reboot/refresh. Completion invalidates the sensor just
    // like the real servo returning to centre. A new request must wait for
    // fresh data, retain the old points, and then reset only the new scan.
    for(unsigned cycle=0;cycle<100;cycle++){
        unsigned old_points=points;
        web("scan");
        assert(!active && points==old_points && g_w.b_scan_test_busy);
        assert(web_command_take()==WEB_CMD_NONE); // No stale queued command.
        stationary_scan_test_task(); assert(starts==n+cycle);
        fresh(); assert(starts==n+cycle+1 && active && points==0);
        stationary_scan_test_task(); assert(points==1);
        finish_next=true;stationary_scan_test_task();finish_next=false;
        assert(!active && !g_app.b_stationary_scan_test && !g_app.b_stationary_scan_starting && !g_w.b_scan_test_busy);
        assert(profile.finished && points==2 && !ready);
    }
    // Startup timeout and STOP during preparation release every busy flag.
    web("scan");now+=SCAN_READY_TIMEOUT_MSEC;stationary_scan_test_task();
    assert(!g_app.b_stationary_scan_test && !g_w.b_scan_test_busy && strstr(gp_scan_test_status,"FAILED"));
    web("scan");web("stop");assert(!g_app.b_stationary_scan_test && !g_app.b_stationary_scan_starting && !g_w.b_scan_test_busy);
    web("scan");fresh();profile.timed_out=true;finish_next=true;
    stationary_scan_test_task();finish_next=false;
    assert(!g_app.b_stationary_scan_test && !g_w.b_scan_test_busy && strstr(gp_scan_test_status,"timed out"));
    web("scan");fresh();key='x';command_run(serial_poll());
    assert(!active && !g_app.b_stationary_scan_test && !g_w.b_scan_test_busy);
    command_parse("stop");command_parse("scan");assert(web_command_take()==WEB_CMD_STOP);
    assert(movement==0 && left==0 && right==0 && cancellations>=3);
    // Actual MQTT callbacks -> shared parser -> same serial command handler.
    key='v'; web_command_t serial_v=serial_poll();
    assert(serial_v==WEB_CMD_BYPASS_TEST);
    g_mq.b_ack_pending=false;
    frame_t const *fv=frame("v");
    incoming_publish_cb(NULL,"other/topic",fv->len);
    incoming_data_cb(NULL,fv->bytes,fv->len,MQTT_DATA_FLAG_LAST);
    assert(!g_mq.b_ack_pending && web_command_take()==WEB_CMD_NONE);
    // An old text command is not a frame: ignored, not acknowledged.
    incoming_publish_cb(NULL,g_topic_cmd,1);
    incoming_data_cb(NULL,(u8_t const *)"v",1,MQTT_DATA_FLAG_LAST);
    assert(!g_mq.b_ack_pending && web_command_take()==WEB_CMD_NONE);
    // A frame that arrives in two parts.
    incoming_publish_cb(NULL,g_topic_cmd,fv->len);
    incoming_data_cb(NULL,fv->bytes,1,0);
    assert(!g_mq.b_ack_pending && web_command_take()==WEB_CMD_NONE);
    incoming_data_cb(NULL,fv->bytes+1,(u16_t)(fv->len-1u),MQTT_DATA_FLAG_LAST);
    assert(g_mq.b_ack_pending && ack_is("v"));
    web_command_t remote_v=web_command_take(); assert(remote_v==serial_v);
    unsigned before=movement; command_run(remote_v); assert(movement==before+1);
    mqtt("v"); mqtt("v"); // Multiple queued copies still dispatch once.
    command_run(web_command_take()); assert(movement==before+2);
    assert(web_command_take()==WEB_CMD_NONE);
    unsigned stops_before=stops;
    mqtt("stop"); mqtt("v"); command_run(web_command_take());
    assert(stops==stops_before+1 && movement==before+2);
    mqtt("v"); mqtt("stop");
    assert(ack_is("stop")); command_run(web_command_take());
    assert(stops==stops_before+2 && movement==before+2);
    mqtt("ft1100");
    assert(ack_is("ft1100"));
    assert(g_w.b_feat_line && g_w.b_feat_avoid && !g_w.b_feat_barcode && !g_w.b_feat_hump);
    assert(web_command_take()==WEB_CMD_FEATURE_TEST);
    mqtt("bcar"); assert(barcode_letters()[0]=='R' && ack_is("bcar"));
    mqtt("bclist"); assert(web_command_take()==WEB_CMD_NONE);
    mqtt("bcreset"); assert(strcmp(barcode_letters(),BARCODE_LETTER_COMMANDS)==0);
    return 0;
}
'''
with tempfile.TemporaryDirectory() as directory:
    source=Path(directory)/"test.c"; binary=Path(directory)/"test"
    source.write_text(code)
    subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror",
                    "-fsanitize=undefined","-fno-sanitize-recover=all",
                    *SRC_INCLUDES,str(source),str(src_path("mqtt_frame.c")),
                    str(src_path("bitpack.c")),"-o",str(binary)],check=True,timeout=30)
    subprocess.run([str(binary)],check=True,timeout=10)
print("Stationary scan checks passed (shared dispatch, rejection, STOP, completion, braking)")

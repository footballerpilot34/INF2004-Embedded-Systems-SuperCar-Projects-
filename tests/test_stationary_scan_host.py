"""Exercise the real serial/web command dispatch and stationary scan lifecycle."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "src/supercar_main.c").read_text()
WEB = (ROOT / "src/web.c").read_text()

def function(source, name):
    start = re.search(r"^(?:static )?(?:void|bool|web_command_t)\n" + name + r"\s*\(", source, re.M)
    assert start, name
    end = re.search(r"^}", source[start.start():], re.M)
    return source[start.start():start.start() + end.end()] + "\n"

code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "web.h"
#include "vehicle.h"
#include "obstacle.h"
#include "ultrasonic.h"
static bool g_stationary_scan_test, g_stationary_scan_starting, g_encoder_marked, active, finish_next;
static bool gb_scan_test_busy, ready = true, settled = true;
static uint32_t g_stationary_scan_since, now = 100, points;
static unsigned starts, cancellations, tasks, movement, stops;
static int32_t left, right;
static int key;
static uint32_t range = 100, g_history_count, g_history_seq, g_log_floor;
static web_command_t g_pending;
static vehicle_state_t state = VEHICLE_READY;
static obstacle_profile_t profile;
static char const * gp_scan_test_status = "";
uint32_t get_absolute_time(void) { return now; }
uint32_t to_ms_since_boot(uint32_t t) { return t; }
int getchar_timeout_us(uint32_t timeout) { (void)timeout; int k=key; key=0; return k; }
void log_printf(char const * fmt, ...) { (void)fmt; }
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
code += MAIN[MAIN.index('#define KEY_START'):MAIN.index('static bool g_encoder_marked;')]
code += function(WEB, 'web_set_scan_test_status') + function(WEB, 'web_set_scan_test_busy')
for name in ['vehicle_is_idle', 'stationary_scan_test_status', 'stationary_scan_test_start',
             'stationary_scan_test_cancel', 'stationary_scan_test_task', 'command_run', 'serial_poll']:
    code += function(MAIN, name)
code += function(WEB, 'command_parse') + function(WEB, 'web_command_take')
code += r'''
static void web(char const * name){command_parse(name);command_run(web_command_take());}
static void fresh(void) { ready=true; settled=true; stationary_scan_test_task(); }
int main(void) {
    key='g'; web_command_t c=serial_poll();
    assert(c==WEB_CMD_STATIONARY_SCAN && starts==0); command_run(c);
    assert(starts==0 && g_stationary_scan_test && g_stationary_scan_starting && gb_scan_test_busy);
    stationary_scan_test_task(); assert(tasks==0); // Wait asynchronously for forward ping.
    fresh(); assert(starts==1 && active && !g_stationary_scan_starting);
    stationary_scan_test_task(); assert(tasks==1 && active);
    web("scan"); assert(starts==1 && strstr(gp_scan_test_status,"already running"));
    web("start"); assert(movement==0);
    web("stop"); assert(!active && !g_stationary_scan_test && !gb_scan_test_busy && stops==1);
    assert(strstr(gp_scan_test_status,"CANCELLED"));
    unsigned n=starts;
    uint32_t invalid[]={0,OBSTACLE_STOP_MM+1,ULTRASONIC_NO_RANGE};
    for(unsigned i=0;i<3;i++){
        range=invalid[i];web("scan");fresh();
        assert(starts==n && !g_stationary_scan_test && !gb_scan_test_busy);
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
        assert(!active && points==old_points && gb_scan_test_busy);
        assert(web_command_take()==WEB_CMD_NONE); // No stale queued command.
        stationary_scan_test_task(); assert(starts==n+cycle);
        fresh(); assert(starts==n+cycle+1 && active && points==0);
        stationary_scan_test_task(); assert(points==1);
        finish_next=true;stationary_scan_test_task();finish_next=false;
        assert(!active && !g_stationary_scan_test && !g_stationary_scan_starting && !gb_scan_test_busy);
        assert(profile.finished && points==2 && !ready);
    }
    // Startup timeout and STOP during preparation release every busy flag.
    web("scan");now+=SCAN_READY_TIMEOUT_MSEC;stationary_scan_test_task();
    assert(!g_stationary_scan_test && !gb_scan_test_busy && strstr(gp_scan_test_status,"FAILED"));
    web("scan");web("stop");assert(!g_stationary_scan_test && !g_stationary_scan_starting && !gb_scan_test_busy);
    web("scan");fresh();profile.timed_out=true;finish_next=true;
    stationary_scan_test_task();finish_next=false;
    assert(!g_stationary_scan_test && !gb_scan_test_busy && strstr(gp_scan_test_status,"timed out"));
    web("scan");fresh();key='x';command_run(serial_poll());
    assert(!active && !g_stationary_scan_test && !gb_scan_test_busy);
    command_parse("stop");command_parse("scan");assert(web_command_take()==WEB_CMD_STOP);
    assert(movement==0 && left==0 && right==0 && cancellations>=3);
    return 0;
}
'''
with tempfile.TemporaryDirectory() as directory:
    source=Path(directory)/"test.c"; binary=Path(directory)/"test"
    source.write_text(code)
    subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror",
                    "-fsanitize=undefined","-fno-sanitize-recover=all",
                    "-I",str(ROOT/"src"),str(source),"-o",str(binary)],check=True,timeout=30)
    subprocess.run([str(binary)],check=True,timeout=10)
print("Stationary scan checks passed (shared dispatch, rejection, STOP, completion, braking)")

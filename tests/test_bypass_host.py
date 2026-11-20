"""Focused checks of the REAL bypass and MQTT event code with hardware stubs.

Internal phase setup isolates clearance/return decisions; it does not simulate
wheel slip, prove a trajectory, or replace physical validation.
"""
from pathlib import Path
import importlib.util
import subprocess
import tempfile
import sys
sys.dont_write_bytecode = True

ROOT = Path(__file__).resolve().parents[1]
code = r'''
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "bypass.c"
static int32_t ml, mr;
static uint32_t seq, left_edges, right_edges;
static bool ready = true, settled = true, black, left_only;
static obstacle_point_t points[31];
static obstacle_profile_t profile;
uint32_t encoder_count(uint32_t pin) { return pin == PIN_ENC_LEFT_A ? left_edges : right_edges; }
void motor_set(int32_t l, int32_t r) { ml=l; mr=r; }
int32_t motor_left_speed(void) { return ml; }
int32_t motor_right_speed(void) { return mr; }
void log_obstacle(char const *fmt, ...) { (void)fmt; }
void servo_set_angle(int32_t a) { (void)a; }
bool servo_is_settled(void) { return settled; }
bool ultrasonic_measurement_ready(void) { return ready; }
uint32_t ultrasonic_sample_sequence(void) { return seq; }
void ultrasonic_invalidate(void) {}
line_state_t line_sensor_read(void) { return (line_state_t){black || left_only, black}; }
obstacle_point_t const *obstacle_scan_points(uint32_t *n) { *n=31; return points; }
obstacle_profile_t const *obstacle_profile(void) { return &profile; }
char const *obstacle_edge_status_name(obstacle_edge_status_t s) { (void)s; return "echo-bracketed"; }
static uint32_t now_msec_get(void) { return 1234; }
static uint32_t hump_count(void) { return 0; }
static int32_t hump_last_height_x10(void) { return 0; }
static int32_t hump_last_tilt_x10(void) { return 0; }
static int32_t hump_run_height_x10(void) { return 0; }
static int32_t hump_run_tilt_x10(void) { return 0; }
#define PAYLOAD_BYTES 512u
static bool gb_scan_done;
static uint32_t g_humps_seen;
static char const *g_topic_obstacle="car/obstacle", *g_topic_hump="car/hump";
static char sent[512];
static unsigned events;
static void publish(char const *topic, char const *payload, unsigned qos, unsigned retain) {
    assert(strcmp(topic,"car/obstacle")==0 && qos==1 && retain==0);
    assert(strlen(payload)<sizeof(sent)); strcpy(sent,payload); ++events;
}
'''
source = (ROOT / 'src/mqtt_comm.c').read_text()
start = source.index('static void\npublish_events (void)')
end = source.index('}   /* publish_events() */', start) + len('}   /* publish_events() */')
code += source[start:end]
code += r'''
static void start_pass(void) {
    reset_state(false, true); g_front_z=200; g_z=100;
    phase_enter(PHASE_PASS,0); gb_pass_straight=true;
}

static void fixed_checks(void) {
    for (int i=0;i<31;i++) points[i]=(obstacle_point_t){-30+2*i,300,OBSTACLE_RETURN_RANGE};
    profile=(obstacle_profile_t){.finished=true,.centre_seen=true,.front_mm=300,
        .left_status=OBSTACLE_EDGE_SCAN_LIMIT,.right_status=OBSTACLE_EDGE_NO_ECHO};
    assert(BYPASS_FIXED_OFFSET_MM==220);
    if (SENSOR_TO_BUMPER_MM < 0) {
        assert(!bypass_begin(&profile,0,true)); return;
    }
    assert(bypass_begin(&profile,0,true)); // Unknown edges/class do not veto the test.
    assert(g_target_x==-220 && g_target_hdg==-(float)BYPASS_DIAGONAL_DEG);
    assert(g_fixed_clear_z==300-SENSOR_TO_BUMPER_MM+100+180+20);
    assert(g_fixed_clear_z==(300-SENSOR_TO_BUMPER_MM+100+180+70+50)-100);
    bypass_cancel();
    // Measured width is ignored even if it is inconsistent with the fixed test.
    profile.width_mm=900; profile.calibrated_width_valid=true; profile.calibrated_width_mm=250;
    assert(bypass_begin(&profile,0,true)); assert(g_target_x==-220);
    bypass_cancel();
    profile.front_mm=150;
    assert(!bypass_begin(&profile,0,true)); // Insufficient pivot envelope.
    profile.front_mm=300; points[15].kind=OBSTACLE_RETURN_NO_ECHO;
    assert(!bypass_begin(&profile,0,true)); // No stable forward obstacle.
    points[15].kind=OBSTACLE_RETURN_RANGE;
    points[0].range_mm=80; assert(!bypass_begin(&profile,0,true));
    points[0].range_mm=300;
    profile.timed_out=true; assert(!bypass_begin(&profile,0,true)); profile.timed_out=false;
    profile.cancelled=true; assert(!bypass_begin(&profile,0,true)); profile.cancelled=false;

    // Full controller/odometry sequence with ideal synthetic wheel motion.
    // Wheel speed here is a test input, not a physical speed calibration.
    for (int dry=0; dry<2; ++dry) {
    black=false; left_edges=right_edges=0;
    assert(dry ? bypass_test_begin(0) : bypass_begin(&profile,0,true));
    assert(profile.width_mm==900); // Rehearsal does not fabricate sensor telemetry.
    unsigned phases=0; float left_mm=0, right_mm=0;
    bool saw_line=false, checked_clear=false;
    bypass_result_t result=BYPASS_RUNNING;
    for (uint32_t t=10;t<60000 && result==BYPASS_RUNNING;t+=10) {
        left_mm+=fabsf((float)ml)*0.06f; right_mm+=fabsf((float)mr)*0.06f;
        left_edges=(uint32_t)lroundf(left_mm*ENC_LEFT_EDGES_PER_M/1000.0f);
        right_edges=(uint32_t)lroundf(right_mm*ENC_RIGHT_EDGES_PER_M/1000.0f);
        if (!dry && g_phase==PHASE_RETURN && g_x>=-(float)BYPASS_LINE_ADVANCE_MM*sinf(BYPASS_DIAGONAL_DEG/DEG_PER_RAD)) {
            black=true; saw_line=true;
        }
        if (g_phase==PHASE_PIVOT && g_after==PHASE_RETURN) {
            assert(g_z>=g_fixed_clear_z); checked_clear=true;
        }
        if (g_phase==PHASE_PASS && !(phases&(1u<<PHASE_PASS))) {
            assert(g_x<=-(float)BYPASS_FIXED_OFFSET_MM); // Full offset before passing.
            assert(fabsf(g_heading)<=BYPASS_ALIGN_TOL_DEG); // Second turn cancels departure.
        }
        if (!dry && g_phase==PHASE_ALIGN) {
            black=fabsf(g_heading)<=BYPASS_ALIGN_TOL_DEG;
        }
        phases|=1u<<g_phase;
        result=bypass_step(t,ULTRASONIC_NO_RANGE); // Explicit controlled empty corridor.
    }
    if (result!=BYPASS_DONE) fprintf(stderr,"fixed failed: offset=%d result=%d phase=%s x=%f z=%f heading=%f distance=%f phases=%x\n", SENSOR_TO_BUMPER_MM,result,bypass_phase_name(),g_x,g_z,g_heading,g_phase_mm,phases);
    assert(result==BYPASS_DONE && (dry || saw_line) && checked_clear && ml==0 && mr==0);
    assert(phases&(1u<<PHASE_OUT)); assert(phases&(1u<<PHASE_PASS));
    assert(phases&(1u<<PHASE_RETURN));
    if (!dry) {
        assert(phases&(1u<<PHASE_ADVANCE));
        assert(phases&(1u<<PHASE_ALIGN));
    } else { assert(phases&(1u<<PHASE_FINISH)); }
    assert(fabsf(g_heading)<=BYPASS_ALIGN_TOL_DEG);
    }
    // A moderate turn may not consume the front/corner clearance budget.
    profile.front_mm=SENSOR_TO_BUMPER_MM+220;
    assert(!bypass_begin(&profile,0,true)); profile.front_mm=300;
    assert(bypass_begin(&profile,0,true));
    phase_enter(PHASE_OUT,0); g_z=g_front_z-CAR_WIDTH_MM/2u-OBSTACLE_BYPASS_MARGIN_MM;
    assert(bypass_step(10,ULTRASONIC_NO_RANGE)==BYPASS_FAILED);

    // Both sensors at an oblique heading are not forward alignment.
    assert(bypass_begin(&profile,0,true)); black=true; g_heading=60;
    phase_enter(PHASE_ALIGN,0);
    assert(bypass_step(10,ULTRASONIC_NO_RANGE)==BYPASS_RUNNING && g_line_ticks==0);
    assert(g_phase==PHASE_ALIGN);
    // One sensor at heading zero must not release the car to following.
    black=false; left_only=true; g_heading=0;
    phase_enter(PHASE_ALIGN,0);
    for (uint32_t t=10;t<=50;t+=10) {
        assert(bypass_step(t,ULTRASONIC_NO_RANGE)==BYPASS_RUNNING);
        assert(ml==0 && mr==0 && g_line_ticks==0);
    }
    assert(bypass_step(BYPASS_LINE_TIMEOUT_MSEC+1,ULTRASONIC_NO_RANGE)==BYPASS_FAILED);
    left_only=false;
    // Stop on the first properly aligned pair, not after rotating past it.
    assert(bypass_begin(&profile,0,true)); g_heading=4; black=true;
    phase_enter(PHASE_ALIGN,0);
    assert(bypass_step(10,ULTRASONIC_NO_RANGE)==BYPASS_RUNNING && ml==0 && mr==0);
    black=false; assert(bypass_step(20,ULTRASONIC_NO_RANGE)==BYPASS_RUNNING);
    black=true;
    assert(bypass_step(30,ULTRASONIC_NO_RANGE)==BYPASS_RUNNING);
    assert(bypass_step(40,ULTRASONIC_NO_RANGE)==BYPASS_RUNNING);
    assert(bypass_step(50,ULTRASONIC_NO_RANGE)==BYPASS_DONE && ml==0 && mr==0);
    assert(fabsf(g_heading)<=BYPASS_ALIGN_TOL_DEG);
    assert(bypass_begin(&profile,0,true)); g_heading=60; phase_enter(PHASE_FINISH,0);
    assert(bypass_step(10,ULTRASONIC_NO_RANGE)==BYPASS_FAILED);

    // Target crossing brakes instead of reversing and applying another boost.
    assert(bypass_begin(&profile,0,true));
    g_heading=g_target_hdg-1;
    assert(bypass_step(10,ULTRASONIC_NO_RANGE)==BYPASS_RUNNING && g_phase==PHASE_OUT);
    assert(ml==0 && mr==0);
    assert(bypass_begin(&profile,0,true)); g_heading=g_target_hdg-10;
    assert(bypass_step(10,ULTRASONIC_NO_RANGE)==BYPASS_FAILED);
    assert(bypass_begin(&profile,0,true)); g_heading=g_target_hdg+10;
    assert(!fixed_turn_step(PIVOT_PULSE_ON_MSEC+10) && ml==0 && mr==0);
    assert(!fixed_turn_step(10) && ml<0 && mr>0);
    g_heading=g_target_hdg-1; motor_set(0,0);
    left_edges+=(uint32_t)lroundf(ENC_LEFT_EDGES_PER_M/1000.0f);
    right_edges+=(uint32_t)lroundf(ENC_RIGHT_EDGES_PER_M/1000.0f);
    float before_coast=g_heading; odo_update();
    assert(g_heading<before_coast); // Coast retains actual last turn direction.
    bypass_cancel();
    // No line cannot fall through to following based on distance alone.
    assert(bypass_begin(&profile,0,true)); black=false;
    phase_enter(PHASE_RETURN,0); g_heading=90;
    assert(bypass_step(BYPASS_LINE_TIMEOUT_MSEC+1,ULTRASONIC_NO_RANGE)==BYPASS_FAILED);
    assert(ml==0 && mr==0);
    assert(bypass_begin(&profile,0,true)); phase_enter(PHASE_RETURN,0);
    g_phase_mm=BYPASS_FIXED_OFFSET_MM+CAR_LENGTH_MM+1;
    assert(bypass_step(10,ULTRASONIC_NO_RANGE)==BYPASS_FAILED);
    assert(bypass_begin(&profile,0,true));
    assert(bypass_step(10,BYPASS_FRONT_STOP_MM)==BYPASS_FAILED);
    assert(ml==0 && mr==0);
    assert(bypass_begin(&profile,0,true)); ready=false; ml=mr=12;
    assert(bypass_step(10,ULTRASONIC_NO_RANGE)==BYPASS_RUNNING && ml==0 && mr==0);
    assert(bypass_step(BYPASS_STEP_MAX_MSEC+1,ULTRASONIC_NO_RANGE)==BYPASS_FAILED);
    ready=true;
    phase_t interrupt_phases[]={PHASE_PIVOT,PHASE_OUT,PHASE_PASS,PHASE_RETURN,PHASE_ADVANCE,PHASE_ALIGN};
    for (unsigned i=0;i<sizeof(interrupt_phases)/sizeof(interrupt_phases[0]);++i) {
        assert(bypass_begin(&profile,0,true));
        phase_enter(interrupt_phases[i],0); ml=12; mr=(g_phase==PHASE_PIVOT ? -12 : 12);
        bypass_cancel();
        assert(g_phase==PHASE_IDLE && ml==0 && mr==0);
    }
}
int main(void) {
    if (BYPASS_FIXED_WIDTH_TEST) { fixed_checks(); return 0; }
    assert(CAR_LENGTH_MM==180 && CAR_WIDTH_MM==140);
    assert(SIDE_CLEARANCE_MM==145); // ceil(hypot(90,70)) + 30.
    assert(BYPASS_ASSUMED_DEPTH_MM==100);
    for (int i=0;i<31;i++) points[i]=(obstacle_point_t){-30+2*i,2000,OBSTACLE_RETURN_RANGE};
    profile=(obstacle_profile_t){.finished=true,.centre_seen=true,
        .left_bounded=true,.right_bounded=true,.front_mm=200,
        .left_status=OBSTACLE_EDGE_CONFIRMED,.right_status=OBSTACLE_EDGE_CONFIRMED,
        .left_mm=-75,.right_mm=75,.left_edge={-10,-12,200,210},
        .right_edge={10,12,200,210}};
    for (uint32_t d=150;d<=250;d+=50) {
        profile.front_mm=d;
        assert(!bypass_begin(&profile,0,true)); // View/turn room insufficient.
        assert(ml==0 && mr==0);
    }
    // Artificial long-range setup exercises geometry decisions only. Real
    // scanner stops at 250 mm; this is not a newly supported operating mode.
    profile.front_mm=1000;
    assert(side_shift(&profile,true)==270); // Unknown: 125 + corner radius/margin.
    profile.calibrated_width_valid=true; profile.calibrated_width_mm=150;
    assert(side_shift(&profile,true)==220); // 75 + same clearance.
    points[0].kind=OBSTACLE_RETURN_NO_ECHO;
    assert(side_shift(&profile,true)==NO_SIDE);
    points[0].kind=OBSTACLE_RETURN_RANGE;
    points[0].range_mm=300;
    assert(side_shift(&profile,true)==NO_SIDE);
    points[0].range_mm=2000;
    profile.front_mm=570;
    assert(!bypass_begin(&profile,0,true)); // Curve too tight even with view.
    assert(!start_backup(PHASE_OUT,80,0)); // Unseen rear path is never used.

    start_pass();
    assert(g_pass_min_mm==100+100+180+30);
    seq++; pass_step(100,200); assert(gb_side_seen);
    pass_step(200,1000); assert(g_side_clear==0); // Same ping, not another vote.
    for (int i=0;i<5;i++) { seq++; pass_step(300+100*i,ULTRASONIC_NO_RANGE); }
    assert(g_pass_end_mm==0 && g_phase==PHASE_PASS);
    for (int i=0;i<3;i++) { seq++; pass_step(900+100*i,1000); }
    assert(g_pass_end_mm >= CAR_LENGTH_MM+BYPASS_SIDE_MARGIN_MM);
    uint32_t target=g_pass_end_mm;
    seq++; pass_step(1200,200); assert(g_pass_end_mm==0); // Side reappears.
    for (int i=0;i<3;i++) { seq++; pass_step(1300+100*i,1000); }
    target=g_pass_end_mm;
    g_phase_mm=g_pass_min_mm-1;
    pass_step(1550,1000); assert(g_phase==PHASE_PASS);
    g_phase_mm=(target>g_pass_min_mm ? target:g_pass_min_mm);
    pass_step(1560,1000); assert(g_phase==PHASE_PIVOT); // Only now return.
    start_pass(); g_phase_mm=g_pass_min_mm;
    assert(pass_step(2000,ULTRASONIC_NO_RANGE)==BYPASS_FAILED); // Never seen.

    reset_state(false,true); phase_enter(PHASE_RETURN,0);
    gb_ret_hold=true; g_hold_end_mm=10; g_phase_mm=20;
    seq++; return_guard(100,ULTRASONIC_NO_RANGE);
    assert(gb_ret_hold && g_hold_end_mm==0);
    black=false; phase_enter(PHASE_FINISH,0);
    assert(bypass_step(10,1000)==BYPASS_FAILED && ml==0 && mr==0);
    black=true; phase_enter(PHASE_FINISH,0);
    assert(bypass_step(10,1000)==BYPASS_RUNNING);
    assert(bypass_step(20,1000)==BYPASS_RUNNING);
    assert(bypass_step(30,1000)==BYPASS_DONE);
    black=false; phase_enter(PHASE_RETURN,0);
    assert(bypass_step(BYPASS_STEP_MAX_MSEC+1,1000)==BYPASS_FAILED);
    phase_enter(PHASE_OUT,0); ml=mr=12; bypass_cancel();
    assert(g_phase==PHASE_IDLE && ml==0 && mr==0);

    // Preserve old event fields; expose calibration as null/150/250.
    profile.front_mm=200; profile.width_mm=333;
    profile.calibrated_width_valid=false; publish_events();
    assert(strstr(sent,"\"width\":333") && strstr(sent,"\"front\":200"));
    assert(strstr(sent,"\"calibrated_width\":null"));
    unsigned previous=events; publish_events(); assert(events==previous);
    for (uint32_t w=150; w<=250; w+=100) {
        gb_scan_done=false; profile.calibrated_width_valid=true;
        profile.calibrated_width_mm=w; publish_events();
        assert(strstr(sent,w==150 ? "\"calibrated_width\":150" : "\"calibrated_width\":250"));
        assert(strstr(sent,"\"width\":333"));
    }
    profile.timed_out=true; gb_scan_done=false; publish_events();
    assert(strstr(sent,"\"calibrated_width\":null"));
    return 0;
}
'''
with tempfile.TemporaryDirectory() as directory:
    work = Path(directory)
    (work / 'test.c').write_text(code)
    for offset, fixed in ((-1, 0), (0, 0), (60, 0), (-1, 1), (0, 1), (60, 1)):
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        f'-DSENSOR_TO_BUMPER_MM={offset}', f'-DBYPASS_FIXED_WIDTH_TEST={fixed}',
                        '-I', str(ROOT / 'src'),
                        str(work / 'test.c'), '-lm', '-o', str(work / 'test')],
                       check=True, timeout=30)
        subprocess.run([str(work / 'test')], check=True, timeout=10)

# Synthetic reference rows test the helper; never write these to firmware.
spec = importlib.util.spec_from_file_location('calibration', ROOT/'tests/calibrate_obstacle_width.py')
calibration = importlib.util.module_from_spec(spec)
spec.loader.exec_module(calibration)
rows = []
for distance in (150, 200, 250):
    for width, span in ((150, 22), (250, 38)):
        for repeat in range(3):
            rows.append(dict(nominal_distance_mm=distance, distance_mm=distance+repeat,
                             left_deg=90-span/2, right_deg=90+span/2, width_mm=width,
                             lst='echo-bracketed', rst='echo-bracketed'))
assert calibration.calibrate(rows)[0] == (150, 152, 220, 220, 380, 380)
assert calibration.calibrate([]) == [(0,0,0,0,0,0)] * 3
for row in rows:
    row['right_deg'], row['left_deg'] = 101, 79
assert calibration.calibrate(rows) == [(0,0,0,0,0,0)] * 3
print('Bypass, MQTT compatibility and calibration helper host checks passed')

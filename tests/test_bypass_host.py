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
static bool ready = true, settled = true, black, left_only, right_only;
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
line_state_t line_sensor_read(void) { return (line_state_t){black || left_only, black || right_only}; }
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

// Advance the incremental planner while time is held at zero in isolated tests.
static bool planned_begin(void) {
    if (!bypass_begin(&profile,0,true)) return false;
    for (unsigned i=0;i<256 && g_phase==PHASE_PLAN;++i) {
        if (bypass_step(0,300)==BYPASS_FAILED) return false;
    }
    if(g_phase==PHASE_BACKUP) {
        // Isolated state tests begin after ideal reverse motion; full-route
        // integration below exercises the actual reverse/settling controller.
        g_x=-g_geometry.reverse_mm*sinf(BYPASS_INITIAL_HEADING_DEG/DEG_PER_RAD);
        g_z=-g_geometry.reverse_mm*cosf(BYPASS_INITIAL_HEADING_DEG/DEG_PER_RAD);
        start_pivot(-g_geometry.departure,PHASE_OUT,0);
        phase_enter(PHASE_PIVOT,0);
    }
    return g_phase==PHASE_PIVOT;
}
static void fixed_checks(void) {
    for (int i=0;i<31;i++) points[i]=(obstacle_point_t){-30+2*i,300,OBSTACLE_RETURN_RANGE};
    profile=(obstacle_profile_t){.finished=true,.centre_seen=true,.front_mm=OBSTACLE_STOP_MM,
        .left_bounded=true,.right_bounded=true,
        .left_status=OBSTACLE_EDGE_CONFIRMED,.right_status=OBSTACLE_EDGE_CONFIRMED,
        .left_mm=-100,.right_mm=100,
        .left_edge={-16,-18,300,315},.right_edge={16,18,300,315}};
    if (SENSOR_TO_BUMPER_MM < 0) {
        assert(!bypass_begin(&profile,0,true)); return;
    }
    assert(planned_begin());
    assert(g_target_x==-210 && g_geometry.departure<=BYPASS_DEPARTURE_MAX_DEG);
    assert(fabsf(g_fixed_clear_z-((OBSTACLE_STOP_MM+BYPASS_AXLE_TO_FRONT_MM-SENSOR_TO_BUMPER_MM)*cosf(BYPASS_INITIAL_HEADING_DEG/DEG_PER_RAD)-BYPASS_AXLE_TO_FRONT_MM+150+180+20+BYPASS_PASS_EXTRA_MM-BYPASS_PASS_TRIM_MM))<0.01f);
    if(SENSOR_TO_BUMPER_MM==60) assert(g_geometry.departure==77 && g_geometry.reverse_mm==0);
    assert(g_geometry.return_deg>=BYPASS_RETURN_MIN_DEG && g_geometry.return_deg<=70);
    profile.corrected_width_mm=140;
    assert(bypass_begin(&profile,0,true)); assert(g_geometry.left==-75 && g_target_x==-210);
    bypass_cancel();
    profile.corrected_width_mm=160;
    assert(bypass_begin(&profile,0,true)); assert(g_geometry.left==-80 && g_target_x==-215);
    bypass_cancel(); profile.corrected_width_mm=150;
    // Fixed controlled-test geometry does not retune itself from scan width/edges.
    profile.left_status=OBSTACLE_EDGE_NO_ECHO;
    assert(bypass_begin(&profile,0,true)); assert(g_target_x==-210);
    bypass_cancel(); profile.left_status=OBSTACLE_EDGE_CONFIRMED;
    profile.left_mm=-150; profile.right_mm=50;
    assert(bypass_begin(&profile,0,true)); assert(g_target_x==-210);
    bypass_cancel(); profile.left_mm=-100; profile.right_mm=100;
    profile.front_mm=SENSOR_TO_BUMPER_MM+20; assert(!planned_begin()); profile.front_mm=OBSTACLE_STOP_MM;
    points[15].kind=OBSTACLE_RETURN_NO_ECHO;
    assert(!bypass_begin(&profile,0,true)); points[15].kind=OBSTACLE_RETURN_RANGE;
    points[0].range_mm=80; assert(!bypass_begin(&profile,0,true)); points[0].range_mm=300;
    profile.timed_out=true; assert(!bypass_begin(&profile,0,true)); profile.timed_out=false;
    profile.cancelled=true; assert(!bypass_begin(&profile,0,true)); profile.cancelled=false;

    // Full ideal encoder routes. Sensor contact is tied to the measured forward
    // sensor position; final success still needs BOTH sensors and forward heading.
    for (int trial=0;trial<4;++trial) {
        int dry=trial==1;
        profile.front_mm=trial==2 ? 140 : OBSTACLE_STOP_MM;
        black=left_only=false; left_edges=right_edges=0;
        assert(dry ? bypass_test_begin(0) : bypass_begin(&profile,0,true));
        unsigned phases=0; float lm=0,rm=0; bool checked_return=false;
        bypass_result_t result=BYPASS_RUNNING;
        for (uint32_t t=10;t<60000 && result==BYPASS_RUNNING;t+=10) {
            lm+=fabsf((float)ml)*0.06f; rm+=fabsf((float)mr)*0.06f;
            left_edges=(uint32_t)lroundf(lm*ENC_LEFT_EDGES_PER_M/1000.0f);
            right_edges=(uint32_t)lroundf(rm*ENC_RIGHT_EDGES_PER_M/1000.0f);
            // Positive cases model the reported effective contact/turn offset.
            // Trial 3 retains the 45 mm ideal mount and must fail safely if
            // empirical over-advance leaves the pair off-line.
            float contact=trial==3 ? BYPASS_LINE_SENSOR_FORWARD_MM : BYPASS_REACQUIRE_ADVANCE_MM;
            float sensor_x=g_x+contact*sinf(g_heading/DEG_PER_RAD);
            black=!dry && (g_phase==PHASE_RETURN || g_phase==PHASE_ADVANCE || g_phase==PHASE_ALIGN) && fabsf(sensor_x)<=9;
            if (g_phase==PHASE_PASS) {
                assert(fabsf(g_heading)<=BYPASS_ALIGN_TOL_DEG);
                assert(fabsf(g_x-g_target_x)<=BYPASS_LATERAL_TOL_MM);
            }
            if ((g_phase==PHASE_BRAKE || g_phase==PHASE_PIVOT) && g_after==PHASE_RETURN &&
                (phases&(1u<<PHASE_PASS))) {
                if(!checked_return) {assert(g_z>=g_fixed_clear_z);checked_return=true;}
                // Unequal-wheel pivot motion is explicitly integrated; it must
                // stay inside the reserved position allowance, not exactly fixed.
                assert(g_z+BYPASS_POSITION_UNCERTAINTY_MM>=g_fixed_clear_z);
            }
            if (g_phase==PHASE_OUT || g_phase==PHASE_PASS || g_phase==PHASE_RETURN || g_phase==PHASE_ADVANCE) assert(ml>=0 && mr>=0);
            assert(fabsf(g_heading)<90);
            phases|=1u<<g_phase;
            result=bypass_step(t,ULTRASONIC_NO_RANGE);
        }
        if(trial==3) {
            assert(phases&(1u<<PHASE_LINE_NUDGE));
            assert(result==BYPASS_FAILED && ml==0 && mr==0);continue;
        }
        if (result!=BYPASS_DONE) fprintf(stderr,"route failed result=%d x=%f y=%f h=%f phases=%x\n",result,g_x,g_z,g_heading,phases);
        assert(result==BYPASS_DONE && ml==0 && mr==0);
        assert(fabsf(g_heading)<=BYPASS_ALIGN_TOL_DEG && fabsf(g_x)<12);
        assert(phases&(1u<<PHASE_PLAN)); assert(phases&(1u<<PHASE_PASS));
        assert(phases&(1u<<PHASE_RETURN));
        assert(phases&(1u<<(dry ? PHASE_FINISH : PHASE_ALIGN)));
        assert(checked_return);
        if(SENSOR_TO_BUMPER_MM==60) {
            if(trial<2) assert(!(phases&(1u<<PHASE_BACKUP)));
            else assert(phases&(1u<<PHASE_BACKUP));
        }
    }
    profile.front_mm=OBSTACLE_STOP_MM;
    black=true;
    assert(bypass_test_begin(0) && !bypass_test_uses_line());
    bypass_cancel();
    black=false;
    assert(bypass_test_begin(0) && !bypass_test_uses_line());
    bypass_cancel();
    // Parallel motor targets are exactly zero heading; drift is bounded.
    assert(planned_begin()); g_x=g_target_x; g_z=g_geometry.departure_y; g_heading=0;
    phase_enter(PHASE_PASS,0); assert(bypass_step(10,1000)==BYPASS_RUNNING && ml==mr);
    g_heading=1.5f; assert(bypass_step(20,1000)==BYPASS_RUNNING && ml<mr);
    g_x=g_target_x-BYPASS_LATERAL_TOL_MM-1;
    assert(bypass_step(30,1000)==BYPASS_FAILED);
    // Both sensors while oblique cannot finish alignment.
    assert(planned_begin()); black=true; g_x=0; g_z=g_fixed_clear_z+200; g_heading=70;
    phase_enter(PHASE_ALIGN,0);
    assert(bypass_step(10,1000)==BYPASS_RUNNING && g_line_ticks==0);
    black=false; left_only=true; g_heading=0; phase_enter(PHASE_ALIGN,0);
    assert(bypass_step(10,1000)==BYPASS_RUNNING && ml==0 && mr==0 && g_line_ticks==0);
    assert(bypass_step(BYPASS_LINE_TIMEOUT_MSEC+1,1000)==BYPASS_FAILED); left_only=false;
    assert(planned_begin()); g_x=0; g_z=g_fixed_clear_z+200; g_heading=4; black=true;
    phase_enter(PHASE_ALIGN,0);
    assert(bypass_step(10,1000)==BYPASS_RUNNING && ml<0 && mr>0);
    g_heading=0; assert(bypass_step(20,1000)==BYPASS_RUNNING && ml==0 && mr==0);
    assert(bypass_step(30,1000)==BYPASS_RUNNING);
    assert(bypass_step(20+BYPASS_TURN_SETTLE_MSEC,1000)==BYPASS_DONE);
    // A small bounded correction is attempted only after the unchanged final
    // pivot has settled. One sensor steers toward the line; BOTH are required.
    assert(planned_begin()); g_x=0; g_z=g_fixed_clear_z+200; g_heading=0;
    black=false; left_only=true; phase_enter(PHASE_ALIGN,0);
    assert(bypass_step(10,1000)==BYPASS_RUNNING && g_phase==PHASE_ALIGN);
    assert(bypass_step(130,1000)==BYPASS_RUNNING && g_phase==PHASE_LINE_NUDGE);
    assert(gb_nudge_used && fabsf(g_target_hdg)<0.01f);
    assert(bypass_step(140,1000)==BYPASS_RUNNING && ml>0 && mr>0 && ml<mr);
    left_only=false; right_only=true;
    assert(bypass_step(150,1000)==BYPASS_RUNNING && ml>0 && mr>0 && ml>mr);
    right_only=false; black=true; g_heading=0;
    assert(bypass_step(160,1000)==BYPASS_RUNNING);
    assert(bypass_step(170,1000)==BYPASS_RUNNING);
    assert(bypass_step(180,1000)==BYPASS_DONE && ml==0 && mr==0);
    // Absence of line is bounded by distance AND time, including sensor waits.
    for(unsigned limit=0;limit<3;++limit) {
        assert(planned_begin()); g_x=0; g_z=g_fixed_clear_z+200; g_heading=0; black=false;
        phase_enter(PHASE_ALIGN,0);
        assert(bypass_step(10,1000)==BYPASS_RUNNING);
        assert(bypass_step(130,1000)==BYPASS_RUNNING && g_phase==PHASE_LINE_NUDGE);
        if(limit==0) g_phase_mm=BYPASS_NUDGE_MAX_MM;
        if(limit==1) ready=false;
        if(limit==2) g_heading=BYPASS_NUDGE_HEADING_DEG+BYPASS_DRIVE_HEADING_TOL_DEG+1;
        assert(bypass_step(limit==1 ? 131+BYPASS_NUDGE_MAX_MSEC : 140,1000)==BYPASS_FAILED);
        assert(ml==0 && mr==0); ready=true;
    }
    // A correction is not allowed while a full turning envelope remains beside the obstacle.
    assert(planned_begin()); black=false; g_x=g_target_x; g_z=g_fixed_clear_z; g_heading=0;
    phase_enter(PHASE_ALIGN,0);
    assert(bypass_step(10,1000)==BYPASS_RUNNING);
    assert(bypass_step(130,1000)==BYPASS_FAILED);
    // Residual turn motion, even during a sensor wait, resets quiet time.
    assert(planned_begin()); g_heading=g_target_hdg;
    assert(bypass_step(10,1000)==BYPASS_RUNNING && g_phase==PHASE_PIVOT);
    ready=false; left_edges++; right_edges++;
    assert(bypass_step(100,1000)==BYPASS_RUNNING && ml==0 && mr==0);
    ready=true;
    assert(bypass_step(130,1000)==BYPASS_RUNNING && g_phase==PHASE_PIVOT);
    assert(bypass_step(100+BYPASS_TURN_SETTLE_MSEC,1000)==BYPASS_RUNNING && g_phase==PHASE_OUT);
    // Translational coast is integrated as translation before a turn,
    // including motion observed while waiting for a sensor measurement.
    assert(planned_begin()); g_x=g_target_x; g_z=g_geometry.departure_y; g_heading=-g_geometry.departure;
    phase_enter(PHASE_OUT,0); start_pivot(0,PHASE_PASS,0);
    assert(g_phase==PHASE_BRAKE);
    float old_x=g_x, old_y=g_z, old_h=g_heading;
    ready=false; left_edges+=100; right_edges+=103;
    assert(bypass_step(100,1000)==BYPASS_RUNNING);
    assert(g_x<old_x && g_z>old_y && fabsf(g_heading-old_h)<0.1f);
    ready=true;
    assert(bypass_step(130,1000)==BYPASS_RUNNING && g_phase==PHASE_BRAKE);
    assert(bypass_step(100+BYPASS_TURN_SETTLE_MSEC,1000)==BYPASS_RUNNING && g_phase==PHASE_PIVOT);
    // Overshoot never triggers an opposite turn/startup boost.
    assert(planned_begin()); g_heading=g_target_hdg-10;
    assert(bypass_step(10,1000)==BYPASS_RUNNING && ml==0 && mr==0);
    assert(bypass_step(10+BYPASS_TURN_SETTLE_MSEC,1000)==BYPASS_FAILED);
    for (int direction=-1;direction<=1;direction+=2) {
        assert(planned_begin()); g_turn_direction=direction; float before=g_heading;
        left_edges+=100; right_edges+=100;
        float mean=(100000.0f/ENC_LEFT_EDGES_PER_M+100000.0f/ENC_RIGHT_EDGES_PER_M)/2;
        float scale=direction<0 ? BYPASS_PIVOT_LEFT_X100 : BYPASS_PIVOT_RIGHT_X100;
        odo_update();
        assert(fabsf(g_heading-before-direction*mean/(WHEEL_TRACK_MM/2.0f)*DEG_PER_RAD*100/scale)<0.001f);
    }
    // Bounded line search and sensor-position advance.
    assert(planned_begin()); black=false; g_x=g_target_x; g_z=g_fixed_clear_z; g_heading=g_geometry.return_deg;
    phase_enter(PHASE_RETURN,0);
    g_phase_mm=g_geometry.offset/sinf((g_geometry.return_deg-BYPASS_ALIGN_TOL_DEG)/DEG_PER_RAD)+BYPASS_RETURN_EXTRA_MM+1;
    assert(bypass_step(10,1000)==BYPASS_FAILED);
    assert(planned_begin()); g_x=0; g_z=g_fixed_clear_z+200; g_heading=0;
    phase_enter(PHASE_RETURN,0); assert(bypass_step(BYPASS_LINE_TIMEOUT_MSEC+1,1000)==BYPASS_FAILED);
    assert(planned_begin()); g_x=-45; g_z=g_fixed_clear_z+200; g_heading=g_geometry.return_deg; black=true;
    phase_enter(PHASE_RETURN,0);
    for (uint32_t t=10;t<=30;t+=10) assert(bypass_step(t,1000)==BYPASS_RUNNING);
    assert(g_phase==PHASE_ADVANCE);
    assert(BYPASS_REACQUIRE_ADVANCE_MM*sinf(g_heading/DEG_PER_RAD)<BYPASS_LINE_CROSS_MAX_MM);
    g_phase_mm=BYPASS_REACQUIRE_ADVANCE_MM-0.1f;
    g_x=g_line_contact_x+g_phase_mm*sinf(g_heading/DEG_PER_RAD);
    assert(bypass_step(40,1000)==BYPASS_RUNNING && g_phase==PHASE_ADVANCE && ml>0 && mr>0);
    g_phase_mm=BYPASS_REACQUIRE_ADVANCE_MM;
    assert(bypass_step(50,1000)==BYPASS_RUNNING && g_phase==PHASE_BRAKE);
    assert(bypass_step(50+BYPASS_TURN_SETTLE_MSEC,1000)==BYPASS_RUNNING && g_phase==PHASE_ALIGN);
    assert(g_target_hdg==0);
    assert(planned_begin()); assert(bypass_step(10,BYPASS_FRONT_STOP_MM)==BYPASS_FAILED);
    assert(planned_begin()); ready=false;
    assert(bypass_step(10,1000)==BYPASS_RUNNING && ml==0 && mr==0);
    assert(bypass_step(BYPASS_STEP_MAX_MSEC+1,1000)==BYPASS_FAILED); ready=true;
    // The planner is cancellable too; no pre-motion blocking search.
    phase_t interrupted[]={PHASE_PLAN,PHASE_BRAKE,PHASE_PIVOT,PHASE_OUT,PHASE_PASS,PHASE_RETURN,PHASE_ADVANCE,PHASE_ALIGN,PHASE_LINE_NUDGE};
    for (unsigned i=0;i<sizeof(interrupted)/sizeof(interrupted[0]);++i) {
        assert(planned_begin()); phase_enter(interrupted[i],0); ml=mr=12;
        bypass_cancel(); assert(g_phase==PHASE_IDLE && ml==0 && mr==0);
    }
}

int main(void) {
    if (BYPASS_FIXED_WIDTH_TEST) { fixed_checks(); return 0; }
    assert(CAR_LENGTH_MM==180 && CAR_WIDTH_MM==140);
    assert(SIDE_CLEARANCE_MM==145); // ceil(hypot(85,70)) + 30.
    assert(BYPASS_ASSUMED_DEPTH_MM==150);
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
    assert(g_pass_min_mm==100+150+180+30);
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
    for offset, fixed, left_scale, right_scale in (
            (-1, 0, 82, 82), (0, 0, 82, 82), (60, 0, 82, 82),
            (-1, 1, 82, 82), (0, 1, 82, 82), (60, 1, 82, 82),
            (60, 1, 76, 90)):

        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        f'-DSENSOR_TO_BUMPER_MM={offset}', f'-DBYPASS_FIXED_WIDTH_TEST={fixed}',
                        f'-DBYPASS_PIVOT_LEFT_X100={left_scale}',
                        f'-DBYPASS_PIVOT_RIGHT_X100={right_scale}',
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

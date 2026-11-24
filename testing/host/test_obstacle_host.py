"""Run the real obstacle scanner against simulated targets."""
from pathlib import Path
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

with tempfile.TemporaryDirectory() as directory:
    work = Path(directory)
    (work / "pico").mkdir()
    (work / "pico/stdlib.h").write_text(
        "#include <stdint.h>\nuint64_t time_us_64(void);\n")
    (work / "pico/time.h").write_text("""
#include "stdlib.h"
static inline uint64_t get_absolute_time(void) { return time_us_64(); }
static inline uint32_t to_ms_since_boot(uint64_t time) { return time / 1000u; }
""")
    (work / "test.c").write_text(r'''
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include "config.h"
#include "obstacle.h"
#include "servo.h"
#include "ultrasonic.h"
#include "vehicle.h"
#include "line_sensor.h"

#include "bypass.h"
static bool mock_bypass_begin_ok;
bool bypass_begin(obstacle_profile_t const *p, uint32_t t, bool l) { (void)p; (void)t; (void)l; return mock_bypass_begin_ok; }
static unsigned mock_bypass_starts;
bool bypass_test_begin(uint32_t t) { (void)t; ++mock_bypass_starts; return true; }
static bool mock_rehearsal_line;
bool bypass_test_uses_line(void) { return mock_rehearsal_line; }
void bypass_cancel(void) {}
static bypass_result_t mock_bypass_result = BYPASS_FAILED;
bypass_result_t bypass_step(uint32_t t, uint32_t r) { (void)t; (void)r; return mock_bypass_result; }
static int32_t mock_height;
int32_t hump_height_x10(void) { return mock_height; }
void hump_task(bool f, bool h) { (void)f; (void)h; }
void hump_finish(int32_t t) { (void)t; }
void hump_new_run(void) {}
int32_t hump_on_mm(void) { return 0; }
int32_t hump_last_height_x10(void) { return 0; }
int32_t hump_run_height_x10(void) { return 0; }
uint32_t hump_count(void) { return 0; }
static uint32_t now, ready_at, sequence, range_mm, invalidations;
static int32_t angle = 90;
static bool measurement_ready;
enum { BACKGROUND, NO_ECHO, WIDE, NOISY_CENTRE, OUTLIER, NOISY_EDGE, ONE_ECHO, TWO_ECHO, UNSTABLE };
static int mode;
static int width = 300, offset;
static uint32_t scan_depth = OBSTACLE_CLEAR_MM - 1u;
static int32_t motor_left, motor_right, tilt;
static bool reader_enabled;
uint64_t time_us_64(void) { return (uint64_t)now * 1000; }
void motor_set(int32_t l, int32_t r) { motor_left = l; motor_right = r; }
int32_t motor_left_speed(void) { return motor_left; }
int32_t motor_right_speed(void) { return motor_right; }
uint32_t encoder_count(uint32_t pin) { (void)pin; return 0u; }
void log_state(char const * format, ...) { (void)format; }
void log_line(char const * format, ...) { (void)format; }
void log_printf(char const * format, ...) { (void)format; }
void barcode_set_hand_mode(bool enabled) { (void)enabled; }
void barcode_set_enabled(bool enabled) { reader_enabled = enabled; }
bool barcode_is_black(void) { return false; }
bool barcode_is_capturing(void) { return false; }
uint32_t barcode_scan_size(void) { return 0u; }
bool barcode_end_scan(void) { return false; }
char const * barcode_command_name(barcode_command_t cmd) { (void)cmd; return "test"; }
int32_t imu_max_tilt_x10(void) { return tilt; }
int32_t imu_total_tilt_x10(void) { return tilt; }
int32_t imu_accel_mg(void) { return 1000; }
int32_t imu_lift_speed_mm_s(void) { return 0; }
int32_t imu_pitch_x10(void) { return tilt; }
int32_t imu_roll_x10(void) { return 0; }
static bool both_line=true, left_line=true;
line_state_t line_sensor_read(void) { return (line_state_t){left_line, both_line}; }
line_junction_t line_junction_detect(bool black, bool enabled, uint32_t time,
                                     uint32_t * mark) {
    (void)black; (void)enabled; (void)time; (void)mark;
    return LINE_JUNCTION_NONE;
}
void log_obstacle(char const * format, ...) { (void)format; }
void servo_set_angle(int32_t value) {
    assert(value >= 60 && value <= 120);
    if (angle != value) {
        angle = value;
        ready_at = now + SERVO_SETTLE_MSEC;
        ultrasonic_invalidate();
    }
}
int32_t servo_angle(void) { return angle; }
bool servo_is_settled(void) { return now >= ready_at; }
void ultrasonic_invalidate(void) { measurement_ready = false; ++invalidations; }
uint32_t ultrasonic_sample_sequence(void) { return sequence; }
bool ultrasonic_measurement_ready(void) { return measurement_ready; }
uint32_t ultrasonic_range_mm(void) { return range_mm; }

static void sample(void) {
    float a = (angle - 90) * 0.017453292519943295f;
    float x = scan_depth * tanf(a);
    bool hit = mode == WIDE || fabsf(x - offset) <= width / 2.0f;
    range_mm = hit ? (uint32_t)lroundf(scan_depth / cosf(a)) :
        (mode == NO_ECHO ? ULTRASONIC_NO_RANGE : (uint32_t)lroundf(2000.0f / cosf(a)));
    if (mode == NOISY_CENTRE || (mode == NOISY_EDGE && !hit)) {
        range_mm = 200 + 200 * (sequence % 3);
    }
    if (mode == OUTLIER && sequence % 3 == 0) { range_mm = 3000; }
    if (mode == ONE_ECHO && sequence % 3 != 0) { range_mm = ULTRASONIC_NO_RANGE; }
    if (mode == TWO_ECHO) {
        if (sequence % 3 == 0) { range_mm = ULTRASONIC_NO_RANGE; }
        else { range_mm += sequence % 3 == 1 ? -10 : 10; }
    }
    if (mode == UNSTABLE) { range_mm = 100 + 200 * (sequence % 3); }
    ++sequence;
    measurement_ready = true;
}

static void run_scan(void) {
    obstacle_scan_start(scan_depth, now);
    assert(obstacle_scan_active());
    // Reading the same completed ping on multiple control ticks must not
    // satisfy the three independent samples required at an angle.
    uint32_t count;
    for (int i = 0; i < 50; ++i) { obstacle_scan_task(now); }
    obstacle_scan_points(&count);
    assert(count == 0);
    uint32_t start = now;
    while (obstacle_scan_active()) {
        now += 10;
        if (servo_is_settled() && now % 60 == 0) { sample(); }
        obstacle_scan_task(now);
        assert(now - start <= OBSTACLE_SCAN_TIMEOUT_MSEC + 10);
    }
    assert(servo_angle() == 90);
    assert(obstacle_profile()->finished);
}


static void estimate_checks(void) {
    obstacle_profile_t p={.width_mm=333};
    obstacle_width_estimate_update(&p);
    assert(p.width_mm==333 && p.corrected_width_mm==0);
}
static void width_calibration_checks(void) {
    for (unsigned distance=150;distance<=350;distance+=25) {
        obstacle_point_t points[31];
        for(unsigned i=0;i<31;i++) {
            int angle=-30+2*(int)i;
            unsigned depth=abs(angle)<=10 ? distance : 1000;
            points[i]=(obstacle_point_t){angle,
                (uint32_t)lroundf(depth/cosf(angle*0.017453292519943295f)),
                OBSTACLE_RETURN_RANGE};
        }
        obstacle_profile_t p;
        obstacle_profile_build(points,31,distance,&p);
        assert(p.edges_confirmed && p.width_mm>0);
        assert(!p.calibrated_width_valid && p.corrected_width_mm==0);
        points[15].kind=OBSTACLE_RETURN_NO_ECHO;
        obstacle_profile_build(points,31,distance,&p);
        assert(!p.centre_seen && !p.left_bounded && !p.right_bounded);
        assert(p.front_mm==ULTRASONIC_NO_RANGE && p.width_mm==0);
    }
}

static void geometry_checks(void) {
    obstacle_point_t a[] = {
        {-4, 800, OBSTACLE_RETURN_RANGE}, {-2, 300, OBSTACLE_RETURN_RANGE},
        {0, 298, OBSTACLE_RETURN_RANGE}, {2, 300, OBSTACLE_RETURN_RANGE},
        {4, 800, OBSTACLE_RETURN_RANGE}
    };
    obstacle_profile_t p;
    obstacle_profile_build(a, 5, 298, &p);
    assert(p.left_bounded && p.right_bounded);
    assert(p.left_angle_x10 == -30 && p.right_angle_x10 == 30);
    assert(p.left_status == OBSTACLE_EDGE_ESTIMATED && !p.edges_confirmed);
    assert(p.width_mm > 25 && p.width_mm < 40 && !p.bypass.available);
    assert(!p.calibrated_width_valid);
    // A SINGLE missing ray bounds an estimate; there is no no-echo streak gate.
    a[0] = (obstacle_point_t){-4, OBSTACLE_POINT_NO_RANGE, OBSTACLE_RETURN_NO_ECHO};
    a[4] = (obstacle_point_t){4, OBSTACLE_POINT_NO_RANGE, OBSTACLE_RETURN_NO_ECHO};
    obstacle_profile_build(a, 5, 298, &p);
    assert(p.left_bounded && p.right_bounded && !p.bypass.available);
    assert(p.left_status == OBSTACLE_EDGE_NO_ECHO && p.right_status == OBSTACLE_EDGE_NO_ECHO);
    assert(p.left_edge.near_angle_deg == -2 && p.left_edge.far_angle_deg == -4);
    // A weak single-echo foreground reading remains usable for geometry.
    a[1].kind = OBSTACLE_RETURN_WEAK_RANGE;
    obstacle_profile_build(a, 5, 298, &p);
    assert(p.left_bounded && p.width_mm > 0 && !p.edges_confirmed);
    // Missing centre uses the pre-scan range for estimation, never clearance.
    a[2] = (obstacle_point_t){0, OBSTACLE_POINT_NO_RANGE, OBSTACLE_RETURN_NO_ECHO};
    obstacle_profile_build(a, 5, 298, &p);
    assert(!p.centre_seen && !p.left_bounded && !p.right_bounded && !p.bypass.available);
    // Every edge is discovered from the scan, including asymmetric observations.
    obstacle_point_t actual[] = {
        {-30,821,OBSTACLE_RETURN_RANGE}, {-28,836,OBSTACLE_RETURN_RANGE},
        {-26,316,OBSTACLE_RETURN_RANGE}, {-24,332,OBSTACLE_RETURN_RANGE},
        {0,298,OBSTACLE_RETURN_RANGE}, {14,355,OBSTACLE_RETURN_RANGE},
        {16,354,OBSTACLE_RETURN_RANGE},
        {18,OBSTACLE_POINT_NO_RANGE,OBSTACLE_RETURN_NO_ECHO}
    };
    obstacle_profile_build(actual, 8, 298, &p);
    assert(p.left_angle_x10 == -270 && p.right_angle_x10 == 170);
    assert(p.width_mm == 249 && !p.edges_confirmed && !p.bypass.available);
    // The same algorithm works with a different pair of physical edges.
    actual[0].angle_deg = -28; actual[1].angle_deg = -26;
    actual[2].angle_deg = -24; actual[3].angle_deg = -22;
    actual[5].angle_deg = 10; actual[6].angle_deg = 12; actual[7].angle_deg = 14;
    obstacle_profile_build(actual, 8, 298, &p);
    assert(p.left_angle_x10 == -250 && p.right_angle_x10 == 130);
    // No distance transition at the range boundary: show a limit estimate.
    obstacle_point_t wide[] = {{-30,340,OBSTACLE_RETURN_RANGE},
        {0,298,OBSTACLE_RETURN_RANGE},{30,340,OBSTACLE_RETURN_RANGE}};
    obstacle_profile_build(wide, 3, 298, &p);
    assert(p.left_status == OBSTACLE_EDGE_SCAN_LIMIT && p.right_status == OBSTACLE_EDGE_SCAN_LIMIT);
    assert(!p.edges_confirmed && !p.bypass.available);
    assert(!p.calibrated_width_valid);
    // Three clean background returns still qualify for the unchanged safety
    // gate; weakening any one must prevent candidate generation.
    obstacle_point_t clean[] = {
        {-8,800,OBSTACLE_RETURN_RANGE},{-6,800,OBSTACLE_RETURN_RANGE},
        {-4,800,OBSTACLE_RETURN_RANGE},{-2,300,OBSTACLE_RETURN_RANGE},
        {0,298,OBSTACLE_RETURN_RANGE},{2,300,OBSTACLE_RETURN_RANGE},
        {4,800,OBSTACLE_RETURN_RANGE},{6,800,OBSTACLE_RETURN_RANGE},
        {8,800,OBSTACLE_RETURN_RANGE}
    };
    obstacle_profile_build(clean, 9, 298, &p);
    assert(p.edges_confirmed && p.bypass.available);
    clean[6].kind = OBSTACLE_RETURN_WEAK_RANGE;
    obstacle_profile_build(clean, 9, 298, &p);
    assert(p.left_bounded && p.right_bounded && !p.edges_confirmed && !p.bypass.available);
    clean[6].kind = OBSTACLE_RETURN_RANGE; clean[7].range_mm = 1600;
    obstacle_profile_build(clean, 9, 298, &p);
    assert(!p.edges_confirmed && !p.bypass.available);
    // A lone jump is displayed immediately even when later readings return
    // to foreground, but it is NOT upgraded to verified clearance.
    clean[6].range_mm = 300; clean[7].range_mm = 300;
    clean[5].range_mm = 800;
    obstacle_profile_build(clean, 9, 298, &p);
    assert(p.right_angle_x10 == 10 && !p.edges_confirmed && !p.bypass.available);
}

int main(void) {
    estimate_checks();
    width_calibration_checks();
    geometry_checks();
    scan_depth = 250; width = 80; offset = 0;
    for (int test = 0; test < 6; ++test) {
        int modes[] = {BACKGROUND, ONE_ECHO, TWO_ECHO, UNSTABLE, NO_ECHO, NOISY_CENTRE};
        mode = modes[test]; run_scan();
        uint32_t count;
        obstacle_point_t const * points = obstacle_scan_points(&count);
        assert(count == 31); // Entire configured 60..120 degree range, even with missing centre.
        for (unsigned i = 0; i < count; ++i) { assert(points[i].angle_deg == -30 + 2 * (int)i); }
        if (mode == ONE_ECHO) {
            assert(points[15].range_mm == scan_depth && points[15].kind == OBSTACLE_RETURN_WEAK_RANGE);
            assert(!obstacle_profile()->centre_seen);
            assert(!obstacle_profile()->left_bounded && !obstacle_profile()->right_bounded);
            assert(!obstacle_profile()->edges_confirmed && !obstacle_profile()->bypass.available);
        }
        if (mode == TWO_ECHO) { assert(points[15].range_mm == scan_depth); }
        if (mode == UNSTABLE) { assert(points[15].range_mm == 300 && points[15].kind == OBSTACLE_RETURN_WEAK_RANGE); }
        if (mode == BACKGROUND) { assert(obstacle_profile()->edges_confirmed); }
        if (mode == NO_ECHO) { assert(!obstacle_profile()->bypass.available); }
    }
    uint32_t before, after;
    obstacle_scan_points(&before); obstacle_scan_cancel(); obstacle_scan_points(&after);
    assert(before == after && !obstacle_scan_active()); // Preserve graph until next scan.
    obstacle_scan_start(250, now); obstacle_scan_points(&after); assert(after == 0);
    now += OBSTACLE_SCAN_TIMEOUT_MSEC; obstacle_scan_task(now);
    assert(obstacle_profile()->timed_out && !obstacle_profile()->bypass.available);
    assert(invalidations > 0);
    // Exercise the real vehicle state machine with the real scanner.

    now = 100; ready_at = 0; mode = NO_ECHO; range_mm = OBSTACLE_STOP_MM + 1;
    vehicle_init(); vehicle_start(); vehicle_task();
    assert(vehicle_state() == VEHICLE_FOLLOW_LINE);
    assert(reader_enabled);
    vehicle_task();
    assert(motor_left > 0 && motor_right > 0);
    range_mm = OBSTACLE_STOP_MM; tilt = HUMP_ENTER_PITCH_X10;
    mock_height=HUMP_ENTER_HEIGHT_X10;
    vehicle_task();
    assert(vehicle_state() == VEHICLE_OBSTACLE_SCAN); // Stop outranks tilt.
    assert(!reader_enabled);
    assert(motor_left == 0 && motor_right == 0);
    tilt = 0; mock_height=0;
    uint32_t scan_start = now;
    while (vehicle_state() == VEHICLE_OBSTACLE_SCAN) {
        now += 10;
        if (servo_is_settled() && now % 60 == 0) { sample(); }
        vehicle_task();
        assert(motor_left == 0 && motor_right == 0);
        assert(now - scan_start <= OBSTACLE_SCAN_TIMEOUT_MSEC + 10);
    }
    assert(vehicle_state() == VEHICLE_BLOCKED);
    assert(obstacle_profile()->finished && !obstacle_profile()->edges_confirmed);
    assert(obstacle_profile()->left_bounded && obstacle_profile()->right_bounded);
    assert(!obstacle_profile()->bypass.available);
    assert(obstacle_profile()->left_status == OBSTACLE_EDGE_NO_ECHO);
    // No echo cannot release BLOCKED; valid distances need sustained clearance.
    range_mm = ULTRASONIC_NO_RANGE;
    for (int i = 0; i < 50; ++i) { now += 10; vehicle_task(); }
    assert(vehicle_state() == VEHICLE_BLOCKED);
    range_mm = OBSTACLE_CLEAR_MM + 1;
    vehicle_task(); now += OBSTACLE_CLEAR_MSEC + 1; vehicle_task();
    assert(vehicle_state() == VEHICLE_FOLLOW_LINE);
    assert(reader_enabled);
    for (uint32_t d = 100; d <= OBSTACLE_STOP_MM; d += 30) {
        vehicle_stop(); vehicle_start(); range_mm = d; vehicle_task();
        assert(vehicle_state() == VEHICLE_OBSTACLE_SCAN);
        assert(motor_left == 0 && motor_right == 0);
    }
    vehicle_stop(); vehicle_start(); range_mm = OBSTACLE_STOP_MM - 1; vehicle_task();
    assert(vehicle_state() == VEHICLE_OBSTACLE_SCAN); // Calibration protected.
    vehicle_stop();
    assert(vehicle_state() == VEHICLE_STOPPED && !obstacle_scan_active());
    assert(!obstacle_profile()->finished && servo_angle() == 90);
    vehicle_barcode_command(BARCODE_CMD_LEFT, true);
    assert(vehicle_state() == VEHICLE_TURN);
    vehicle_task();
    assert(vehicle_state() == VEHICLE_OBSTACLE_SCAN); // Turn protected.
    vehicle_stop(); vehicle_barcode_test(); vehicle_task();
    assert(vehicle_state() == VEHICLE_BLOCKED && !obstacle_scan_active());
    vehicle_stop(); vehicle_task();
    assert(vehicle_state() == VEHICLE_STOPPED && motor_left == 0 && motor_right == 0);
    vehicle_bypass_test();
    assert(vehicle_state() == VEHICLE_BYPASS);
    unsigned starts_before_repeat = mock_bypass_starts;
    vehicle_bypass_test(); vehicle_bypass_test();
    assert(vehicle_state() == VEHICLE_BYPASS && mock_bypass_starts == starts_before_repeat);
    mock_bypass_result = BYPASS_DONE; vehicle_task();
    assert(vehicle_state() == VEHICLE_STOPPED); // Dry run must not start driving straight.
    mock_rehearsal_line=true;
    vehicle_bypass_test();
    assert(vehicle_state()==VEHICLE_BYPASS);
    mock_bypass_result=BYPASS_DONE; vehicle_task();
    assert(vehicle_state()==VEHICLE_STOPPED); // v always ends stopped, even over a line.
    vehicle_stop();
    vehicle_bypass_test(); mock_bypass_result=BYPASS_FAILED; vehicle_task();
    assert(vehicle_state()==VEHICLE_STOPPED);
    mock_rehearsal_line=false;
    mock_bypass_begin_ok = true; mock_bypass_result = BYPASS_RUNNING;
    mode = BACKGROUND; width = 80; scan_depth = OBSTACLE_STOP_MM; range_mm = OBSTACLE_STOP_MM;
    vehicle_start(); vehicle_task();
    assert(vehicle_state() == VEHICLE_OBSTACLE_SCAN);
    uint32_t integration_start = now;
    while (vehicle_state() == VEHICLE_OBSTACLE_SCAN) {
        now += 10;
        if (servo_is_settled() && now % 60 == 0) { sample(); }
        vehicle_task();
        assert(now - integration_start <= OBSTACLE_SCAN_TIMEOUT_MSEC + 10);
    }
    assert(vehicle_state() == VEHICLE_BYPASS);
    range_mm = 80; vehicle_task();
    assert(vehicle_state() == VEHICLE_BYPASS); // No nested detection during return.
    mock_bypass_result = BYPASS_DONE; vehicle_task();
    assert(vehicle_state() == VEHICLE_FOLLOW_LINE && reader_enabled);
    range_mm=OBSTACLE_CLEAR_MM+1;
    vehicle_task();
    assert(vehicle_state()==VEHICLE_FOLLOW_LINE && motor_left>0 && motor_right>0);
    // A single black sensor is a valid normal-follower input. Seed its
    // history and steer immediately instead of the straight-launch delay.
    vehicle_stop(); vehicle_start(); range_mm=OBSTACLE_STOP_MM;
    mock_bypass_result=BYPASS_RUNNING; vehicle_task();
    uint32_t handoff_start=now;
    while(vehicle_state()==VEHICLE_OBSTACLE_SCAN) {
        now+=10;
        if(servo_is_settled() && now%60==0) sample();
        vehicle_task();
        assert(now-handoff_start<=OBSTACLE_SCAN_TIMEOUT_MSEC+10);
    }
    assert(vehicle_state()==VEHICLE_BYPASS);
    both_line=false; mock_bypass_result=BYPASS_DONE; vehicle_task();
    assert(vehicle_state()==VEHICLE_FOLLOW_LINE);
    range_mm=OBSTACLE_CLEAR_MM+1; vehicle_task();
    assert(motor_left>0 && motor_right>0 && motor_left<motor_right);
    vehicle_stop(); both_line=true;
    vehicle_start(); range_mm=OBSTACLE_STOP_MM; mock_bypass_result=BYPASS_RUNNING; vehicle_task();
    handoff_start=now;
    while(vehicle_state()==VEHICLE_OBSTACLE_SCAN) {
        now+=10; if(servo_is_settled() && now%60==0) sample(); vehicle_task();
        assert(now-handoff_start<=OBSTACLE_SCAN_TIMEOUT_MSEC+10);
    }
    assert(vehicle_state()==VEHICLE_BYPASS);
    both_line=false; left_line=false; mock_bypass_result=BYPASS_DONE; vehicle_task();
    assert(vehicle_state()==VEHICLE_STOPPED && motor_left==0 && motor_right==0);
    both_line=left_line=true;
    vehicle_stop();
    return 0;
}
'''.replace("#include <math.h>", "#include <math.h>\n#include <stdlib.h>"))
    for definitions in (["-DTEST_CALIBRATED=0"],
                        ["-DTEST_CALIBRATED=0", "-DOBSTACLE_WIDTH_CALIBRATION_ENABLED=false"],
                        ["-DTEST_CALIBRATED=1",
                         "-DOBSTACLE_WIDTH_CALIBRATION_ROWS={140,160,200,240,360,400},{190,210,200,240,360,400},{240,260,200,240,360,400}"],
                        ["-DTEST_CALIBRATED=0",
                         "-DOBSTACLE_WIDTH_CALIBRATION_ROWS={140,260,200,240,360,400},{140,260,200,240,360,400}"],
                        ["-DTEST_CALIBRATED=0",
                         "-DOBSTACLE_WIDTH_CALIBRATION_ROWS={140,260,200,400,360,400}"]):
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-fsanitize=undefined", "-fno-sanitize-recover=all",
                        *definitions, "-I", str(work),
                        *SRC_INCLUDES,
                        str(work / "test.c"), str(src_path("obstacle.c")),
                        str(src_path("vehicle.c")),
                        "-lm", "-o", str(work / "test")], check=True, timeout=30)
        subprocess.run([str(work / "test")], check=True, timeout=10)
print("Obstacle scan host checks passed (geometry, freshness, cancellation, timeout)")

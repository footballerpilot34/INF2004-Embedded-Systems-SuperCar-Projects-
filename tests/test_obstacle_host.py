"""Run the real coarse/fine scanner against simulated rectangular targets."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as directory:
    work = Path(directory)
    (work / "pico").mkdir()
    (work / "pico/stdlib.h").write_text(
        "#include <stdint.h>\nuint64_t time_us_64(void);\n")
    (work / "test.c").write_text(r'''
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include "config.h"
#include "obstacle.h"
#include "servo.h"
#include "ultrasonic.h"
#include "vehicle.h"
#include "line_sensor.h"

static uint32_t now, ready_at, sequence, range_mm, invalidations;
static int32_t angle = 90;
static bool measurement_ready;
static int mode; // 0 background echo, 1 no echo, 2 wide, 3 noisy centre.
static int width = 300, offset;
static int32_t motor_left, motor_right, tilt;
uint64_t time_us_64(void) { return (uint64_t)now * 1000; }
void motor_set(int32_t l, int32_t r) { motor_left = l; motor_right = r; }
void log_state(char const * format, ...) { (void)format; }
void log_line(char const * format, ...) { (void)format; }
void log_printf(char const * format, ...) { (void)format; }
void barcode_set_hand_mode(bool enabled) { (void)enabled; }
void barcode_set_enabled(bool enabled) { (void)enabled; }
bool barcode_is_black(void) { return false; }
bool barcode_is_capturing(void) { return false; }
bool barcode_end_scan(void) { return false; }
char const * barcode_command_name(barcode_command_t cmd) { (void)cmd; return "test"; }
int32_t imu_max_tilt_x10(void) { return tilt; }
line_state_t line_sensor_read(void) { return (line_state_t){true, true}; }
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
    float x = 400.0f * tanf(a);
    bool hit = mode == 2 || fabsf(x - offset) <= width / 2.0f;
    range_mm = hit ? (uint32_t)lroundf(400.0f / cosf(a)) :
        (mode == 1 ? ULTRASONIC_NO_RANGE : (uint32_t)lroundf(2000.0f / cosf(a)));
    if (mode == 3) { range_mm = sequence % 2 ? 200 : 400; }
    ++sequence;
    measurement_ready = true;
}

static void run_scan(void) {
    obstacle_scan_start(400, now);
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

int main(void) {
    for (width = 200; width <= 300; width += 50) {
        for (offset = -40; offset <= 40; offset += 40) {
            mode = 0; run_scan();
            obstacle_profile_t const * p = obstacle_profile();
            assert(!p->timed_out && p->centre_seen);
            assert(p->left_bounded && p->right_bounded && p->edges_confirmed);
            assert(fabsf((float)p->width_mm - width) / width <= 0.10f);
            assert(abs(p->left_mm - (offset - width / 2)) <= 10);
            assert(abs(p->right_mm - (offset + width / 2)) <= 10);
            assert(p->width_min_mm <= (uint32_t)width);
            assert(p->width_max_mm >= (uint32_t)width);
            assert(!p->exceeds_expected_width);
        }
    }
    width = 300; offset = 0; mode = 1; run_scan();
    assert(obstacle_profile()->left_bounded && obstacle_profile()->right_bounded);
    assert(!obstacle_profile()->edges_confirmed);
    assert(obstacle_profile()->left_ray_mm == ULTRASONIC_NO_RANGE);
    mode = 2; run_scan(); // Obstacle extends beyond both mechanical limits.
    assert(!obstacle_profile()->left_bounded && !obstacle_profile()->right_bounded);
    mode = 3; run_scan(); // Inconsistent samples are not labelled no-echo/clear.
    assert(!obstacle_profile()->centre_seen);
    obstacle_scan_start(400, now);
    obstacle_scan_cancel();
    assert(!obstacle_scan_active() && !obstacle_profile()->finished);
    assert(servo_angle() == 90);
    // Stuck sensor/task must terminate the scan with an explicit timeout.
    obstacle_scan_start(400, now);
    now += OBSTACLE_SCAN_TIMEOUT_MSEC;
    obstacle_scan_task(now);
    assert(!obstacle_scan_active() && obstacle_profile()->timed_out);
    // Unsigned elapsed-time handling must also work across clock wrap.
    now = UINT32_MAX - 50;
    obstacle_scan_start(400, now);
    now += OBSTACLE_SCAN_TIMEOUT_MSEC;
    obstacle_scan_task(now);
    assert(obstacle_profile()->timed_out);
    assert(invalidations > 0);

    // Exercise the real vehicle state machine with the real scanner.
    now = 100; ready_at = 0; mode = 0; range_mm = 401;
    vehicle_init(); vehicle_start(); vehicle_task();
    assert(vehicle_state() == VEHICLE_FOLLOW_LINE);
    vehicle_task();
    assert(motor_left > 0 && motor_right > 0);
    range_mm = 400; tilt = HUMP_ENTER_PITCH_X10;
    vehicle_task();
    assert(vehicle_state() == VEHICLE_OBSTACLE_SCAN); // Stop outranks tilt.
    assert(motor_left == 0 && motor_right == 0);
    tilt = 0;
    uint32_t scan_start = now;
    while (vehicle_state() == VEHICLE_OBSTACLE_SCAN) {
        now += 10;
        if (servo_is_settled() && now % 60 == 0) { sample(); }
        vehicle_task();
        assert(motor_left == 0 && motor_right == 0);
        assert(now - scan_start <= OBSTACLE_SCAN_TIMEOUT_MSEC + 10);
    }
    assert(vehicle_state() == VEHICLE_BLOCKED);
    assert(obstacle_profile()->finished && obstacle_profile()->edges_confirmed);
    // No echo cannot release BLOCKED; valid distances need sustained clearance.
    range_mm = ULTRASONIC_NO_RANGE;
    for (int i = 0; i < 50; ++i) { now += 10; vehicle_task(); }
    assert(vehicle_state() == VEHICLE_BLOCKED);
    range_mm = OBSTACLE_CLEAR_MM + 1;
    vehicle_task(); now += OBSTACLE_CLEAR_MSEC + 1; vehicle_task();
    assert(vehicle_state() == VEHICLE_FOLLOW_LINE);
    vehicle_stop(); vehicle_start(); range_mm = 399; vehicle_task();
    assert(vehicle_state() == VEHICLE_OBSTACLE_SCAN); // Calibration protected.
    vehicle_stop();
    assert(vehicle_state() == VEHICLE_STOPPED && !obstacle_scan_active());
    assert(!obstacle_profile()->finished && servo_angle() == 90);
    vehicle_barcode_command(BARCODE_CMD_LEFT, true);
    assert(vehicle_state() == VEHICLE_TURN);
    vehicle_task();
    assert(vehicle_state() == VEHICLE_OBSTACLE_SCAN); // Turn protected.
    vehicle_stop(); vehicle_barcode_test(); vehicle_task();
    assert(vehicle_state() == VEHICLE_STOPPED && !obstacle_scan_active());
    vehicle_task(); // STOP remains stopped even with the obstacle still present.
    assert(motor_left == 0 && motor_right == 0);
    return 0;
}
'''.replace("#include <math.h>", "#include <math.h>\n#include <stdlib.h>"))
    subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-I", str(work),
                    "-I", str(ROOT / "src"),
                    str(work / "test.c"), str(ROOT / "src/obstacle.c"),
                    str(ROOT / "src/vehicle.c"),
                    "-lm", "-o", str(work / "test")], check=True, timeout=30)
    subprocess.run([str(work / "test")], check=True, timeout=10)
print("Obstacle scan host checks passed (geometry, freshness, cancellation, timeout)")

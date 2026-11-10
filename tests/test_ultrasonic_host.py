"""Exercise the real ultrasonic driver with simulated GPIO and time.

Run with python3 tests/test_ultrasonic_host.py; requires a host C compiler.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as directory:
    work = Path(directory)
    (work / "pico").mkdir()
    (work / "hardware").mkdir()
    (work / "pico/stdlib.h").write_text("""
#include <stdint.h>
uint64_t time_us_64(void);
uint32_t time_us_32(void);
void sleep_us(uint64_t us);
""")
    (work / "hardware/gpio.h").write_text("""
#include <stdbool.h>
#define GPIO_OUT 1
#define GPIO_IN 0
#define GPIO_IRQ_EDGE_RISE 1
#define GPIO_IRQ_EDGE_FALL 2
void gpio_init(unsigned pin);
void gpio_set_dir(unsigned pin, bool out);
void gpio_put(unsigned pin, bool value);
void gpio_pull_down(unsigned pin);
""")
    (work / "test.c").write_text("""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "ultrasonic.h"
static uint64_t now = 1000;
static unsigned triggers;
uint64_t time_us_64(void) { return now; }
uint32_t time_us_32(void) { return (uint32_t)now; }
void sleep_us(uint64_t us) { now += us; }
void gpio_init(unsigned p) { (void)p; }
void gpio_set_dir(unsigned p, bool v) { (void)p; (void)v; }
void gpio_pull_down(unsigned p) { (void)p; }
void gpio_put(unsigned p, bool v) { (void)p; if (v) ++triggers; }
static void echo(uint32_t mm) {
    now += 200;
    ultrasonic_echo_isr(1);
    now += mm * 58 / 10;
    ultrasonic_echo_isr(2);
    ultrasonic_task();
}
int main(void) {
    _Static_assert(OBSTACLE_STOP_MM == 400, "Stop at 40 cm");
    _Static_assert(OBSTACLE_CLEAR_MM > OBSTACLE_STOP_MM, "Hysteresis");
    ultrasonic_init();
    assert(!ultrasonic_measurement_ready());
    ultrasonic_task();
    echo(400);
    assert(ultrasonic_measurement_ready());
    assert(ultrasonic_range_mm() == 400);
    ultrasonic_invalidate();
    assert(!ultrasonic_measurement_ready());
    assert(ultrasonic_range_mm() == ULTRASONIC_NO_RANGE);
    unsigned old_triggers = triggers;
    ultrasonic_task();
    assert(triggers == old_triggers);
    // Late edges from an abandoned ping must not become a new range.
    ultrasonic_echo_isr(1); now += 2320; ultrasonic_echo_isr(2);
    now += US_PING_PERIOD_USEC;
    ultrasonic_task();
    assert(!ultrasonic_measurement_ready());
    echo(450);
    assert(ultrasonic_range_mm() == 450);
    now += (uint64_t)US_STALE_MSEC * 1000 + 1;
    assert(ultrasonic_range_mm() == ULTRASONIC_NO_RANGE);
    ultrasonic_invalidate();
    now += US_PING_PERIOD_USEC;
    ultrasonic_task();
    now += US_ECHO_TIMEOUT_USEC;
    ultrasonic_task();
    assert(ultrasonic_measurement_ready());
    assert(ultrasonic_range_mm() == ULTRASONIC_NO_RANGE);
    return 0;
}
""")
    subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-I", str(work), "-I", str(ROOT / "src"),
                    str(work / "test.c"), str(ROOT / "src/ultrasonic.c"),
                    "-o", str(work / "test")], check=True)
    subprocess.run([str(work / "test")], check=True)
print("Ultrasonic host checks passed")

"""Feed full Code 39 GPIO edge streams into the actual barcode driver."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as directory:
    work = Path(directory)
    (work / "pico").mkdir()
    (work / "hardware").mkdir()
    (work / "pico/stdlib.h").write_text("""
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
typedef struct { int unused; } repeating_timer_t;
uint32_t time_us_32(void);
bool add_repeating_timer_us(int64_t us, bool (*cb)(repeating_timer_t *),
                            void * data, repeating_timer_t * timer);
""")
    (work / "hardware/gpio.h").write_text("""
#include <stdbool.h>
#define GPIO_IN 0
void gpio_init(unsigned pin);
void gpio_set_dir(unsigned pin, bool out);
void gpio_disable_pulls(unsigned pin);
bool gpio_get(unsigned pin);
""")
    (work / "hardware/adc.h").write_text("""
#include <stdint.h>
void adc_init(void);
void adc_gpio_init(unsigned pin);
void adc_select_input(unsigned input);
uint16_t adc_read(void);
""")
    (work / "test.c").write_text(r'''
#include <assert.h>
#include <string.h>
#include "pico/stdlib.h"
#include "barcode.h"
#include "config.h"
#include "encoder.h"
static uint32_t now = 10000, position;
static bool black;
uint32_t time_us_32(void) { return now; }
uint32_t encoder_position(void) { return position; }
void gpio_init(unsigned p) { (void)p; }
void gpio_set_dir(unsigned p, bool v) { (void)p; (void)v; }
void gpio_disable_pulls(unsigned p) { (void)p; }
bool gpio_get(unsigned p) { (void)p; return black == BARCODE_BLACK_IS_HIGH; }
void adc_init(void) {}
void adc_gpio_init(unsigned p) { (void)p; }
void adc_select_input(unsigned p) { (void)p; }
uint16_t adc_read(void) { return 0; }
bool add_repeating_timer_us(int64_t us, bool (*cb)(repeating_timer_t *),
                            void * data, repeating_timer_t * t) {
    (void)us; (void)cb; (void)data; (void)t; return true;
}
void log_line(char const * format, ...) { (void)format; }

// mode 0 good distance, 1 quantised/distorted distance, 2 stationary hand
// reading, 3 malformed equal-width marks, 4 incomplete barcode.
static barcode_result_t read_code(uint16_t letter, bool reverse, int mode) {
    uint32_t widths[29], n = 0;
    uint16_t symbols[] = {0x094, letter, 0x094};
    for (unsigned ch = 0; ch < 3; ++ch) {
        for (int bit = 8; bit >= 0; --bit) {
            widths[n++] = (symbols[ch] & (1u << bit)) ? 3 : 1;
        }
        if (ch < 2) { widths[n++] = 1; }
    }
    barcode_clear_message();
    barcode_set_enabled(true);
    barcode_set_hand_mode(mode == 2);
    black = true; now += 10000; barcode_edge_isr(0); barcode_task();
    unsigned elements = mode == 4 ? 12 : 29;
    for (unsigned i = 0; i < elements; ++i) {
        uint32_t units = widths[reverse ? 28 - i : i];
        now += (mode == 3 ? 1 : units) * 20000;
        if (mode != 2) {
            position += (mode == 1 || mode == 3 ? 1 : units) * 8 * ENCODER_POS_SCALE;
        }
        black = !black;
        barcode_edge_isr(0);
        barcode_task();
    }
    now += BARCODE_STALL_USEC + 1;
    if (mode != 2) { position += 100 * ENCODER_POS_SCALE; }
    return barcode_task();
}

int main(void) {
    barcode_init();
    for (int mode = 0; mode <= 2; ++mode) {
        for (int reverse = 0; reverse < 2; ++reverse) {
            assert(read_code(0x109, reverse, mode) == BARCODE_DECODED);
            assert(strcmp(barcode_message(), "A") == 0);
            assert(barcode_command() == BARCODE_CMD_LEFT);
            assert(barcode_last_result() == BARCODE_DECODED);
            assert(read_code(0x0d0, reverse, mode) == BARCODE_DECODED);
            assert(strcmp(barcode_message(), "Z") == 0);
            assert(barcode_command() == BARCODE_CMD_RIGHT);
        }
    }
    assert(read_code(0x109, false, 3) == BARCODE_INVALID);
    assert(barcode_last_result() == BARCODE_INVALID);
    assert(read_code(0x109, false, 4) == BARCODE_INVALID);
    barcode_set_enabled(false);
    assert(!barcode_is_enabled());
    uint32_t previous = barcode_edge_count();
    black = !black; now += 10000; barcode_edge_isr(0);
    barcode_task();
    assert(barcode_edge_count() == previous + 1);
    assert(!barcode_is_capturing());
    barcode_set_enabled(true);
    assert(barcode_is_enabled());
    assert(read_code(0x109, false, 0) == BARCODE_DECODED);
    return 0;
}
''')
    subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-I", str(work), "-I", str(ROOT / "src"),
                    str(work / "test.c"), str(ROOT / "src/barcode.c"),
                    "-o", str(work / "test")], check=True, timeout=30)
    subprocess.run([str(work / "test")], check=True, timeout=10)
print("Barcode host checks passed (GPIO streams, distance/time, reverse, malformed, re-enable)")

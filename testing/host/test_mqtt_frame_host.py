"""Check the REAL bit packer and binary MQTT frames (change requirement 1).

Compiles app_program/bitpack.c and app_program/comms/mqtt_frame.c with a
small C driver, then checks every frame byte for byte against the Python
reference decoder/encoder in tools/mqtt_frame.py (the one the laptop's
tools use), so the car and the laptop agree on every layout.

Run with python3 testing/host/test_mqtt_frame_host.py; requires a host C
compiler invoked as cc.
"""
import importlib.util
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]

SRC_DIRS = [ROOT / d for d in (
    'config', 'app_program', 'app_program/line_barcode',
    'app_program/motion', 'app_program/obstacle', 'app_program/comms',
    'app_program/imu_hump')]
SRC_INCLUDES = [flag for d in SRC_DIRS for flag in ('-I', str(d))]


def src_path(name):
    """Return the path of a project source file, wherever it now lives."""
    return next(d / name for d in SRC_DIRS if (d / name).exists())


spec = importlib.util.spec_from_file_location(
    'mqtt_frame', ROOT / 'tools' / 'mqtt_frame.py')
ref = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ref)

# The same readings for C and Python, including values at and past the
# ends of each field's range.
TELE = {'t_msec': 429306, 'state': 11, 'bypass_phase': 4,
        'motor_left': -100, 'motor_right': 100, 'enc_left': 24721,
        'enc_right': 25744, 'line_left': 1, 'line_right': 0, 'barcode': 1,
        'range_mm': 1455, 'pitch_x10': -1800, 'roll_x10': 1799,
        'servo_deg': 180, 'hump_x10': -1024, 'accel_mg': 8191,
        'lift_mm_s': -2048, 'pending_turn': 4, 'faults': 0xA5}
BEAT = {'seq': 65535, 'up_s': 429, 'vsys_mv': 4987, 'sent': 309,
        'dropped': 20, 'connects': 1, 'wifi_mode': 2, 'hotspot': 1,
        'ip': 0x060A14AC, 'rssi_dbm': -70, 'ping_ok': 1, 'ping_msec': 30,
        'ping_age_s': 1, 'link_drops': 0, 'restart': 0x2D, 'faults': 0x10,
        'loop_max_us': 812, 'overruns': 3}
SCAN = {'t_msec': 1, 'front_mm': 300, 'left_mm': -150,
        'right_mm': ref.NO_MM, 'mid_mm': 0, 'width_mm': 0, 'geometric': 0,
        'left_status': 3, 'right_status': 1}
HUMP = {'t_msec': 325089, 'height_x10': 100, 'tilt_x10': 105,
        'run_height_x10': 100, 'run_tilt_x10': 105, 'count': 1}
# Every opcode's word (the longest, "calibrate", fills the C word buffer),
# plus letter and feature commands with arguments.
COMMANDS = ([w for w in ref.WORDS if w not in ('bc', 'ft')] +
            ['bcal', 'bczn', 'ft1011', 'ft0000'])

code = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "bitpack.h"
#include "mqtt_frame.h"

static void hex(char const *name, uint8_t const *data, uint32_t length) {
    printf("%s ", name);
    for (uint32_t i = 0; i < length; i++) { printf("%02x", data[i]); }
    printf("\n");
}

static void bitpack_checks(void) {
    uint8_t buf[8];
    bitpack_t p;
    memset(buf, 0xFF, sizeof(buf));
    bitpack_init(&p, buf, sizeof(buf));
    bitpack_put(&p, 5u, 3u);             // 101
    bitpack_put_signed(&p, -1, 4u);      // 1111
    bitpack_put_signed(&p, -300, 8u);    // Held at -128.
    bitpack_put_sat(&p, 9999u, 12u);     // Held at 4095.
    bitpack_put(&p, 0x1FFFFu, 16u);      // Counter: low 16 bits.
    bitpack_put(&p, 0u, 5u);             // Clears the 0xFF already there.
    assert(bitpack_bytes(&p) == 6u && !p.b_overrun);
    bitpack_init(&p, buf, sizeof(buf));
    assert(bitpack_get(&p, 3u) == 5u);
    assert(bitpack_get_signed(&p, 4u) == -1);
    assert(bitpack_get_signed(&p, 8u) == -128);
    assert(bitpack_get(&p, 12u) == 4095u);
    assert(bitpack_get(&p, 16u) == 0xFFFFu);
    assert(bitpack_get(&p, 5u) == 0u);
    assert(bitpack_max(1u) == 1u && bitpack_max(32u) == UINT32_MAX);
    // A field that does not fit is not written, and is reported.
    bitpack_init(&p, buf, 1u);
    bitpack_put(&p, 1u, 6u);
    bitpack_put(&p, 3u, 3u);
    assert(p.b_overrun && bitpack_bytes(&p) == 1u);
    bitpack_init(&p, buf, 1u);
    (void)bitpack_get(&p, 9u);
    assert(p.b_overrun);
}

int main(void) {
    uint8_t out[MQTT_FRAME_MAX_BYTES];
    uint32_t n;
    bitpack_checks();

    mqtt_tele_t tele = {
        .t_msec = 429306u, .state = 11u, .bypass_phase = 4u,
        .motor_left = -100, .motor_right = 100, .enc_left = 24721u,
        .enc_right = 25744u, .b_line_left = true, .b_line_right = false,
        .b_barcode = true, .range_mm = 1455u, .pitch_x10 = -1800,
        .roll_x10 = 1799, .servo_deg = 180u, .hump_x10 = -1024,
        .accel_mg = 8191u, .lift_mm_s = -2048, .pending_turn = 4u,
        .faults = 0xA5u};
    n = mqtt_frame_telemetry(&tele, out, sizeof(out));
    assert(n == 24u); hex("telemetry", out, n);
    tele.range_mm = 9999u; tele.pitch_x10 = -9999;    // Held, not wrapped.
    n = mqtt_frame_telemetry(&tele, out, sizeof(out)); hex("held", out, n);
    assert(mqtt_frame_telemetry(&tele, out, 23u) == 0u); // Too small.

    mqtt_heartbeat_t beat = {
        .seq = 65535u, .up_s = 429u, .vsys_mv = 4987u, .sent = 309u,
        .dropped = 20u, .connects = 1u, .wifi_mode = 2u, .hotspot = 1u,
        .ip = 0x060A14ACu, .rssi_dbm = -70, .b_ping_ok = true,
        .ping_msec = 30u, .ping_age_s = 1u, .link_drops = 0u,
        .restart = 0x2Du, .faults = 0x10u, .loop_max_us = 812u,
        .overruns = 3u};
    n = mqtt_frame_heartbeat(&beat, out, sizeof(out));
    assert(n == 28u); hex("heartbeat", out, n);

    mqtt_obstacle_t scan = {
        .t_msec = 1u, .front_mm = 300u, .left_mm = -150,
        .right_mm = MQTT_FRAME_NO_MM, .mid_mm = 0, .width_mm = 0u,
        .b_geometric = false, .left_status = 3u, .right_status = 1u};
    n = mqtt_frame_obstacle(&scan, out, sizeof(out));
    assert(n == 14u); hex("obstacle", out, n);

    mqtt_hump_t hump = {.t_msec = 325089u, .height_x10 = 100,
                        .tilt_x10 = 105, .run_height_x10 = 100,
                        .run_tilt_x10 = 105, .count = 1u};
    n = mqtt_frame_hump(&hump, out, sizeof(out));
    assert(n == 12u); hex("hump", out, n);

    n = mqtt_frame_barcode(7u, 1u, "hello-1", out, sizeof(out));
    hex("barcode", out, n);
    assert(mqtt_frame_code39_index('A') == 10u);
    assert(mqtt_frame_code39_index('a') == 10u);
    assert(mqtt_frame_code39_index('#') == 63u);

    uint8_t log_out[16];
    n = mqtt_frame_log(3u, "State: A -> B", log_out, sizeof(log_out));
    assert(n == 15u && log_out[0] == 0xA6u && log_out[1] == 3u);
    assert(memcmp(log_out + 2, "State: A -> B", 13u) == 0);
    n = mqtt_frame_log(1u, "a much longer line", log_out, sizeof(log_out));
    assert(n == sizeof(log_out)); // Cut to fit.

    // Commands (Python-built frames, below) and their acknowledgements.
    static struct { char const *word; uint8_t frame[4]; uint32_t len; } const cmds[] = {
''' + ',\n'.join('        {"%s", {%s}, %d}' % (
        w, ', '.join('0x%02xu' % b for b in ref.encode_command(w)),
        len(ref.encode_command(w))) for w in COMMANDS) + r'''
    };
    for (uint32_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        char word[MQTT_FRAME_WORD_BYTES];
        uint8_t ack[8];
        assert(mqtt_frame_command(cmds[i].frame, cmds[i].len, word, sizeof(word)));
        assert(strcmp(word, cmds[i].word) == 0);
        assert(mqtt_frame_ack(cmds[i].frame, cmds[i].len, ack, sizeof(ack)) == cmds[i].len);
        assert(ack[0] == 0xA8u && memcmp(ack + 1, cmds[i].frame + 1, cmds[i].len - 1u) == 0);
    }
    // Not command frames: old text, wrong type or version, bad opcode,
    // a letter past Z, a truncated frame, a too-small word buffer.
    char word[MQTT_FRAME_WORD_BYTES];
    uint8_t const text[] = {'s', 't'};
    uint8_t const typed[] = {0xA1u, 0x01u};
    uint8_t const old[] = {0xB7u, 0x01u};
    uint8_t const opcode[] = {0xA7u, 0x1Fu};
    uint8_t const letter[] = {0xA7u, 0x4Fu, 0x03u}; // Letter 26.
    uint8_t const cut[] = {0xA7u, 0x0Fu};
    assert(!mqtt_frame_command(text, 2u, word, sizeof(word)));
    assert(!mqtt_frame_command(typed, 2u, word, sizeof(word)));
    assert(!mqtt_frame_command(old, 2u, word, sizeof(word)));
    assert(!mqtt_frame_command(opcode, 2u, word, sizeof(word)));
    assert(!mqtt_frame_command(letter, 3u, word, sizeof(word)));
    assert(!mqtt_frame_command(cut, 2u, word, sizeof(word)));
    assert(!mqtt_frame_command(typed, 1u, word, sizeof(word)));
    assert(!mqtt_frame_command(cmds[0].frame, cmds[0].len, word, 4u));
    return 0;
}
'''

with tempfile.TemporaryDirectory() as directory:
    work = Path(directory)
    (work / 'test.c').write_text(code)
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=undefined', '-fno-sanitize-recover=all',
                    *SRC_INCLUDES, str(work / 'test.c'),
                    str(src_path('bitpack.c')), str(src_path('mqtt_frame.c')),
                    '-o', str(work / 'test')], check=True, timeout=30)
    result = subprocess.run([str(work / 'test')], check=True, timeout=10,
                            capture_output=True, text=True)

frames = dict(line.split() for line in result.stdout.splitlines())
expected = {
    'telemetry': ref.encode('telemetry', TELE),
    'held': ref.encode('telemetry', dict(TELE, range_mm=9999,
                                         pitch_x10=-9999)),
    'heartbeat': ref.encode('heartbeat', BEAT),
    'obstacle': ref.encode('obstacle', SCAN),
    'hump': ref.encode('hump', HUMP),
    'barcode': ref.encode_barcode(7, 1, 'hello-1'),
}
for name, data in expected.items():
    assert frames[name] == data.hex(), (name, frames[name], data.hex())
assert ref.decode(bytes.fromhex(frames['telemetry'])) == ('telemetry', TELE)
held = ref.decode(bytes.fromhex(frames['held']))[1]
assert held['range_mm'] == 4095 and held['pitch_x10'] == -2048
assert ref.decode(bytes.fromhex(frames['barcode']))[1]['message'] == 'HELLO-1'
ref.selftest()
print('MQTT frame host checks passed (C packer == Python reference)')

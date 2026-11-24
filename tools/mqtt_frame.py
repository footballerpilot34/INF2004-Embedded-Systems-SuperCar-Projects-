#!/usr/bin/env python3
"""Binary MQTT frames of the SuperCar (change requirement 1).

The reference for the layouts in app_program/comms/mqtt_frame.h: decode
any frame the car sends, and build the command frames it accepts.

    python tools/mqtt_frame.py --decode TOPIC HEX   decode one payload
    python tools/mqtt_frame.py --cmd WORD           command frame, as hex
    python tools/mqtt_frame.py --selftest           round-trip checks

Byte 0 is the header (type in bits 0-3, version 0xA in bits 4-7); the
fields follow LSB first, no padding (app_program/bitpack.h). Keep this
file, mqtt_frame.c and tools/mqtt_car.ps1 in step.
"""

import sys

VERSION = 0xA
TYPES = {1: 'telemetry', 2: 'heartbeat', 3: 'barcode', 4: 'obstacle',
         5: 'hump', 6: 'log', 7: 'cmd', 8: 'cmd/ack'}
TYPE_OF = {name: code for code, name in TYPES.items()}

# (name, bits, signed)
TELEMETRY = [('t_msec', 32, False), ('state', 4, False),
             ('bypass_phase', 4, False), ('motor_left', 8, True),
             ('motor_right', 8, True), ('enc_left', 16, False),
             ('enc_right', 16, False), ('line_left', 1, False),
             ('line_right', 1, False), ('barcode', 1, False),
             ('range_mm', 12, False), ('pitch_x10', 12, True),
             ('roll_x10', 12, True), ('servo_deg', 8, False),
             ('hump_x10', 11, True), ('accel_mg', 13, False),
             ('lift_mm_s', 12, True), ('pending_turn', 3, False),
             ('faults', 8, False)]
HEARTBEAT = [('seq', 16, False), ('up_s', 24, False), ('vsys_mv', 13, False),
             ('sent', 16, False), ('dropped', 16, False),
             ('connects', 8, False), ('wifi_mode', 3, False),
             ('hotspot', 3, False), ('ip', 32, False), ('rssi_dbm', 8, True),
             ('ping_ok', 1, False), ('ping_msec', 10, False),
             ('ping_age_s', 8, False), ('link_drops', 8, False),
             ('restart', 8, False), ('faults', 8, False),
             ('loop_max_us', 16, False), ('overruns', 16, False)]
OBSTACLE = [('t_msec', 32, False), ('front_mm', 12, False),
            ('left_mm', 13, True), ('right_mm', 13, True),
            ('mid_mm', 13, True), ('width_mm', 12, False),
            ('geometric', 1, False), ('left_status', 3, False),
            ('right_status', 3, False)]
HUMP = [('t_msec', 32, False), ('height_x10', 11, True),
        ('tilt_x10', 12, True), ('run_height_x10', 11, True),
        ('run_tilt_x10', 12, True), ('count', 8, False)]
LAYOUTS = {'telemetry': TELEMETRY, 'heartbeat': HEARTBEAT,
           'obstacle': OBSTACLE, 'hump': HUMP}

NO_MM = -4096
CODE39 = '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*'
STATES = ['READY', 'FOLLOW_LINE', 'PROBE', 'HUMP', 'BLOCKED', 'STOPPED',
          'LINE_CAL', 'TURN', 'FEATURE_TEST', 'BARCODE_STATIC',
          'OBSTACLE_SCAN', 'BYPASS']
PHASES = ['idle', 'pivot', 'out', 're-scan', 'pass', 'return', 'advance',
          'align', 'finish', 'brake', 'plan', 'line-nudge', 'back-up']
TURNS = ['MESSAGE', 'TURN LEFT', 'TURN RIGHT', 'U-TURN', 'GO STRAIGHT']
EDGES = ['edge not observed', 'missing echoes', 'unreliable observations',
         'echo-bracketed', 'estimated at scan limit',
         'estimated distance jump']
LOG_CATS = ['system', 'obstacle', 'line', 'state', 'servo']
FAULTS = ['motor stall', 'encoder', 'IMU', 'no echo', 'Wi-Fi loss',
          'MQTT down', 'battery low', 'loop overrun']
WIFI_MODES = ['off', 'joining', 'joined', 'rejoining', 'own network']
RESTARTS = ['unknown', 'power-on or brown-out', 'reset button',
            'debugger restart', 'software reset', 'watchdog (froze)',
            'CPU hard fault', 'network stack check failed']
STAGES = ['unknown', 'Wi-Fi chip/network polling', 'web server',
          'hotspot joining', 'battery reading', 'buttons', 'serial keys',
          'running a command', 'servo', 'ultrasonic', 'IMU', 'barcode',
          'vehicle control', 'obstacle scan test', 'IR diag', 'buzzer',
          'telemetry', 'MQTT', 'waiting for the next tick', 'fault check']

# Command words by opcode (the web page's words); see mqtt_frame.h.
WORDS = ['stop', 'start', 'calibrate', 'wheel', 'diag', 'black', 'white',
         'bt', 'bs', 'scan', 'v', 'mark', 'clear', 'bclist', 'bcreset',
         'bc', 'ft']
ACTIONS = 'nlrsu'


class Packer:
    """Bit writer, LSB first (the same as bitpack.c)."""

    def __init__(self):
        self.bits = []

    def put(self, value, width, signed=False):
        if signed:
            high = (1 << (width - 1)) - 1
            value = max(-high - 1, min(high, value))
            value &= (1 << width) - 1
        else:
            value = max(0, value) & ((1 << width) - 1)
        self.bits.extend((value >> i) & 1 for i in range(width))

    def put_sat(self, value, width):
        self.put(min(max(0, value), (1 << width) - 1), width)

    def data(self):
        out = bytearray((len(self.bits) + 7) // 8)
        for i, bit in enumerate(self.bits):
            out[i // 8] |= bit << (i % 8)
        return bytes(out)


class Unpacker:
    """Bit reader, LSB first."""

    def __init__(self, data):
        self.data = data
        self.pos = 0

    def get(self, width, signed=False):
        if self.pos + width > len(self.data) * 8:
            raise ValueError('frame too short')
        value = 0
        for i in range(width):
            bit = (self.data[(self.pos + i) // 8] >> ((self.pos + i) % 8)) & 1
            value |= bit << i
        self.pos += width
        if signed and value & (1 << (width - 1)):
            value -= 1 << width
        return value


def header(kind):
    return (VERSION << 4) | TYPE_OF[kind]


def encode(kind, fields):
    """Build a fixed-layout frame from a dict (used by the tests)."""
    pack = Packer()
    pack.put(TYPE_OF[kind], 4)
    pack.put(VERSION, 4)
    for name, width, signed in LAYOUTS[kind]:
        value = int(fields.get(name, 0))
        if signed:
            pack.put(value, width, True)
        elif name in ('enc_left', 'enc_right', 'seq', 'sent', 'dropped',
                      't_msec', 'ip', 'faults', 'restart', 'connects',
                      'ping_age_s', 'link_drops', 'loop_max_us', 'overruns',
                      'line_left', 'line_right', 'barcode', 'ping_ok',
                      'geometric'):
            pack.put(value, width)
        else:
            pack.put_sat(value, width)
    return pack.data()


def encode_barcode(t_msec, command, message):
    pack = Packer()
    pack.put(TYPE_OF['barcode'], 4)
    pack.put(VERSION, 4)
    pack.put(t_msec, 32)
    pack.put_sat(command, 3)
    message = message[:31]
    pack.put(len(message), 5)
    for char in message:
        upper = char.upper()
        pack.put(CODE39.index(upper) if upper in CODE39 else 63, 6)
    return pack.data()


def encode_command(word):
    """The binary command frame for a web-page command word, or None."""
    word = word.strip().lower().replace(' ', '')
    pack = Packer()
    pack.put(TYPE_OF['cmd'], 4)
    pack.put(VERSION, 4)
    if len(word) == 4 and word.startswith('bc') and word[2].isalpha() and \
            word[3] in ACTIONS and word not in WORDS:
        pack.put(WORDS.index('bc'), 5)
        pack.put(ord(word[2]) - ord('a'), 5)
        pack.put(ACTIONS.index(word[3]), 3)
    elif word.startswith('ft') and len(word) in (5, 6) and \
            set(word[2:]) <= {'0', '1'}:
        digits = word[2:] + ('1' if len(word) == 5 else '')
        pack.put(WORDS.index('ft'), 5)
        for digit in digits:
            pack.put(int(digit), 1)
    elif word in WORDS and word not in ('bc', 'ft'):
        pack.put(WORDS.index(word), 5)
    else:
        return None
    return pack.data()


def command_word(data):
    """The web-page word a command (or ack) frame carries."""
    unpack = Unpacker(data)
    unpack.get(8)
    opcode = unpack.get(5)
    if opcode >= len(WORDS):
        raise ValueError('bad opcode')
    word = WORDS[opcode]
    if word == 'bc':
        letter = unpack.get(5)
        action = unpack.get(3)
        word = 'bc' + chr(ord('a') + letter) + ACTIONS[action]
    elif word == 'ft':
        word = 'ft' + ''.join(str(unpack.get(1)) for _ in range(4))
    return word


def fault_names(mask):
    names = [FAULTS[i] for i in range(8) if mask & (1 << i)]
    return ', '.join(names) if names else 'none'


def decode(data):
    """Decode one payload: (kind, dict of fields)."""
    if not data:
        raise ValueError('empty')
    if (data[0] >> 4) != VERSION or (data[0] & 0xF) not in TYPES:
        raise ValueError(f'not a frame (header 0x{data[0]:02x})')
    kind = TYPES[data[0] & 0xF]
    unpack = Unpacker(data)
    unpack.get(8)
    if kind in LAYOUTS:
        fields = {name: unpack.get(width, signed)
                  for name, width, signed in LAYOUTS[kind]}
    elif kind == 'barcode':
        fields = {'t_msec': unpack.get(32), 'command': unpack.get(3)}
        length = unpack.get(5)
        codes = [unpack.get(6) for _ in range(length)]
        fields['message'] = ''.join(CODE39[c] if c < len(CODE39) else '?'
                                    for c in codes)
    elif kind == 'log':
        fields = {'category': unpack.get(3)}
        fields['text'] = bytes(data[2:]).decode('utf-8', 'replace')
    else:
        fields = {'word': command_word(data)}
    return kind, fields


def name(table, index):
    return table[index] if 0 <= index < len(table) else f'?{index}'


def describe(kind, f):
    """One line for people."""
    if kind == 'telemetry':
        state = name(STATES, f['state'])
        if state == 'BYPASS':
            state += '/' + name(PHASES, f['bypass_phase'])
        rng = f['range_mm'] or '-'
        text = (f"{state:<13} motors {f['motor_left']:4} {f['motor_right']:4}"
                f"  IR {f['line_left']}{f['line_right']}{f['barcode']}"
                f"  range {rng:>5}  pitch {f['pitch_x10'] / 10:5.1f}"
                f"  roll {f['roll_x10'] / 10:5.1f}"
                f"  height {f['hump_x10'] / 10:4.1f} mm"
                f"  servo {f['servo_deg']:3}  enc {f['enc_left']}/"
                f"{f['enc_right']}  am {f['accel_mg']}  vz {f['lift_mm_s']}")
        if f['pending_turn']:
            text += '  next ' + name(TURNS, f['pending_turn'])
        if f['faults']:
            text += '  FAULTS: ' + fault_names(f['faults'])
        return text
    if kind == 'heartbeat':
        ip = '.'.join(str((f['ip'] >> s) & 0xFF) for s in (0, 8, 16, 24))
        restart = f['restart'] & 7
        why = name(RESTARTS, restart)
        if restart in (5, 6):
            why += ' in ' + name(STAGES, f['restart'] >> 3)
        return (f"heartbeat #{f['seq']}  up {f['up_s']} s  battery "
                f"{f['vsys_mv']} mV  sent {f['sent']}  dropped "
                f"{f['dropped']}  connections {f['connects']}  Wi-Fi "
                f"{name(WIFI_MODES, f['wifi_mode'])} #{f['hotspot'] + 1} "
                f"{ip} {f['rssi_dbm']} dBm ping "
                f"{f['ping_msec'] if f['ping_ok'] else '-'} ms  loop max "
                f"{f['loop_max_us']} us ({f['overruns']} overruns)  "
                f"faults: {fault_names(f['faults'])}  last restart: {why}")
    if kind == 'barcode':
        return f"BARCODE '{f['message']}' -> {name(TURNS, f['command'])}"
    if kind == 'obstacle':
        def mm(value, none):
            return '-' if value == none else str(value)
        return (f"OBSTACLE {mm(f['front_mm'], 0)} mm ahead, "
                f"{mm(f['width_mm'], 0)} mm wide (edges "
                f"{mm(f['left_mm'], NO_MM)} / {mm(f['right_mm'], NO_MM)} mm, "
                f"{name(EDGES, f['left_status'])} / "
                f"{name(EDGES, f['right_status'])})")
    if kind == 'hump':
        return (f"HUMP peak {f['height_x10'] / 10:.1f} mm, tilt "
                f"{f['tilt_x10'] / 10:.1f} deg (highest this run "
                f"{f['run_height_x10'] / 10:.1f} mm, {f['count']} humps)")
    if kind == 'log':
        return f"[{name(LOG_CATS, f['category'])}] {f['text']}"
    return f"{kind}: {f['word']}"


def selftest():
    tele = {'t_msec': 429306, 'state': 11, 'bypass_phase': 4,
            'motor_left': -100, 'motor_right': 100, 'enc_left': 24721,
            'enc_right': 25744, 'line_left': 1, 'line_right': 0,
            'barcode': 1, 'range_mm': 1455, 'pitch_x10': -1800,
            'roll_x10': 1799, 'servo_deg': 180, 'hump_x10': -1024,
            'accel_mg': 8191, 'lift_mm_s': -2048, 'pending_turn': 4,
            'faults': 0xA5}
    beat = {'seq': 65535, 'up_s': 429, 'vsys_mv': 4987, 'sent': 309,
            'dropped': 20, 'connects': 1, 'wifi_mode': 2, 'hotspot': 1,
            'ip': 0x060A14AC, 'rssi_dbm': -70, 'ping_ok': 1,
            'ping_msec': 30, 'ping_age_s': 1, 'link_drops': 0,
            'restart': 1, 'faults': 0, 'loop_max_us': 812, 'overruns': 3}
    scan = {'t_msec': 1, 'front_mm': 300, 'left_mm': -150,
            'right_mm': NO_MM, 'mid_mm': 0, 'width_mm': 0, 'geometric': 0,
            'left_status': 3, 'right_status': 1}
    hump = {'t_msec': 325089, 'height_x10': 100, 'tilt_x10': 105,
            'run_height_x10': 100, 'run_tilt_x10': 105, 'count': 1}
    expected_sizes = {'telemetry': 24, 'heartbeat': 28, 'obstacle': 14,
                      'hump': 12}
    for kind, fields in (('telemetry', tele), ('heartbeat', beat),
                         ('obstacle', scan), ('hump', hump)):
        data = encode(kind, fields)
        assert len(data) == expected_sizes[kind], (kind, len(data))
        got_kind, got = decode(data)
        assert got_kind == kind and got == fields, (kind, got)
    # Held (saturated) values.
    data = encode('telemetry', dict(tele, range_mm=9999, pitch_x10=-9999))
    got = decode(data)[1]
    assert got['range_mm'] == 4095 and got['pitch_x10'] == -2048
    # Barcode: 6 bits per character.
    data = encode_barcode(7, 1, 'hello-1')
    assert len(data) == 1 + (32 + 3 + 5 + 6 * 7 + 7) // 8
    assert decode(data) == ('barcode', {'t_msec': 7, 'command': 1,
                                        'message': 'HELLO-1'})
    # Commands: every word round-trips; text is never a frame.
    for word in WORDS:
        if word in ('bc', 'ft'):
            continue
        data = encode_command(word)
        assert len(data) == 2 and decode(data) == ('cmd', {'word': word})
    for word in ('bcal', 'bczn', 'ft1011', 'ft0000'):
        data = encode_command(word)
        assert decode(data)[1]['word'] == word, word
    assert decode(encode_command('ft110'))[1]['word'] == 'ft1101'
    assert encode_command('mb1.2.3.4') is None
    assert encode_command('bogus') is None
    for text in (b'{"t":1}', b'start', b'1'):
        try:
            decode(text)
            raise AssertionError('text decoded as a frame')
        except ValueError:
            pass
    print('mqtt_frame.py self-test passed')


def main():
    if '--selftest' in sys.argv:
        selftest()
    elif '--cmd' in sys.argv:
        data = encode_command(' '.join(sys.argv[sys.argv.index('--cmd') + 1:]))
        print(data.hex() if data else 'unknown command')
    elif '--decode' in sys.argv:
        hex_text = sys.argv[sys.argv.index('--decode') + 2]
        kind, fields = decode(bytes.fromhex(hex_text))
        print(describe(kind, fields))
    else:
        print(__doc__)


if __name__ == '__main__':
    main()

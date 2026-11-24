#!/usr/bin/env python3
"""MQTT message sizes from a saved log (change requirement 1).

Reads a log written by tools/mqtt_car.ps1 (one "HH:MM:SS.mmm topic payload"
line per message) and prints, per topic, how many messages there were and
their payload size, plus the size of the whole MQTT PUBLISH packet on the
wire (fixed header + topic + packet id for QoS 1 + payload).

Binary payloads are logged as "x:" followed by hex (and the decoded text
after " | "), so both the old (JSON text) and the new (binary) format are
measured the same way.

Also: python tools/mqtt_size_report.py --frames
    prints the binary frame sizes from the layouts (tools/mqtt_frame.py),
    for comparison before the car has been run with the new firmware.

Usage:
    python tools/mqtt_size_report.py LOG [--since HH:MM:SS] [--grep TEXT]
"""

import sys
from collections import defaultdict

QOS1_TOPICS = {'barcode', 'obstacle', 'hump', 'cmd/ack', 'online', 'cmd'}


def remaining_length_bytes(length):
    """Bytes MQTT needs to encode a remaining length."""
    count = 1
    while length > 127:
        length //= 128
        count += 1
    return count


def payload_bytes(payload):
    """Size of one logged payload in bytes. A binary payload is logged as
    "x:" + hex, then " | " and the decoded text (tools/mqtt_car.ps1)."""
    if payload.startswith('x:'):
        return len(payload[2:].split(' ', 1)[0]) // 2
    return len(payload.encode('utf-8'))


def load_frames():
    """Import tools/mqtt_frame.py (the frame reference)."""
    import importlib.util
    from pathlib import Path
    spec = importlib.util.spec_from_file_location(
        'mqtt_frame', Path(__file__).with_name('mqtt_frame.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def index_of(table, name):
    return table.index(name) if name in table else 0


def as_int(value, scale=1):
    """A JSON number (or null) as an int, scaled."""
    if value is None:
        return 0
    return int(round(float(value) * scale))


def binary_payload(frames, topic, payload):
    """The binary frame the new firmware sends for an old JSON payload, or
    None if there is no equivalent (e.g. a text "mb" broker command)."""
    import json
    if topic == 'online':
        return payload.encode('utf-8')
    if topic in ('cmd', 'cmd/ack'):
        return frames.encode_command(payload)
    try:
        d = json.loads(payload)
    except ValueError:
        if topic == 'log':
            return bytes(2) + payload.encode('utf-8')
        return None
    if topic == 'telemetry':
        rng = as_int(d.get('rg'))
        return frames.encode('telemetry', {
            't_msec': as_int(d.get('t')),
            'state': index_of(frames.STATES, d.get('st')),
            'bypass_phase': index_of(frames.PHASES, d.get('bp')),
            'motor_left': as_int(d.get('ml')),
            'motor_right': as_int(d.get('mr')),
            'enc_left': as_int(d.get('el')), 'enc_right': as_int(d.get('er')),
            'line_left': as_int(d.get('l')), 'line_right': as_int(d.get('r')),
            'barcode': as_int(d.get('b')),
            'range_mm': max(0, rng), 'pitch_x10': as_int(d.get('pt')),
            'roll_x10': as_int(d.get('rl')), 'servo_deg': as_int(d.get('sv')),
            'hump_x10': as_int(d.get('hh'), 10),
            'accel_mg': as_int(d.get('am')),
            'lift_mm_s': as_int(d.get('vz')),
            'pending_turn': index_of(frames.TURNS, d.get('pend')),
            'faults': 0})
    if topic == 'heartbeat':
        return frames.encode('heartbeat', {
            'seq': as_int(d.get('seq')), 'up_s': as_int(d.get('up')),
            'vsys_mv': as_int(d.get('vb')), 'sent': as_int(d.get('sent')),
            'dropped': as_int(d.get('drop')),
            'connects': as_int(d.get('conn'))})
    if topic == 'barcode':
        return frames.encode_barcode(as_int(d.get('t')),
                                     index_of(frames.TURNS, d.get('cmd')),
                                     str(d.get('msg', '')))
    if topic == 'obstacle':
        return frames.encode('obstacle', {
            't_msec': as_int(d.get('t')),
            'front_mm': as_int(d.get('front')),
            'width_mm': as_int(d.get('width'))})
    if topic == 'hump':
        return frames.encode('hump', {
            't_msec': as_int(d.get('t')),
            'height_x10': as_int(d.get('height_mm'), 10),
            'tilt_x10': as_int(d.get('peak_tilt_x10')),
            'run_height_x10': as_int(d.get('run_height_mm'), 10),
            'run_tilt_x10': as_int(d.get('run_peak_tilt_x10')),
            'count': as_int(d.get('n'))})
    if topic == 'log':
        return bytes(2) + str(d.get('m', '')).encode('utf-8')
    return None


def wire_bytes(topic_full, topic, size):
    """A whole MQTT PUBLISH packet carrying a payload of this size."""
    body = 2 + len(topic_full) + size
    if topic in QOS1_TOPICS:
        body += 2
    return 1 + remaining_length_bytes(body) + body


def convert(path):
    """Old (JSON text) vs new (binary) sizes for every logged message."""
    frames = load_frames()
    old = defaultdict(list)
    new = defaultdict(list)
    old_wire = defaultdict(int)
    new_wire = defaultdict(int)
    skipped = defaultdict(int)
    with open(path, encoding='utf-8', errors='replace') as handle:
        for line in handle:
            parts = line.rstrip('\r\n').split(' ', 2)
            if len(parts) < 3 or '/' not in parts[1] or \
                    parts[2].startswith('x:'):
                continue
            topic_full = parts[1]
            topic = topic_full.split('supercar/', 1)[-1]
            data = binary_payload(frames, topic, parts[2])
            if data is None:
                skipped[topic] += 1
                continue
            size_old = len(parts[2].encode('utf-8'))
            old[topic].append(size_old)
            new[topic].append(len(data))
            old_wire[topic] += wire_bytes(topic_full, topic, size_old)
            new_wire[topic] += wire_bytes(topic_full, topic, len(data))

    print(f'{"topic":<12}{"count":>8}{"JSON mean":>11}{"binary mean":>13}'
          f'{"payload saved":>15}{"wire saved":>12}')
    for topic in sorted(old):
        mean_old = sum(old[topic]) / len(old[topic])
        mean_new = sum(new[topic]) / len(new[topic])
        print(f'{topic:<12}{len(old[topic]):>8}{mean_old:>11.1f}'
              f'{mean_new:>13.1f}{100 * (1 - mean_new / mean_old):>14.1f}%'
              f'{100 * (1 - new_wire[topic] / old_wire[topic]):>11.1f}%')
    total_old = sum(sum(v) for v in old.values())
    total_new = sum(sum(v) for v in new.values())
    print(f'{"all":<12}{sum(len(v) for v in old.values()):>8}'
          f'{"":>11}{"":>13}{100 * (1 - total_new / total_old):>14.1f}%'
          f'{100 * (1 - sum(new_wire.values()) / sum(old_wire.values())):>11.1f}%')
    print(f'payload bytes: {total_old} JSON -> {total_new} binary; '
          f'on the wire: {sum(old_wire.values())} -> {sum(new_wire.values())}')
    for topic, count in sorted(skipped.items()):
        print(f'not convertible ({topic}): {count}')


def main():
    if '--convert' in sys.argv:
        convert(sys.argv[sys.argv.index('--convert') + 1])
        return
    if '--frames' in sys.argv:
        frames = load_frames()
        for kind in ('telemetry', 'heartbeat', 'obstacle', 'hump'):
            print(f'{kind:<10}{len(frames.encode(kind, {})):>4} bytes')
        print(f'{"barcode":<10} 6 bytes + 6 bits per character (e.g. 5 chars: '
              f'{len(frames.encode_barcode(0, 0, "HELLO"))} bytes)')
        print(f'{"log":<10} 2 + the line\'s text')
        print(f'{"command":<10} 2 bytes (3 for set-letter and features)')
        return
    path = sys.argv[1]
    keep = None
    if '--grep' in sys.argv:
        keep = sys.argv[sys.argv.index('--grep') + 1]
    sizes = defaultdict(list)
    wire = defaultdict(list)
    with open(path, encoding='utf-8', errors='replace') as handle:
        for line in handle:
            parts = line.rstrip('\r\n').split(' ', 2)
            if len(parts) < 3 or '/' not in parts[1]:
                continue
            topic_full = parts[1]
            topic = topic_full.split('supercar/', 1)[-1]
            if keep and (keep not in parts[2]) and (topic == 'telemetry'):
                continue
            size = payload_bytes(parts[2])
            body = 2 + len(topic_full) + size
            if topic in QOS1_TOPICS:
                body += 2
            sizes[topic].append(size)
            wire[topic].append(1 + remaining_length_bytes(body) + body)

    print(f'{"topic":<12}{"count":>8}{"min":>6}{"mean":>8}{"max":>6}'
          f'{"wire mean":>11}')
    for topic in sorted(sizes):
        values = sizes[topic]
        print(f'{topic:<12}{len(values):>8}{min(values):>6}'
              f'{sum(values) / len(values):>8.1f}{max(values):>6}'
              f'{sum(wire[topic]) / len(wire[topic]):>11.1f}')


if __name__ == '__main__':
    main()

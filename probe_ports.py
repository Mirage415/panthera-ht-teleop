"""Query-only USB/CAN discovery using the official SDK's CDC framing.

Only MOTOR_VERSION (0x0b) and MOTOR_STATE2 (0x0a) are transmitted.
No SDK Robot construction, enable, position, torque, reset or zero commands.
"""
import json
import struct
import time
from pathlib import Path

import serial


def crc(data, initial, polynomial):
    value = initial
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (polynomial if value & 1 else 0)
    return value


def packet(command, payload=b'\x7f'):
    if command not in (0x0b, 0x0a):
        raise ValueError('Discovery only allows version/state queries')
    header = struct.pack('<BH', command, len(payload))
    return b'\xf7' + header + bytes([crc(header, 255, 0x8c)]) + struct.pack('<H', crc(payload, 65535, 0x8408)) + payload


def frames(buffer):
    result = []
    while len(buffer) >= 7:
        if buffer[0] != 0xf7 or crc(buffer[1:4], 255, 0x8c) != buffer[4]:
            del buffer[0]
            continue
        length = int.from_bytes(buffer[2:4], 'little')
        if length > 256:
            del buffer[0]
            continue
        if len(buffer) < 7 + length:
            break
        payload = bytes(buffer[7:7 + length])
        if crc(payload, 65535, 0x8408) == int.from_bytes(buffer[5:7], 'little'):
            result.append((buffer[1], payload))
        del buffer[:7 + length]
    return result


def query(path, duration=0.4):
    versions, states = {}, {}
    port = serial.Serial(str(path), 4000000, timeout=0.02, write_timeout=0.2, exclusive=True)
    try:
        port.reset_input_buffer()
        buf = bytearray()
        end = time.monotonic() + duration
        next_send = 0
        while time.monotonic() < end:
            if time.monotonic() >= next_send:
                port.write(packet(0x0b))
                next_send = time.monotonic() + 0.1
            buf.extend(port.read(max(1, port.in_waiting)))
            for command, payload in frames(buf):
                if command == 0x0b and len(payload) % 4 == 0:
                    for mid, major, minor, patch in struct.iter_unpack('<BBBB', payload):
                        if major:
                            versions[mid] = f'{major}.{minor}.{patch}'
                if command == 0x0a and len(payload) % 9 == 0:
                    for mid, mode, fault, pos, vel, tqe in struct.iter_unpack('<BBBhhh', payload):
                        states[mid] = dict(mode=mode, fault=fault, raw_position=pos, raw_velocity=vel, raw_torque=tqe)
    finally:
        port.reset_output_buffer()
        port.close()
    return dict(versions=versions, states=states)


if __name__ == '__main__':
    report = {}
    config = json.loads(Path(__file__).with_name('teleop-config.json').read_text())
    for role in ('leader', 'follower'):
        selected = Path(config[role])
        if selected.parent != Path('/dev/serial/by-path') or ':1.' not in selected.name:
            raise ValueError(f'{role}: configure a verified /dev/serial/by-path CAN interface')
        report[role] = {}
        pattern = selected.name.rsplit(':1.', 1)[0] + ':1.*'
        for path in sorted(selected.parent.glob(pattern)):
            print('Query', role, str(path.resolve()), flush=True)
            try:
                found = query(path)
            except serial.SerialException as exc:
                found = {'error': str(exc)}
            report[role][str(path)] = dict(device=str(path.resolve()), **found)
            print(role, path.name, str(path.resolve()), json.dumps(found), flush=True)
    Path(__file__).with_name('port-probe.json').write_text(json.dumps(report, indent=2) + '\n')

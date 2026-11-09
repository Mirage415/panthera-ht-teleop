"""Shared sample validation and CSV schema; capture runs inside Viser."""
import numpy as np
from telemetry import NAMES

CSV_FIELDS = ['timestamp_ns', 'monotonic_ns', 'session', 'sequence'] + [
    f'{name}_{field}' for field in ('rad', 'rad_s', 'nm', 'feedback_time_s') for name in NAMES]
CSV_FIELDS += [f'{name}_action_rad' for name in NAMES]

# Explicit schema for recording and offline data readers.
JOINT_SCHEMA = {
    'type': 'object',
    'properties': {
        **{key: {'type': 'array', 'items': {'type': 'number'},
                 'minItems': 7, 'maxItems': 7}
           for key in ('position', 'velocity', 'effort', 'feedback_time_s', 'action', 'leader_position')},
        'name': {'type': 'array', 'items': {'type': 'string'}},
        **{key: {'type': 'integer'} for key in
           ('timestamp_ns', 'monotonic_ns', 'sequence', 'supervisor_pid')},
        **{key: {'type': 'string'} for key in
           ('session', 'position_unit', 'action_unit', 'action_source')},
    },
}


def validate_joint(row):
    if row['name'] != NAMES or row['position_unit'] != 'rad':
        raise ValueError('unexpected joints or units')
    for key in ('position', 'velocity', 'effort', 'feedback_time_s'):
        values = np.asarray(row[key], dtype=float)
        if values.shape != (7,) or not np.isfinite(values).all():
            raise ValueError(f'invalid {key}')
    for key in ('timestamp_ns', 'monotonic_ns', 'sequence'):
        if type(row[key]) is not int or row[key] < 0:
            raise ValueError(f'invalid {key}')
    if not isinstance(row['session'], str) or not row['session']:
        raise ValueError('invalid session')
    if 'action' in row:
        values = np.asarray(row['action'], dtype=float)
        if values.shape != (7,) or not np.isfinite(values).all() or row.get('action_unit') != 'rad':
            raise ValueError('invalid action')
    if 'leader_position' in row:
        values = np.asarray(row['leader_position'], dtype=float)
        if values.shape != (7,) or not np.isfinite(values).all():
            raise ValueError('invalid leader_position')
    return row


def csv_row(row):
    return [row[k] for k in CSV_FIELDS[:4]] + sum(
        [row[k] for k in ('position', 'velocity', 'effort', 'feedback_time_s')], []) + row.get('action', [''] * 7)


def camera_latency_ms(frame_time_ms, domain, received_ns):
    # Only global_time is a device capture timestamp mapped onto the host clock.
    # system_time may be assigned on arrival, so cannot measure capture latency.
    if str(domain).split('.')[-1] != 'global_time':
        return None
    latency = received_ns / 1e6 - frame_time_ms
    return round(latency, 2) if np.isfinite(latency) and 0 <= latency <= 5000 else None

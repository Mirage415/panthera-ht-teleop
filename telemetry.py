"""Best-effort loopback telemetry; never owns or commands a motor."""
import json
import os
import socket
import time
import uuid

PORT = 9877
NAMES = [f'joint_{i}' for i in range(1, 7)] + ['gripper']


class Publisher:
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.session = uuid.uuid4().hex
        self.sequence = 0

    def send(self, state, feedback_times, action=None, leader_position=None):
        row = dict(session=self.session, sequence=self.sequence,
                   supervisor_pid=os.getppid(),
                   timestamp_ns=time.time_ns(), monotonic_ns=time.monotonic_ns(),
                   name=NAMES, position=state[:, 0].tolist(),
                   velocity=state[:, 1].tolist(), effort=state[:, 2].tolist(),
                   feedback_time_s=feedback_times, position_unit='rad')
        if action is not None:
            row.update(action=action.tolist(), action_source='follower_command_target', action_unit='rad')
        if leader_position is not None:
            row['leader_position'] = leader_position.tolist()
        self.sequence += 1
        try:
            self.socket.sendto(json.dumps(row, allow_nan=False).encode(), ('127.0.0.1', PORT))
        except OSError:
            pass  # Acquisition must never block the motor control loop.

    def close(self):
        self.socket.close()

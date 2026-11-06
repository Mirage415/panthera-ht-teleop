"""Telemetry round trip and persisted timestamps; no camera or motors opened."""
import csv
import io
import json
from pathlib import Path
import socket
import tempfile
import time
import unittest
from unittest.mock import patch

import foxglove
from mcap.reader import make_reader
import numpy as np

from capture import CSV_FIELDS, csv_row, validate_joint, joint_channel, camera_latency_ms
import pyrealsense2 as rs
import telemetry


class CaptureTests(unittest.TestCase):
    def test_camera_latency_requires_comparable_device_clock(self):
        self.assertEqual(camera_latency_ms(1000, rs.timestamp_domain.global_time, 1012500000), 12.5)
        for domain in (rs.timestamp_domain.hardware_clock, rs.timestamp_domain.system_time):
            self.assertIsNone(camera_latency_ms(1000, domain, 1012500000))
        self.assertIsNone(camera_latency_ms(2000, rs.timestamp_domain.global_time, 1012500000))

    def test_transport_and_recording_preserve_feedback(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1', 0))
            receiver.settimeout(1)
            with patch.object(telemetry, 'PORT', receiver.getsockname()[1]):
                publisher = telemetry.Publisher()
                self.addCleanup(publisher.close)
                values = np.arange(21, dtype=float).reshape(7, 3) / 10
                stamps = [time.time() - .01 * i for i in range(7)]
                publisher.send(values, stamps)
                row = validate_joint(json.loads(receiver.recv(65535)))
                self.assertEqual(row['position'], values[:, 0].tolist())
                self.assertEqual(row['feedback_time_s'], stamps)
                publisher.send(values, stamps)
                second = validate_joint(json.loads(receiver.recv(65535)))
                self.assertEqual(second['sequence'], row['sequence'] + 1)
        output = io.StringIO()
        writer = csv.writer(output)
        writer.writerow(CSV_FIELDS)
        writer.writerow(csv_row(row))
        output.seek(0)
        saved = next(csv.DictReader(output))
        self.assertEqual(int(saved['timestamp_ns']), row['timestamp_ns'])
        self.assertEqual(float(saved['gripper_rad']), values[6, 0])
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'test.mcap'
            context = foxglove.Context()
            channel = joint_channel(context)
            with foxglove.open_mcap(str(path), context=context):
                channel.log(row, log_time=row['timestamp_ns'])
            with path.open('rb') as handle:
                messages = list(make_reader(handle).iter_messages())
            self.assertEqual(len(messages), 1)
            schema = messages[0][0]
            self.assertEqual(schema.name, 'panthera.JointState')
            self.assertEqual(json.loads(schema.data)['properties']['position']['items'],
                             {'type': 'number'})
            self.assertEqual(messages[0][2].log_time, row['timestamp_ns'])
            self.assertEqual(json.loads(messages[0][2].data), row)

    def test_reject_malformed_feedback(self):
        for positions in ([1] * 6, [float('nan')] * 7):
            with self.assertRaises(ValueError):
                validate_joint(dict(name=telemetry.NAMES, position_unit='rad', position=positions))

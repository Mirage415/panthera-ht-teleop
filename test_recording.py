"""Episode boundaries, repeated page clicks and service queue behavior."""
from concurrent.futures import ThreadPoolExecutor
import csv
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import foxglove
from mcap.reader import make_reader

from recording import Recorder, RecordingControl


class RecordingTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        self.context = foxglove.Context()
        self.channel = foxglove.Channel('/follower/joint_states', message_encoding='json', context=self.context)
        self.stats = dict(rgb_frames=0, joint_samples=0)
        self.recorder = Recorder(self.root, {}, ['q'], context=self.context)
        self.addCleanup(lambda: self.recorder.stop(self.stats))

    def sample(self, q):
        self.channel.log({'q': q})
        self.recorder.write_joint([q])
        self.stats['joint_samples'] += 1
        self.stats['rgb_frames'] += 1

    def test_multiple_episodes_no_idle_data_and_exact_csv_counts(self):
        self.sample(-1)
        self.assertEqual(list(self.root.iterdir()), [])
        for q in (1, 2):
            started = self.recorder.start(self.stats)
            duplicate = self.recorder.start(self.stats)
            self.assertEqual(started['episode_path'], duplicate['episode_path'])
            self.sample(q)
            stopped = self.recorder.stop(self.stats)
            self.assertFalse(stopped['recording'])
            self.assertTrue(stopped['last_episode_complete'])
            self.assertEqual(stopped['last_episode_statistics']['joint_samples'], 1)
            self.assertEqual(self.recorder.stop(self.stats), stopped)
            self.sample(-2)
            folder = Path(stopped['last_episode_path'])
            with (folder / 'data.mcap').open('rb') as f:
                values = [json.loads(m.data) for _, _, m in make_reader(f).iter_messages()]
            self.assertEqual(values, [{'q': q}])
            with (folder / 'joints.csv').open() as f:
                self.assertEqual(list(csv.reader(f)), [['q'], [str(q)]])
            self.assertTrue(json.loads((folder / 'metadata.json').read_text())['complete'])
            self.assertTrue(self.recorder.decide('saved')['ok'])
        self.assertEqual(len(list(self.root.iterdir())), 2)

    def test_interrupted_recording_incomplete(self):
        self.recorder.start(self.stats)
        self.sample(1)
        result = self.recorder.stop(self.stats, reason='shutdown', error='camera disconnected')
        self.assertFalse(result['last_episode_complete'])
        self.assertIn('camera disconnected', result['recording_error'])

    def test_mcap_close_failure_reported(self):
        self.recorder.start(self.stats)
        self.sample(1)
        real_writer = self.recorder.writer
        self.recorder.writer = SimpleNamespace(close=lambda: (_ for _ in ()).throw(OSError('disk full')))
        result = self.recorder.stop(self.stats)
        real_writer.close()
        self.assertFalse(result['last_episode_complete'])
        self.assertIn('disk full', result['recording_error'])

    def call(self, control, action, live):
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(control.handler(action), SimpleNamespace(encoding='json', payload=b'{}'))
            deadline = time.monotonic() + 2
            while control.queue.empty() and time.monotonic() < deadline:
                time.sleep(.001)
            control.drain(self.recorder, self.stats, live)
            return json.loads(future.result(timeout=2))

    def test_page_start_requires_live_streams(self):
        control = RecordingControl()
        result = self.call(control, 'start', dict(camera_live=True, follower_live=False))
        self.assertFalse(result['ok'])
        self.assertFalse(self.recorder.active)
        self.assertEqual(list(self.root.iterdir()), [])
        result = self.call(control, 'start', dict(camera_live=True, follower_live=True))
        self.assertTrue(result['ok'])
        self.assertTrue(result['recording'])
        self.sample(1)
        result = self.call(control, 'stop', dict(camera_live=True, follower_live=True))
        self.assertTrue(result['last_episode_complete'])

    def test_expired_start_never_runs_later(self):
        control = RecordingControl(timeout=.01)
        with self.assertRaisesRegex(RuntimeError, '超时'):
            control.handler('start')(SimpleNamespace(encoding='json', payload=b'{}'))
        control.drain(self.recorder, self.stats, dict(camera_live=True, follower_live=True))
        self.assertFalse(self.recorder.active)

    def test_failed_open_does_not_leave_active_recording(self):
        with patch('recording.foxglove.open_mcap', side_effect=OSError('cannot write')):
            with self.assertRaisesRegex(OSError, 'cannot write'):
                self.recorder.start(self.stats)
        self.assertFalse(self.recorder.active)
        self.assertIsNone(self.recorder.csv_file)
        self.assertFalse(self.recorder.last_result['complete'])

    def test_stop_requires_review_and_discard_is_durable(self):
        self.recorder.start(self.stats)
        self.sample(1)
        self.recorder.stop(self.stats)
        self.assertTrue(self.recorder.has_pending)
        with self.assertRaisesRegex(ValueError, 'Save'):
            self.recorder.start(self.stats)
        restored = Recorder(self.root, {}, ['q'], context=self.context)
        self.assertTrue(restored.has_pending)
        self.assertTrue(restored.decide('discarded')['ok'])
        metadata = json.loads((Path(restored.last_path) / 'metadata.json').read_text())
        self.assertEqual(metadata['review_status'], 'discarded')
        self.assertTrue((Path(restored.last_path) / 'data.mcap').exists())
        self.assertFalse(restored.decide('saved')['ok'])
        self.assertFalse(Recorder(self.root, {}, ['q']).has_pending)

    def test_save_button_closes_active_episode_and_accepts(self):
        self.recorder.start(self.stats)
        self.sample(1)
        result = self.call(RecordingControl(), 'save', dict(camera_live=True, follower_live=True))
        self.assertTrue(result['ok'])
        self.assertFalse(result['recording'])
        self.assertEqual(result['review_status'], 'saved')
        self.assertFalse(self.recorder.decide('discarded')['ok'])

    def test_cannot_save_incomplete_episode(self):
        self.recorder.start(self.stats)
        self.recorder.stop(self.stats)
        self.assertFalse(self.recorder.decide('saved')['ok'])
        self.assertTrue(self.recorder.has_pending)
        self.assertTrue(self.recorder.decide('discarded')['ok'])

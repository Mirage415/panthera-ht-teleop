"""Page-controlled Teleop lifecycle; never launches the real motor program."""
from pathlib import Path
import signal
import tempfile
import unittest
from unittest.mock import Mock, patch

from teleop_session import TeleopSession


class TeleopSessionTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.session = TeleopSession(Path(temp.name))
        self.recorder = Mock(active=False, error='', has_pending=False)
        self.stats = dict(rgb_frames=1, joint_samples=1)
        self.live = dict(camera_live=True, follower_live=True, follower_supervisor_pid=123)
        self.process = Mock(pid=123)
        self.process.poll.return_value = None
        self.process.wait.return_value = 0
        launch = patch('teleop_session.subprocess.Popen', return_value=self.process)
        self.launch = launch.start()
        self.addCleanup(launch.stop)
        self.addCleanup(lambda: self.session.close(self.recorder, self.stats))

    def start(self):
        return self.session.start(self.recorder, self.stats, self.live)

    def test_start_waits_for_owned_feedback_and_is_idempotent(self):
        self.assertTrue(self.start()['ok'])
        self.assertEqual(self.session.phase, 'starting')
        self.start()
        self.launch.assert_called_once()
        self.session.tick(self.recorder, self.stats, dict(self.live, follower_supervisor_pid=999))
        self.recorder.start.assert_not_called()
        self.session.tick(self.recorder, self.stats, self.live)
        self.recorder.start.assert_called_once_with(self.stats)
        self.assertEqual(self.session.phase, 'running')
        self.session.tick(self.recorder, self.stats, self.live)
        self.recorder.start.assert_called_once()
        self.assertIn('--owner-pid', self.launch.call_args.args[0])

    def test_stop_cancels_startup_and_blocks_restart_until_reaped(self):
        self.start()
        self.session.stop(self.recorder, self.stats)
        self.process.send_signal.assert_called_once_with(signal.SIGTERM)
        self.assertFalse(self.start()['ok'])
        self.session.tick(self.recorder, self.stats, self.live)
        self.recorder.start.assert_not_called()
        self.process.poll.return_value = 0
        self.session.tick(self.recorder, self.stats, self.live)
        self.assertEqual(self.session.phase, 'idle')
        self.assertIsNone(self.session.process)

    def test_failed_preflight_is_visible_and_does_not_record(self):
        self.start()
        self.session.log.write('端口未连接\n')
        self.session.log.flush()
        self.process.poll.return_value = 1
        self.session.tick(self.recorder, self.stats, self.live)
        self.assertEqual(self.session.phase, 'error')
        self.assertIn('端口未连接', self.session.error)
        self.recorder.start.assert_not_called()

    def test_recording_failure_stops_motor_process(self):
        self.start()
        self.recorder.start.side_effect = OSError('disk full')
        self.session.tick(self.recorder, self.stats, self.live)
        self.assertEqual(self.session.phase, 'stopping')
        self.process.send_signal.assert_called_once_with(signal.SIGTERM)
        self.assertIn('disk full', self.session.error)

    def test_feedback_loss_stops_and_marks_incomplete(self):
        self.start()
        self.session.tick(self.recorder, self.stats, self.live)
        self.session.tick(self.recorder, self.stats, dict(self.live, follower_live=False))
        self.assertEqual(self.session.phase, 'stopping')
        self.process.send_signal.assert_called_once_with(signal.SIGTERM)
        self.assertTrue(self.recorder.stop.call_args.kwargs['error'])

    def test_startup_timeout_stops(self):
        self.start()
        self.session.deadline = 0
        self.session.tick(self.recorder, self.stats, dict(self.live, follower_live=False))
        self.assertEqual(self.session.phase, 'stopping')
        self.assertIn('超时', self.session.error)

    def test_camera_not_ready_does_not_launch(self):
        result = self.session.start(self.recorder, self.stats, dict(self.live, camera_live=False))
        self.assertFalse(result['ok'])
        self.launch.assert_not_called()

    def test_reset_never_records_and_requires_completion_marker(self):
        self.session.reset(dict(unit='rad', leader=[0]*7, follower=[0]*7))
        self.session.tick(self.recorder, self.stats, self.live)
        self.recorder.start.assert_not_called()
        self.process.poll.return_value = 0
        self.session.tick(self.recorder, self.stats, self.live)
        self.assertFalse(self.session.reset_completed)
        self.assertEqual(self.session.phase, 'error')

    def test_successful_reset_and_cancelled_reset_are_distinct(self):
        for cancelled in (False, True):
            self.process.poll.return_value = None
            self.session.reset(dict(unit='rad', leader=[0]*7, follower=[0]*7))
            self.session.reset_target.with_suffix('.done').touch()
            if cancelled:
                self.session.stop(self.recorder, self.stats)
            self.process.poll.return_value = 0
            self.session.tick(self.recorder, self.stats, self.live)
            self.assertEqual(self.session.reset_completed, not cancelled)
            self.recorder.start.assert_not_called()

    def test_supervisor_stops_worker_when_capture_owner_exits(self):
        import teleop
        from types import SimpleNamespace
        worker = Mock(pid=456)
        worker.poll.return_value = None
        with patch.object(teleop.subprocess, 'Popen', return_value=worker), \
             patch.object(teleop.os, 'getppid', return_value=1), \
             patch.object(teleop.os, 'killpg') as kill, \
             patch.object(teleop, 'RUNTIME', self.session.root):
            with self.assertRaisesRegex(RuntimeError, '页面采集服务已退出'):
                teleop.supervise(SimpleNamespace(owner_pid=999))
        kill.assert_called_once_with(456, signal.SIGINT)
        worker.wait.assert_called_once_with(timeout=2)

    def test_single_arm_reset_is_independent_of_teleop_feedback_and_recording(self):
        self.session.reset(dict(unit='rad', follower=[0]*7), role='follower')
        command=self.launch.call_args.args[0]
        self.assertEqual(command[command.index('--role')+1], 'follower')
        self.assertIn('--reset-target',command)
        self.session.tick(self.recorder,self.stats,dict(camera_live=False,follower_live=False))
        self.assertEqual(self.session.state()['teleop_state'],'idle')
        self.assertEqual(self.session.state()['reset_state'],'resetting')
        self.recorder.start.assert_not_called()
        self.process.poll.return_value=1
        self.session.tick(self.recorder,self.stats,{})
        self.assertIn('follower 回位失败',self.session.state()['reset_error'])
        self.assertEqual(self.session.state()['teleop_error'],'')
        self.recorder.stop.assert_not_called()

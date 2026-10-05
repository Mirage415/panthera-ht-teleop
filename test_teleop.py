"""Software safety checks; no physical devices are opened."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np

import teleop
from probe_ports import frames, packet


class FakeArm:
    def __init__(self):
        self.position = np.zeros(7)
        self.joint_limits = {'lower': np.full(6, -2.), 'upper': np.full(6, 2.)}
        self.gripper_limits = {'lower': 0., 'upper': 2.}
        self.commands = []
        self.full_commands = []
        self.age = 0.
        self.fault = 0
        self.reject = False

    def get_current_state(self):
        return [SimpleNamespace(ID=i + 1, position=q, velocity=0., torque=0., time=time.time() - self.age, fault=self.fault) for i, q in enumerate(self.position[:6])]

    def get_current_state_gripper(self):
        return SimpleNamespace(ID=7, position=self.position[6], velocity=0., torque=0., time=time.time() - self.age, fault=self.fault)

    def get_Gravity(self, q):
        return np.zeros(6)

    def get_friction_compensation(self, vel, *args):
        return np.zeros_like(vel)

    def pos_vel_tqe_kp_kd(self, p, v, t, kp, kd):
        self.commands.append(np.array(p))
        self.full_commands.append(tuple(np.array(x) for x in (p, v, t, kp, kd)))
        if np.any(kp):
            self.position[:6] = p
        return not self.reject

    def gripper_control_MIT(self, p, *args):
        return True


class TeleopTests(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads((teleop.ROOT / 'teleop-config.example.json').read_text())
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        runtime = patch.object(teleop, 'RUNTIME', Path(tmp.name))
        runtime.start()
        self.addCleanup(runtime.stop)

    def test_stale_state_rejected(self):
        arm = FakeArm(); arm.age = 1
        with self.assertRaisesRegex(RuntimeError, '超时'):
            teleop.snapshot(arm, .2)

    def test_fault_rejected(self):
        arm = FakeArm(); arm.fault = 3
        with self.assertRaisesRegex(RuntimeError, '故障码'):
            teleop.snapshot(arm, .2)

    def test_nonfinite_rejected(self):
        arm = FakeArm(); arm.position[2] = np.nan
        with self.assertRaisesRegex(RuntimeError, '非有限'):
            teleop.snapshot(arm, .2)

    def test_joint_limits_rejected(self):
        arm = FakeArm(); arm.position[0] = 3
        with self.assertRaisesRegex(RuntimeError, '超出'):
            teleop.validate_positions(arm, arm.position)

    def test_target_speed_and_no_overshoot(self):
        previous = np.zeros(7)
        goal = np.array([1., -1., .001, 0., 2., 0., 1.])
        result = teleop.bounded_target(previous, goal, .5, .01)
        self.assertLessEqual(float(np.max(np.abs(result))), .005)
        self.assertEqual(result[2], .001)

    def test_settings_fail_closed(self):
        teleop.validate_settings(self.cfg)
        for key, value in [('motor_timeout_ms', 0), ('max_target_speed_rad_s', float('nan')), ('joint_kp', [1, 2])]:
            cfg = copy.deepcopy(self.cfg); cfg[key] = value
            with self.assertRaises(ValueError):
                teleop.validate_settings(cfg)

    def test_query_rejects_motion_packet(self):
        with self.assertRaises(ValueError):
            packet(0xb0)

    def test_fragmented_and_corrupt_frames(self):
        data = packet(0x0b, bytes([1, 4, 7, 3]))
        buf = bytearray(data[:4]); self.assertEqual(frames(buf), [])
        buf.extend(data[4:]); self.assertEqual(frames(buf), [(0x0b, bytes([1, 4, 7, 3]))])
        bad = bytearray(data); bad[-1] ^= 1
        self.assertEqual(frames(bad), [])

    def test_control_loop_rate_limited_following(self):
        leader, follower = FakeArm(), FakeArm()
        leader.position[0] = 1.
        with tempfile.TemporaryDirectory() as tmp, patch.object(teleop, 'RUNTIME', Path(tmp)):
            teleop.run_loop(leader, follower, self.cfg, {}, .08)
        self.assertGreater(len(follower.commands), 2)
        increments = np.diff(np.vstack([np.zeros(6), follower.commands]), axis=0)
        self.assertTrue(np.all(np.abs(increments) <= .030001))

    def test_usb_change_stops_before_commands(self):
        leader, follower = FakeArm(), FakeArm()
        cfg = dict(self.cfg, leader='/nonexistent/panthera-device')
        with self.assertRaisesRegex(RuntimeError, 'USB'):
            teleop.run_loop(leader, follower, cfg, {'leader': Path('/dev/ttyACM0')}, .1)
        self.assertEqual(len(follower.commands), 0)

    def test_sdk_rejection_aborts(self):
        leader, follower = FakeArm(), FakeArm(); leader.reject = True
        with self.assertRaisesRegex(RuntimeError, 'SDK 拒绝'):
            teleop.run_loop(leader, follower, self.cfg, {}, .1)
        self.assertEqual(len(follower.commands), 0)

    def test_tracking_error_stops_before_commands(self):
        leader, follower = FakeArm(), FakeArm()
        original = follower.get_current_state
        calls = 0
        def displaced_state():
            nonlocal calls
            calls += 1
            if calls > 1:
                follower.position[0] = 1.
            return original()
        follower.get_current_state = displaced_state
        with self.assertRaisesRegex(RuntimeError, '跟踪误差'):
            teleop.run_loop(leader, follower, self.cfg, {}, .1)
        self.assertEqual(len(follower.commands), 0)

    def test_generated_config_watchdog_and_paths(self):
        import yaml
        with tempfile.TemporaryDirectory() as tmp, patch.object(teleop, 'RUNTIME', Path(tmp)):
            path = teleop.generate_config('leader', Path('/dev/ttyACM0'), self.cfg, True)
            high = yaml.safe_load(path.read_text())
            low = yaml.safe_load(Path(high['robot']['param_file']).read_text())
            self.assertTrue(Path(high['urdf']['file_path']).is_file())
            self.assertEqual(low['robot']['motor_timeout_ms'], 250)
            self.assertEqual(low['robot']['Serial_Type'], '/dev/ttyACM0')
            self.assertTrue(low['robot']['exit_motor_brake_flag'])

    def test_leader_remains_dissipative_with_bounded_follower_feedforward(self):
        leader, follower = FakeArm(), FakeArm()
        leader.position[0] = .2
        def forbid_friction(*args):
            raise AssertionError('Uncalibrated friction must remain disabled')
        leader.get_friction_compensation = forbid_friction
        follower.get_friction_compensation = forbid_friction
        teleop.run_loop(leader, follower, self.cfg, {}, .05)
        for position, velocity, torque, kp, kd in leader.full_commands:
            self.assertTrue(np.all(velocity == 0))
            self.assertTrue(np.all(kp == 0))
            self.assertTrue(np.all(kd > 0))
            self.assertEqual(torque[0], 0)
            for measured_velocity in [-.1, .1]:
                power = (-kd[0] * measured_velocity) * measured_velocity
                self.assertLess(power, 0)
        for _, velocity, _, _, _ in follower.full_commands:
            self.assertTrue(np.all(np.abs(velocity) <= 0.75))
        self.assertGreater(follower.full_commands[-1][1][0], 0)
        rows = [json.loads(line) for line in (teleop.RUNTIME / 'teleop-latest.jsonl').read_text().splitlines()]
        self.assertGreater(len(rows), 0)
        self.assertLessEqual(max(abs(v) for v in rows[0]['target_velocity']), .75)

    def test_feedforward_stops_and_reverses_without_old_direction(self):
        previous = np.array([.8, .8, -.8])
        filtered, velocity = teleop.reference_velocity(previous, np.array([0., -.01, .01]), .01, 1., .5, .04)
        self.assertEqual(velocity[0], 0)
        self.assertLess(velocity[1], 0)
        self.assertGreater(velocity[2], 0)
        self.assertTrue(np.all(np.abs(velocity) <= .5))

    def test_new_speed_limit_tracks_moderate_leader_motion(self):
        target = np.zeros(7)
        for tick in range(1, 101):
            desired = np.full(7, tick * .008)  # 0.8 rad/s leader motion
            target = teleop.bounded_target(target, desired, self.cfg['max_target_speed_rad_s'], .01)
        np.testing.assert_allclose(target, desired)

    def test_gripper_has_independent_faster_limit(self):
        limits = np.r_[np.full(6, self.cfg['max_target_speed_rad_s']), self.cfg['gripper_max_target_speed_rad_s']]
        step = teleop.bounded_target(np.zeros(7), np.ones(7), limits, .01)
        np.testing.assert_allclose(step[:6], .015)
        self.assertAlmostEqual(step[6], .04)
        filtered = np.zeros(7)
        for _ in range(100):
            filtered, velocity = teleop.reference_velocity(filtered, step, .01, limits, .5, .04)
        self.assertTrue(np.all(velocity[:6] <= .75))
        self.assertGreater(velocity[6], 1.99)
        self.assertLessEqual(velocity[6], 2.)

    def test_selected_leader_gravity_terms_are_reduced(self):
        arm = FakeArm()
        arm.get_Gravity = lambda q: np.array([0., 2., 3.8, .64, .04, .001])
        gravity = arm.get_Gravity(np.zeros(6))
        result = teleop.leader_gravity_torque(arm, np.zeros(6), self.cfg)
        self.assertAlmostEqual(result[2], 3.61)
        self.assertAlmostEqual(result[3], .576)
        np.testing.assert_array_equal(result[[0,1,4,5]], gravity[[0,1,4,5]])
        cfg = copy.deepcopy(self.cfg)
        cfg['leader_gravity_scale'][3] = float('nan')
        with self.assertRaises(ValueError):
            teleop.validate_settings(cfg)

    def test_cleanup_does_not_report_control_heartbeat_failure(self):
        import os
        for name in ['running', 'heartbeat', 'cleanup']:
            (teleop.RUNTIME / name).touch()
        os.utime(teleop.RUNTIME / 'heartbeat', (time.time()-2, time.time()-2))
        teleop.check_worker_health(time.monotonic())
        os.utime(teleop.RUNTIME / 'cleanup', (time.time()-4, time.time()-4))
        with self.assertRaisesRegex(RuntimeError, '退出清理'):
            teleop.check_worker_health(time.monotonic())

    def test_active_heartbeat_timeout_is_preserved(self):
        import os
        for name in ['running', 'heartbeat']:
            (teleop.RUNTIME / name).touch()
        os.utime(teleop.RUNTIME / 'heartbeat', (time.time()-1, time.time()-1))
        with self.assertRaisesRegex(RuntimeError, '心跳超时'):
            teleop.check_worker_health(time.monotonic())

    def test_device_rule_generation_uses_actual_distinct_boards(self):
        from device_rules import render_rules
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            config = {}
            for role, number in [('leader', 4), ('follower', 8)]:
                board = root / f'1-{number}.2'
                board.mkdir()
                (board / 'idVendor').write_text('caf1')
                (board / 'idProduct').write_text('ffff')
                interface = board / 'interface'; interface.mkdir()
                tty = root / f'tty{role}'; tty.mkdir()
                (tty / 'device').symlink_to(interface)
                config[role] = str(tty)
            rules = render_rules(config, 'operator', root)
            self.assertIn('KERNELS=="1-4.2"', rules)
            self.assertIn('KERNELS=="1-8.2"', rules)
            self.assertNotIn('0777', rules)
            config['follower'] = config['leader']
            with self.assertRaisesRegex(ValueError, 'different boards'):
                render_rules(config, 'operator', root)


if __name__ == '__main__':
    unittest.main()

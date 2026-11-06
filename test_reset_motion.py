"""Exercise reset bounds, convergence, feedback faults and cancellation without hardware."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
import numpy as np
from reset_motion import run_reset, validated_pose

class ResetTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        self.root=Path(temp.name)
        self.settings=json.loads((Path(__file__).parent/'teleop-config.example.json').read_text())
        self.robots={}
        self.devices={}
        for role in ('leader','follower'):
            device=self.root/role;device.touch();self.devices[role]=device;self.settings[role]=str(device)
            robot=Mock(joint_limits=dict(lower=np.full(6,-2),upper=np.full(6,2)),gripper_limits=dict(lower=-2,upper=2))
            robot.q=np.zeros(7);robot.history=[]
            robot.get_Gravity.return_value=np.zeros(6)
            def command(pos,*args,r=robot): r.q[:6]=pos;r.history.append(pos.copy());return True
            def gripper(pos,*args,r=robot): r.q[6]=pos;return True
            robot.pos_vel_tqe_kp_kd.side_effect=command;robot.gripper_control_MIT.side_effect=gripper
            self.robots[role]=robot
        self.target=self.root/'target.json';self.target.write_text(json.dumps(dict(leader=[.2]*7,follower=[.25]*7,unit='rad')))
        self.clock=0.

    def simulate(self, failed=False):
        def now(): return self.clock
        def sleep(dt): self.clock+=max(.01,dt)
        def snapshot(robot,age,feedback=None):
            if failed: raise RuntimeError('stale feedback')
            if feedback is not None: feedback.extend([0]*7)
            return np.c_[robot.q,np.zeros((7,2))]
        with patch('reset_motion.time.monotonic',side_effect=now),patch('reset_motion.time.sleep',side_effect=sleep),patch('reset_motion.snapshot',side_effect=snapshot),patch('reset_motion.RUNTIME',self.root),patch('reset_motion.Publisher'):
            run_reset(self.robots,self.settings,self.devices,self.target)

    def test_reset_is_bounded_and_only_marks_success_after_settling(self):
        self.simulate()
        self.assertTrue(self.target.with_suffix('.done').exists())
        for robot in self.robots.values():
            self.assertLessEqual(np.max(np.abs(np.diff(robot.history,axis=0))),.35*.02+1e-9)
        self.assertLessEqual(np.max(np.abs(self.robots['follower'].q-.25)),.04)

    def test_invalid_pose_never_sends_motion(self):
        for values in ([float('nan')]*7,[3]*7,[0]*6):
            with self.assertRaises(ValueError): validated_pose(dict(unit='rad',leader=values,follower=[0]*7),self.robots)
        for r in self.robots.values(): r.pos_vel_tqe_kp_kd.assert_not_called()

    def test_stale_feedback_fails_without_success_marker(self):
        with self.assertRaisesRegex(RuntimeError,'stale'): self.simulate(failed=True)
        self.assertFalse(self.target.with_suffix('.done').exists())

    def test_each_arm_resets_with_other_arm_absent(self):
        for role in ('leader', 'follower'):
            with self.subTest(role=role):
                other = 'follower' if role == 'leader' else 'leader'
                absent = self.robots.pop(other)
                self.clock = 0.
                self.simulate()
                self.assertTrue(self.target.with_suffix('.done').exists())
                self.robots[other] = absent

    def test_encoder_boundary_is_clipped_without_expanding_command_limits(self):
        robot = self.robots['leader']
        robot.joint_limits['lower'][1] = 0.
        pose = [0., -.00314159, 0., 0., 0., 0., 0.]
        result = validated_pose(dict(unit='rad', leader=pose), {'leader': robot})
        self.assertEqual(result['leader'][1], 0.)
        pose[1] = -.02
        with self.assertRaises(ValueError):
            validated_pose(dict(unit='rad', leader=pose), {'leader': robot})

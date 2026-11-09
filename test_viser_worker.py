"""Exercise the real worker loop with simulated SDK/clock, never real ports."""
import contextlib
import io
import json
from pathlib import Path
import queue
import tempfile
import types
import unittest
from unittest.mock import patch
import numpy as np
import yaml
import viser_worker


class WorkerTests(unittest.TestCase):
    def run_worker(self, enabled, mode="web", continuous=False, speed=.15, feedback_pose=0., feedback_j2=0., gui_targets=None, replay_feedback=False, measured_velocity=0., measured_torque=0., switch_to_reset=False, capture=False):
        clock = [0.]
        robots = []
        stopped = []
        feedback = queue.Queue()
        samples=queue.Queue(maxsize=4)
        recording=types.SimpleNamespace(is_set=lambda:capture)
        class Robot:
            def __init__(self, path):
                self.moves = []; self.homes=[]; self.grips=[]; self.queries=0; self.index=len(robots); robots.append(self)
            def send_get_motor_state_cmd(self): self.queries+=1
            def get_Gravity(self, q): return np.zeros(6)
            def pos_vel_tqe_kp_kd(self, q, *args):
                self.moves.append(q.copy()); return True
            def Joint_Pos_Vel(self,q,*args):self.homes.append(q.copy());return True
            def gripper_control_MIT(self, *args):self.grips.append(args);return True
        # A fresh command is published once; expiry must stop the worker.
        class Commands:
            delivered = False
            def get_nowait(self):
                if self.delivered:
                    if continuous and clock[0]<1.6:self.delivered=False
                    raise queue.Empty
                self.delivered = True
                payload=gui_targets if gui_targets is not None else [[1.,0,0,0,0,0],[0.]*6]
                if replay_feedback:
                    rows=[m for m in list(feedback.queue) if m.get('kind')=='state']
                    if rows:payload=rows[-1]['desired']
                reset = switch_to_reset and clock[0] >= .3
                return dict(stamp=clock[0], targets=payload,
                            enabled=enabled, epochs=[2,2] if reset else [1,1], speed=speed,
                            mode='reset' if reset else mode, operation=2 if reset else 1)
        def sleep(dt): clock[0] += max(.001, dt)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); sdk = root/'sdk'; param = sdk/'robot_param'
            (param/'motor_param').mkdir(parents=True)
            low = {'robot': {'CANboard': {'No_1_CANboard': {'CANport': {'CANport_1': {}}}}}}
            (param/'motor_param/6dof_Panthera_params_follower.yaml').write_text(yaml.safe_dump(low))
            (param/'Follower.yaml').write_text(yaml.safe_dump({'robot': {}, 'urdf': {'file_path': 'dummy'}}))
            (root/'teleop-config.json').write_text(json.dumps(dict(leader='A', follower='B',
                joint_kp=[1]*6, joint_kd=[1]*6, leader_kd=[.1]*6, torque_limits_nm=[5]*6)))
            fake = types.ModuleType('teleop')
            fake.SDK = sdk
            fake.reference_velocity = lambda previous,step,dt,*a:(np.asarray(step)/dt,np.asarray(step)/dt*.5)
            fake.leader_gravity_torque = lambda *a:np.zeros(6)
            fake.resolve_device = lambda p:p
            def snapshot(robot,*args):
                if len(args)>1:args[1].extend([1000.]*7)
                out=np.zeros((7,3));out[1,0]=feedback_j2;out[5,1]=measured_velocity;out[5,2]=measured_torque
                if robot.index==0:out[0,0]=feedback_pose;out[6,0]=feedback_pose
                return out
            fake.snapshot = snapshot
            fake.validate_positions = lambda *a:None
            fake.shutdown = lambda rs:stopped.append(list(rs))
            lib = types.ModuleType('Panthera_lib');lib.Panthera = Robot
            with patch.dict('sys.modules', {'teleop':fake,'Panthera_lib':lib}), \
                 patch.object(viser_worker,'ROOT',root), \
                 patch.object(viser_worker.signal,'signal'), \
                 patch.object(viser_worker.time,'monotonic',lambda:clock[0]), \
                 patch.object(viser_worker.time,'sleep',sleep), \
                 contextlib.redirect_stderr(io.StringIO()), contextlib.redirect_stdout(io.StringIO()):
                viser_worker.worker(Commands(),feedback,samples,recording)
        messages=[]
        while not feedback.empty():messages.append(feedback.get_nowait())
        while not samples.empty():messages.insert(0,dict(kind="sample",row=samples.get_nowait()))
        return robots, stopped, messages

    def test_live_mode_transition_to_reset_aligns_actual_without_brake(self):
        for mode in ('web','teleop'):
            with self.subTest(mode=mode):
                robots,stopped,messages=self.run_worker([True,True],mode=mode,continuous=True,
                    feedback_pose=.05,switch_to_reset=True)
                self.assertTrue(all(r.moves and r.homes for r in robots))
                self.assertEqual(stopped,[[0,1]])  # Only final heartbeat shutdown.
                self.assertAlmostEqual(robots[0].homes[0][0],.05-.0008,places=5)
                self.assertLessEqual(abs(robots[1].homes[0][0]),.00081)
                self.assertEqual(messages[-1]['fault']['code'],'heartbeat_timeout')
                self.assertTrue(any(m.get('mode')=='reset' and m.get('operation')==2 for m in messages))

    def test_recording_queue_full_never_blocks_motor_loop(self):
        robots,stopped,messages=self.run_worker([True,True],mode='teleop',capture=True)
        samples=[m['row'] for m in messages if m['kind']=='sample']
        self.assertEqual(len(samples),4)
        self.assertGreater(len(robots[0].moves),20)
        self.assertEqual(messages[-1]['fault']['code'],'heartbeat_timeout')
        row=samples[0];self.assertEqual(len(row['arms']),2)
        self.assertEqual(len(row['arms'][0]['feedback_time_s']),7)
        self.assertEqual(len(row['arms'][1]['action']),7)
        self.assertIsInstance(row['timestamp_ns'],int)

    def test_disabled_arms_never_receive_motion(self):
        robots, stopped, messages = self.run_worker([False,False])
        self.assertEqual([len(r.moves) for r in robots],[0,0])
        self.assertEqual(stopped[-1],[0,1])
        self.assertIn('心跳超时',messages[-1]['message'])

    def test_enabled_arm_is_bounded_and_heartbeat_failure_brakes_both(self):
        robots, stopped, messages = self.run_worker([True,False])
        self.assertGreater(len(robots[0].moves),2)
        self.assertEqual(len(robots[1].moves),0)
        q=np.asarray(robots[0].moves)
        self.assertLessEqual(np.max(np.diff(q[:,0])),.00150001)
        self.assertLess(q[-1,0],.08)
        self.assertEqual(stopped[-1],[0,1])
        self.assertIn('心跳超时',messages[-1]['message'])

    def test_reset_uses_position_speed_sdk_and_reports_settled(self):
        robots,stopped,messages=self.run_worker([True,True],mode='reset',continuous=True)
        self.assertTrue(all(len(r.homes)>100 for r in robots))
        self.assertTrue(all(len(r.moves)==0 for r in robots))
        self.assertTrue(any(m['kind']=='reset_done' and m['operation']==1 for m in messages))
        self.assertEqual(stopped[-1],[0,1])

    def test_teleop_uses_leader_gravity_and_follower_positions(self):
        robots,stopped,messages=self.run_worker([True,True],mode='teleop',feedback_pose=.1)
        self.assertGreater(len(robots[0].moves),2)
        self.assertTrue(all(np.all(q==0) for q in robots[0].moves))
        self.assertGreater(robots[1].moves[-1][0],0)
        self.assertGreater(robots[1].grips[-1][0],0)

    def test_tracking_failure_reports_exact_joint_and_brakes(self):
        robots,stopped,messages=self.run_worker([True,False],continuous=True,speed=1.)
        fault=messages[-1]['fault']
        self.assertEqual(fault['code'],'tracking_error')
        self.assertEqual(fault['detail']['joint'],1)
        self.assertEqual(fault['detail']['arm'],'A')
        self.assertGreater(fault['detail']['error_rad'],.3)
        self.assertEqual(stopped[-1],[0,1])

    def test_reset_rejects_distant_pose_without_motion(self):
        robots,stopped,messages=self.run_worker([True,True],mode='reset',feedback_pose=.9)
        self.assertEqual(messages[-1]['fault']['code'],'home_range')
        self.assertTrue(all(not r.moves and not r.homes for r in robots))

    def test_teleop_negative_zero_feedback_roundtrip_is_not_a_target_fault(self):
        robots,stopped,messages=self.run_worker([True,True],mode='teleop',feedback_j2=-.0006283185,replay_feedback=True)
        self.assertGreater(len(robots[0].moves),20)
        self.assertEqual(messages[-1]['fault']['code'],'heartbeat_timeout')
        for m in messages:
            if m['kind']=='state':
                self.assertLess(m['positions'][0][1],0.)
                self.assertGreaterEqual(m['desired'][0][1],-.1)
                self.assertGreaterEqual(m['desired'][1][1],-.1)

    def test_reconnect_ignores_stale_gui_targets_while_idle(self):
        stale=[[0.,-.000628,0,0,0,0]]*2
        robots,stopped,messages=self.run_worker([False,False],mode='idle',gui_targets=stale,feedback_j2=-.000628)
        self.assertTrue(any(m['kind']=='state' for m in messages))
        self.assertEqual(messages[-1]['fault']['code'],'heartbeat_timeout')
        self.assertTrue(all(not r.moves and not r.homes for r in robots))

    def test_reset_ignores_stale_gui_target_and_still_reaches_zero(self):
        stale=[[0.,-.000628,0,0,0,0]]*2
        robots,stopped,messages=self.run_worker([True,True],mode='reset',continuous=True,gui_targets=stale,feedback_j2=-.000628)
        self.assertTrue(any(m['kind']=='reset_done' for m in messages))
        self.assertEqual(messages[-1]['fault']['code'],'heartbeat_timeout')

    def test_real_web_target_violation_still_stops_before_motion(self):
        robots,stopped,messages=self.run_worker([True,False],mode='web',gui_targets=[[0.,-.2,0,0,0,0],[0.]*6])
        self.assertEqual(messages[-1]['fault']['code'],'target_limit')
        self.assertIn('J2',messages[-1]['message'])
        self.assertTrue(all(not r.moves and not r.homes for r in robots))

    def test_active_teleop_does_not_interleave_state_queries(self):
        robots,stopped,messages=self.run_worker([True,True],mode='teleop')
        self.assertGreater(len(robots[0].moves),20)
        self.assertTrue(all(r.queries<=4 for r in robots))

    def test_passive_feedback_is_polled_at_bounded_rate(self):
        robots,stopped,messages=self.run_worker([False,False],mode='idle')
        self.assertTrue(all(8<=r.queries<=15 for r in robots))
        self.assertTrue(all(not r.moves for r in robots))

    def test_homing_does_not_switch_to_gripper_mit_between_joint_commands(self):
        robots,stopped,messages=self.run_worker([True,True],mode='reset',continuous=True)
        self.assertTrue(any(m['kind']=='reset_done' for m in messages))
        self.assertTrue(all(len(r.homes)>100 and not r.grips for r in robots))
        self.assertTrue(all(r.queries<=4 for r in robots))

    def test_normal_control_has_no_extra_measured_velocity_or_torque_cutoff(self):
        for mode in ('web','teleop'):
            robots,stopped,messages=self.run_worker([True,True],mode=mode,measured_velocity=3.,measured_torque=7.)
            self.assertEqual(messages[-1]['fault']['code'],'heartbeat_timeout')
            self.assertGreater(len(robots[1].moves),20)

    def test_home_original_velocity_guard_remains(self):
        robots,stopped,messages=self.run_worker([True,True],mode='reset',measured_velocity=5.23)
        self.assertEqual(messages[-1]['fault']['code'],'home_velocity')
        self.assertTrue(all(not r.homes for r in robots))

if __name__=='__main__':unittest.main()

import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from viser_control import ConsoleState, ControlFault, tracking_guard
from viser_console import EventLog, RecoveryRequest

class ConsoleTests(unittest.TestCase):
    def state(self):
        s=ConsoleState(lambda:10.);s.feedback(np.zeros((2,6)));return s
    def test_modes_require_stop_and_single_owner(self):
        s=self.state();s.begin('teleop',4)
        with self.assertRaises(ValueError):s.target(0,np.zeros(6),4)
        with self.assertRaises(ValueError):s.begin('web',5)
        s.disable(0);self.assertEqual(s.enabled,[False,False]);self.assertEqual(s.mode,'idle')
        s.begin('web',5);self.assertEqual(s.enabled,[True,True])
    def test_reset_ack_after_stop_never_reenables(self):
        s=self.state();s.begin('reset',1);op=s.operation;s.disable()
        self.assertFalse(s.finish_reset(op));self.assertEqual(s.enabled,[False,False])
    def test_reset_done_holds_zero_and_allows_web_edit(self):
        s=self.state();s.actual[:]=.1;s.begin('reset',1)
        self.assertTrue(s.finish_reset(s.operation));self.assertEqual(s.mode,'hold')
        np.testing.assert_equal(s.desired,np.zeros((2,6)))
        s.target(0,[.1,0,0,0,0,0],1);self.assertEqual(s.mode,'web')
    def test_home_hold_to_teleop_without_intermediate_disable(self):
        s=self.state();s.begin('reset',1);s.finish_reset(s.operation)
        s.begin('teleop',1);self.assertEqual(s.mode,'teleop');self.assertEqual(s.enabled,[True,True])

    def test_active_modes_reset_without_disabling(self):
        for mode in ('web','teleop','hold'):
            with self.subTest(mode=mode):
                s=self.state();s.begin('web',1);s.mode=mode
                s.feedback(np.full((2,6),.1));before=list(s.epochs);op=s.operation
                s.begin('reset',1)
                self.assertEqual(s.mode,'reset');self.assertEqual(s.owner,1)
                self.assertEqual(s.enabled,[True,True]);self.assertEqual(s.operation,op+1)
                self.assertEqual(s.epochs,[e+1 for e in before])
                np.testing.assert_equal(s.desired,np.zeros((2,6)))

    def test_rejected_active_reset_leaves_previous_control_intact(self):
        for reason in ('owner','range','stale','duplicate'):
            with self.subTest(reason=reason):
                s=self.state();s.begin('web',1)
                if reason=='range':s.actual[0,0]=.9
                if reason=='stale':s.updated=0.
                if reason=='duplicate':s.begin('reset',1)
                before=s.command();owner=s.owner
                with self.assertRaises(ValueError):s.begin('reset',2 if reason=='owner' else 1)
                self.assertEqual(s.command(),before);self.assertEqual(s.owner,owner)

    def test_previous_reset_completion_cannot_complete_new_reset(self):
        s=self.state();s.begin('reset',1);old=s.operation;s.finish_reset(old)
        s.begin('reset',1)
        self.assertFalse(s.finish_reset(old));self.assertEqual(s.mode,'reset')
        self.assertTrue(s.finish_reset(s.operation))

    def test_active_reset_callback_and_button(self):
        import threading
        from unittest.mock import Mock
        from viser_app import App, ARMS
        for mode in ('web','teleop','hold'):
            with self.subTest(mode=mode):
                app=App.__new__(App)
                app.state=ConsoleState();app.state.feedback(np.zeros((2,6)))
                app.state.begin('web',1);app.state.mode=mode
                app.hardware=True;app.lock=threading.RLock();app.recovery=RecoveryRequest()
                app.process=Mock();app.process.is_alive.return_value=True
                app.log=Mock(last_fault=None,events=[]);app.worker_state={};app.faulted=False
                app.page_name='机械臂';app.arm_index=0;app.data_arm=Mock(value=ARMS[0])
                app.actual_urdfs=[Mock(),Mock()]
                for urdf in app.actual_urdfs:urdf.get_transform.return_value=np.eye(4)
                for name in ('status','fault_banner','start_button','operation_mode','home_button','retry','sync_button','speed','gizmo','progress','reset_info','readout','data_table','system_info','events_view'):
                    setattr(app,name,Mock())
                app.enable_buttons=[Mock(),Mock()];app.sliders=[]
                app.message='';app.refresh_sliders=Mock();app.fail_event=Mock()
                app.client_id=lambda event:1;app.dragging=True
                app.update_status();self.assertFalse(app.home_button.disabled)
                app.reset_both(None);app.fail_event.assert_not_called()
                self.assertEqual(app.state.mode,'reset');self.assertFalse(app.dragging)
                app.update_status();self.assertTrue(app.home_button.disabled)

    def test_reset_far_pose_rejected_before_enabling(self):
        s=self.state();s.actual[1,3]=-.9
        with self.assertRaises(ValueError):s.begin('reset',1)
        self.assertEqual(s.enabled,[False,False])
    def test_disconnect_cancels_paired_operation(self):
        s=self.state();s.begin('reset',3);s.disconnected(3)
        self.assertEqual(s.enabled,[False,False]);self.assertEqual(s.mode,'idle')
    def test_recovery_expiry_disconnect_and_cancel(self):
        clock=[0.];r=RecoveryRequest(lambda:clock[0])
        r.request(1);clock[0]=26.;self.assertIsNone(r.take({1:None}))
        r.request(1);self.assertIsNone(r.take({2:None}))
        r.request(1);r.cancel();self.assertIsNone(r.take({1:None}))
        r.request(1);self.assertEqual(r.take({1:None}),1);self.assertIsNone(r.take({1:None}))
    def test_tracking_fault_has_specific_joint_reference_actual_and_limit(self):
        ref=np.zeros(6);ref[3]=.4
        with self.assertRaises(ControlFault) as caught:
            tracking_guard('A',ref,np.zeros(6),ref,.3,1.,'web')
        d=caught.exception.detail
        self.assertEqual(d['joint'],4);self.assertEqual(d['error_rad'],.4)
        self.assertEqual(d['limit_rad'],.3);self.assertEqual(d['actual_rad'],0.)
    def test_stop_and_restart_do_not_erase_fault(self):
        with tempfile.TemporaryDirectory() as tmp:
            log=EventLog(tmp);fault=dict(time=123.,code='tracking_error',message='A J4',detail={'joint':4})
            log.fault(fault);log.add('用户停止','stop');log.add('重新连接成功')
            again=EventLog(tmp);self.assertEqual(again.last_fault,fault)
            self.assertEqual(len(again.events),3)

class CleanupTests(unittest.TestCase):
    def test_fault_cleanup_is_bounded_and_does_not_restart(self):
        from unittest.mock import Mock
        from viser_app import App
        app=App.__new__(App)
        app.process=Mock();app.process.is_alive.return_value=True
        app.faulted=False;app.fault_at=10.;app.terminating=False;app.killed=False
        app.reap_faulted_worker(20.);app.process.terminate.assert_not_called()
        app.faulted=True
        app.reap_faulted_worker(11.);app.process.terminate.assert_not_called()
        app.reap_faulted_worker(12.);app.reap_faulted_worker(13.)
        self.assertEqual(app.process.terminate.call_count,1)
        app.reap_faulted_worker(15.);app.reap_faulted_worker(16.)
        self.assertEqual(app.process.kill.call_count,1)
        app.process.start.assert_not_called()

if __name__=='__main__':unittest.main()

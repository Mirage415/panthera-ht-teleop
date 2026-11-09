"""Exercise complete file lifecycle and camera/frame pairing without hardware."""
import io
import json
from pathlib import Path
import queue
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zipfile
import numpy as np
from PIL import Image
from mcap.reader import make_reader
from viser_collection import Collection
from viser_control import ConsoleState
from viser_playback import Playback

class CollectionTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        state=ConsoleState();state.feedback(np.zeros((2,6)));state.begin('web',1)
        self.app=SimpleNamespace(lock=threading.RLock(),state=state,hardware=True,
            sample_queue=queue.Queue(),recording_event=threading.Event(),
            server=SimpleNamespace(get_clients=lambda:{1:object()}),operation_mode=SimpleNamespace(value='网页双臂控制'))
        with patch('threading.Thread.start'):
            self.c=Collection(self.app,self.root)
        self.addCleanup(self.c.writer_lock.close)
        self.addCleanup(lambda:self.c.stop_recording())
        self.c.workflow.configure('catalog_create',dict(name='桌面任务'),self.c.recorder,None)
        self.c.workflow.configure('task_create',dict(catalog_id=self.c.workflow.data['selected_catalog'],prompt='拿起方块'),self.c.recorder,None)
        self.c.camera=dict(connected=True,timestamp_ns=time.time_ns(),session='cam',depth_scale=.001)
    def revision(self):return self.c.workflow.state(self.c.recorder,None)['workflow_revision']
    def sample(self, sequence=0, mode='teleop'):
        arm=dict(position=[.1]*7,velocity=[.01]*7,effort=[.2]*7,feedback_time_s=[time.time()]*7,reference=[.1]*6,target=[.2]*6)
        return dict(timestamp_ns=time.time_ns(),monotonic_ns=time.monotonic_ns(),sequence=sequence,session='worker',
            arms=[arm,dict(arm,action=[.2]*7)],mode=mode,enabled=[True,True])
    def batch(self, sequence=1):
        rgb=io.BytesIO();Image.new('RGB',(32,24),'red').save(rgb,format='JPEG')
        depth=io.BytesIO();Image.fromarray(np.full((24,32),1234,dtype=np.uint16)).save(depth,format='PNG')
        data=io.BytesIO();stamp=time.time_ns()
        with zipfile.ZipFile(data,'w') as z:
            z.writestr('metadata.json',json.dumps(dict(self.c.camera,timestamp_ns=stamp)))
            z.writestr('frames.json',json.dumps([dict(sequence=sequence,timestamp_ns=stamp)]))
            z.writestr(f'{sequence}.jpg',rgb.getvalue());z.writestr(f'{sequence}.png',depth.getvalue())
        return data.getvalue()
    def test_rgb_depth_dual_feedback_review_and_replay(self):
        self.c.command('start',1,self.revision(),dict(depth=True))
        self.c.joint(self.sample())
        with patch('viser_collection.urlopen',return_value=io.BytesIO(self.batch())):self.c.camera_batch()
        self.c.command('stop',1,None,{})
        folder=Path(self.c.recorder.last_path)
        self.assertTrue(self.c.recorder.last_result['complete'])
        self.assertTrue(self.app.state.enabled[0])  # Recording stop is not motor stop.
        with (folder/'data.mcap').open('rb') as f:
            messages=list(make_reader(f).iter_messages())
        self.assertEqual({ch.topic for _,ch,_ in messages},{'/robot/dual_arm_state','/leader/joint_states','/follower/joint_states','/camera/color/image','/camera/depth/image','/camera/frame_info'})
        dual=next(json.loads(m.data) for _,ch,m in messages if ch.topic=='/robot/dual_arm_state')
        self.assertEqual(len(dual['arms']),2);self.assertIsInstance(dual['timestamp_ns'],int)
        with patch('threading.Thread.start'):playback=Playback(self.root)
        replay=playback.read(folder.name,.5)
        self.assertEqual(replay['image'].shape,(24,32,3));self.assertEqual(len(replay['joints']),2)
        self.c.command('save',1,self.revision(),{})
        self.assertEqual(self.c.workflow.state(self.c.recorder,None)['workflow_phase'],'ready')
        self.assertEqual(self.c.recorder.last_result['task']['prompt'],'拿起方块')
    def test_camera_missing_or_feedback_stale_never_enables(self):
        self.app.state.disable();self.c.camera['connected']=False
        with self.assertRaises(ValueError):self.c.command('start',1,self.revision(),{})
        self.assertFalse(any(self.app.state.enabled));self.assertFalse(self.c.recorder.active)
        self.c.camera['connected']=True;self.app.state.updated-=1
        with self.assertRaises(ValueError):self.c.command('start',1,self.revision(),{})
        self.assertFalse(any(self.app.state.enabled))
    def test_stale_action_and_other_owner_rejected(self):
        old=self.revision()
        self.c.workflow.configure('catalog_create',dict(name='新任务'),self.c.recorder,None)
        with self.assertRaises(ValueError):self.c.command('start',1,old,{})
        with self.assertRaises(ValueError):self.c.start(2,{})
        self.assertFalse(self.c.recorder.active)
    def test_sequence_gaps_and_old_samples(self):
        self.c.start(1,{})
        self.c.joint(self.sample(0));self.c.joint(self.sample(4));self.c.joint(self.sample(4))
        self.assertEqual(self.c.stats['sequence_gaps'],3);self.assertEqual(self.c.stats['joint_samples'],2)
        stale=self.sample(5);stale['monotonic_ns']-=1_000_000_000
        with self.assertRaises(ValueError):self.c.joint(stale)
    def test_duplicate_start_and_pending_review(self):
        self.c.start(1,{});path=self.c.recorder.path
        self.c.start(1,{});self.assertEqual(path,self.c.recorder.path)
        self.c.stop_recording()
        with self.assertRaises(ValueError):self.c.start(1,{})
        self.c.command('discard',1,self.revision(),{})
        self.assertTrue((path/'data.mcap').exists())
        self.assertEqual(self.c.workflow.state(self.c.recorder,None)['workflow_phase'],'ready')
    def test_disconnected_client_never_starts_motion(self):
        self.app.state.disable();self.app.server.get_clients=lambda:{}
        with self.assertRaises(ValueError):self.c.start(1,{})
        self.assertFalse(any(self.app.state.enabled));self.assertFalse(self.c.recorder.active)
    def test_single_writer_lock_and_crash_recovery(self):
        with patch('threading.Thread.start'):
            with self.assertRaises(BlockingIOError):Collection(self.app,self.root)
        p=self.root/'crashed';p.mkdir();(p/'metadata.json').write_text(json.dumps(dict(recording=True,review_status='pending',complete=False)))
        from recording import Recorder
        restored=Recorder(self.root,{},[])
        self.assertTrue(restored.has_pending);self.assertFalse(restored.last_result['complete'])
        self.assertIn('异常退出',restored.last_result['error'])
    def test_review_survives_browser_reconnect(self):
        self.c.start(1,{});self.c.stop_recording()
        self.app.server.get_clients=lambda:{2:object()}
        self.c.command('discard',2,self.revision(),{})
        self.assertFalse(self.c.recorder.has_pending)
        self.assertIsNone(self.c.owner)

    def test_playback_rejects_paths_outside_recordings(self):
        with patch('threading.Thread.start'):p=Playback(self.root)
        with self.assertRaises(ValueError):p.seek('../outside',.5)

class CameraServiceTests(unittest.TestCase):
    def test_batch_keeps_depth_rgb_and_timestamps_together(self):
        import camera_service as s
        with s.condition:
            s.frame_history.clear();s.frame_history.append((dict(sequence=10,timestamp_ns=123),'rgb'.encode(),'depth'.encode()))
            s.state.update(connected=True,frames=10,updated=time.monotonic())
        response=s.app.test_client().get('/api/frames?after=9&depth=1')
        self.assertEqual(response.status_code,200)
        with zipfile.ZipFile(io.BytesIO(response.data)) as z:
            self.assertEqual(json.loads(z.read('frames.json'))[0]['timestamp_ns'],123)
            self.assertEqual(z.read('10.jpg'),b'rgb');self.assertEqual(z.read('10.png'),b'depth')
        with s.condition:s.state['connected']=False
        self.assertEqual(s.app.test_client().get('/api/frames?after=9').status_code,503)

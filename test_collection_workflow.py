"""Task/review/reset state machine; no motors or external services."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import foxglove
from collection_workflow import CollectionWorkflow
from recording import Recorder, RecordingControl


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        self.workflow = CollectionWorkflow(self.root / '.workflow.json')
        self.stats = dict(rgb_frames=0, joint_samples=0)
        self.recorder = Recorder(self.root, {}, ['q'], context=foxglove.Context())
        self.addCleanup(lambda: self.recorder.stop(self.stats))
        self.teleop = SimpleNamespace(phase='idle', reset_completed=False, reset=Mock(return_value={'ok': True}),
                                     state=lambda: {})
        self.live = dict(camera_live=True, follower_live=True)

    def create(self):
        self.workflow.configure('catalog_create', {'name': '桌面抓取'}, self.recorder, self.teleop)
        catalog = self.workflow.data['selected_catalog']
        self.workflow.configure('task_create', {'catalog_id': catalog, 'prompt': 'Put the red block in the bowl.'}, self.recorder, self.teleop)
        return catalog

    def sample(self):
        self.recorder.capture_start_pose(dict(position=[.1]*7, leader_position=[.2]*7, timestamp_ns=123))
        self.stats.update(rgb_frames=1, joint_samples=1)
        self.workflow.tick(self.teleop, self.recorder)

    def test_full_saved_and_discarded_episode_reset_cycle(self):
        self.create()
        for decision in ('saved', 'discarded'):
            self.workflow.before_start(self.recorder, self.teleop)
            self.recorder.start(self.stats)
            self.stats['rgb_frames'] += 1
            self.stats['joint_samples'] += 1
            self.recorder.capture_start_pose(dict(position=[.1]*7, leader_position=[.2]*7, timestamp_ns=self.stats['joint_samples']))
            self.workflow.tick(self.teleop, self.recorder)
            self.assertEqual(self.workflow.state(self.recorder,self.teleop)['workflow_phase'], 'recording')
            self.recorder.stop(self.stats)
            with self.assertRaises(ValueError): self.workflow.before_start(self.recorder,self.teleop)
            with self.assertRaises(ValueError): self.workflow.reset(self.recorder,self.teleop)
            self.assertTrue(self.recorder.decide(decision)['ok'])
            self.workflow.remember_episode(self.recorder)
            self.assertEqual(self.workflow.state(self.recorder,self.teleop)['workflow_phase'], 'reset_required')
            self.workflow.reset(self.recorder,self.teleop)
            self.teleop.reset.assert_called_with(self.recorder.last_result['initial_pose'], role='both')
            self.teleop.reset_completed=True
            self.workflow.tick(self.teleop,self.recorder)
            self.assertEqual(self.workflow.state(self.recorder,self.teleop)['workflow_phase'], 'ready')
        self.assertEqual(self.workflow.state(self.recorder,self.teleop)['task_episode_counts'],dict(saved=1,discarded=1))

    def test_prompt_snapshot_and_restore_without_relabeling(self):
        catalog = self.create()
        exact = self.workflow.selection()
        self.workflow.before_start(self.recorder,self.teleop)
        self.recorder.start(self.stats); self.sample(); self.recorder.stop(self.stats)
        with self.assertRaises(ValueError):
            self.workflow.configure('task_create',dict(catalog_id=catalog,prompt='Another task'),self.recorder,self.teleop)
        restored = CollectionWorkflow(self.workflow.path)
        self.assertEqual(restored.selection(),exact)
        self.assertTrue(restored.data['needs_reset'])
        self.assertEqual(self.recorder.last_result['task'],exact)
        self.assertEqual(self.recorder.last_result['initial_pose']['follower'],[.1]*7)

    def test_duplicate_names_prompts_rejected(self):
        catalog = self.create()
        with self.assertRaises(ValueError): self.workflow.configure('catalog_create',dict(name='桌面抓取'),self.recorder,self.teleop)
        with self.assertRaises(ValueError): self.workflow.configure('task_create',dict(catalog_id=catalog,prompt='Put the red block in the bowl.'),self.recorder,self.teleop)
        with self.assertRaises(ValueError): self.workflow.configure('task_create',dict(catalog_id=catalog,prompt='a\nb'),self.recorder,self.teleop)

    def test_stale_revision_rejected_after_transition_and_restart(self):
        revision = self.workflow.state(self.recorder,self.teleop)['workflow_revision']
        self.create()
        with self.assertRaises(ValueError): self.workflow.check_revision(revision,self.recorder,self.teleop)
        current = self.workflow.state(self.recorder,self.teleop)['workflow_revision']
        restored = CollectionWorkflow(self.workflow.path)
        with self.assertRaises(ValueError): restored.check_revision(current,self.recorder,self.teleop)

    def test_reset_requires_recorded_pose(self):
        self.create()
        with self.assertRaises(ValueError): self.workflow.reset(self.recorder,self.teleop)
        self.teleop.reset.assert_not_called()

    def test_failed_start_without_episode_can_retry(self):
        self.create()
        self.workflow.before_start(self.recorder,self.teleop)
        self.teleop.phase='error'
        self.workflow.tick(self.teleop,self.recorder)
        self.assertEqual(self.workflow.state(self.recorder,self.teleop)['workflow_phase'],'ready')

    def test_structured_command_service_and_replay_guard(self):
        control=RecordingControl(workflow=self.workflow,teleop=self.teleop)
        revision=self.workflow.state(self.recorder,self.teleop)['workflow_revision']
        payload=json.dumps(dict(action='catalog_create',revision=revision,name='Test')).encode()
        def call():
            with ThreadPoolExecutor(1) as pool:
                future=pool.submit(control.handler('command'),SimpleNamespace(encoding='json',payload=payload))
                deadline=time.monotonic()+2
                while control.queue.empty() and time.monotonic()<deadline: time.sleep(.001)
                control.drain(self.recorder,self.stats,self.live)
                return json.loads(future.result(timeout=2))
        self.assertTrue(call()['ok'])
        with self.assertRaises(ValueError): call()
        self.assertEqual(len(self.workflow.data['catalogs']),1)

    def test_separate_resets_both_required_and_durable(self):
        self.create()
        self.workflow.before_start(self.recorder,self.teleop)
        self.recorder.start(self.stats);self.sample();self.recorder.stop(self.stats)
        self.recorder.decide('saved');self.workflow.remember_episode(self.recorder)
        self.workflow.reset(self.recorder,self.teleop,role='follower')
        self.teleop.reset_role='follower';self.teleop.reset_completed=True
        self.workflow.tick(self.teleop,self.recorder)
        self.assertTrue(self.workflow.data['needs_reset'])
        self.assertEqual(self.workflow.data['reset_completed_roles'],['follower'])
        restored=CollectionWorkflow(self.workflow.path)
        self.assertEqual(restored.data['reset_completed_roles'],['follower'])
        restored.reset(self.recorder,self.teleop,role='leader')
        self.teleop.reset_role='leader';self.teleop.reset_completed=True
        restored.tick(self.teleop,self.recorder)
        self.assertFalse(restored.data['needs_reset'])
        restored.remember_episode(self.recorder)
        self.assertFalse(restored.data['needs_reset'])

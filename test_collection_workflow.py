"""Task/review/reset state machine; no motors or external services."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from collection_workflow import CollectionWorkflow
from recording import Recorder


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        self.workflow = CollectionWorkflow(self.root / '.workflow.json')
        self.stats = dict(rgb_frames=0, joint_samples=0)
        self.recorder = Recorder(self.root, {}, ['q'])
        self.addCleanup(lambda: self.recorder.stop(self.stats))
        self.teleop = SimpleNamespace(phase='idle', reset_completed=False, reset=Mock(return_value={'ok': True}),
                                     state=lambda: {})
        self.live = dict(camera_live=True, follower_live=True)

    def create(self):
        self.workflow.configure('catalog_create', {'name': '桌面抓取'}, self.recorder, self.teleop)
        catalog = self.workflow.data['selected_catalog']
        self.workflow.configure('task_create', {'catalog_id': catalog, 'prompt': 'Put the red block in the bowl.'}, self.recorder, self.teleop)
        return catalog

    def test_save_and_discard_do_not_require_pose_reset(self):
        self.create()
        for decision in ('saved','discarded'):
            self.workflow.before_start(self.recorder,None)
            self.recorder.start(self.stats)
            self.stats['rgb_frames']+=1;self.stats['joint_samples']+=1
            self.recorder.stop(self.stats)
            with self.assertRaises(ValueError):self.workflow.before_start(self.recorder,None)
            self.recorder.decide(decision)
            self.assertEqual(self.workflow.state(self.recorder,None)['workflow_phase'],'ready')
        self.assertFalse(hasattr(self.workflow,'reset'))

    def test_old_reset_gate_is_ignored_and_prompt_preserved(self):
        self.create();selected=self.workflow.selection()
        self.workflow.data['needs_reset']=True;self.workflow.persist()
        restored=CollectionWorkflow(self.workflow.path)
        self.assertEqual(restored.state(self.recorder,None)['workflow_phase'],'ready')
        self.assertEqual(restored.selection(),selected)

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

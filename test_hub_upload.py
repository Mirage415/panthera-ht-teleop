"""Upload queue tests use no network and never launch a worker."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch, Mock

from hub_upload import HubUploader, upload_episode


class UploadTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.config = self.root / 'config.json'
        self.config.write_text(json.dumps(dict(enabled=True, repo_id='user/dataset')))
        self.output = self.root / 'episodes'
        self.output.mkdir()

    def episode(self, name, review, complete=True):
        folder = self.output / name
        folder.mkdir()
        (folder / 'metadata.json').write_text(json.dumps(dict(complete=complete, recording=False, review_status=review)))
        return folder

    def test_only_saved_complete_episodes_are_queued(self):
        self.episode('pending', 'pending')
        self.episode('discarded', 'discarded')
        self.episode('incomplete', 'saved', False)
        self.episode('accepted', 'saved')
        uploader = HubUploader(self.output, self.config)
        with patch('hub_upload.subprocess.Popen') as launch:
            uploader.tick(force=True)
        launch.assert_called_once()
        self.assertEqual([p.parent.name for p in self.output.glob('*/upload.json')], ['accepted'])
        job = json.loads((self.output / 'accepted/upload.json').read_text())
        self.assertEqual(job['format'], 'lerobot')

    def test_worker_also_rejects_unsaved_data(self):
        folder = self.episode('pending', 'pending')
        api = Mock()
        with self.assertRaises(ValueError):
            upload_episode(folder, dict(repo_id='user/dataset'), api)
        api.create_repo.assert_not_called()

    def test_task_prompt_is_frozen_in_upload_job(self):
        folder = self.episode('accepted', 'saved')
        path = folder / 'metadata.json'
        metadata = json.loads(path.read_text())
        metadata['task'] = dict(catalog_id='set1', task_id='task1', prompt='Place the red block in the bowl.')
        path.write_text(json.dumps(metadata))
        with patch('hub_upload.subprocess.Popen'):
            HubUploader(self.output, self.config).tick(force=True)
        job = json.loads((folder/'upload.json').read_text())
        self.assertEqual(job['task'], metadata['task']['prompt'])

    def test_no_configuration_does_not_launch_or_upload(self):
        self.episode('accepted', 'saved')
        with patch('hub_upload.subprocess.Popen') as launch:
            uploader = HubUploader(self.output, self.root/'missing.json')
            uploader.tick(force=True)
        launch.assert_not_called()
        self.assertEqual(uploader.state()['upload_state'], 'awaiting_configuration')

"""Durable background LeRobot conversion and upload of finalized episodes."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
FILES = ['data.mcap', 'joints.csv', 'metadata.json']


def read_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return {}


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(f'.{os.getpid()}.tmp')
    temporary.write_text(json.dumps(value, indent=2))
    temporary.replace(path)


def upload_episode(folder, job, api):
    folder = Path(folder)
    metadata = read_json(folder / 'metadata.json')
    if metadata.get('complete') is not True or metadata.get('recording') is not False or metadata.get('review_status') != 'saved':
        raise ValueError('只上传已关闭且完整的 episode')
    if any(not (folder / name).is_file() or (folder / name).is_symlink() for name in FILES):
        raise ValueError('episode 文件缺失或包含符号链接')
    repo = job['repo_id']
    api.create_repo(repo_id=repo, repo_type='dataset', private=True, exist_ok=True)
    destination = ROOT / 'lerobot-export' / repo.replace('/', '--')
    python = ROOT / '.venv-lerobot/bin/python'
    with (folder / 'conversion.log').open('a') as log:
        subprocess.run([str(python), str(ROOT / 'export_lerobot.py'), '--episode', str(folder.resolve()),
            '--destination', str(destination), '--repo', repo, '--task', job.get('task', 'Panthera teleoperation')],
            stdout=log, stderr=subprocess.STDOUT, check=True)
    info = api.repo_info(repo_id=repo, repo_type='dataset')
    names = {entry.rfilename for entry in info.siblings}
    if 'meta/info.json' in names:
        if 'source_episodes.json' not in names:
            raise ValueError('远端已有其他数据集，拒绝覆盖')
        remote = read_json(api.hf_hub_download(repo_id=repo, repo_type='dataset', filename='source_episodes.json', revision=info.sha))
        local = read_json(destination / 'source_episodes.json')
        remote_episodes = [x['source_episode'] for x in remote.get('episodes', [])]
        local_episodes = [x['source_episode'] for x in local.get('episodes', [])]
        if local_episodes[:len(remote_episodes)] != remote_episodes:
            raise ValueError('远端已有其他采集机添加的 episode，需要先合并，未覆盖')
    result = api.upload_folder(repo_id=repo, repo_type='dataset', folder_path=str(destination),
        allow_patterns=['README.md', 'source_episodes.json', 'meta/**', 'data/**', 'videos/**'],
        parent_commit=info.sha, commit_message=f'Add LeRobot episode {folder.name}')
    return dict(commit_url=str(result.commit_url),
                url=f'https://huggingface.co/datasets/{repo}')


def worker(output):
    from huggingface_hub import HfApi, get_token
    output = Path(output)
    with (output / '.hf-upload.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return
        for path in sorted(output.glob('*/upload.json')):
            job = read_json(path)
            if job.get('state') not in ('queued', 'uploading'):
                continue
            job.update(state='uploading', attempts=job.get('attempts', 0) + 1, updated_at=time.time())
            write_json(path, job)
            try:
                if not get_token():
                    raise RuntimeError('采集机尚未登录 Hugging Face，请完成浏览器授权')
                result = upload_episode(path.parent, job, HfApi())
                job.update(state='uploaded', error='', **result)
            except Exception as exc:
                # Never put HTTP request headers or credentials in the page status.
                message = str(exc) if isinstance(exc, (ValueError, RuntimeError)) else f'{type(exc).__name__}：上传失败，请检查账号权限和网络'
                job.update(state='failed', error=message, retry_after=time.time() + 60)
            job['updated_at'] = time.time()
            write_json(path, job)


class HubUploader:
    def __init__(self, output, config=ROOT / 'hf-config.json'):
        self.output = Path(output)
        self.config = Path(config)
        self.process = None
        self.checked = 0
        self.snapshot = dict(upload_state='awaiting_configuration', upload_repo='', upload_error='',
                             upload_pending=0, upload_failed=0, upload_completed=0, upload_url='')

    def state(self):
        return self.snapshot.copy()

    def tick(self, force=False):
        if not force and time.monotonic() - self.checked < 2:
            return
        self.checked = time.monotonic()
        config = read_json(self.config)
        repo = config.get('repo_id', '')
        if not config.get('enabled') or not repo:
            self.snapshot['upload_state'] = 'awaiting_configuration'
            return
        from huggingface_hub.utils import validate_repo_id
        try:
            validate_repo_id(repo)
            if '/' not in repo:
                raise ValueError('repo_id 必须为 用户名或组织/仓库名')
        except ValueError as exc:
            self.snapshot.update(upload_state='error', upload_error=str(exc))
            return
        self.output.mkdir(parents=True, exist_ok=True)
        jobs = []
        for folder in sorted(self.output.iterdir()):
            if not folder.is_dir() or folder.is_symlink():
                continue
            metadata = read_json(folder / 'metadata.json')
            if metadata.get('complete') is not True or metadata.get('recording') is not False or metadata.get('review_status') != 'saved':
                continue
            path = folder / 'upload.json'
            job = read_json(path)
            if not job:
                job = dict(repo_id=repo, format='lerobot',
                           task=metadata.get('task', {}).get('prompt') or config.get('task', 'Panthera teleoperation'),
                           state='queued', attempts=0, error='')
                write_json(path, job)
            if job.get('repo_id') != repo:
                continue  # Changing config never redirects an existing queued episode.
            if job.get('state') == 'failed' and job.get('attempts', 0) < 3 and time.time() >= job.get('retry_after', 0):
                job['state'] = 'queued'
                write_json(path, job)
            jobs.append(job)
        if self.process is not None and self.process.poll() is not None:
            self.process = None
        pending = sum(j['state'] in ('queued', 'uploading') for j in jobs)
        failures = [j for j in jobs if j['state'] == 'failed']
        completed = [j for j in jobs if j['state'] == 'uploaded']
        if pending and self.process is None:
            with (self.output / '.hf-upload.log').open('a') as log:
                self.process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()),
                    '--worker', str(self.output.resolve())], stdin=subprocess.DEVNULL,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.snapshot.update(upload_state='uploading' if pending else ('failed' if failures else ('uploaded' if completed else 'ready')),
            upload_repo=repo, upload_pending=pending, upload_failed=len(failures), upload_completed=len(completed),
            upload_error=failures[-1].get('error', '') if failures else '',
            upload_url=completed[-1].get('url', '') if completed else '')

    def retry(self):
        repo = read_json(self.config).get('repo_id')
        for path in self.output.glob('*/upload.json'):
            job = read_json(path)
            if job.get('repo_id') == repo and job.get('state') == 'failed':
                job.update(state='queued', attempts=0, error='')
                write_json(path, job)
        self.tick(force=True)
        return dict(ok=bool(repo), message='已重新排队' if repo else '请先配置 Hugging Face 仓库')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--worker', type=Path, required=True)
    worker(parser.parse_args().worker)

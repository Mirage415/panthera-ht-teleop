"""Episode lifecycle; all file mutations belong to the collection thread."""
from copy import deepcopy
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from mcap_io import McapSink


class Recorder:
    def __init__(self, output, metadata, csv_fields, *, joints_only=False):
        self.output = Path(output)
        self.template = metadata
        self.csv_fields = csv_fields
        self.joints_only = joints_only
        self.writer = self.csv_file = self.csv_writer = None
        self.path = None
        self.last_path = None
        self.last_result = None
        self.error = ''
        self.base = {}
        self.started = None
        self.meta = None
        # Restore an unreviewed, closed episode after a service restart.
        for path in sorted(self.output.glob('*/metadata.json')):
            try:
                previous = json.loads(path.read_text())
            except (OSError, ValueError):
                continue
            if previous.get('recording') is True:
                previous.update(recording=False, complete=False, error='上次服务异常退出，录制未正常关闭', review_status='pending')
                tmp=path.with_suffix('.recovery.tmp');tmp.write_text(json.dumps(previous,indent=2));tmp.replace(path)
            if previous.get('review_status') == 'pending' and previous.get('recording') is False:
                self.last_path, self.last_result = str(path.parent), previous
                break

    @property
    def active(self):
        return self.writer is not None

    @property
    def has_pending(self):
        return bool(self.last_result and self.last_result.get('review_status') == 'pending')

    def start(self, stats):
        if self.active:
            return self.state(stats)  # Repeated clicks do not split an episode.
        if self.has_pending:
            raise ValueError('上一段 episode 尚未处理，请先 Save 或 Discard')
        self.error = ''
        self.path = self.output / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        self.base = stats.copy()
        self.started = time.monotonic()
        self.meta = deepcopy(self.template)
        self.meta.update(start_time_ns=time.time_ns(), complete=False, recording=True, review_status='pending')
        try:
            self.path.mkdir(parents=True, exist_ok=False)
            self._save_metadata()
            self.csv_file = (self.path / 'joints.csv').open('x', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(self.csv_fields)
            self.writer = McapSink(self.path / 'data.mcap')
        except Exception as exc:
            self.error = str(exc)
            self.stop(stats, reason='start_error', error=str(exc))
            raise
        return self.state(stats)

    def _save_metadata(self):
        temporary = self.path / 'metadata.json.tmp'
        temporary.write_text(json.dumps(self.meta, indent=2))
        temporary.replace(self.path / 'metadata.json')

    def counts(self, stats):
        return {key: value - self.base.get(key, 0) for key, value in stats.items()}

    def write_joint(self, row):
        if self.active:
            self.csv_writer.writerow(row)

    def flush(self):
        if self.csv_file:
            self.csv_file.flush()

    def stop(self, stats, *, reason='user', error=''):
        if self.path is None:
            return self.state(stats)
        counts = self.counts(stats)
        failures = [error] if error else []
        for resource in (self.writer, self.csv_file):
            if resource is not None:
                try:
                    resource.close()
                except Exception as exc:
                    failures.append(str(exc))
        self.writer = self.csv_file = self.csv_writer = None
        self.error = '; '.join(failures)
        self.meta.update(end_time_ns=time.time_ns(), statistics=counts, recording=False,
                         duration_s=time.monotonic() - self.started, stop_reason=reason,
                         error=self.error, complete=not failures and bool(counts['rgb_frames'] and counts['joint_samples']))
        try:
            self._save_metadata()
        except Exception as exc:
            self.error = '; '.join(filter(None, [self.error, str(exc)]))
            self.meta['complete'] = False
            self.meta['error'] = self.error
        self.last_path = str(self.path)
        self.last_result = deepcopy(self.meta)
        self.path = None
        self.started = None
        return self.state(stats)

    def state(self, stats):
        return dict(recording=self.active, recording_state='recording' if self.active else ('error' if self.error else 'idle'),
                    episode_path=str(self.path) if self.path else '', last_episode_path=self.last_path or '',
                    elapsed_s=round(time.monotonic() - self.started, 2) if self.active else 0,
                    episode_statistics=self.counts(stats) if self.active else {},
                    last_episode_complete=self.last_result['complete'] if self.last_result else None,
                    last_episode_statistics=self.last_result.get('statistics', {}) if self.last_result else {},
                    recording_error=self.error, pending_review=self.has_pending and not self.active,
                    review_status='recording' if self.active else (self.last_result.get('review_status', '') if self.last_result else ''))

    def decide(self, decision):
        if self.active:
            raise ValueError('请先停止录制')
        if not self.last_path or not self.last_result:
            return dict(ok=False, message='没有待处理的 episode')
        previous = self.last_result.get('review_status')
        if previous == decision:
            return dict(ok=True, message='已处理，无需重复操作')
        if previous != 'pending':
            return dict(ok=False, message='此 episode 已处理；不会修改已保存或已丢弃的数据')
        if decision == 'saved' and not self.last_result.get('complete'):
            return dict(ok=False, message='此 episode 不完整，不能作为训练数据上传；可选择 Discard')
        if decision not in ('saved', 'discarded'):
            raise ValueError('invalid review decision')
        updated = deepcopy(self.last_result)
        updated.update(review_status=decision, reviewed_at_ns=time.time_ns())
        path = Path(self.last_path) / 'metadata.json'
        temporary = path.with_suffix('.review.tmp')
        temporary.write_text(json.dumps(updated, indent=2))
        temporary.replace(path)
        self.last_result = updated
        return dict(ok=True, message='已保留；导出与上传按数据集配置处理' if decision == 'saved' else '已丢弃，不进入训练集、不上传；本地原始文件保留供恢复')

"""Episode lifecycle and Foxglove services; mutations run on the capture thread."""
from concurrent.futures import Future, TimeoutError
from copy import deepcopy
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
from queue import Queue, Empty, Full
import threading
import time

import foxglove
from foxglove import MessageSchema, Schema
from foxglove.websocket import Service, ServiceSchema


class Recorder:
    def __init__(self, output, metadata, csv_fields, *, joints_only=False, context=None):
        self.output = Path(output)
        self.template = metadata
        self.csv_fields = csv_fields
        self.joints_only = joints_only
        self.context = context
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
            options = {'context': self.context}
            if self.joints_only:
                options['channel_filter'] = lambda channel: not channel.topic.startswith('/camera/')
            self.writer = foxglove.open_mcap(str(self.path / 'data.mcap'), **options)
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

    def capture_start_pose(self, row):
        if self.active and 'initial_pose' not in self.meta and 'leader_position' in row:
            self.meta['initial_pose'] = dict(leader=row['leader_position'], follower=row['position'],
                                             timestamp_ns=row['timestamp_ns'], unit='rad')
            self._save_metadata()

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
        return dict(ok=True, message='已保留，后台将转换并上传 LeRobot 数据集' if decision == 'saved' else '已丢弃，不进入训练集、不上传；本地原始文件保留供恢复')


class RecordingControl:
    """SDK service handlers enqueue; the camera thread opens/closes MCAP sinks."""
    def __init__(self, timeout=5, teleop=None, uploader=None, workflow=None):
        self.teleop = teleop
        self.uploader = uploader
        self.workflow = workflow
        self.queue = Queue(maxsize=16)
        self.timeout = timeout
        self.closed = threading.Event()

    def services(self):
        empty = MessageSchema(encoding='json', schema=Schema(name='CaptureRequest', encoding='jsonschema',
            data=b'{"type":"object","properties":{},"additionalProperties":false}'))
        response = MessageSchema(encoding='json', schema=Schema(name='CaptureResponse', encoding='jsonschema',
            data=b'{"type":"object","additionalProperties":true}'))
        services = [Service(name=f'/capture/{action}', schema=ServiceSchema(name=f'capture_{action}',
                    request=empty, response=response), handler=self.handler(action))
                for action in ('start', 'stop', 'save', 'discard', 'reset', 'reset_leader', 'reset_follower', 'status', 'upload_retry')]
        command = MessageSchema(encoding='json', schema=Schema(name='CollectionCommand', encoding='jsonschema',
            data=json.dumps(dict(type='object', properties={key: dict(type='string') for key in
                ('action', 'revision', 'name', 'catalog_id', 'task_id', 'prompt')},
                required=['action', 'revision'], additionalProperties=False)).encode()))
        services.append(Service(name='/capture/command', schema=ServiceSchema(name='collection_command',
            request=command, response=response), handler=self.handler('command')))
        return services

    def handler(self, action):
        def handle(request):
            payload = json.loads(request.payload or b'{}')
            if request.encoding != 'json' or not isinstance(payload, dict) or (action != 'command' and payload != {}):
                raise ValueError('请求必须是 JSON 空对象 {}')
            if self.closed.is_set():
                raise RuntimeError('采集服务正在关闭')
            future = Future()
            try:
                self.queue.put_nowait((action, future, payload))
            except Full:
                raise RuntimeError('采集操作队列已满，请稍后重试') from None
            try:
                return json.dumps(future.result(timeout=self.timeout)).encode()
            except TimeoutError:
                future.cancel()  # An expired, unprocessed start must never record later.
                raise RuntimeError('操作超时，请查看 /capture/status 确认当前状态') from None
        return handle

    def drain(self, recorder, stats, live):
        for _ in range(16):
            try:
                action, future, payload = self.queue.get_nowait()
            except Empty:
                break
            if not future.set_running_or_notify_cancel():
                continue
            try:
                if action == 'command':
                    if self.workflow is None:
                        raise ValueError('任务流程不可用')
                    action = payload.get('action')
                    if action not in ('start', 'stop', 'save', 'discard', 'reset', 'reset_leader', 'reset_follower', 'catalog_create', 'task_create', 'task_select'):
                        raise ValueError('未知操作')
                    if action != 'stop':
                        self.workflow.check_revision(payload.get('revision'), recorder, self.teleop)
                if action in ('catalog_create', 'task_create', 'task_select'):
                    result = self.workflow.configure(action, payload, recorder, self.teleop)
                elif action in ('reset', 'reset_leader', 'reset_follower'):
                    if self.workflow is None:
                        raise ValueError('尚未配置回位流程')
                    result = self.workflow.reset(recorder, self.teleop, role=action.removeprefix('reset_') if action != 'reset' else 'both')
                elif action in ('save', 'discard'):
                    was_starting = self.teleop is not None and self.teleop.phase == 'starting' and not recorder.active
                    if self.teleop is not None:
                        self.teleop.stop(recorder, stats)
                    else:
                        recorder.stop(stats)
                    if was_starting:
                        result = dict(ok=action == 'discard', message='已取消 Teleop 启动，尚未生成 episode')
                    else:
                        result = recorder.decide('saved' if action == 'save' else 'discarded')
                    if self.workflow is not None:
                        self.workflow.remember_episode(recorder)
                elif action == 'upload_retry':
                    result = self.uploader.retry() if self.uploader else dict(ok=False, message='上传尚未配置')
                elif self.teleop is not None and action in ('start', 'stop'):
                    if action == 'start' and self.workflow is not None:
                        self.workflow.before_start(recorder, self.teleop)
                    result = getattr(self.teleop, action)(recorder, stats, live) if action == 'start' else self.teleop.stop(recorder, stats)
                elif action == 'start' and not recorder.active:
                    if not live['camera_live'] or not live['follower_live']:
                        result = dict(ok=False, message='尚未开始：请确认相机有画面，并先启动 Teleop 使 follower 数据在线。')
                    else:
                        recorder.start(stats)
                        result = dict(ok=True, message='采集已开始')
                elif action == 'stop':
                    recorder.stop(stats)
                    result = dict(ok=not bool(recorder.error), message='采集已停止，文件已关闭' if not recorder.error else recorder.error)
                else:
                    result = dict(ok=True, message='正在采集' if recorder.active else '预览中')
                result.update(recorder.state(stats), **live)
                if self.teleop is not None:
                    result.update(self.teleop.state())
                if self.uploader is not None:
                    result.update(self.uploader.state())
                if self.workflow is not None:
                    result.update(self.workflow.state(recorder, self.teleop))
                future.set_result(result)
            except Exception as exc:
                future.set_exception(exc)

    def close(self):
        self.closed.set()
        while True:
            try:
                _, future, _ = self.queue.get_nowait()
            except Empty:
                return
            if not future.done():
                future.set_exception(RuntimeError('采集服务已关闭'))

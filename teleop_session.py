"""Own a Teleop supervisor without blocking the camera or starting motors on connect."""
import os
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent


class TeleopSession:
    def __init__(self, root=ROOT):
        self.root = Path(root)
        self.process = None
        self.log = None
        self.log_path = None
        self.phase = 'idle'
        self.error = ''
        self.deadline = 0
        self.reset_target = None
        self.reset_completed = False
        self.operation = 'teleop'
        self.reset_role = 'both'

    def state(self):
        return dict(teleop_state=self.phase if self.operation == 'teleop' else 'idle',
                    teleop_error=self.error if self.operation == 'teleop' else '',
                    reset_state=self.phase if self.operation == 'reset' else 'idle',
                    reset_error=self.error if self.operation == 'reset' else '',
                    reset_role=self.reset_role,
                    teleop_pid=self.process.pid if self.process and self.operation == 'teleop' else None,
                    teleop_log=str(self.log_path) if self.log_path and self.operation == 'teleop' else '',
                    reset_pid=self.process.pid if self.process and self.operation == 'reset' else None,
                    reset_log=str(self.log_path) if self.log_path and self.operation == 'reset' else '')

    def start(self, recorder, stats, live):
        if self.phase in ('starting', 'running'):
            return dict(ok=True, message='正在启动' if self.phase == 'starting' else '正在遥操作并采集')
        if self.process is not None:
            return dict(ok=False, message='机械臂操作仍在运行，请等待停止后再开始')
        if not live['camera_live']:
            return dict(ok=False, message='相机尚未就绪')
        if recorder.active:
            return dict(ok=False, message='已有录制，请先点击停止再启动一键遥操作')
        if recorder.has_pending:
            return dict(ok=False, message='上一段 episode 尚未处理，请先 Save 或 Discard')
        runtime = self.root / '.runtime'
        runtime.mkdir(exist_ok=True)
        self.log_path = runtime / f'page-teleop-{time.time_ns()}.log'
        self.log = self.log_path.open('x')
        try:
            self.process = subprocess.Popen(
                [sys.executable, '-u', str(self.root / 'teleop.py'), '--owner-pid', str(os.getpid())],
                cwd=self.root, stdin=subprocess.DEVNULL, stdout=self.log, stderr=subprocess.STDOUT,
                start_new_session=True)
        except Exception:
            self.log.close()
            self.log = None
            raise
        self.operation = 'teleop'
        self.phase = 'starting'
        self.error = ''
        self.deadline = time.monotonic() + 30
        return dict(ok=True, message='正在启动 Teleop；follower 就绪后自动开始采集')

    def reset(self, pose, role='both'):
        if role not in ('leader', 'follower', 'both'):
            raise ValueError('未知机械臂')
        if self.process is not None:
            raise ValueError('请等待当前机械臂进程停止')
        runtime = self.root / '.runtime'
        runtime.mkdir(exist_ok=True)
        self.reset_target = runtime / f'reset-{time.time_ns()}.json'
        self.reset_target.write_text(json.dumps(pose))
        self.log_path = self.reset_target.with_suffix('.log')
        self.log = self.log_path.open('x')
        try:
            self.process = subprocess.Popen(
                [sys.executable, '-u', str(self.root / 'teleop.py'), '--owner-pid', str(os.getpid()),
                 '--reset-target', str(self.reset_target), '--role', role], cwd=self.root, stdin=subprocess.DEVNULL,
                stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        except Exception:
            self.log.close()
            self.log = None
            raise
        self.reset_completed = False
        self.error = ''
        self.operation = 'reset'
        self.reset_role = role
        self.phase = 'resetting'
        return dict(ok=True, message=f'正在独立回位：{role}，不启动遥操作或录制')

    def stop(self, recorder, stats, *, error=''):
        if error:
            self.error = error
        # Signal the supervisor before potentially slow disk finalization.
        if self.process is not None and self.process.poll() is None:
            if self.phase != 'stopping':
                try:
                    self.process.send_signal(signal.SIGTERM)
                except ProcessLookupError:
                    pass
            self.phase = 'stopping'
        try:
            recorder.stop(stats, reason='teleop_error' if error else 'user', error=error)
        finally:
            if self.process is None:
                self.phase = 'error' if self.error else 'idle'
        return dict(ok=not bool(self.error or recorder.error),
                    message=self.error or recorder.error or ('正在停止回位' if self.operation == 'reset' else '正在停止 Teleop，采集文件已关闭'))

    def tick(self, recorder, stats, live):
        if self.process is None:
            return
        code = self.process.poll()
        if code is not None:
            reset_ok = (self.phase == 'resetting' and code == 0 and self.reset_target is not None
                        and self.reset_target.with_suffix('.done').exists())
            unexpected = self.phase != 'stopping' and not reset_ok
            self.log.close()
            self.log = None
            self.process = None
            if unexpected:
                with self.log_path.open('rb') as handle:
                    handle.seek(0, 2)
                    handle.seek(max(0, handle.tell() - 2000))
                    tail = handle.read().decode(errors='replace').strip()
                if self.operation == 'reset':
                    self.error = f'{self.reset_role} 回位失败（退出码 {code}）：{tail}'
                else:
                    self.error = f'Teleop 已退出（{code}）：{tail}'
                    recorder.stop(stats, reason='teleop_exit', error=self.error)
            self.phase = 'error' if self.error else 'idle'
            self.reset_completed = reset_ok
            return
        if self.phase in ('stopping', 'resetting'):
            return
        owned_feedback = (live['follower_live'] and live.get('follower_supervisor_pid') == self.process.pid)
        if self.phase == 'starting':
            if not live['camera_live']:
                self.stop(recorder, stats, error='启动期间相机断流')
            elif owned_feedback:
                try:
                    recorder.start(stats)
                except Exception as exc:
                    self.stop(recorder, stats, error=f'无法开始保存：{exc}')
                else:
                    self.phase = 'running'
            elif time.monotonic() > self.deadline:
                self.stop(recorder, stats, error='Teleop 启动超时，未收到本次进程的 follower 数据')
        elif self.phase == 'running' and (not live['camera_live'] or not owned_feedback):
            self.stop(recorder, stats, error='相机或 follower 数据断流，已停止本次遥操作和采集')

    def close(self, recorder, stats, error=''):
        self.stop(recorder, stats, error=error)
        if self.process is not None:
            try:
                self.process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                # Do not kill the supervisor: it owns worker cleanup and also
                # detects our exit via --owner-pid.
                self.error = ('回位' if self.operation == 'reset' else 'Teleop') + '退出未完成，请检查机械臂状态'
            finally:
                if self.log:
                    self.log.close()
                    self.log = None

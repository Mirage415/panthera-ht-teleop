"""Persistent task catalogs and episode gates, inspired by embodied-ops.

Hardware motion remains in TeleopSession; this module never opens a motor.
"""
from copy import deepcopy
import json
from pathlib import Path
import uuid


class CollectionWorkflow:
    def __init__(self, path):
        self.path = Path(path)
        self.data = json.loads(self.path.read_text()) if self.path.exists() else dict(
            version=1, catalogs=[], selected_catalog='', selected_task='', needs_reset=False)
        self.generation = uuid.uuid4().hex
        self.revision = 0
        self.signature = None

    def persist(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix('.tmp')
        tmp.write_text(json.dumps(self.data, ensure_ascii=False, indent=2))
        tmp.replace(self.path)
        self.revision += 1

    def selection(self):
        for catalog in self.data['catalogs']:
            if catalog['id'] == self.data['selected_catalog']:
                for task in catalog['tasks']:
                    if task['id'] == self.data['selected_task']:
                        return dict(catalog_id=catalog['id'], catalog_name=catalog['name'],
                                    task_id=task['id'], prompt=task['prompt'])
        return None

    def state(self, recorder, teleop):
        phase = teleop.phase if teleop else 'idle'
        if phase in ('starting', 'stopping', 'resetting'):
            stage = phase
        elif recorder.active:
            stage = 'recording'
        elif recorder.has_pending:
            stage = 'review'
        elif self.data['needs_reset']:
            stage = 'reset_required'
        elif not self.selection():
            stage = 'needs_task'
        else:
            stage = 'ready'
        signature = (stage, phase, str(recorder.path), recorder.last_path,
                     recorder.last_result.get('review_status') if recorder.last_result else None)
        if signature != self.signature:
            self.signature = signature
            self.revision += 1
        counts = dict(saved=0, discarded=0)
        # Episode totals are refreshed only on transitions, not at camera rate.
        if getattr(self, '_counts_signature', None) != (signature, self.data['selected_task']):
            for path in recorder.output.glob('*/metadata.json'):
                try:
                    meta = json.loads(path.read_text())
                except (OSError, ValueError):
                    continue
                if meta.get('task', {}).get('task_id') == self.data['selected_task']:
                    decision = meta.get('review_status')
                    if decision in counts:
                        counts[decision] += 1
            self._counts = counts
            self._counts_signature = (signature, self.data['selected_task'])
        return dict(workflow_phase=stage, workflow_revision=f'{self.generation}:{self.revision}',
                    task_catalogs=deepcopy(self.data['catalogs']), selected_task=self.selection(),
                    selected_catalog_id=self.data['selected_catalog'],
                    task_episode_counts=self._counts.copy(), reset_mode='automatic',
                    reset_available=bool(self.data.get('reset_pose')),
                    reset_completed_roles=self.data.get('reset_completed_roles', []))

    def check_revision(self, value, recorder, teleop):
        if value != self.state(recorder, teleop)['workflow_revision']:
            raise ValueError('状态已变化，请等待面板更新后重试')

    @staticmethod
    def text(value, label, maximum):
        if not isinstance(value, str) or not value.strip() or len(value) > maximum:
            raise ValueError(f'{label} 不能为空，最多 {maximum} 个字符')
        if value != value.strip() or '\n' in value or '\r' in value:
            raise ValueError(f'{label} 需为单行文本且首尾不能有空格')
        return value

    def configure(self, action, payload, recorder, teleop):
        stage = self.state(recorder, teleop)['workflow_phase']
        if stage not in ('ready', 'needs_task'):
            raise ValueError('先完成当前 episode 的 Save / Discard 和 Reset，再切换任务')
        if action == 'catalog_create':
            name = self.text(payload.get('name'), '任务集名称', 120)
            if any(c['name'] == name for c in self.data['catalogs']):
                raise ValueError('任务集名称已存在')
            catalog = dict(id=uuid.uuid4().hex, name=name, tasks=[])
            self.data['catalogs'].append(catalog)
            self.data.update(selected_catalog=catalog['id'], selected_task='')
        else:
            catalog = next((c for c in self.data['catalogs'] if c['id'] == payload.get('catalog_id')), None)
            if catalog is None:
                raise ValueError('任务集不存在')
            if action == 'task_create':
                prompt = self.text(payload.get('prompt'), 'Prompt', 2000)
                if any(t['prompt'] == prompt for t in catalog['tasks']):
                    raise ValueError('该 Prompt 已存在，请直接选择')
                task = dict(id=uuid.uuid4().hex, prompt=prompt)
                catalog['tasks'].append(task)
                self.data.update(selected_catalog=catalog['id'], selected_task=task['id'])
            elif action == 'task_select':
                task_id = payload.get('task_id', '')
                if task_id and not any(t['id'] == task_id for t in catalog['tasks']):
                    raise ValueError('子任务不存在')
                self.data.update(selected_catalog=catalog['id'], selected_task=task_id)
            else:
                raise ValueError('未知任务操作')
        self.persist()
        return dict(ok=True, message='任务已更新')

    def before_start(self, recorder, teleop):
        if self.state(recorder, teleop)['workflow_phase'] != 'ready':
            raise ValueError('请先选择子任务，并完成上一段的 Save / Discard → Reset')
        recorder.template['task'] = deepcopy(self.selection())
        self.persist()

    def reset(self, recorder, teleop, role='both'):
        if role not in ('leader', 'follower', 'both'):
            raise ValueError('未知机械臂')
        if recorder.active or recorder.has_pending or (teleop and teleop.phase in ('starting', 'running', 'stopping', 'resetting')):
            raise ValueError('先停止 Teleop 并 Save / Discard，再 Reset')
        if not self.data.get('reset_pose') or teleop is None:
            raise ValueError('本轮没有完整的主从臂起始姿态，不能自动回位')
        completed = set(self.data.get('reset_completed_roles', []))
        completed.difference_update(('leader', 'follower') if role == 'both' else (role,))
        self.data['reset_completed_roles'] = sorted(completed)
        self.data['needs_reset'] = True
        self.persist()
        return teleop.reset(self.data['reset_pose'], role=role)

    def remember_episode(self, recorder):
        if (recorder.last_result and recorder.last_result.get('initial_pose')
                and recorder.last_result['initial_pose'] != self.data.get('reset_pose')):
            self.data['reset_pose'] = deepcopy(recorder.last_result['initial_pose'])
            self.data['needs_reset'] = True
            self.data['reset_completed_roles'] = []
            self.persist()

    def tick(self, teleop, recorder):
        pose = (recorder.meta if recorder.active else recorder.last_result) or {}
        if pose.get('initial_pose') and pose['initial_pose'] != self.data.get('reset_pose'):
            self.data['reset_pose'] = deepcopy(pose['initial_pose'])
            self.data['needs_reset'] = True
            self.data['reset_completed_roles'] = []
            self.persist()
        if teleop.reset_completed:
            completed = set(self.data.get('reset_completed_roles', []))
            role = getattr(teleop, 'reset_role', 'both')
            completed.update(('leader', 'follower') if role == 'both' else (role,))
            self.data['reset_completed_roles'] = sorted(completed)
            self.data['needs_reset'] = not {'leader', 'follower'}.issubset(completed)
            self.persist()
            teleop.reset_completed = False

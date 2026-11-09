"""Compact collection page, with task management and diagnostics folded away."""
import json
from pathlib import Path
import time
import numpy as np
from viser_playback import Playback

class CollectionPanel:
    def __init__(self, app, collection):
        self.app=app;self.collection=collection;self.rendering=False;self.revision=None;self.updated=0
        self.catalog_ids={};self.task_ids={};self.last_playback=None
        self.playback=Playback(collection.output)
        gui=app.server.gui
        self.status=gui.add_markdown('正在连接采集服务…')
        self.catalog=gui.add_dropdown('任务集',('暂无任务集',))
        self.task=gui.add_dropdown('任务',('暂无任务',))
        self.catalog.on_update(self.select_catalog);self.task.on_update(self.select_task)
        self.start=gui.add_button('开始采集',color='teal');self.start.on_click(lambda e:self.send('start',e,depth=self.depth.value))
        self.stop=gui.add_button('停止录制');self.stop.on_click(lambda e:self.send('stop',e))
        self.save=gui.add_button('保存',color='teal');self.save.on_click(lambda e:self.send('save',e))
        self.discard=gui.add_button('丢弃');self.discard.on_click(lambda e:self.send('discard',e))
        self.notice=gui.add_markdown('停止录制只结束文件；顶部“停止”同时停止机械臂。')
        with gui.add_folder('任务管理',expand_by_default=False):
            self.name=gui.add_text('新任务集名称',initial_value='')
            gui.add_button('新建任务集').on_click(lambda e:self.send('catalog_create',e,name=self.name.value))
            self.prompt=gui.add_text('新任务描述',initial_value='')
            gui.add_button('新建任务').on_click(lambda e:self.send('task_create',e,catalog_id=self.catalog_ids.get(self.catalog.value,''),prompt=self.prompt.value))
        with gui.add_folder('录制与上传详情',expand_by_default=False):
            self.depth=gui.add_checkbox('同时保存原始深度',False)
            self.details=gui.add_markdown('')
            gui.add_button('重试上传').on_click(lambda e:self.send('upload_retry',e))
        with gui.add_folder('历史与回放',expand_by_default=False):
            self.episodes=gui.add_dropdown('录制',('暂无录制',))
            gui.add_button('刷新列表').on_click(self.refresh_episodes)
            self.seek=gui.add_slider('进度 %',min=0,max=100,step=.1,initial_value=0)
            self.episodes.on_update(self.seek_episode);self.seek.on_update(self.seek_episode)
            self.replay_image=gui.add_image(np.zeros((240,320,3),dtype=np.uint8),label='离线回放')
            self.replay_info=gui.add_markdown('拖动进度查看图像和关节；回放不会控制实机。')
        self.refresh_episodes(None)
    def send(self, action, event, **payload):
        if event.client is None or self.rendering:return
        try:self.collection.submit(action,event.client.client_id,self.revision,**payload)
        except Exception as exc:self.status.content=str(exc)
    def select_catalog(self,event):
        self.send('task_select',event,catalog_id=self.catalog_ids.get(self.catalog.value,''),task_id='')
    def select_task(self,event):
        self.send('task_select',event,catalog_id=self.catalog_ids.get(self.catalog.value,''),task_id=self.task_ids.get(self.task.value,''))
    def refresh_episodes(self,event):
        names=[]
        for path in sorted(self.collection.output.glob('*/metadata.json'),reverse=True):
            try:meta=json.loads(path.read_text())
            except (OSError,ValueError):continue
            if not meta.get('recording') and (path.parent/'data.mcap').exists():names.append(path.parent.name)
        self.episodes.options=tuple(names) or ('暂无录制',)
    def seek_episode(self,event):
        if event.client is None:return
        try:self.playback.seek(self.episodes.value,self.seek.value/100)
        except Exception as exc:self.replay_info.content=str(exc)
    def update(self):
        s=self.collection.snapshot
        if not s:return
        self.rendering=True
        try:
            self.revision=s['workflow_revision'];catalogs=s['task_catalogs']
            self.catalog_ids={c['name']:c['id'] for c in catalogs}
            options=tuple(self.catalog_ids) or ('暂无任务集',)
            if self.catalog.options!=options:self.catalog.options=options
            selected=next((c for c in catalogs if c['id']==s['selected_catalog_id']),None)
            if selected and self.catalog.value!=selected['name']:self.catalog.value=selected['name']
            self.task_ids={t['prompt']:t['id'] for t in selected['tasks']} if selected else {}
            options=tuple(self.task_ids) or ('暂无任务',)
            if self.task.options!=options:self.task.options=options
            task=s['selected_task']
            if task and self.task.value!=task['prompt']:self.task.value=task['prompt']
            active=s['recording'];pending=s['pending_review'];ready=s['workflow_phase']=='ready'
            self.start.disabled=not ready or not self.app.hardware or not self.app.state.ready or not s.get('camera',{}).get('connected')
            self.start.label='开始控制并录制' if self.app.state.mode in ('idle','hold') else '开始录制'
            self.stop.disabled=not active;self.save.disabled=not (active or pending);self.discard.disabled=not(active or pending)
            self.depth.disabled=active;self.catalog.disabled=self.task.disabled=active or pending
            counts=s['task_episode_counts'];stats=s['episode_statistics'] if active else s.get('last_episode_statistics',{})
            self.status.content=(f'**{"录制中" if active else "待审核" if pending else "待采集"}**'
                +(f' · {s["elapsed_s"]:.1f} 秒' if active else '')+(' · 等待机械臂' if not self.app.state.ready else '')+(' · 等待相机' if not s.get('camera',{}).get('connected') else '')+f'\n\n{s["message"]}\n\n已保存 {counts["saved"]} 段 · 已丢弃 {counts["discarded"]} 段')
            upload_labels={'awaiting_configuration':'未配置','ready':'就绪','uploading':'转换/上传中','uploaded':'已上传','failed':'失败','error':'错误'}
            path=s['episode_path'] or s['last_episode_path']
            cam=s.get('camera',{});latency=cam.get('camera_latency_ms')
            self.details.content=(f'RGB {stats.get("rgb_frames",0)} 帧 · 深度 {stats.get("depth_frames",0)} 帧 · 关节 {stats.get("joint_samples",0)} 组'
                f'\n\n反馈缺口 {stats.get("sequence_gaps",0)} · 图像缺口 {stats.get("camera_gaps",0)}'
                f'\n\n相机采集延迟：{str(latency)+" ms" if latency is not None else "不可用"}（不含网络与显示）'
                f'\n\n文件：`{path or "—"}`\n\n上传：{upload_labels.get(s["upload_state"],s["upload_state"])} · 待处理 {s["upload_pending"]} / 完成 {s["upload_completed"]} / 失败 {s["upload_failed"]}'
                f'\n\n{s.get("recording_error","")}\n\n{s.get("camera_error","")}\n\n{s.get("upload_error","")}'
                +(f'\n\n[查看数据集]({s["upload_url"]})' if s.get('upload_url') else '')
                +'\n\n主从模式保留可导出的 B 臂目标；网页模式保存双臂原始状态和六关节目标，不伪造夹爪动作。')
            replay=self.playback.snapshot
            if replay and replay is not self.last_playback:
                self.last_playback=replay
                if 'error' in replay:self.replay_info.content=replay['error']
                else:
                    if replay['image'] is not None:self.replay_image.image=replay['image']
                    text=f'**离线回放** · {replay["seconds"]:.2f} / {replay["duration"]:.2f} 秒\n\n'
                    for topic,row in replay['joints'].items():
                        arm='A' if topic.startswith('/leader') else 'B'
                        text+=f'{arm} 臂（rad）：'+', '.join(f'{q:.3f}' for q in row['position'])+'\n\n'
                    self.replay_info.content=text+'回放不向电机发送指令。'
                    if replay['image'] is None:self.replay_image.image=np.zeros((240,320,3),dtype=np.uint8)
        finally:self.rendering=False

#!/usr/bin/env python3
"""Viser dual-arm joint editor. Preview is always available; hardware is opt-in."""
import argparse
from functools import partial
from collections import deque
import os
import json
import multiprocessing as mp
from pathlib import Path
import queue
import signal
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation
import viser
from viser.extras import ViserUrdf
import yourdfpy
from viser_control import ConsoleState, JOINT_NAMES, LOWER, UPPER, ring_angle
from viser_worker import worker, put_latest
from viser_console import EventLog, RecoveryRequest
from viser_camera import CameraBridge
from viser_collection import Collection
from viser_collection_ui import CollectionPanel

ROOT = Path(__file__).resolve().parent
ASSET = ROOT / 'Panthera-HT_SDK/panthera_python/Panthera-HT_description'
URDF = ASSET / 'urdf/Panthera-HT_description_follower.urdf'
ARMS = ('A · CAN1', 'B · CAN2')
JOINTS = ('J1 · 基座', 'J2 · 肩关节', 'J3 · 肘关节', 'J4 · 腕部俯仰', 'J5 · 腕部旋转', 'J6 · 末端旋转')


def load_model():
    def filename(fname):
        name = fname
        prefix = 'package://Panthera-HT_description/'
        return str(ASSET / name[len(prefix):]) if name.startswith(prefix) else str(URDF.parent / name)
    return yourdfpy.URDF.load(str(URDF), filename_handler=filename, load_meshes=True,
                             build_scene_graph=True, load_collision_meshes=False)


class App:
    def __init__(self, host, port, hardware):
        self.hardware = hardware
        self.lock = threading.RLock()
        self.state = ConsoleState()
        self.settings=json.loads((ROOT/'teleop-config.json').read_text()) if (ROOT/'teleop-config.json').exists() else {}
        self.state.speed=float(self.settings.get('max_target_speed_rad_s',1.5))
        self.page_name='机械臂';self.camera_aspect=16/9;self.view_lock=threading.RLock()
        self.log=EventLog(ROOT/'.runtime/viser')
        self.recovery=RecoveryRequest()
        self.faulted=False
        self.fault_at=0.
        self.terminating=False
        self.killed=False
        self.worker_state={}
        self.history=deque(maxlen=150)
        self.last_plot=0.
        self.last_status=0.
        self.boot=time.monotonic()
        self.alive = True
        self.process = None
        self.commands = None
        self.feedback_queue = None
        self.start_time = 0.
        self.last_feedback = 0.
        self.worker_active = [False, False]
        self.ctx = mp.get_context('spawn')
        self.sample_queue=self.ctx.Queue(maxsize=512)
        self.recording_event=self.ctx.Event()
        self.message = '预览模式：未连接电机' if not hardware else '正在连接两只机械臂…'
        self.dragging = False
        self.arm_index = 0
        self.joint_index = 0
        self.anchor_rotation = Rotation.identity()
        # Viser 1.1.1 uses websockets' 20s defaults. Bound loss detection to ~3s.
        import websockets.asyncio.server as ws
        original_serve = ws.serve
        ws.serve = partial(original_serve, ping_interval=1., ping_timeout=2.)
        try:
            self.server = viser.ViserServer(host=host, port=port, label='Viser')
        finally:
            ws.serve = original_serve
        self.server.gui.configure_theme(dark_mode=True,
                                        control_width='medium', show_logo=False,
                                        show_share_button=False, brand_color=(39, 185, 171))
        self.server.scene.set_up_direction('+z')
        self.server.initial_camera.position = (1.4, -1.7, 1.25)
        self.server.initial_camera.look_at = (0., 0., .20)
        self.server.initial_camera.up_direction = (0., 0., 1.)
        self.server.scene.world_axes.visible = False
        self.floor=self.server.scene.add_grid('/floor', width=2.4, height=1.8, cell_size=.1, section_size=.5,
                                   cell_color=(48,62,77), section_color=(76,94,110),
                                   plane_color=(20,28,39), plane_opacity=.85)
        self.roots=[]
        self.actual_urdfs=[]
        self.actual_models = []
        self.target_models = []
        self.urdfs = []
        self.bases = [np.array([0.,-.42,0.]), np.array([0.,.42,0.])]
        for i in range(2):
            base = f'/arm{i}'
            self.roots.append(self.server.scene.add_frame(base, position=self.bases[i], show_axes=False))
            actual_urdf=load_model();self.actual_urdfs.append(actual_urdf)
            actual = ViserUrdf(self.server, actual_urdf, root_node_name=base+'/actual',
                              mesh_color_override=(.70,.76,.82,1.))
            urdf = load_model()
            target = ViserUrdf(self.server, urdf, root_node_name=base+'/target',
                              mesh_color_override=(.12,.82,.70,.30))
            self.actual_models.append(actual); self.target_models.append(target); self.urdfs.append(urdf)
            self.server.scene.add_label(base+'/label', ARMS[i], position=(0.,0.,.62),
                                        font_screen_scale=.9, anchor='center')
        self.gizmo = self.server.scene.add_transform_controls('/joint_ring', scale=80., fixed=True, line_width=3.,
                    active_axes=(True,True,False), disable_axes=True, disable_sliders=True,
                    depth_test=False, rotation_limits=((0.,0.),(0.,0.),(-10.,10.)))
        self.gizmo.on_update(self.ring_update)
        self.status=self.server.gui.add_markdown('正在连接…')
        self.fault_banner=self.server.gui.add_markdown('')
        self.stop_button=self.server.gui.add_button('停止',color='red')
        self.stop_button.on_click(self.stop_control)
        self.pages=self.server.gui.add_button_group('页面',('机械臂','相机','采集','数据','日志'))
        self.pages.on_click(self.switch_page)
        self.page_panels={}
        self.page_panels['机械臂']=self.server.gui.add_folder(None)
        with self.page_panels['机械臂']:
            self.operation_mode=self.server.gui.add_dropdown('控制模式',('网页双臂控制','主从遥操 A → B'))
            self.start_button=self.server.gui.add_button('开始',color='teal',disabled=True)
            self.start_button.on_click(lambda e:self.begin_mode('web' if self.operation_mode.value=='网页双臂控制' else 'teleop',e))
            self.home_button=self.server.gui.add_button('双臂复位',disabled=not hardware)
            self.home_button.on_click(self.reset_both)
            self.progress=self.server.gui.add_progress_bar(0,visible=False,color='teal')
            self.reset_info=self.server.gui.add_markdown('')
            self.retry=self.server.gui.add_button('重新连接',disabled=not hardware,visible=False)
            self.retry.on_click(self.retry_worker)
            self.arm=self.server.gui.add_dropdown('机械臂',ARMS,initial_value=ARMS[0])
            self.joint=self.server.gui.add_dropdown('关节',JOINTS,initial_value=JOINTS[0])
            self.arm.on_update(self.select);self.joint.on_update(self.select)
            with self.server.gui.add_folder('关节角度',expand_by_default=False):
                self.sliders=[]
                for j in range(6):
                    h=self.server.gui.add_slider(JOINTS[j]+' (°)',min=float(np.degrees(LOWER[j])),
                        max=float(np.degrees(UPPER[j])),step=.1,initial_value=0.)
                    h.on_update(lambda e,j=j:self.slider_update(j,e));self.sliders.append(h)
                self.sync_button=self.server.gui.add_button('目标对齐实测');self.sync_button.on_click(self.sync)
                self.readout=self.server.gui.add_markdown('')
            with self.server.gui.add_folder('单臂与参数',expand_by_default=False):
                self.enable_buttons=[]
                for i in range(2):
                    btn=self.server.gui.add_button(f'仅启用 {ARMS[i]}',disabled=True)
                    btn.on_click(lambda e,i=i:self.toggle_arm(i,e));self.enable_buttons.append(btn)
                self.speed=self.server.gui.add_slider('目标速度 (rad/s)',min=.05,max=1.5,step=.05,initial_value=self.state.speed)
                self.speed.on_update(self.speed_update)
                self.show_target=self.server.gui.add_checkbox('显示目标模型',True);self.show_target.on_update(self.visibility)
                self.server.gui.add_markdown('沿用仓库参数：关节 1.5 rad/s、夹爪 4.0 rad/s、跟踪阈值 0.6 rad。\n\n回零为六关节低速运动，不重设零位；停止时请承托两臂。')
        self.page_panels['相机']=self.server.gui.add_folder(None,visible=False)
        with self.page_panels['相机']:
            self.camera_view=self.server.gui.add_dropdown('画面',('彩色','深度'))
            self.camera_show=self.server.gui.add_checkbox('实时画面',True)
            self.camera_status=self.server.gui.add_markdown('正在连接 RealSense…')
            self.server.gui.add_markdown('[打开独立相机页面](http://10.209.66.60:8080/)')
        output=ROOT/'recordings' if hardware else ROOT/'.runtime/preview-recordings'
        self.collection=Collection(self,output)
        self.page_panels['采集']=self.server.gui.add_folder(None,visible=False)
        with self.page_panels['采集']:
            self.collection_panel=CollectionPanel(self,self.collection)
        self.page_panels['数据']=self.server.gui.add_folder(None,visible=False)
        with self.page_panels['数据']:
            self.data_arm=self.server.gui.add_dropdown('机械臂',ARMS,initial_value=ARMS[0])
            self.data_joint=self.server.gui.add_dropdown('曲线关节',JOINTS+('夹爪',),initial_value=JOINTS[0])
            self.data_metric=self.server.gui.add_dropdown('曲线',('位置','速度','力矩'))
            self.data_table=self.server.gui.add_markdown('等待数据…')
            series=({},dict(label='实测',stroke='#d4e0ef',width=2),dict(label='参考',stroke='#29baaa',width=2),dict(label='目标',stroke='#f2ae58',width=1))
            self.plot=self.server.gui.add_uplot((np.array([0.]),)*4,series,title='关节位置 · 30 秒',scales={'x':{'time':False}},height=190)
            self.error_plot=self.server.gui.add_uplot((np.array([0.]),)*2,({},dict(label='误差 rad',stroke='#ed7979',width=2)),
                title='跟踪误差',scales={'x':{'time':False}},height=130)
            self.system_info=self.server.gui.add_markdown('')
        self.page_panels['日志']=self.server.gui.add_folder(None,visible=False)
        with self.page_panels['日志']:
            self.fault_detail=self.server.gui.add_markdown('暂无故障')
            self.events_view=self.server.gui.add_markdown('')
        self.server.on_client_disconnect(self.disconnect)
        if hardware: self.start_worker()
        else: self.state.feedback(np.zeros((2,6)))
        self.update_scene()
        self.camera=CameraBridge(self)
        self.log.add("Viser 控制台启动；默认不使能电机")

    def switch_page(self,event):
        if event.client is None:return
        with self.view_lock:
            self.page_name=event.target.value
            for name,panel in self.page_panels.items():panel.visible=name==self.page_name
            camera=self.page_name=='相机'
            if camera:self.camera_aspect=max(.2,float(event.client.camera.aspect))
            self.floor.visible=not camera
            for root in self.roots:root.visible=not camera
            self.gizmo.visible=not camera and self.state.mode not in ('reset','teleop')
            self.server.scene.set_background_image(None)

    def disconnect(self,client):
        with self.lock:
            owned=self.state.owner==client.client_id
            pending=self.recovery.pending and self.recovery.pending['client']==client.client_id
            self.state.disconnected(client.client_id)
            if pending:self.recovery.cancel()
            if owned or pending:
                self.dragging=False;self.message='控制窗口断开，已停止运动并取消待执行复位'
                self.log.add(self.message,'stop')

    def client_id(self, event):
        return event.client.client_id if event.client is not None else None

    def fail_event(self, exc):
        self.message = str(exc)
        self.log.add(self.message,'rejected')

    def select(self, event):
        with self.lock:
            client=self.client_id(event)
            if client is not None and self.state.owner not in (None,client):
                self.arm.value=ARMS[self.arm_index];self.joint.value=JOINTS[self.joint_index]
                self.message='另一浏览器正在控制';return
            self.dragging = False
            self.arm_index = ARMS.index(self.arm.value)
            self.joint_index = JOINTS.index(self.joint.value)
            self.refresh_sliders()
            self.update_scene()

    def refresh_sliders(self):
        for j,h in enumerate(self.sliders): h.value = float(np.degrees(self.state.desired[self.arm_index,j]))

    def slider_update(self, j, event):
        client = self.client_id(event)
        if client is None:return
        with self.lock:
            q = self.state.desired[self.arm_index].copy()
            q[j] = np.radians(event.target.value)
            try:
                self.state.target(self.arm_index,q,client)
                self.joint_index=j;self.joint.value=JOINTS[j]
                self.message = '目标已更新' if self.state.enabled[self.arm_index] else '模型预览；实机尚未启用'
            except ValueError as exc:self.fail_event(exc);self.refresh_sliders()
            self.update_scene()

    def speed_update(self,event):
        client=self.client_id(event)
        if client is None:return
        with self.lock:
            if self.state.mode=='reset':
                self.speed.value=self.state.speed;return
            if self.state.owner not in (None,client):
                self.speed.value=self.state.speed;self.message='另一浏览器正在控制';return
            self.state.speed=float(event.target.value)

    def begin_mode(self,mode,event):
        with self.lock:
            try:
                if not self.hardware:raise ValueError('预览服务不连接电机')
                if self.recovery.pending:raise ValueError('正在恢复连接，请等待或停止')
                self.state.begin(mode,self.client_id(event))
                self.message={'web':'网页双臂控制已启动','teleop':'主从遥操已启动：手动拖动 A，B 跟随'}[mode]
                self.log.add(self.message,'start');self.refresh_sliders()
            except ValueError as exc:self.fail_event(exc)

    def toggle_arm(self,i,event):
        with self.lock:
            try:
                if self.state.enabled[i]:self.state.disable(i);self.message=f'{ARMS[i]} 已停止'
                else:
                    if not self.hardware:raise ValueError('预览服务不连接电机')
                    if self.recovery.pending:raise ValueError('正在恢复连接，请等待或停止')
                    self.state.enable(i,self.client_id(event));self.message=f'{ARMS[i]} 网页控制已启动'
                self.log.add(self.message);self.refresh_sliders();self.update_scene()
            except ValueError as exc:self.fail_event(exc)

    def reset_both(self,event):
        with self.lock:
            try:
                client=self.client_id(event);self.state.require_owner(client)
                if not self.hardware:raise ValueError('预览服务不连接电机')
                if self.recovery.pending:raise ValueError('复位请求已提交，请等待或停止')
                if self.state.ready:
                    self.state.begin('reset',client)
                    self.dragging=False
                    self.message='双臂正在低速回零，完成后保持零位';self.log.add(self.message,'reset')
                    self.refresh_sliders()
                else:
                    if self.process is not None and self.process.is_alive():raise ValueError('控制进程尚在停止/连接中，请稍后重试')
                    self.recovery.request(client)
                    self.start_worker()
                    self.message='复位请求已提交：先重新连接，再从实测姿态低速回零'
                    self.log.add(self.message,'reset')
            except ValueError as exc:
                suffix='；当前控制保持运行，未执行新的复位' if any(self.state.enabled) else ''
                self.fail_event(ValueError(str(exc)+suffix))

    def stop_control(self,event):
        with self.lock:
            self.recovery.cancel();self.state.disable();self.dragging=False
            self.message='用户停止：已请求双臂制动，复位请求已取消；请承托机械臂'
            self.log.add(self.message,'stop')

    def sync(self,event):
        with self.lock:
            try:self.state.target(self.arm_index,np.clip(self.state.actual[self.arm_index],LOWER,UPPER),self.client_id(event))
            except ValueError as exc:self.fail_event(exc)
            self.refresh_sliders();self.update_scene()

    def visibility(self,event):
        for model in self.target_models:model.show_visual=bool(self.show_target.value)

    def ring_update(self,event):
        client=self.client_id(event)
        if client is None:return
        with self.lock:
            if self.state.mode in ('reset','teleop'):
                self.dragging=False;self.update_scene();return
            if event.phase == 'start': self.dragging=True
            i,j=self.arm_index,self.joint_index
            rotation=Rotation.from_quat(np.roll(np.asarray(event.target.wxyz),-1))
            relative=(self.anchor_rotation.inv()*rotation).as_matrix()
            q=self.state.desired[i].copy();q[j]=ring_angle(relative,q[j],LOWER[j],UPPER[j])
            try:self.state.target(i,q,client)
            except ValueError as exc:self.fail_event(exc)
            self.refresh_sliders()
            # Updating the URDF is safe during a drag; do not overwrite the gizmo pose.
            for k in range(2):self.target_models[k].update_cfg(self.state.desired[k])
            if event.phase == 'end':
                self.dragging=False;self.update_scene()

    def update_scene(self):
        for i in range(2):
            self.actual_models[i].update_cfg(self.state.actual[i])
            self.target_models[i].update_cfg(self.state.desired[i])
        if self.dragging:return
        i,j=self.arm_index,self.joint_index
        model=self.urdfs[i];joint=model.joint_map[JOINT_NAMES[j]]
        origin=np.eye(4) if joint.origin is None else joint.origin
        transform=model.get_transform(joint.parent)@origin
        axis=np.asarray(joint.axis,dtype=float)
        alignment,_=Rotation.align_vectors(axis.reshape(1,3),np.array([[0.,0.,1.]]))
        self.anchor_rotation=Rotation.from_matrix(transform[:3,:3])*alignment
        rotation=self.anchor_rotation*Rotation.from_rotvec([0.,0.,self.state.desired[i,j]])
        self.gizmo.position=transform[:3,3]+self.bases[i]
        self.gizmo.wxyz=np.roll(rotation.as_quat(),1)

    def start_worker(self):
        if self.process is not None and self.process.is_alive():return
        if self.process is not None:self.process.join(timeout=0)
        self.recording_event.clear()
        self.state.disable();self.state.ready=False;self.state.initialized=False
        self.state.desired=np.clip(self.state.actual,LOWER,UPPER)
        self.faulted=False;self.fault_at=0.;self.terminating=False;self.killed=False;self.worker_active=[False,False];self.worker_state={}
        self.commands=self.ctx.Queue(maxsize=16);self.feedback_queue=self.ctx.Queue(maxsize=32)
        self.process=self.ctx.Process(target=worker,args=(self.commands,self.feedback_queue,self.sample_queue,self.recording_event),daemon=True)
        self.worker_started_wall=time.time()
        self.process.start();self.start_time=time.monotonic();self.last_feedback=0
        self.message='正在初始化两臂，请稍候…';self.log.add(self.message)

    def retry_worker(self,event):
        with self.lock:
            if self.process is not None and self.process.is_alive():
                self.message='连接已建立或正在处理连接，请稍候';return
            self.recovery.cancel()
            if self.hardware:self.start_worker()

    def record_fault(self,fault):
        self.state.disable();self.state.ready=False;self.worker_active=[False,False]
        self.recovery.cancel();self.faulted=True;self.fault_at=time.monotonic();self.message=fault['message']
        self.log.fault(fault)

    def recorded_worker_fault(self):
        try:
            fault=json.loads((ROOT/'.runtime/viser/worker-fault.json').read_text())
            if fault['time']>=self.worker_started_wall:return fault
        except (OSError,ValueError,KeyError):pass
        return None

    def terminate_worker(self,message,code):
        if self.terminating:return
        self.record_fault(self.recorded_worker_fault() or dict(time=time.time(),code=code,message=message,detail={}))
        self.terminating=True;self.process.terminate()

    def reap_faulted_worker(self,now):
        # SDK/queue cleanup may leave threads alive after a fault. Bound cleanup,
        # while retaining the ORIGINAL fault and never restarting motion.
        if not self.faulted or self.process is None or not self.process.is_alive():return
        if now-self.fault_at>4. and not self.killed:
            self.process.kill();self.killed=True
        elif now-self.fault_at>1.5 and not self.terminating:
            self.process.terminate();self.terminating=True

    def update_status(self):
        fresh=self.state.ready and time.monotonic()-self.state.updated<.3
        modes={'idle':'待机','web':'网页控制','teleop':'主从遥操 A → B','reset':'双臂回零','hold':'零位保持'}
        connection='实机在线' if self.hardware and fresh else ('纯模型预览' if not self.hardware else '等待连接 / 已停止')
        mode=modes.get(self.worker_state.get('mode',self.state.mode),'待机') if fresh else '已停止'
        self.status.content=f'**{connection} · {mode}**'
        fault=self.log.last_fault
        if fault:
            when=time.strftime('%H:%M:%S',time.localtime(fault['time']))
            self.fault_banner.content=f'**已停止** · {fault["message"]}' if self.faulted else ''
            self.fault_detail.content=f'### 最近一次故障\n{when} · `{fault.get("code","unknown")}`\n\n{fault["message"]}\n\n```json\n{json.dumps(fault.get("detail",{}),ensure_ascii=False,indent=2)}\n```'
        else:self.fault_banner.content=''
        busy=(any(self.state.enabled) and self.state.mode!='hold') or self.recovery.pending is not None
        self.start_button.disabled=not(self.hardware and fresh) or busy
        self.operation_mode.disabled=busy
        self.home_button.disabled=not self.hardware or self.state.mode=='reset' or self.recovery.pending is not None or (self.process is not None and self.process.is_alive() and not fresh)
        self.retry.disabled=not self.hardware or (self.process is not None and self.process.is_alive())
        self.retry.visible=self.hardware and not fresh
        paired=self.state.mode in ('reset','teleop')
        for i,b in enumerate(self.enable_buttons):
            b.disabled=not(self.hardware and fresh) or paired or self.recovery.pending is not None
            b.label=('停止 ' if self.state.enabled[i] else '仅启用 ')+ARMS[i]
        self.sync_button.disabled=paired
        self.speed.disabled=self.state.mode=='reset'
        for h in self.sliders:h.disabled=paired
        self.gizmo.visible=not paired and self.page_name!='相机'
        self.progress.visible=self.state.mode=='reset'
        if self.state.mode=='reset':
            remaining=float(np.max(np.abs(self.state.actual)))
            self.progress.value=float(np.clip(100*(1-remaining/.8),0,99))
            self.reset_info.content=f'正在回零 · 剩余最大角度 {np.degrees(remaining):.1f}° · {self.worker_state.get("reset_elapsed_s",0):.1f} 秒；可随时停止'
        elif self.state.mode=='hold':self.reset_info.content='**双臂回零完成，正在保持零位。** 可以直接开始网页控制或主从遥操。'
        else:self.reset_info.content=self.message if self.message not in ('两臂已连接；可开始控制或双臂复位','预览模式：未连接电机') and not self.faulted else ''
        actual=np.degrees(self.state.actual[self.arm_index]);desired=np.degrees(self.state.desired[self.arm_index])
        self.readout.content='|关节|实测 °|目标 °|\n|---|---:|---:|\n'+'\n'.join(f'|J{j+1}|{actual[j]:.1f}|{desired[j]:.1f}|' for j in range(6))
        i=ARMS.index(self.data_arm.value);ws=self.worker_state
        vel=np.asarray(ws.get('velocities',np.zeros((2,6))))[i];tq=np.asarray(ws.get('torques',np.zeros((2,6))))[i]
        refs=ws.get('references',[None,None]);ref=self.state.actual[i] if refs[i] is None else np.asarray(refs[i])
        pos=self.state.actual[i];goal=self.state.desired[i]
        self.data_table.content=('|关节|实测 °|目标 °|速度 rad/s|力矩 Nm|参考误差 rad|\n|---|---:|---:|---:|---:|---:|\n'+
            '\n'.join(f'|J{j+1}|{np.degrees(pos[j]):.1f}|{np.degrees(goal[j]):.1f}|{vel[j]:.2f}|{tq[j]:.2f}|{ref[j]-pos[j]:.3f}|' for j in range(6)))
        tf=self.actual_urdfs[i].get_transform('tool_link')
        self.data_table.content+=f'\n\n末端 XYZ（底座系，m）：`{tf[0,3]:.3f}, {tf[1,3]:.3f}, {tf[2,3]:.3f}`'
        grip=ws.get('grippers',[[0,0,0],[0,0,0]])[i]
        self.data_table.content+=f'\n\n夹爪：{grip[0]:.3f} rad · {grip[1]:.3f} rad/s · {grip[2]:.3f} Nm'
        age=time.monotonic()-self.state.updated
        self.system_info.content=f'反馈延迟：{age*1000:.0f} ms' if fresh else '反馈：已失效，表格为最后一次读数'
        self.system_info.content+=f'\n\n目标限速 {self.state.speed:.2f} rad/s · 跟踪误差保护 0.6 rad（回零 0.15）\n\n控制 100 Hz · 反馈约 20 Hz · 图表 5 Hz · NUC 负载 {os.getloadavg()[0]:.2f}'
        self.events_view.content='### 最近事件\n\n'+'\n\n'.join(f'`{time.strftime("%H:%M:%S",time.localtime(e["time"]))}` **{e["kind"]}** · {e["message"]}' for e in self.log.events[-25:][::-1])

    def update_plot(self):
        if not self.history:return
        i=ARMS.index(self.data_arm.value);j=(JOINTS+('夹爪',)).index(self.data_joint.value)
        x=np.array([r[0] for r in self.history]);metric=self.data_metric.value
        if metric=='位置' and j<6:
            actual=np.array([r[1][i][j] for r in self.history]);ref=np.array([r[2][i][j] for r in self.history]);goal=np.array([r[3][i][j] for r in self.history])
            values=(np.degrees(actual),np.degrees(ref),np.degrees(goal));error=ref-actual
        else:
            index={'位置':0,'速度':1,'力矩':2}[metric]
            key='velocities' if metric=='速度' else 'torques'
            actual=np.array([r[4].get('grippers',[[0,0,0]]*2)[i][index] if j==6 else r[4].get(key,[[0]*6]*2)[i][j] for r in self.history])
            if metric=='位置':actual=np.degrees(actual)
            values=(actual,np.full(len(x),np.nan),np.full(len(x),np.nan));error=np.zeros(len(x))
        unit={'位置':'°','速度':'rad/s','力矩':'Nm'}[metric]
        self.plot.title=f'{metric} ({unit}) · 30 秒'
        self.plot.data=(x,*values);self.error_plot.visible=metric=='位置' and j<6
        self.error_plot.data=(x,error)

    def handle_feedback(self,msg):
        if msg['kind']=='state' and not self.faulted:
            first=not self.state.initialized
            self.state.feedback(msg['positions'],msg['stamp']);self.last_feedback=time.monotonic()
            self.worker_active=msg['enabled'];self.worker_state=msg
            if self.state.mode=='teleop' and msg.get('mode')=='teleop':
                self.state.desired=np.clip(np.asarray(msg['desired']),LOWER,UPPER);self.refresh_sliders()
            if first:
                self.refresh_sliders();self.message='两臂已连接；可开始控制或双臂复位';self.log.add(self.message)
                if self.recovery.pending:
                    client=self.recovery.take(self.server.get_clients())
                    if client is None:self.message='复位请求已过期或控制窗口已关闭；保持待机';self.log.add(self.message,'cancel')
                    else:
                        try:
                            self.state.begin('reset',client);self.message='恢复连接成功，双臂正在回零';self.log.add(self.message,'reset')
                        except ValueError as exc:self.fail_event(exc)
            if msg.get('reset_done'):self.reset_complete(msg['operation'])
        elif msg['kind']=='reset_done':self.reset_complete(msg['operation'])
        elif msg['kind']=='error':self.record_fault(msg['fault'])
        elif msg['kind']=='starting' and not self.faulted:self.message=msg['message']

    def reset_complete(self,operation):
        if self.state.finish_reset(operation):
            self.message='双臂回零完成，正在保持零位';self.log.add(self.message,'reset_done');self.refresh_sliders()

    def run(self):
        runtime=ROOT/'.runtime/viser';runtime.mkdir(parents=True,exist_ok=True)
        status_time=0
        try:
            while self.alive:
                with self.lock:
                    now=time.monotonic()
                    if self.recovery.expired():
                        self.recovery.cancel();self.message='复位等待连接超时，已取消运动请求';self.log.add(self.message,'cancel')
                    if self.process is not None:
                        put_latest(self.commands,self.state.command())
                        while True:
                            try:msg=self.feedback_queue.get_nowait()
                            except queue.Empty:break
                            self.handle_feedback(msg)
                        if not self.process.is_alive():
                            if not self.faulted:
                                try:
                                    fault=json.loads((runtime/'worker-fault.json').read_text())
                                    if fault['time']<time.time()-(now-self.start_time)-1:raise ValueError('old fault')
                                except (OSError,ValueError):fault=dict(time=time.time(),code='worker_exit',message='控制进程已退出，请重新连接',detail={})
                                self.record_fault(fault)
                            self.state.ready=False;self.worker_active=[False,False]
                        elif not self.faulted and self.last_feedback and now-self.last_feedback>.7:
                            self.terminate_worker('实机反馈中断，已停止双臂','feedback_timeout')
                        elif not self.faulted and not self.last_feedback and now-self.start_time>25:
                            self.terminate_worker('初始化超过 25 秒，已取消连接与复位','startup_timeout')
                    self.reap_faulted_worker(now)
                    self.update_scene()
                    if now-self.last_plot>.2:
                        if self.state.ready:
                            refs=self.worker_state.get('references',[None,None])
                            refs=[self.state.actual[i].tolist() if refs[i] is None else refs[i] for i in range(2)]
                            self.history.append((now-self.boot,self.state.actual.tolist(),refs,self.state.desired.tolist(),self.worker_state.copy()))
                        self.update_status();self.update_plot();self.collection_panel.update();self.last_plot=now
                    if now-status_time>.5:
                        row=dict(hardware=self.hardware,ready=self.state.ready,feedback_age_s=now-self.state.updated if self.state.ready else None,
                            enabled=self.state.enabled,worker_enabled=self.worker_active,mode=self.state.mode,owner=self.state.owner,
                            positions=self.state.actual.tolist(),targets=self.state.desired.tolist(),speed=self.state.speed,message=self.message,
                            last_fault=self.log.last_fault,pending_reset=self.recovery.pending,worker_pid=self.process.pid if self.process else None,wall_time=time.time(),collection=self.collection.snapshot)
                        p=runtime/'status.tmp';p.write_text(json.dumps(row,ensure_ascii=False));p.replace(runtime/'status.json');status_time=now
                time.sleep(.04)
        finally:
            self.camera.stop.set()
            with self.lock:
                self.recovery.cancel();self.state.disable()
                if self.commands is not None:put_latest(self.commands,self.state.command())
            if self.process is not None and self.process.is_alive():
                self.process.terminate();self.process.join(3)
                if self.process.is_alive():self.process.kill();self.process.join(1)
            self.collection.close();self.collection_panel.playback.closed.set()
            self.server.stop()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--host',default='0.0.0.0');parser.add_argument('--port',type=int,default=8082)
    parser.add_argument('--hardware',action='store_true')
    args=parser.parse_args();app=App(args.host,args.port,args.hardware)
    def stop(*_):app.alive=False
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    app.run()
if __name__=='__main__':main()

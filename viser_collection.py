"""Viser collection actor: one writer thread; no serial ports or camera pipelines."""
import base64
import io
import json
from pathlib import Path
import queue
import threading
import time
import zipfile
from urllib.request import urlopen

from capture import CSV_FIELDS, csv_row
from collection_workflow import CollectionWorkflow
from hub_upload import HubUploader
from recording import Recorder
from telemetry import NAMES

class Collection:
    def __init__(self, app, output, camera_url='http://10.209.66.60:8080'):
        self.app=app;self.output=Path(output);self.camera_url=camera_url
        self.output.mkdir(parents=True,exist_ok=True)
        import fcntl
        self.writer_lock=(self.output/'.collection.lock').open('a')
        try:fcntl.flock(self.writer_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except Exception:
            self.writer_lock.close();raise
        self.recorder=Recorder(output,dict(format_version=2,source='viser',joint_names=NAMES,
            timestamp_policy='Host snapshot/receive nanoseconds; device timestamps retained; no hardware synchronization'),CSV_FIELDS)
        self.workflow=CollectionWorkflow(self.output/'.collection-workflow.json')
        self.uploader=HubUploader(output)
        self.stats=dict(rgb_frames=0,depth_frames=0,joint_samples=0,sequence_gaps=0,camera_gaps=0)
        self.requests=queue.Queue(maxsize=16);self.closed=threading.Event();self.snapshot={}
        self.message='选择任务后开始采集';self.camera_error='';self.camera={};self.after=-1;self.last_sample=None
        self.last_joint=0.;self.last_image=0.;self.started_wall=0;self.owner=None;self.depth=False
        self.thread=threading.Thread(target=self.run,daemon=True,name='collection-writer')
        self.publish();self.thread.start()
    def submit(self, action, client, revision=None, **payload):
        self.requests.put_nowait((action,client,revision,payload,time.monotonic()))
    def camera_batch(self):
        after=self.after if self.recorder.active else -1
        with urlopen(self.camera_url+f'/api/frames?after={after}&depth={int(self.depth)}',timeout=.7) as response:
            data=response.read()
        with zipfile.ZipFile(io.BytesIO(data)) as z:
            meta=json.loads(z.read('metadata.json'));rows=json.loads(z.read('frames.json'))
            if self.camera.get('session') and meta['session']!=self.camera['session']:
                self.after=-1
                if self.recorder.active:raise RuntimeError('相机服务重启，本段采集结束')
            self.camera=meta
            for row in rows:
                seq=row['sequence'];stamp=row['timestamp_ns']
                if self.after>=0 and seq<=self.after:continue
                if self.recorder.active and stamp>=self.started_wall:
                    if self.after>=0:self.stats['camera_gaps']+=max(0,seq-self.after-1)
                    self.recorder.writer.image('/camera/color/image',z.read(f'{seq}.jpg'),stamp,frame_id='agent_color_optical')
                    if self.depth:
                        self.recorder.writer.image('/camera/depth/image',z.read(f'{seq}.png'),stamp,format='png16',
                            frame_id='agent_depth_optical',depth_scale=meta['depth_scale'])
                        self.stats['depth_frames']+=1
                    self.recorder.writer.log('/camera/frame_info',row,stamp)
                    self.stats['rgb_frames']+=1;self.last_image=time.monotonic()
                self.after=seq
    def start(self, client, payload):
        if self.recorder.active:return
        self.workflow.before_start(self.recorder,None)
        # Camera and motor readiness precede any enable operation.
        if not self.camera.get('connected') or time.time_ns()-self.camera.get('timestamp_ns',0)>500_000_000:
            raise ValueError('相机未就绪，尚未开始')
        with self.app.lock:
            self.app.state.require_owner(client)
            if client not in self.app.server.get_clients():raise ValueError('发起窗口已断开')
            if not self.app.hardware or not self.app.state.ready or time.monotonic()-self.app.state.updated>.3:
                raise ValueError('两臂未连接，尚未开始')
            if self.app.state.mode=='reset':raise ValueError('正在回零，请等待完成')
            mode=self.app.state.mode
            if mode in ('idle','hold'):
                mode='web' if self.app.operation_mode.value=='网页双臂控制' else 'teleop'
            if mode not in ('web','teleop'):raise ValueError('请先选择控制模式')
            self.owner=client;self.depth=bool(payload.get('depth'))
            self.recorder.template.update(camera=self.camera.copy(),control_mode=mode,
                depth_recorded=self.depth,action_available=mode=='teleop',
                camera_intrinsics=self.camera.get('color_intrinsics'),depth_intrinsics=self.camera.get('depth_intrinsics'),
                depth_to_color=self.camera.get('depth_to_color'),depth_scale_m=self.camera.get('depth_scale'))
        # Opening files can be slow; never hold the control lock during disk work.
        self.recorder.start(self.stats);self.started_wall=time.time_ns()
        try:
            with self.app.lock:
                if client not in self.app.server.get_clients():raise ValueError('发起窗口已断开')
                self.app.state.require_owner(client)
                if self.app.state.mode in ('idle','hold'):self.app.state.begin(mode,client)
                if self.app.state.mode not in ('web','teleop'):raise ValueError('控制模式已变化')
                while True:
                    try:self.app.sample_queue.get_nowait()
                    except queue.Empty:break
                self.app.recording_event.set()
        except Exception as exc:
            self.recorder.stop(self.stats,error=str(exc));raise
        self.last_sample=None;self.last_joint=self.last_image=time.monotonic()
        self.message='正在录制双臂反馈与相机画面'
    def joint(self, sample):
        if not self.recorder.active or sample['timestamp_ns']<self.started_wall:return
        if time.monotonic_ns()-sample['monotonic_ns']>500_000_000:raise ValueError('录制反馈积压超过 0.5 秒')
        if self.last_sample:
            if sample['session']!=self.last_sample['session']:raise ValueError('控制进程已重启')
            if sample['sequence']<=self.last_sample['sequence']:return
            self.stats['sequence_gaps']+=max(0,sample['sequence']-self.last_sample['sequence']-1)
        self.last_sample=sample
        stamp=sample['timestamp_ns'];self.recorder.writer.log('/robot/dual_arm_state',sample,stamp)
        for i,arm in enumerate(sample['arms']):
            row=dict(arm,name=NAMES,position_unit='rad',session=sample['session'],sequence=sample['sequence'],
                timestamp_ns=stamp,monotonic_ns=sample['monotonic_ns'])
            if i==1:
                row['leader_position']=sample['arms'][0]['position']
                if 'action' in row:row.update(action_unit='rad',action_source='follower_command_target')
                self.recorder.write_joint(csv_row(row))
            self.recorder.writer.log('/'+('leader' if i==0 else 'follower')+'/joint_states',row,stamp,
                schema_name='panthera.JointState')
        self.stats['joint_samples']+=1;self.last_joint=time.monotonic()
    def stop_recording(self, reason='user', error=''):
        self.app.recording_event.clear()
        # Samples before stop are drained on the actor before file finalization.
        for _ in range(512):
            try:row=self.app.sample_queue.get_nowait()
            except queue.Empty:break
            try:self.joint(row)
            except Exception as exc:error=error or str(exc)
        if self.recorder.active:
            self.recorder.stop(self.stats,reason=reason,error=error)
            self.message='录制已结束，请保存或丢弃' if not error else '录制中断：'+error
    def command(self, action, client, revision, payload):
        if action not in ('stop','upload_retry'):
            self.workflow.check_revision(revision,self.recorder,None)
        if action in ('start','save','discard') and self.owner not in (None,client):
            if self.owner in self.app.server.get_clients():raise ValueError('请在发起采集的窗口操作')
            if not self.recorder.active:self.owner=None
            else:raise ValueError('正在结束已断开窗口的录制，请稍候')
        if action in ('catalog_create','task_create','task_select'):
            self.message=self.workflow.configure(action,payload,self.recorder,None)['message']
        elif action=='start':self.start(client,payload)
        elif action=='stop':self.stop_recording()
        elif action in ('save','discard'):
            self.stop_recording()
            result=self.recorder.decide('saved' if action=='save' else 'discarded')
            self.message=result['message']
            if result['ok']:self.owner=None
        elif action=='upload_retry':self.message=self.uploader.retry()['message']
        else:raise ValueError('未知采集操作')
    def publish(self):
        self.snapshot=dict(self.recorder.state(self.stats),**self.workflow.state(self.recorder,None),
            **self.uploader.state(),message=self.message,camera_error=self.camera_error,camera=self.camera.copy(),
            feedback_age_s=time.monotonic()-self.last_joint if self.last_joint else None)
    def run(self):
        flushed=0.;last_camera=0.
        try:
            while not self.closed.is_set():
                try:
                    now=time.monotonic()
                    if now-last_camera>(.03 if self.recorder.active else .5):
                        self.camera_batch();last_camera=time.monotonic();self.camera_error=''
                    for _ in range(256):
                        try:row=self.app.sample_queue.get_nowait()
                        except queue.Empty:break
                        self.joint(row)
                    for _ in range(16):
                        try:action,client,rev,payload,created=self.requests.get_nowait()
                        except queue.Empty:break
                        try:
                            if action!='stop' and time.monotonic()-created>3:raise ValueError('操作已过期，请重试')
                            self.command(action,client,rev,payload)
                        except Exception as exc:self.message=str(exc)
                    if self.recorder.active:
                        with self.app.lock:
                            active=self.app.state.ready and self.app.state.mode in ('web','teleop') and any(self.app.state.enabled)
                        if not active:self.stop_recording('control_stopped')
                        elif time.monotonic()-min(self.last_joint,self.last_image)>.7:
                            raise RuntimeError('相机或双臂反馈超过 0.7 秒未更新')
                    if time.monotonic()-flushed>1:
                        self.recorder.flush();flushed=time.monotonic()
                except Exception as exc:
                    last_camera=time.monotonic()
                    self.camera['connected']=False
                    self.camera_error='相机采集暂不可用，请检查连接或相机服务'
                    if self.recorder.active:self.stop_recording('stream_error',str(exc))
                    else:self.message=self.message or self.camera_error
                    # Even without a camera, tasks/review/stop must remain usable.
                    for _ in range(16):
                        try:action,client,rev,payload,created=self.requests.get_nowait()
                        except queue.Empty:break
                        try:
                            if action!='stop' and time.monotonic()-created>3:raise ValueError('操作已过期，请重试')
                            if action=='start':raise ValueError('相机未就绪，尚未开始')
                            self.command(action,client,rev,payload)
                        except Exception as error:self.message=str(error)
                try:
                    if self.app.hardware:self.uploader.tick()
                except Exception as exc:self.message='上传状态异常：'+str(exc)
                self.publish();self.closed.wait(.02)
        finally:
            self.stop_recording('shutdown',error='服务退出时结束录制' if self.recorder.active else '')
            self.publish()
    def close(self):
        self.closed.set();self.thread.join(timeout=3)
        if not self.thread.is_alive():self.writer_lock.close()

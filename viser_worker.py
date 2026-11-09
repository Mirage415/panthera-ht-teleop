"""Exclusive SDK worker: web joint control, paired homing, and A→B teleop."""
import fcntl
import json
from pathlib import Path
import queue
import signal
import time
import traceback
import uuid
import numpy as np
import yaml
from viser_control import checked_target, slew, LOWER, UPPER, ControlFault, tracking_guard, effective_targets

ROOT = Path(__file__).resolve().parent


def put_latest(out, value):
    try: out.put_nowait(value)
    except queue.Full:
        try: out.get_nowait()
        except queue.Empty: pass
        try: out.put_nowait(value)
        except queue.Full: pass


def worker(commands, feedback, samples=None, recording=None):
    from teleop import (resolve_device, snapshot, validate_positions, shutdown, SDK,
                        reference_velocity, leader_gravity_torque)
    from Panthera_lib import Panthera
    robots = {}
    sample_sequence=0; sample_session=uuid.uuid4().hex
    def stop(*_): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop); signal.signal(signal.SIGINT, stop)
    runtime = ROOT / '.runtime/viser'; runtime.mkdir(parents=True,exist_ok=True)
    lock = (ROOT / '.runtime/teleop.lock').open('w')
    context = {}; latest = None
    try:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        settings = json.loads((ROOT/'teleop-config.json').read_text())
        devices = [resolve_device(settings[r]) for r in ('leader','follower')]
        if devices[0] == devices[1]: raise RuntimeError('两臂端口重复')
        for i,dev in enumerate(devices):
            context = dict(arm='AB'[i],stage='initialization')
            low=yaml.safe_load((SDK/'robot_param/motor_param/6dof_Panthera_params_follower.yaml').read_text())
            low['robot'].update(Serial_Type=str(dev),motor_timeout_ms=250,exit_motor_brake_flag=True)
            low['robot']['CANboard']['No_1_CANboard']['CANport']['CANport_1']['serial_id']=1
            lowpath=runtime/f'arm{i}-motors.yaml';lowpath.write_text(yaml.safe_dump(low))
            high=yaml.safe_load((SDK/'robot_param/Follower.yaml').read_text())
            high['robot']['param_file']=str(lowpath)
            high['urdf']['file_path']=str((SDK/'robot_param'/high['urdf']['file_path']).resolve())
            path=runtime/f'arm{i}.yaml';path.write_text(yaml.safe_dump(high))
            put_latest(feedback,dict(kind='starting',message=f'正在连接机械臂 {"AB"[i]}'))
            robots[i]=Panthera(str(path))
        for _ in range(3):
            for r in robots.values():r.send_get_motor_state_cmd()
            time.sleep(.03)
        active=[False,False]; epochs=[-1,-1];refs=[None,None];armed_at=[0.,0.]
        filtered=[np.zeros(6),np.zeros(6)]
        previous_mode='idle'; operation=-1;reset_start=0.;settled_since=None;reset_done=False
        grip_ref=0.;grip_velocity=0.
        last=time.monotonic();sent=0;last_poll=[-1.,-1.]
        kp=np.asarray(settings['joint_kp']);kd=np.asarray(settings['joint_kd'])
        limits=np.asarray(settings['torque_limits_nm']);home_caps=np.array([8.,15.,15.,8.,3.,1.])
        while True:
            now=time.monotonic();dt=now-last;last=now
            if dt>.1:raise ControlFault('loop_timeout','控制循环延迟超过 100ms，已停止双臂',delay_s=dt)
            while True:
                try:latest=commands.get_nowait()
                except queue.Empty:break
            if latest is None or now-latest['stamp']>.5:
                raise ControlFault('heartbeat_timeout','网页服务心跳超时，已停止双臂')
            mode=latest.get('mode','web');op=latest.get('operation',0)
            if mode not in ('idle','web','teleop','reset','hold'):raise RuntimeError('未知控制模式')
            states=[]; feedback_times=[]
            # Validate BOTH arms before issuing any active command in this cycle.
            for i,r in robots.items():
                context=dict(arm='AB'[i],mode=mode,stage='feedback')
                if resolve_device(settings[('leader','follower')[i]])!=devices[i]:
                    raise ControlFault('usb_changed',f'{"AB"[i]} 臂 USB 连接发生变化')
                # Active SDK commands already return motor state. Do not interleave
                # STATE2 at 100Hz: it changes the shared CAN command buffer/mode.
                # Match the established teleop loop; poll only passive arms at 20Hz.
                wanted=bool(latest['enabled'][i]) and mode!='idle'
                if (not active[i] or not wanted) and now-last_poll[i]>=.05:
                    r.send_get_motor_state_cmd();last_poll[i]=now
                stamps=[]
                try:
                    v=snapshot(r,.2,stamps)
                except RuntimeError as exc:
                    detail=dict(arm='AB'[i],mode=mode,stage='feedback',was_active=active[i])
                    try:
                        raw=r.get_current_state()+[r.get_current_state_gripper()]
                        wall=time.time()
                        detail['motors']=[dict(id=int(m.ID),age_ms=round((wall-m.time)*1000,2),
                            fault=int(m.fault),position_rad=float(m.position),velocity_rad_s=float(m.velocity)) for m in raw]
                    except Exception:pass
                    raise ControlFault('feedback_timeout' if '状态超时' in str(exc) else 'feedback_error',
                        f'{"AB"[i]} 臂反馈异常：{exc}',**detail) from exc
                validate_positions(r,v[:,0]);states.append(v);feedback_times.append(stamps)
            targets=effective_targets(latest,[v[:6,0] for v in states])
            paired=mode in ('reset','teleop')
            if paired and not all(latest['enabled']):raise RuntimeError('双臂模式使能状态不一致')
            entering=mode!=previous_mode or op!=operation
            if mode=='reset' and entering:
                if max(np.max(np.abs(v[:6,0])) for v in states)>.8:
                    raise ControlFault('home_range','距零位超过 0.8 rad，拒绝自动回零；请先低速移近零位')
                reset_start=now;settled_since=None;reset_done=False
            if mode=='teleop' and entering:
                grip_ref=float(np.clip(states[1][6,0],0.,2.));grip_velocity=0.
            previous_mode=mode;operation=op
            for i in range(2):
                wanted=bool(latest['enabled'][i]) and mode!='idle'
                if wanted and (not active[i] or epochs[i]!=latest['epochs'][i]):
                    refs[i]=np.clip(states[i][:6,0],LOWER,UPPER)
                    armed_at[i]=now;filtered[i]=np.zeros(6)
                if not wanted and active[i]:shutdown({i:robots[i]})
                active[i]=wanted;epochs[i]=latest['epochs'][i]
            velocities=[np.zeros(6),np.zeros(6)]
            for i in range(2):
                if not active[i]:continue
                v=states[i]
                # Retain the established home_arms checks; do not impose new
                # measured-speed/torque cutoffs on normal web or teleop operation.
                if mode=='reset':
                    if np.max(np.abs(v[:6,1]))>1.0:
                        j=int(np.argmax(np.abs(v[:6,1])))
                        raise ControlFault('home_velocity',f'{"AB"[i]} 臂 J{j+1} 回零反馈速度异常',
                            arm='AB'[i],joint=j+1,velocity_rad_s=float(v[j,1]))
                    if np.any(np.abs(v[:6,2])>home_caps+.5):
                        j=int(np.argmax(np.abs(v[:6,2])-home_caps))
                        raise ControlFault('home_torque',f'{"AB"[i]} 臂 J{j+1} 回零反馈力矩异常',
                            arm='AB'[i],joint=j+1,torque_nm=float(v[j,2]))
                if mode=='teleop' and i==0:
                    refs[i]=v[:6,0].copy();continue
                target=np.zeros(6) if mode=='reset' else (np.clip(states[0][:6,0],LOWER,UPPER) if mode=='teleop' else targets[i])
                speed=.08 if mode=='reset' else latest['speed']
                previous=refs[i].copy();refs[i]=slew(previous,target,speed,dt)
                threshold=.15 if mode=='reset' else settings.get('max_tracking_error_rad',.6)
                tracking_guard('AB'[i],refs[i],v[:6,0],target,threshold,speed,mode)
                targets[i]=target
                filtered[i],velocities[i]=reference_velocity(filtered[i],refs[i]-previous,max(dt,.001),speed,
                    settings.get('velocity_feedforward_gain',.5),settings.get('velocity_filter_time_s',.04))
            if mode=='reset':
                if now-reset_start>25:raise ControlFault('home_timeout','25 秒内双臂未完成回零，已停止')
                settled=all(np.max(np.abs(v[:6,0]))<.02 and np.max(np.abs(v[:6,1]))<.08 for v in states)
                settled=settled and all(np.max(np.abs(q))<1e-6 for q in refs)
                settled_since=(now if settled_since is None else settled_since) if settled else None
                if settled_since is not None and now-settled_since>=1. and not reset_done:
                    reset_done=True
                    put_latest(feedback,dict(kind='reset_done',operation=op,message='双臂回零完成，正在保持零位'))
            if mode=='teleop':
                old=grip_ref;gs=settings.get('gripper_max_target_speed_rad_s',4.)
                grip_ref=float(old+np.clip(np.clip(states[0][6,0],0.,2.)-old,-gs*min(dt,.02),gs*min(dt,.02)))
                if abs(grip_ref-states[1][6,0])>settings.get('max_tracking_error_rad',.6):
                    raise ControlFault('gripper_tracking',f'B 臂夹爪跟踪误差超过 {settings.get("max_tracking_error_rad",.6):.2f} rad',arm='B',joint=7,
                        actual_rad=float(states[1][6,0]),reference_rad=grip_ref,error_rad=abs(grip_ref-float(states[1][6,0])))
                grip_velocity,gv=reference_velocity(np.array([grip_velocity]),np.array([grip_ref-old]),max(dt,.001),gs,.5,.04)
                grip_velocity=float(grip_velocity[0])
            # All state/limit checks precede active sends.
            for i,r in robots.items():
                if not active[i]:continue
                context=dict(arm='AB'[i],mode=mode,stage='command')
                v=states[i];ramp=min(1.,(now-armed_at[i])/1.)
                if mode=='reset':
                    ok=r.Joint_Pos_Vel(refs[i],np.full(6,.08),home_caps)
                else:
                    gravity=(leader_gravity_torque(r,v[:6,0],settings) if mode=='teleop' and i==0 else np.asarray(r.get_Gravity(v[:6,0])))
                    if not np.isfinite(gravity).all():raise RuntimeError('重力计算异常')
                    if mode=='teleop' and i==0:
                        ok=r.pos_vel_tqe_kp_kd(np.zeros(6),np.zeros(6),ramp*np.clip(gravity,-limits,limits),np.zeros(6),np.asarray(settings['leader_kd']))
                    else:
                        ok=r.pos_vel_tqe_kp_kd(refs[i],velocities[i],ramp*np.clip(gravity,-limits,limits),kp,kd)
                if not ok:raise RuntimeError('SDK 拒绝关节命令')
                if mode=='teleop' and i==1:
                    ok=r.gripper_control_MIT(grip_ref,float(gv[0]),0.,4.,.4)
                elif mode=='reset':
                    # Keep the SDK position/velocity mode intact during homing.
                    # The original home_arms loop likewise commands only six joints.
                    ok=True
                else:ok=r.gripper_control_MIT(0.,0.,0.,0.,.05)
                if not ok:raise RuntimeError('SDK 拒绝夹爪命令')
            if samples is not None and recording.is_set():
                row=dict(timestamp_ns=time.time_ns(),monotonic_ns=time.monotonic_ns(),
                    session=sample_session,sequence=sample_sequence,mode=mode,operation=op,enabled=list(active),
                    arms=[dict(position=v[:,0].tolist(),velocity=v[:,1].tolist(),effort=v[:,2].tolist(),
                        reference=refs[i].tolist() if refs[i] is not None else None,
                        target=targets[i].tolist(),feedback_time_s=feedback_times[i]) for i,v in enumerate(states)])
                if mode=='teleop':row['arms'][1]['action']=refs[1].tolist()+[grip_ref]
                try:samples.put_nowait(row)
                except queue.Full:pass
                sample_sequence+=1
            if now-sent>=.05:
                put_latest(feedback,dict(kind='state',stamp=now,positions=[v[:6,0].tolist() for v in states],
                    velocities=[v[:6,1].tolist() for v in states],torques=[v[:6,2].tolist() for v in states],
                    grippers=[v[6].tolist() for v in states],enabled=list(active),mode=mode,operation=op,
                    reset_done=reset_done and mode=='reset',reset_elapsed_s=now-reset_start if mode=='reset' else 0.,
                    references=[q.tolist() if q is not None else None for q in refs],desired=[q.tolist() for q in targets]))
                sent=now
            time.sleep(max(0,.01-(time.monotonic()-now)))
    except BaseException as exc:
        if not isinstance(exc,KeyboardInterrupt):
            traceback.print_exc()
            fault=dict(time=time.time(),code=getattr(exc,'code','sdk_error'),message=str(exc),
                       detail={**context,**getattr(exc,'detail',{})})
            if isinstance(exc,BlockingIOError):fault.update(code='port_busy',message='其他程序占用机械臂，请先停止旧遥操')
            try:
                p=runtime/'worker-fault.tmp';p.write_text(json.dumps(fault,ensure_ascii=False));p.replace(runtime/'worker-fault.json')
            except OSError:pass
            put_latest(feedback,dict(kind='error',fault=fault,message=fault['message']))
    finally:
        shutdown(robots)
        lock.close()
        print('Viser motor worker stopped; brake sent',flush=True)

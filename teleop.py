#!/usr/bin/env python3
"""Panthera leader/follower teleoperation, based on official 5_teleop_control.py.

Adds USB topology selection, preflight, bounded reference speed, feedback checks,
motor watchdog, process lock, bounded startup and explicit shutdown.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import yaml

from probe_ports import query

ROOT = Path(__file__).resolve().parent
SDK = ROOT / 'Panthera-HT_SDK/panthera_python'
RUNTIME = ROOT / '.runtime'
sys.path.insert(0, str(SDK / 'scripts'))


def resolve_device(path):
    link = Path(path)
    if not link.exists():
        raise RuntimeError(f'通信板未连接或枚举失败：{link}')
    dev = link.resolve(strict=True)
    if not os.access(dev, os.R_OK | os.W_OK):
        raise RuntimeError(f'{dev} 无读写权限；执行 sudo bash {ROOT}/setup-device-access.sh')
    # The vendor library interprets Serial_Type as a prefix, not an exact path.
    matches = list(dev.parent.glob(dev.name + '*'))
    if matches != [dev]:
        raise RuntimeError(f'SDK 串口前缀不唯一：{dev}，候选 {matches}；拒绝选择可能错误的设备')
    sysdev = (Path('/sys/class/tty') / dev.name / 'device').resolve()
    for parent in [sysdev, *sysdev.parents]:
        if (parent / 'idVendor').exists():
            if ((parent / 'idVendor').read_text().strip(), (parent / 'idProduct').read_text().strip()) != ('caf1', 'ffff'):
                raise RuntimeError(f'{dev} 不是已确认的 Livelybot 通信板')
            break
    else:
        raise RuntimeError(f'无法识别 USB 设备：{dev}')
    return dev


def generate_config(role, device, settings, active):
    name = role.capitalize()
    low = yaml.safe_load((SDK / f'robot_param/motor_param/6dof_Panthera_params_{role}.yaml').read_text())
    low['robot']['Serial_Type'] = str(device)
    low['robot']['motor_timeout_ms'] = settings['motor_timeout_ms'] if active else 0
    low['robot']['exit_motor_brake_flag'] = True
    low['robot']['CANboard']['No_1_CANboard']['CANport']['CANport_1']['serial_id'] = 1
    lowfile = RUNTIME / f'{role}-motors.yaml'
    lowfile.write_text(yaml.safe_dump(low))
    high = yaml.safe_load((SDK / f'robot_param/{name}.yaml').read_text())
    high['robot']['param_file'] = str(lowfile)
    high['urdf']['file_path'] = str((SDK / 'robot_param' / high['urdf']['file_path']).resolve())
    path = RUNTIME / f'{role}.yaml'
    path.write_text(yaml.safe_dump(high))
    return path


def preflight(settings, active, roles):
    devices = {role: resolve_device(settings[role]) for role in roles}
    if len(set(devices.values())) != len(devices):
        raise RuntimeError('主从端口重复，拒绝启动')
    configs = {}
    for role, dev in devices.items():
        print(f'{role}: {settings[role]} -> {dev}', flush=True)
        result = query(dev, duration=0.8)
        if set(result['versions']) != set(range(1, 8)):
            raise RuntimeError(f'{role} 未找到完整的电机 ID 1–7：{result["versions"]}；检查电源和 CAN 接线')
        if any(int(v.split('.')[0]) < 4 for v in result['versions'].values()):
            raise RuntimeError(f'{role} 电机固件版本过旧，需要单独适配')
        print(f'{role}: 7 个电机通信正常 {result["versions"]}', flush=True)
        configs[role] = generate_config(role, dev, settings, active)
    return devices, configs


def snapshot(robot, max_age):
    states = robot.get_current_state() + [robot.get_current_state_gripper()]
    now = time.time()
    if len(states) != 7 or [s.ID for s in states] != list(range(1, 8)):
        raise RuntimeError('电机状态数量或 ID 不匹配')
    values = np.array([[s.position, s.velocity, s.torque] for s in states])
    if not np.isfinite(values).all():
        raise RuntimeError('电机状态包含非有限数值')
    for s in states:
        if not np.isfinite(s.time) or not -0.05 <= now - s.time <= max_age:
            raise RuntimeError(f'电机 {s.ID} 状态超时：{now - s.time:.3f}s')
        if s.fault:
            raise RuntimeError(f'电机 {s.ID} 故障码：{s.fault}')
    return values


def bounded_target(previous, desired, speed, dt):
    return previous + np.clip(desired - previous, -speed * dt, speed * dt)


def reference_velocity(previous, step, dt, speed_limit, gain, time_constant):
    raw = np.clip(step / dt, -speed_limit, speed_limit)
    alpha = dt / (time_constant + dt)
    # No residual forward push when the target stops or reverses direction.
    previous = np.where(previous * raw > 0, previous, 0.0)
    filtered = previous + alpha * (raw - previous)
    filtered = np.where(np.abs(step) > 1e-9, filtered, 0.0)
    return filtered, gain * filtered


def leader_gravity_torque(robot, position, settings):
    return np.asarray(robot.get_Gravity(position)) * np.asarray(settings['leader_gravity_scale'])


def validate_settings(settings):
    ranges = {
        'frequency_hz': (20, 200), 'motor_timeout_ms': (100, 1000),
        'state_timeout_s': (0.02, 0.25), 'max_target_speed_rad_s': (0.01, 1.5),
        'gripper_max_target_speed_rad_s': (0.01, 4.0),
        'max_tracking_error_rad': (0.05, 0.6),
        'velocity_feedforward_gain': (0, 0.5), 'velocity_filter_time_s': (0.02, 0.2),
    }
    for key, (low, high) in ranges.items():
        value = settings[key]
        if not np.isfinite(value) or not low <= value <= high:
            raise ValueError(f'{key} 必须在 [{low}, {high}] 内')
    if settings['motor_timeout_ms'] != int(settings['motor_timeout_ms']):
        raise ValueError('motor_timeout_ms 必须为整数')
    if settings['state_timeout_s'] > settings['motor_timeout_ms'] / 1000:
        raise ValueError('状态超时不得大于电机 watchdog 超时')
    scale = np.asarray(settings['leader_gravity_scale'], dtype=float)
    if scale.shape != (6,) or not np.isfinite(scale).all() or np.any(scale < 0.8) or np.any(scale > 1.0):
        raise ValueError('leader_gravity_scale 必须是六个 0.8–1.0 范围内的数值')
    for key, cap in [('joint_kp', 100), ('joint_kd', 10), ('leader_kd', 10), ('torque_limits_nm', [15, 30, 30, 15, 5, 5])]:
        value = np.asarray(settings[key], dtype=float)
        if value.shape != (6,) or not np.isfinite(value).all() or np.any(value < 0) or np.any(value > cap):
            raise ValueError(f'{key} 包含无效参数')


def validate_positions(robot, positions):
    lower, upper = robot.joint_limits['lower'], robot.joint_limits['upper']
    if np.any(positions[:6] < lower - 0.1) or np.any(positions[:6] > upper + 0.1):
        raise RuntimeError(f'关节位置超出配置范围：{positions[:6]}；检查零位，程序不会自动重设零位')
    if not robot.gripper_limits['lower'] - 0.1 <= positions[6] <= robot.gripper_limits['upper'] + 0.1:
        raise RuntimeError(f'夹爪位置超出配置范围：{positions[6]}')


def shutdown(robots):
    # Vendor BRAKE is damping; it does not promise static gravity support.
    for robot in robots.values():
        try:
            for motor in robot.Motors:
                motor.brake()
            robot.motor_send_cmd()
        except Exception as exc:
            print(f'退出制动发送失败（依赖电机 watchdog）：{exc}', file=sys.stderr)


def run_loop(leader, follower, settings, devices, duration):
    period = 1 / settings['frequency_hz']
    limit = np.array(settings['torque_limits_nm'])
    kp, kd = np.array(settings['joint_kp']), np.array(settings['joint_kd'])
    leader_kd = np.array(settings['leader_kd'])
    speed_limits = np.r_[np.full(6, settings['max_target_speed_rad_s']),
                         settings['gripper_max_target_speed_rad_s']]
    initial = snapshot(follower, settings['state_timeout_s'])[:, 0]
    lower = np.r_[follower.joint_limits['lower'], follower.gripper_limits['lower']]
    upper = np.r_[follower.joint_limits['upper'], follower.gripper_limits['upper']]
    target = np.clip(initial, lower, upper)
    filtered_velocity = np.zeros(7)
    logfile = RUNTIME / 'teleop-latest.jsonl'
    if logfile.exists():
        logfile.rename(RUNTIME / f'teleop-{time.time_ns()}.jsonl')
    start = last = time.monotonic()
    printed = 0
    logged = -1.0
    print('遥操作开始：主臂重力补偿及阻尼，从臂及夹爪位置跟随。Ctrl+C 退出；退出前扶稳机械臂。', flush=True)
    while duration is None or time.monotonic() - start < duration:
        now = time.monotonic()
        elapsed, dt = now - start, max(now - last, period)
        if now - last > 0.1:
            raise RuntimeError('控制循环延迟超过 100ms，停止遥操作')
        last = now
        for role, dev in devices.items():
            if not Path(settings[role]).exists() or Path(settings[role]).resolve() != dev:
                raise RuntimeError(f'{role} USB 连接发生变化')
        l = snapshot(leader, settings['state_timeout_s'])
        f = snapshot(follower, settings['state_timeout_s'])
        validate_positions(leader, l[:, 0])
        validate_positions(follower, f[:, 0])
        desired = np.clip(l[:, 0], lower, upper)
        new_target = bounded_target(target, desired, speed_limits, min(dt, 0.02))
        filtered_velocity, target_velocity = reference_velocity(
            filtered_velocity, new_target - target, dt,
            speed_limits, settings['velocity_feedforward_gain'],
            settings['velocity_filter_time_s'])
        target = new_target
        if np.max(np.abs(target - f[:, 0])) > settings['max_tracking_error_rad']:
            raise RuntimeError('从臂跟踪误差过大，停止遥操作')
        ramp = min(1.0, elapsed / 1.0)
        lt = leader_gravity_torque(leader, l[:6, 0], settings)
        ft = follower.get_Gravity(f[:6, 0])
        if not np.isfinite(lt).all() or not np.isfinite(ft).all():
            raise RuntimeError('动力学计算返回非有限值')
        if not leader.pos_vel_tqe_kp_kd(np.zeros(6), np.zeros(6), ramp * np.clip(lt, -limit, limit), np.zeros(6), leader_kd):
            raise RuntimeError('主臂控制命令被 SDK 拒绝')
        if not follower.pos_vel_tqe_kp_kd(target[:6], target_velocity[:6], ramp * np.clip(ft, -limit, limit), kp, kd):
            raise RuntimeError('从臂控制命令被 SDK 拒绝')
        # Passive leader gripper; no uncalibrated force feedback or opening spring.
        if not leader.gripper_control_MIT(0, 0, 0, 0, 0.05):
            raise RuntimeError('主臂夹爪命令被拒绝')
        if not follower.gripper_control_MIT(float(target[6]), float(target_velocity[6]), 0, 4.0, 0.4):
            raise RuntimeError('从臂夹爪命令被拒绝')
        if elapsed - logged >= 0.05:
            row = dict(t=elapsed, wall_time=time.time(), leader=l.tolist(), follower=f.tolist(),
                       desired=desired.tolist(), target=target.tolist(), target_velocity=target_velocity.tolist(),
                       leader_gravity=lt.tolist(), follower_gravity=ft.tolist(), leader_kd=leader_kd.tolist(),
                       speed_limit=settings['max_target_speed_rad_s'],
                       gripper_speed_limit=settings['gripper_max_target_speed_rad_s'],
                       velocity_feedforward_gain=settings['velocity_feedforward_gain'],
                       leader_gravity_scale=settings['leader_gravity_scale'])
            with (RUNTIME / 'teleop-latest.jsonl').open('a') as log:
                log.write(json.dumps(row) + '\n')
            logged = elapsed
        (RUNTIME / 'heartbeat').touch()
        if elapsed >= printed:
            print(f'运行 {elapsed:.0f}s | 目标跟踪误差 {np.max(np.abs(target - f[:, 0])):.3f} rad', flush=True)
            printed += 2
        time.sleep(max(0, period - (time.monotonic() - now)))


def gravity_observation(robot, settings, device):
    """Eight-second supported observation; no position servo or parameter fitting."""
    destination = ROOT / 'calibration'
    destination.mkdir(exist_ok=True)
    path = destination / f'leader-gravity-{time.time_ns()}.jsonl'
    initial = snapshot(robot, settings['state_timeout_s'])[:6, 0].copy()
    limit = np.asarray(settings['torque_limits_nm'])
    kd = np.asarray(settings['leader_kd'])
    start = last = time.monotonic()
    print('主臂补偿观察开始，共 8 秒，前 2 秒逐渐加载。请持续承托；不要松手。', flush=True)
    with path.open('x') as log:
        while time.monotonic() - start < 8:
            now = time.monotonic()
            if now - last > 0.1:
                raise RuntimeError('补偿观察循环延迟过大')
            last = now
            if not Path(settings['leader']).exists() or Path(settings['leader']).resolve() != device:
                raise RuntimeError('主臂 USB 连接变化')
            values = snapshot(robot, settings['state_timeout_s'])
            validate_positions(robot, values[:, 0])
            if np.max(np.abs(values[:6, 0] - initial)) > 0.15 or np.max(np.abs(values[:6, 1])) > 0.5:
                log.write(json.dumps(dict(event='motion_guard', elapsed=now-start, state=values.tolist(), initial=initial.tolist())) + '\n')
                raise RuntimeError(f'静态观察移动过大：角度变化={np.round(values[:6, 0]-initial, 3)}，速度={np.round(values[:6, 1], 3)}；结束补偿，请托稳主臂')
            gravity = leader_gravity_torque(robot, values[:6, 0], settings)
            if not np.isfinite(gravity).all() or np.any(np.abs(gravity) > limit):
                raise RuntimeError('重力模型力矩无效或超限')
            elapsed = now - start
            torque = min(1., elapsed / 2.) * gravity
            if not robot.pos_vel_tqe_kp_kd(np.zeros(6), np.zeros(6), torque, np.zeros(6), kd):
                raise RuntimeError('主臂补偿命令被拒绝')
            if not robot.gripper_control_MIT(0, 0, 0, 0, 0.05):
                raise RuntimeError('主臂夹爪阻尼命令被拒绝')
            log.write(json.dumps(dict(elapsed=elapsed, wall_time=time.time(), state=values.tolist(),
                gravity=gravity.tolist(), commanded_feedforward=torque.tolist(), kd=kd.tolist(),
                leader_gravity_scale=settings['leader_gravity_scale'], mode='supported_gravity_observation')) + '\n')
            (RUNTIME / 'heartbeat').touch()
            time.sleep(max(0, .01 - (time.monotonic() - now)))
    print(f'8 秒观察结束，退出补偿，请继续承托。数据：{path}', flush=True)


def worker(args, settings):
    roles = ['leader', 'follower'] if args.role == 'both' else [args.role]
    active = not args.check and not args.state and not args.sample
    devices, configs = preflight(settings, active, roles)
    from Panthera_lib import Panthera
    if args.check:
        print('预检通过：端口、电机版本、SDK 导入和配置正常；未发送运动命令。', flush=True)
        return
    robots = {}
    try:
        for role in roles:
            robots[role] = Panthera(str(configs[role]))
        for _ in range(3):
            for r in robots.values():
                r.send_get_motor_state_cmd()
            time.sleep(0.05)
        for role, r in robots.items():
            values = snapshot(r, settings['state_timeout_s'])
            print(f'{role} 关节及夹爪角度(rad)：{np.round(values[:, 0], 4)}', flush=True)
            validate_positions(r, values[:, 0])
        if args.sample:
            destination = ROOT / 'calibration'
            destination.mkdir(exist_ok=True)
            path = destination / f'leader-baseline-{time.time_ns()}.jsonl'
            print('开始 8 秒主臂基线采样。请扶稳并保持姿态；本模式不提供重力支撑。', flush=True)
            start = time.monotonic()
            with path.open('x') as log:
                while time.monotonic() - start < 8:
                    robot = robots['leader']
                    robot.send_get_motor_state_cmd()
                    time.sleep(0.05)
                    values = snapshot(robot, settings['state_timeout_s'])
                    validate_positions(robot, values[:, 0])
                    gravity = robot.get_Gravity(values[:6, 0])
                    if not np.isfinite(gravity).all():
                        raise RuntimeError('模型重力力矩无效')
                    log.write(json.dumps(dict(wall_time=time.time(),
                        elapsed=time.monotonic() - start, state=values.tolist(),
                        model_gravity=gravity.tolist(),
                        mode='supported_state_only_not_gravity_identification')) + '\n')
            print(f'采样已保存：{path}。手扶状态下的电机力矩不能直接用于拟合重力参数。', flush=True)
            return
        if args.state:
            print('所选机械臂状态检查通过；未发送位置/力矩目标。', flush=True)
            return
        (RUNTIME / 'heartbeat').touch()
        (RUNTIME / 'running').touch()
        if args.gravity_observe:
            gravity_observation(robots['leader'], settings, devices['leader'])
            return
        run_loop(robots['leader'], robots['follower'], settings, devices, args.duration)
    finally:
        (RUNTIME / 'cleanup').touch()
        shutdown(robots)


def check_worker_health(started):
    if (RUNTIME / 'cleanup').exists():
        if time.time() - (RUNTIME / 'cleanup').stat().st_mtime > 3:
            raise RuntimeError('SDK 退出清理超过 3 秒，终止工作进程；不是控制循环心跳超时')
    elif (RUNTIME / 'running').exists():
        if time.time() - (RUNTIME / 'heartbeat').stat().st_mtime > 0.5:
            raise RuntimeError('控制进程心跳超时；终止进程，电机 watchdog 进入阻尼')
    elif time.monotonic() - started > 25:
        raise RuntimeError('通信预检/SDK 初始化超时')


def supervise(args):
    for name in ['heartbeat', 'running', 'cleanup']:
        (RUNTIME / name).unlink(missing_ok=True)
    child = subprocess.Popen([sys.executable, '-u', __file__, *sys.argv[1:], '--worker'], start_new_session=True)
    started = time.monotonic()
    interrupted = False
    try:
        while child.poll() is None:
            check_worker_health(started)
            time.sleep(0.05)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
    return 0 if interrupted else child.returncode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true', help='只检查端口和电机版本，不构造 SDK Robot')
    mode.add_argument('--state', action='store_true', help='连接 SDK 读取状态，退出时发送制动')
    mode.add_argument('--gravity-observe', action='store_true', help='单独主臂，8 秒手扶重力补偿观察')
    mode.add_argument('--sample', action='store_true', help='手扶主臂静态采样 8 秒；不施加重力补偿')
    parser.add_argument('--role', choices=['both', 'leader', 'follower'], default='both')
    parser.add_argument('--duration', type=float, help='遥操作自动结束秒数')
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.sample and args.role != 'leader':
        parser.error('--sample 必须使用 --role leader')
    if args.gravity_observe and args.role != 'leader':
        parser.error('--gravity-observe 必须使用 --role leader')
    if args.role != 'both' and not (args.check or args.state or args.sample or args.gravity_observe):
        parser.error('遥操作必须同时使用主从两臂')
    if args.duration is not None and (not np.isfinite(args.duration) or args.duration <= 0):
        parser.error('--duration 必须为正数')
    RUNTIME.mkdir(exist_ok=True)
    config_file = ROOT / 'teleop-config.json'
    if not config_file.exists():
        raise RuntimeError('缺少 teleop-config.json；从 teleop-config.example.json 复制并填入已验证的 USB 路径')
    settings = json.loads(config_file.read_text())
    validate_settings(settings)
    def stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    if args.worker:
        worker(args, settings)
        return 0
    with (RUNTIME / 'teleop.lock').open('w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('已有遥操作/检查进程在运行')
        return supervise(args)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('\n已停止。退出制动不保证抗重力支撑，请扶稳机械臂。', flush=True)
        sys.exit(0)
    except Exception as exc:
        print(f'错误：{exc}', file=sys.stderr, flush=True)
        sys.exit(1)

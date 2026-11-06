"""Independent single-arm or paired return, sharing the hardware watchdog."""
import json
from pathlib import Path
import time

import numpy as np

from telemetry import Publisher
from teleop import (RUNTIME, snapshot, validate_positions, bounded_target,
                    leader_gravity_torque)


def validated_pose(payload, robots):
    if payload.get('unit') != 'rad':
        raise ValueError('回位姿态单位必须为 rad')
    poses = {}
    for role, robot in robots.items():
        pose = np.asarray(payload.get(role), dtype=float)
        lower = np.r_[robot.joint_limits['lower'], robot.gripper_limits['lower']]
        upper = np.r_[robot.joint_limits['upper'], robot.gripper_limits['upper']]
        if pose.shape != (7,) or not np.isfinite(pose).all() or np.any(pose < lower - .01) or np.any(pose > upper + .01):
            raise ValueError(f'{role} 起始姿态无效或超出关节限位')
        clipped = np.clip(pose, lower, upper)
        if np.any(clipped != pose):
            print(f'{role} 起始读数在限位边界 0.01 rad 内，目标投影至合法范围：{clipped.tolist()}', flush=True)
        poses[role] = clipped
    return poses


def run_reset(robots, settings, devices, target_path):
    target_path = Path(target_path)
    poses = validated_pose(json.loads(target_path.read_text()), robots)
    previous = {}
    for role, robot in robots.items():
        measured = snapshot(robot, settings['state_timeout_s'])[:, 0]
        validate_positions(robot, measured)
        previous[role] = np.clip(measured,
            np.r_[robot.joint_limits['lower'], robot.gripper_limits['lower']],
            np.r_[robot.joint_limits['upper'], robot.gripper_limits['upper']])
    speed = np.r_[np.full(6, min(.35, settings['max_target_speed_rad_s'])),
                  min(.5, settings['gripper_max_target_speed_rad_s'])]
    period = 1 / settings['frequency_hz']
    limits = np.asarray(settings['torque_limits_nm'])
    kp, kd = np.asarray(settings['joint_kp']), np.asarray(settings['joint_kd'])
    start = last = time.monotonic()
    settled_since = None
    publisher = Publisher()
    try:
        while time.monotonic() - start < 45:
            now = time.monotonic()
            if now - last > .1:
                raise RuntimeError('回位循环延迟超过 100ms')
            dt = min(max(now - last, period), .02)
            last = now
            states = {}
            feedback = []
            settled = True
            for role, robot in robots.items():
                if not Path(settings[role]).exists() or Path(settings[role]).resolve() != devices[role]:
                    raise RuntimeError(f'{role} USB 连接变化，停止回位')
                state = snapshot(robot, settings['state_timeout_s'], feedback if role == 'follower' else None)
                states[role] = state
                validate_positions(robot, state[:, 0])
                # Ramp the reference speed as well as gravity compensation.
                ramp = min(1., (now - start) / 1.)
                target = bounded_target(previous[role], poses[role], speed * ramp, dt)
                if np.max(np.abs(target - state[:, 0])) > min(.3, settings['max_tracking_error_rad']):
                    raise RuntimeError(f'{role} 回位跟踪误差过大')
                gravity = (leader_gravity_torque(robot, state[:6, 0], settings)
                           if role == 'leader' else np.asarray(robot.get_Gravity(state[:6, 0])))
                if not np.isfinite(gravity).all():
                    raise RuntimeError('回位动力学返回非有限值')
                if not robot.pos_vel_tqe_kp_kd(target[:6], np.zeros(6), ramp * np.clip(gravity, -limits, limits), kp, kd):
                    raise RuntimeError(f'{role} 回位命令被拒绝')
                if not robot.gripper_control_MIT(float(target[6]), 0, 0, 4., .4):
                    raise RuntimeError(f'{role} 夹爪回位命令被拒绝')
                previous[role] = target
                settled &= bool(np.max(np.abs(poses[role] - state[:, 0])) <= .04 and
                                np.max(np.abs(state[:, 1])) <= .08)
            if 'follower' in states:
                publisher.send(states['follower'], feedback, action=previous['follower'],
                               leader_position=states['leader'][:, 0] if 'leader' in states else None)
            (RUNTIME / 'heartbeat').touch()
            if settled:
                if settled_since is None:
                    settled_since = now
                elif now - settled_since >= .5:
                    target_path.with_suffix('.done').write_text('reached\n')
                    print(f'{", ".join(robots)} 及夹爪已回到本轮采集起始姿态。', flush=True)
                    return
            else:
                settled_since = None
            time.sleep(max(0, period - (time.monotonic() - now)))
        raise RuntimeError('45 秒内未到达起始姿态，回位失败')
    finally:
        publisher.close()

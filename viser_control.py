"""Hardware-independent state and limits for the dual-arm Viser controller."""
import math
import time
import numpy as np

LOWER = np.array([-2.4, -.1, -.1, -1.6, -1.7, -2.5])
UPPER = np.array([2.4, 3.2, 4., 1.6, 1.7, 2.5])
JOINT_NAMES = tuple(f'joint{i}' for i in range(1, 7))


def checked_target(value):
    q = np.asarray(value, dtype=float)
    if q.shape != (6,) or not np.isfinite(q).all():
        raise ValueError('目标必须是六个有效的关节角度')
    invalid=(q < LOWER-1e-7) | (q > UPPER+1e-7)
    if np.any(invalid):
        j=int(np.flatnonzero(invalid)[0])
        raise ValueError(f'J{j+1} 目标 {q[j]:.6f} rad（{np.degrees(q[j]):.3f}°）超出允许范围 '
                         f'[{LOWER[j]:.3f}, {UPPER[j]:.3f}] rad；这不是实测姿态越界提示')
    return np.clip(q, LOWER, UPPER)


def slew(previous, desired, speed, dt):
    if not math.isfinite(speed) or not .03 <= speed <= 1.5:
        raise ValueError('速度必须在 0.03–1.5 rad/s 内')
    if not math.isfinite(dt) or dt < 0 or dt > .1:
        raise ValueError('控制循环时间异常')
    return previous + np.clip(desired - previous, -speed * min(dt, .02), speed * min(dt, .02))


def ring_angle(matrix, previous, lower, upper):
    angle = math.atan2(matrix[1, 0], matrix[0, 0])
    angle += round((previous - angle) / (2 * math.pi)) * 2 * math.pi
    return float(np.clip(angle, lower, upper))


class ControlState:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.actual = np.zeros((2, 6))
        self.desired = self.actual.copy()
        self.enabled = [False, False]
        self.owner = None
        self.ready = False
        self.updated = -math.inf
        self.initialized = False
        self.speed = 1.5
        self.error = ''
        self.epochs = [0, 0]

    def feedback(self, positions, stamp=None):
        q = np.asarray(positions, dtype=float)
        if q.shape != (2, 6) or not np.isfinite(q).all():
            raise ValueError('反馈无效')
        self.actual = q.copy()
        self.updated = self.clock() if stamp is None else stamp
        self.ready = True
        if not self.initialized:
            self.desired = np.clip(q, LOWER, UPPER)
            self.initialized = True

    def enable(self, arm, client):
        if not self.ready or self.clock() - self.updated > .3:
            raise ValueError('实机反馈未就绪，请等待连接')
        if self.owner is not None and self.owner != client:
            raise ValueError('另一浏览器正在控制，请先在该窗口停止')
        self.owner = client
        self.desired[arm] = np.clip(self.actual[arm], LOWER, UPPER)
        self.enabled[arm] = True
        self.epochs[arm] += 1
        self.error = ''

    def disable(self, arm=None):
        for i in range(2) if arm is None else (arm,):
            self.enabled[i] = False
            self.epochs[i] += 1
        if not any(self.enabled):
            self.owner = None

    def disconnected(self, client):
        if self.owner == client:
            self.disable()

    def target(self, arm, q, client):
        if self.owner is not None and self.owner != client:
            raise ValueError('另一浏览器正在控制')
        self.desired[arm] = checked_target(q)

    def command(self):
        return dict(stamp=self.clock(), targets=self.desired.tolist(), enabled=list(self.enabled),
                    speed=self.speed, epochs=list(self.epochs))


class ConsoleState(ControlState):
    """Explicit operation state: resetting/teleop cannot be edited as web targets."""
    def __init__(self, clock=time.monotonic):
        super().__init__(clock)
        self.mode = 'idle'
        self.operation = 0

    def require_owner(self, client):
        if client is None:
            raise ValueError('操作必须来自已连接的浏览器')
        if self.owner not in (None, client):
            raise ValueError('另一浏览器正在控制，请先停止当前操作')

    def enable(self, arm, client):
        self.require_owner(client)
        if self.mode in ('reset', 'teleop'):
            raise ValueError('请先停止回零/主从遥操，再启用网页控制')
        super().enable(arm, client)
        self.mode = 'web'

    def begin(self, mode, client):
        if mode not in ('web', 'reset', 'teleop'):
            raise ValueError('未知控制模式')
        self.require_owner(client)
        if mode == 'reset' and self.mode == 'reset':
            raise ValueError('双臂正在回零，请等待完成或点停止')
        if any(self.enabled) and mode != 'reset' and not (self.mode=='hold' and mode in ('web','teleop')):
            raise ValueError('请先停止当前操作，再切换控制模式')
        if not self.ready or self.clock() - self.updated > .3:
            raise ValueError('两臂反馈未就绪，请先重新连接')
        if mode == 'reset' and np.max(np.abs(self.actual)) > .8:
            raise ValueError('当前有关节距零位超过 0.8 rad，请先低速移近零位再回零')
        # Both are aligned atomically before the first active command.
        for i in range(2): super().enable(i, client)
        self.mode = mode
        self.operation += 1
        if mode == 'reset': self.desired[:] = 0.

    def finish_reset(self, operation):
        if self.mode == 'reset' and self.operation == operation and all(self.enabled):
            self.mode = 'hold'
            self.desired[:] = 0.
            return True
        return False

    def disable(self, arm=None):
        # Paired modes stop together even if an old single-arm button is clicked.
        if self.mode in ('reset', 'teleop'): arm = None
        super().disable(arm)
        if not any(self.enabled):
            self.mode = 'idle'
            self.operation += 1

    def target(self, arm, q, client):
        self.require_owner(client)
        if self.mode in ('reset', 'teleop'):
            raise ValueError('回零/主从遥操期间不能编辑网页目标')
        super().target(arm,q,client)
        if self.mode == 'hold': self.mode = 'web'

    def command(self):
        return dict(super().command(), mode=self.mode, operation=self.operation)


class ControlFault(RuntimeError):
    def __init__(self, code, message, **detail):
        super().__init__(message)
        self.code = code
        self.detail = detail


def tracking_guard(arm, reference, actual, desired, limit, speed, mode):
    error = np.abs(reference - actual)
    j = int(np.argmax(error))
    if error[j] > limit:
        raise ControlFault('tracking_error',
            f'{arm} 臂 J{j+1} 跟踪误差 {error[j]:.3f} rad（{np.degrees(error[j]):.1f}°），'
            f'超过 {limit:.2f} rad；已停止双臂',
            arm=arm, joint=j+1, actual_rad=float(actual[j]), reference_rad=float(reference[j]),
            desired_rad=float(desired[j]), error_rad=float(error[j]), limit_rad=float(limit),
            speed_rad_s=float(speed), mode=mode)


def effective_targets(command, positions):
    """Only active web/hold arms consume GUI targets. Feedback is never a command."""
    actual=np.asarray(positions,dtype=float)
    if actual.shape!=(2,6) or not np.isfinite(actual).all():
        raise ValueError('两臂反馈形状或数值无效')
    mode=command.get('mode','web')
    bounded=np.clip(actual,LOWER,UPPER)
    if mode=='idle':return [q.copy() for q in bounded]
    if mode=='reset':return [np.zeros(6),np.zeros(6)]
    if mode=='teleop':return [bounded[0].copy(),bounded[0].copy()]
    if mode not in ('web','hold'):raise ValueError('未知目标模式')
    targets=[]
    for i in range(2):
        if not command['enabled'][i]:targets.append(bounded[i].copy());continue
        try:targets.append(checked_target(command['targets'][i]))
        except ValueError as exc:
            raise ControlFault('target_limit',f'{"AB"[i]} 臂：{exc}',arm='AB'[i],mode=mode,stage='target_validation') from exc
    return targets

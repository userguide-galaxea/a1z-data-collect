import time

import numpy as np

from utils.interfaces import LeaderArmInterface


def _quat_xyzw_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def _quat_inv_xyzw(q):
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)


def _quat_mul_xyzw(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ], dtype=np.float64)


_JOINT_LIMITS = [
    (-2.094, 2.094),
    (0.0, 3.142),
    (-3.142, 0.0),
    (-1.484, 1.484),
    (-1.484, 1.484),
    (-2.007, 2.007),
]

_MAX_JOINT_VEL = np.array([1.0, 1.0, 1.0, 1.5, 3.0, 4.0], dtype=np.float64)
_SLOW_JOINT_VEL = np.array([0.3, 0.3, 0.3, 0.5, 0.5, 0.8], dtype=np.float64)

# --- VR 遥操作参数 (参考 vr_joy/arm_control.py) ---
TRIGGER_DEADZONE = 0.05
TRIGGER_CURVE_EXP = 1.0   # 1.0=线性响应 (0.7=前段灵敏, >1.0=后段灵敏)
_BASE_SAFETY_RADIUS = 0.12
_OPEN_TIMEOUT_SEC = 60.0
_CONN_DROP_TIMEOUT = 3.0
_GRIP_ACTIVATE_THRESH = 0.15


def remap_trigger(press_index, deadzone=TRIGGER_DEADZONE, curve_exp=TRIGGER_CURVE_EXP):
    """trigger 原始值 → 归一化夹爪指令: 死区 + 线性重映射 + power-law 曲线。"""
    if press_index <= deadzone:
        return 0.0
    effective = (press_index - deadzone) / (1.0 - deadzone)
    effective = float(np.clip(effective, 0.0, 1.0))
    if curve_exp != 1.0:
        effective = effective ** curve_exp
    return effective


class VRControl(LeaderArmInterface):
    """VR 遥操作控制器 — 使用 A1ZArmIK (5-DOF IK + roll 分离) 做逆向运动学。

    arm_ik_left / arm_ik_right 应为 robots.vr_arm_ik.A1ZArmIK 实例。
    """

    def __init__(self, vr_store, arm_ik_left, arm_ik_right, scale=1.0):
        self._vr_store = vr_store
        self._arm_ik_l = arm_ik_left
        self._arm_ik_r = arm_ik_right
        self._scale = scale

        self._enabled = False
        self._returning_to_zero = False
        self._anchored_r = False
        self._anchored_l = False

        self._anchor_r_pos = None
        self._anchor_r_quat = None
        self._anchor_l_pos = None
        self._anchor_l_quat = None
        self._anchor_ee_pos_r = None
        self._anchor_ee_rot_r = None
        self._anchor_ee_pos_l = None
        self._anchor_ee_rot_l = None

        self._last_cmd_l = np.zeros(6, dtype=np.float64)
        self._last_cmd_r = np.zeros(6, dtype=np.float64)
        self._last_grip_l = 1.0
        self._last_grip_r = 1.0

        # 连接掉线状态追踪
        self._was_disconnected_l = False
        self._was_disconnected_r = False

    @property
    def vr_store(self):
        return self._vr_store

    # --- 右手柄全局监听调用的控制方法 ---
    # 由 VRHandSupervisor 在独立线程里调用，不经过 events 字典。

    def toggle_enabled(self):
        """A键(下): 切换使能/失能。"""
        self._enabled = not self._enabled
        self._vr_store.send_haptic("right", count=1, amp=0.8)
        if self._enabled:
            print("[VR] 右手 A键(下) → 遥操作使能 (握住手柄即可操作)")
        else:
            self._anchored_r = False
            self._anchored_l = False
            print("[VR] 右手 A键(下) → 遥操作失能 (双臂冻结)")

    def trigger_return_to_zero(self):
        """B键(上): 回零。"""
        if self._returning_to_zero:
            return
        self._returning_to_zero = True
        self._enabled = False
        self._anchored_r = False
        self._anchored_l = False
        self._vr_store.send_haptic("right", count=2, amp=0.6)
        print("[VR] 右手 B键(上) → 双臂回零中...")

    def emergency_stop(self):
        """右摇杆按下: 急停。"""
        self._enabled = False
        self._anchored_r = False
        self._anchored_l = False
        self._vr_store.send_haptic("right", count=3, amp=1.0)
        print("[VR] 右手摇杆按下 → 急停 (双臂冻结)")

    @property
    def teleop_enabled(self):
        """供 UI 提示查询当前是否使能。"""
        return self._enabled

    def open(self):
        """阻塞直到 Quest 双手均上报过数据，或超时抛 TimeoutError。"""
        print("[VRControl] 等待 Quest 双手连接 (左手 + 右手) ...")
        deadline = time.monotonic() + _OPEN_TIMEOUT_SEC
        while self._vr_store.right.timestamp == 0 or self._vr_store.left.timestamp == 0:
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"VR 连接超时: 在 {_OPEN_TIMEOUT_SEC}s 内未收到 Quest 双手数据 "
                    f"(left.timestamp={self._vr_store.left.timestamp:.3f}, "
                    f"right.timestamp={self._vr_store.right.timestamp:.3f}). "
                    "请确认 Quest 浏览器已打开 http://<主机IP>:8000 并进入 VR。"
                )
            time.sleep(0.1)
        print("[VRControl] 双手已连接, 遥操作就绪。")

    def close(self):
        bridge = getattr(self, "_bridge", None)
        if bridge is not None:
            bridge.stop()

    def sync_state(self, q_left, q_right):
        """将从臂当前关节状态同步到 VR 控制器和 IK 热启动初值。"""
        self._last_cmd_l = np.asarray(q_left, dtype=np.float64).copy()
        self._last_cmd_r = np.asarray(q_right, dtype=np.float64).copy()
        self._arm_ik_l.sync_state(q_left)
        self._arm_ik_r.sync_state(q_right)

    def _state_key(self):
        """返回当前状态的摘要 key，用于检测状态变化。"""
        return (self._enabled, self._anchored_r, self._anchored_l, self._returning_to_zero)

    def read_as_vector(self):
        # 状态变化时打印一行
        sk = self._state_key()
        if not hasattr(self, "_last_state_key"):
            self._last_state_key = None
        if sk != self._last_state_key:
            self._last_state_key = sk
            if self._returning_to_zero:
                print("[VR] 状态: 回零中...")
            elif self._enabled and self._anchored_r and self._anchored_l:
                print("[VR] 状态: 双手遥操作中")
            elif self._enabled and self._anchored_r:
                print("[VR] 状态: 右手遥操作中 (左手待握持)")
            elif self._enabled and self._anchored_l:
                print("[VR] 状态: 左手遥操作中 (右手待握持)")
            elif self._enabled:
                print("[VR] 状态: 已使能 — 握住 side grip 开始遥操作")
            else:
                print("[VR] 状态: 已锁定 (按右手 A键 使能)")

        r_pos = self._vr_store.right.pos.copy()
        r_quat = self._vr_store.right.quat.copy()
        r_trigger = self._vr_store.right.trigger
        r_grip = self._vr_store.right.grip

        l_pos = self._vr_store.left.pos.copy()
        l_quat = self._vr_store.left.quat.copy()
        l_trigger = self._vr_store.left.trigger
        l_grip = self._vr_store.left.grip

        # 连接掉线检测
        r_connected = self._vr_store.is_connected("right", _CONN_DROP_TIMEOUT)
        l_connected = self._vr_store.is_connected("left", _CONN_DROP_TIMEOUT)

        if not r_connected:
            if not self._was_disconnected_r:
                print("[VRControl] 右手掉线, 冻结右臂目标。")
                self._was_disconnected_r = True
            self._anchored_r = False
        else:
            if self._was_disconnected_r:
                print("[VRControl] 右手恢复连接。")
                self._vr_store.send_haptic("right", count=2, amp=0.6)
                self._was_disconnected_r = False

        if not l_connected:
            if not self._was_disconnected_l:
                print("[VRControl] 左手掉线, 冻结左臂目标。")
                self._was_disconnected_l = True
            self._anchored_l = False
        else:
            if self._was_disconnected_l:
                print("[VRControl] 左手恢复连接。")
                self._vr_store.send_haptic("left", count=2, amp=0.6)
                self._was_disconnected_l = False

        max_delta = _MAX_JOINT_VEL / 30.0

        if self._returning_to_zero:
            target_l = np.zeros(6, dtype=np.float64)
            target_r = np.zeros(6, dtype=np.float64)
            grip_l = 1.0
            grip_r = 1.0
            max_delta = _SLOW_JOINT_VEL / 30.0
        else:
            teleop_active_r = self._enabled and r_grip > _GRIP_ACTIVATE_THRESH and r_connected
            teleop_active_l = self._enabled and l_grip > _GRIP_ACTIVATE_THRESH and l_connected

            if teleop_active_r:
                if not self._anchored_r:
                    self._anchor_r_pos = r_pos.copy()
                    self._anchor_r_quat = r_quat.copy()
                    # 用 arm_ik 的 sync_state 替代旧 fk(): 同时设置热启动初值 + 获取当前 EE 位姿
                    self._arm_ik_r.sync_state(self._last_cmd_r)
                    self._anchor_ee_pos_r, self._anchor_ee_rot_r = self._arm_ik_r.get_ee_pose()
                    self._anchored_r = True

                target_r = self._solve_ik(
                    self._arm_ik_r,
                    r_pos, r_quat,
                    self._anchor_r_pos, self._anchor_r_quat,
                    self._anchor_ee_pos_r, self._anchor_ee_rot_r,
                )
            else:
                target_r = self._last_cmd_r
                self._anchored_r = False

            if teleop_active_l:
                if not self._anchored_l:
                    self._anchor_l_pos = l_pos.copy()
                    self._anchor_l_quat = l_quat.copy()
                    self._arm_ik_l.sync_state(self._last_cmd_l)
                    self._anchor_ee_pos_l, self._anchor_ee_rot_l = self._arm_ik_l.get_ee_pose()
                    self._anchored_l = True

                target_l = self._solve_ik(
                    self._arm_ik_l,
                    l_pos, l_quat,
                    self._anchor_l_pos, self._anchor_l_quat,
                    self._anchor_ee_pos_l, self._anchor_ee_rot_l,
                )
            else:
                target_l = self._last_cmd_l
                self._anchored_l = False

            grip_l = float(np.clip(1.0 - remap_trigger(l_trigger), 0.0, 1.0))
            grip_r = float(np.clip(1.0 - remap_trigger(r_trigger), 0.0, 1.0))

        cmd_l, cmd_r = self._apply_velocity_limit(
            target_l, target_r, max_delta
        )

        for i in range(6):
            lo, hi = _JOINT_LIMITS[i]
            cmd_l[i] = float(np.clip(cmd_l[i], lo, hi))
            cmd_r[i] = float(np.clip(cmd_r[i], lo, hi))

        self._last_cmd_l = cmd_l.copy()
        self._last_cmd_r = cmd_r.copy()
        self._last_grip_l = grip_l
        self._last_grip_r = grip_r

        if self._returning_to_zero:
            if np.max(np.abs(cmd_l)) < 0.01 and np.max(np.abs(cmd_r)) < 0.01:
                self._returning_to_zero = False
                self._anchored_r = False
                self._anchored_l = False
                self._arm_ik_l.calibrate()
                self._arm_ik_r.calibrate()
                print("[VR] 回零完成，双臂已锁定。按右手 A键 重新使能。")

        action = np.concatenate([cmd_l, [grip_l], cmd_r, [grip_r]])
        velocity = np.zeros(12, dtype=np.float64)
        return action, velocity

    def _solve_ik(self, arm_ik, vr_pos, vr_quat, anchor_pos, anchor_quat,
                  anchor_ee_pos, anchor_ee_rot):
        """计算 delta → 目标位姿 → 调用 A1ZArmIK.set_ee_target() 求解 IK。"""
        delta_pos = (vr_pos - anchor_pos) * self._scale
        target_pos = anchor_ee_pos + delta_pos

        # 基座安全半径: 防止末端目标进入基座周围球内 (防撞操作者胸口)
        target_dist = float(np.linalg.norm(target_pos))
        if target_dist < _BASE_SAFETY_RADIUS:
            if target_dist > 1e-6:
                target_pos = target_pos / target_dist * _BASE_SAFETY_RADIUS
            else:
                target_pos = np.array([_BASE_SAFETY_RADIUS, 0.0, 0.0], dtype=np.float64)

        q_delta = _quat_mul_xyzw(vr_quat, _quat_inv_xyzw(anchor_quat))
        R_delta = _quat_xyzw_to_R(q_delta)
        target_rot = R_delta @ anchor_ee_rot

        # A1ZArmIK.set_ee_target 内部做增量 IK + 滤波，返回 N_TOTAL_DOFS 维关节目标
        return arm_ik.set_ee_target(target_pos, target_rot)

    def _apply_velocity_limit(self, target_l, target_r, max_delta):
        delta_l = target_l - self._last_cmd_l
        delta_l_clipped = np.clip(delta_l, -max_delta, max_delta)
        cmd_l = self._last_cmd_l + delta_l_clipped

        delta_r = target_r - self._last_cmd_r
        delta_r_clipped = np.clip(delta_r, -max_delta, max_delta)
        cmd_r = self._last_cmd_r + delta_r_clipped

        return cmd_l, cmd_r

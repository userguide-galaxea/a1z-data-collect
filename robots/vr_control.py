import time

import numpy as np

from utils.interfaces import LeaderArmInterface
from robots.vr_filter import WeightedMovingFilter


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

# --- VR 遥操作细化参数 (参考 vr_joy/arm_control.py) ---
TRIGGER_DEADZONE = 0.05      # trigger 死区: 小于此值视为未按
TRIGGER_CURVE_EXP = 0.7     # power-law 曲线指数, 使轻按更线性、更易控
_BASE_SAFETY_RADIUS = 0.12   # 末端目标位姿不得进入基座周围此半径球内 (防撞操作者胸口)
_FILTER_WEIGHTS = (0.1, 0.2, 0.3, 0.4)  # 4 点加权滑动平均权重 (和=1.0)
_OPEN_TIMEOUT_SEC = 60.0    # open() 等待 Quest 双手连接的最大秒数
_CONN_DROP_TIMEOUT = 3.0    # 超过此秒未收到某手数据视为掉线


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
    def __init__(self, vr_store, ik_left, ik_right, scale=1.0):
        self._vr_store = vr_store
        self._ik_left = ik_left
        self._ik_right = ik_right
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

        # 关节目标滤波器: 对 IK 解算的 q[:6] 做加权滑动平均, 抑制抖动。
        self._filter_l = WeightedMovingFilter(_FILTER_WEIGHTS, data_size=6)
        self._filter_r = WeightedMovingFilter(_FILTER_WEIGHTS, data_size=6)

        # 连接掉线状态追踪: 进入掉线时清锚; 恢复时震动一次提示操作者。
        self._was_disconnected_l = False
        self._was_disconnected_r = False

        self._nq_l = self._ik_left._model.nq
        self._nq_r = self._ik_right._model.nq

        # [DBG] 调试: 定位"按下按键无响应/卡顿"。每秒打印一次状态摘要。
        self._dbg_frames = 0
        self._dbg_last_t = time.monotonic()

    @property
    def vr_store(self):
        return self._vr_store

    def open(self):
        """阻塞直到 Quest 双手均上报过数据, 或超时抛 TimeoutError。

        原 Bug 3: 无超时无限阻塞, Quest 没连接或只有一只手时程序看起来卡死。
        现加打印 + 超时, 操作者能看到"正在等什么"。
        """
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
        self._last_cmd_l = np.asarray(q_left, dtype=np.float64).copy()
        self._last_cmd_r = np.asarray(q_right, dtype=np.float64).copy()
        # 锚定/回零后, 滤波器窗口已不再代表当前构型, 必须清空, 否则首帧会
        # 用旧窗口的加权结果, 出现一帧跳变。
        self._filter_l.reset()
        self._filter_r.reset()

    def read_as_vector(self):
        self._dbg_frames += 1
        now_dbg = time.monotonic()
        if now_dbg - self._dbg_last_t >= 1.0:
            rs = self._vr_store.right
            ls = self._vr_store.left
            r_conn = self._vr_store.is_connected("right", _CONN_DROP_TIMEOUT)
            l_conn = self._vr_store.is_connected("left", _CONN_DROP_TIMEOUT)
            print(
                f"[DBG] frames={self._dbg_frames} "
                f"enabled={self._enabled} ret_home={self._returning_to_zero} "
                f"R{('C' if r_conn else 'X')}{('A' if self._anchored_r else '-')} "
                f"grip={rs.grip:.2f} trig={rs.trigger:.2f} lo={int(rs.button_lower)} up={int(rs.button_upper)} ts={rs.timestamp:.2f} "
                f"L{('C' if l_conn else 'X')}{('A' if self._anchored_l else '-')} "
                f"grip={ls.grip:.2f} trig={ls.trigger:.2f} lo={int(ls.button_lower)} up={int(ls.button_upper)} ts={ls.timestamp:.2f}"
            )
            self._dbg_last_t = now_dbg
            self._dbg_frames = 0

        r_pos = self._vr_store.right.pos.copy()
        r_quat = self._vr_store.right.quat.copy()
        r_trigger = self._vr_store.right.trigger
        r_grip = self._vr_store.right.grip

        l_pos = self._vr_store.left.pos.copy()
        l_quat = self._vr_store.left.quat.copy()
        l_trigger = self._vr_store.left.trigger
        l_grip = self._vr_store.left.grip

        # 连接掉线检测: 某只手超过 _CONN_DROP_TIMEOUT 秒没上报 → 视为掉线。
        # 掉线期间冻结对应臂 (清锚, 回到 last_cmd); 恢复时震动一次提示操作者。
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

        if self._vr_store.get_button_upper_edge("right") and not self._returning_to_zero:
            self._returning_to_zero = True
            self._enabled = False
            self._vr_store.send_haptic("right", count=2, amp=0.6)
            print("[DBG] 右上键按下 → 进入回零模式")

        max_delta = _MAX_JOINT_VEL / 30.0

        if self._returning_to_zero:
            target_l = np.zeros(6, dtype=np.float64)
            target_r = np.zeros(6, dtype=np.float64)
            grip_l = 1.0
            grip_r = 1.0
            max_delta = _SLOW_JOINT_VEL / 30.0
        else:
            if self._vr_store.get_button_lower_edge("right"):
                self._enabled = not self._enabled
                self._vr_store.send_haptic("right", count=1, amp=0.8)
                print(f"[DBG] 右下键按下 → enabled 切换为 {self._enabled}")

            teleop_active_r = self._enabled and r_grip > 0.5 and r_connected
            teleop_active_l = self._enabled and l_grip > 0.5 and l_connected

            if teleop_active_r:
                if not self._anchored_r:
                    self._anchor_r_pos = r_pos.copy()
                    self._anchor_r_quat = r_quat.copy()
                    T_r = self._ik_right.fk(self._last_cmd_r)
                    self._anchor_ee_pos_r = T_r[:3, 3].copy()
                    self._anchor_ee_rot_r = T_r[:3, :3].copy()
                    self._anchored_r = True

                target_r = self._solve_ik(
                    self._ik_right, self._nq_r,
                    r_pos, r_quat,
                    self._anchor_r_pos, self._anchor_r_quat,
                    self._anchor_ee_pos_r, self._anchor_ee_rot_r,
                    self._last_cmd_r, self._filter_r,
                )
            else:
                target_r = self._last_cmd_r
                self._anchored_r = False

            if teleop_active_l:
                if not self._anchored_l:
                    self._anchor_l_pos = l_pos.copy()
                    self._anchor_l_quat = l_quat.copy()
                    T_l = self._ik_left.fk(self._last_cmd_l)
                    self._anchor_ee_pos_l = T_l[:3, 3].copy()
                    self._anchor_ee_rot_l = T_l[:3, :3].copy()
                    self._anchored_l = True

                target_l = self._solve_ik(
                    self._ik_left, self._nq_l,
                    l_pos, l_quat,
                    self._anchor_l_pos, self._anchor_l_quat,
                    self._anchor_ee_pos_l, self._anchor_ee_rot_l,
                    self._last_cmd_l, self._filter_l,
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

        action = np.concatenate([cmd_l, [grip_l], cmd_r, [grip_r]])
        velocity = np.zeros(12, dtype=np.float64)
        return action, velocity

    def _solve_ik(self, ik, nq, vr_pos, vr_quat, anchor_pos, anchor_quat,
                  anchor_ee_pos, anchor_ee_rot, last_cmd, filt):
        delta_pos = (vr_pos - anchor_pos) * self._scale
        target_pos = anchor_ee_pos + delta_pos

        # 基座安全半径: 末端目标不得进入基座周围 _BASE_SAFETY_RADIUS 米球内,
        # 防止操作者把目标拉到自己胸口导致机械臂撞基座/操作者。
        target_dist = float(np.linalg.norm(target_pos))
        if target_dist < _BASE_SAFETY_RADIUS:
            if target_dist > 1e-6:
                target_pos = target_pos / target_dist * _BASE_SAFETY_RADIUS
            else:
                target_pos = np.array([_BASE_SAFETY_RADIUS, 0.0, 0.0], dtype=np.float64)

        q_delta = _quat_mul_xyzw(vr_quat, _quat_inv_xyzw(anchor_quat))
        R_delta = _quat_xyzw_to_R(q_delta)
        target_rot = R_delta @ anchor_ee_rot

        target_pose = np.eye(4, dtype=np.float64)
        target_pose[:3, :3] = target_rot
        target_pose[:3, 3] = target_pos

        init_q = np.zeros(nq, dtype=np.float64)
        init_q[:6] = last_cmd

        converged, q = ik.ik(target_pose, init_q=init_q, damping=1e-3, max_iters=50)
        q6 = q[:6].copy() if converged else last_cmd.copy()
        # 对 IK 解算结果做加权滑动平均, 抑制高频抖动。
        return filt.next(q6)

    def _apply_velocity_limit(self, target_l, target_r, max_delta):
        delta_l = target_l - self._last_cmd_l
        delta_l_clipped = np.clip(delta_l, -max_delta, max_delta)
        cmd_l = self._last_cmd_l + delta_l_clipped

        delta_r = target_r - self._last_cmd_r
        delta_r_clipped = np.clip(delta_r, -max_delta, max_delta)
        cmd_r = self._last_cmd_r + delta_r_clipped

        return cmd_l, cmd_r

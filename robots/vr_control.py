import threading
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

    右手柄控制（A使能 / B回零 / 摇杆急停）由 VRHandSupervisor 在独立线程里
    直接调用本类方法，全局生效——不依赖外层采集主循环是否在轮询。本类在
    read_as_vector 里根据 _enabled / _returning_to_zero / _estopped 三个互斥
    状态产出动作：
        - 使能:       从 VR 手柄位姿解算 IK 驱动从臂
        - 回零:       朝零位匀速收，由主循环发到从臂真正运动
        - 急停:       冻结双臂目标（保持当前位置不动）
    """

    def __init__(self, vr_store, arm_ik_left, arm_ik_right, scale=1.0):
        self._vr_store = vr_store
        self._arm_ik_l = arm_ik_left
        self._arm_ik_r = arm_ik_right
        self._scale = scale

        # follower 句柄在 attach_follower() 时填上，用于回零模式下真正驱动从臂。
        # 不在 __init__ 传是为了不破坏 VRControl 与底层硬件的解耦（IK 测试时不要求 follower）。
        self._follower = None

        self._enabled = False
        self._returning_to_zero = False
        self._anchored_r = False
        self._anchored_l = False

        # 录制中标志：由 record_data.py 在 arm_collector.start/stop 时设置。
        # episode 之间（非录制态）按 B 键回零时，由 _zero_loop 独立线程直接
        # 驱动从臂 glide 回零；录制态下则由 read_as_vector() 的软回零处理。
        self._recording = False
        self._zero_thread = None

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

        # 锁：右手柄全局监听线程（VRHandSupervisor）与主控制循环（read_as_vector）
        # 都会改 _enabled / _returning_to_zero / 急停标志，加锁保证状态原子。
        self._state_lock = threading.Lock()
        # 急停标志：置位后冻结双臂目标，只有重新使能(A键)才清除。
        self._estopped = False

    def attach_follower(self, follower):
        """绑定从臂句柄，供回零模式下真正驱动从臂运动。"""
        self._follower = follower

    def set_recording(self, recording: bool):
        """由 record_data.py 在 arm_collector.start/stop 时调用。

        episode 之间（非录制态）按 B 键回零时，由 _zero_loop 独立线程直接
        驱动从臂 glide 回零；录制态下则由 read_as_vector() 的软回零处理。
        """
        self._recording = recording
        if not recording:
            # 录制刚结束：若此时有挂起的回零请求，启动独立回零线程。
            self._maybe_start_zero_thread()

    @property
    def vr_store(self):
        return self._vr_store

    # --- 右手柄全局监听调用的控制方法 ---
    # 由 VRHandSupervisor 在独立线程里调用，不经过 events 字典。

    def toggle_enabled(self):
        """A键(下): 切换使能/失能。"""
        with self._state_lock:
            if self._estopped:
                # 急停状态：A键清除急停并使能。
                self._estopped = False
                self._enabled = True
            else:
                self._enabled = not self._enabled
            new_enabled = self._enabled
            if not new_enabled:
                self._anchored_r = False
                self._anchored_l = False
        self._vr_store.send_haptic("right", count=1, amp=0.8)
        if new_enabled:
            print("[VR] 右手 A键(下) → 遥操作使能 (握住手柄即可操作)")
        else:
            print("[VR] 右手 A键(下) → 遥操作失能 (双臂冻结)")

    def trigger_return_to_zero(self):
        """B键(上): 回零。

        录制中：设 _returning_to_zero 标志，由 read_as_vector() 软回零。
        非录制中（episode 之间）：启动独立线程 _zero_loop 直接驱动从臂 glide 回零。
        """
        with self._state_lock:
            if self._returning_to_zero or (self._zero_thread is not None and self._zero_thread.is_alive()):
                return
            self._enabled = False
            self._estopped = False
            self._anchored_r = False
            self._anchored_l = False
            if self._recording:
                # 录制中：软回零，由 read_as_vector() 消费 _returning_to_zero 标志
                self._returning_to_zero = True
                self._vr_store.send_haptic("right", count=2, amp=0.6)
                print("[VR] 右手 B键(上) → 双臂回零中 (软回零, 录制中)...")
            else:
                # 非录制中：独立线程硬回零
                self._vr_store.send_haptic("right", count=2, amp=0.6)
                print("[VR] 右手 B键(上) → 双臂回零中 (独立线程, episode 间)...")
                self._zero_thread = threading.Thread(target=self._zero_loop, daemon=True)
                self._zero_thread.start()

    def _maybe_start_zero_thread(self):
        """录制刚结束时，若操作者已按过 B 键（_returning_to_zero 挂起），启动独立回零。"""
        with self._state_lock:
            if self._returning_to_zero and not self._recording:
                self._returning_to_zero = False
                if self._zero_thread is None or not self._zero_thread.is_alive():
                    self._zero_thread = threading.Thread(target=self._zero_loop, daemon=True)
                    self._zero_thread.start()
                    print("[VR] 检测到录制结束时有挂起的回零请求, 启动独立回零线程...")

    def _zero_loop(self):
        """独立线程：用 utils.homing.glide_to 把从臂匀速收到零位。

        不依赖 ArmCollector/read_as_vector，因此 episode 之间也能回零。
        回零完成后更新 _last_cmd_* 与 IK 热启动初值，保证下次使能无跳变。
        """
        if self._follower is None:
            print("[VR] 回零失败: 未绑定从臂 (follower is None)")
            return
        try:
            from utils.homing import glide_to, home_vector
            dim = len(self._follower.get_joint_pos())
            home = home_vector(dim)
            glide_to(self._follower, home, secs=3.0)
            # 同步回零后的真实关节角到控制器内部状态
            pos = self._follower.get_joint_pos()
            self._last_cmd_l = np.asarray(pos[:6], dtype=np.float64).copy()
            self._last_cmd_r = np.asarray(pos[7:13], dtype=np.float64).copy()
            self._arm_ik_l.sync_state(self._last_cmd_l)
            self._arm_ik_r.sync_state(self._last_cmd_r)
            print("[VR] 独立回零完成, 双臂已锁定。按右手 A键 重新使能。")
        except Exception as e:
            print(f"[VR] 独立回零异常: {e}")
        finally:
            with self._state_lock:
                self._returning_to_zero = False

    def emergency_stop(self):
        """右摇杆按下: 急停。"""
        with self._state_lock:
            self._enabled = False
            self._estopped = True
            self._returning_to_zero = False
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
        return (self._enabled, self._anchored_r, self._anchored_l,
                self._returning_to_zero, self._estopped)

    def read_as_vector(self):
        # 状态变化时打印一行
        sk = self._state_key()
        if not hasattr(self, "_last_state_key"):
            self._last_state_key = None
        if sk != self._last_state_key:
            self._last_state_key = sk
            if self._estopped:
                print("[VR] 状态: 已急停 (双臂冻结, 按右手 A键 使能恢复)")
            elif self._returning_to_zero:
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

        # 急停：冻结双臂目标（保持当前位置），不做任何移动。
        # 与回零/使能互斥（emergency_stop() 已把另外两个标志清零）。
        if self._estopped:
            target_l = self._last_cmd_l.copy()
            target_r = self._last_cmd_r.copy()
            grip_l = self._last_grip_l
            grip_r = self._last_grip_r
            self._anchored_r = False
            self._anchored_l = False
        elif self._returning_to_zero:
            # 回零：target 朝零位匀速收（受 max_delta 限速），由主循环发到从臂真正运动。
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
            else:
                # 回零中：把当前 command 反馈给 IK 热启动初值，避免下次使能时 IK 从
                # 旧构型出发产生跳变（read_as_vector 每帧都会更新 self._last_cmd_*）。
                self._arm_ik_l.sync_state(cmd_l)
                self._arm_ik_r.sync_state(cmd_r)

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

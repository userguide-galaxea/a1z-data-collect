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

        self._nq_l = self._ik_left._model.nq
        self._nq_r = self._ik_right._model.nq

    @property
    def vr_store(self):
        return self._vr_store

    def open(self):
        while self._vr_store.right.timestamp == 0 or self._vr_store.left.timestamp == 0:
            time.sleep(0.1)

    def close(self):
        bridge = getattr(self, "_bridge", None)
        if bridge is not None:
            bridge.stop()

    def sync_state(self, q_left, q_right):
        self._last_cmd_l = np.asarray(q_left, dtype=np.float64).copy()
        self._last_cmd_r = np.asarray(q_right, dtype=np.float64).copy()

    def read_as_vector(self):
        r_pos = self._vr_store.right.pos.copy()
        r_quat = self._vr_store.right.quat.copy()
        r_trigger = self._vr_store.right.trigger
        r_grip = self._vr_store.right.grip

        l_pos = self._vr_store.left.pos.copy()
        l_quat = self._vr_store.left.quat.copy()
        l_trigger = self._vr_store.left.trigger

        if self._vr_store.get_button_upper_edge("right") and not self._returning_to_zero:
            self._returning_to_zero = True
            self._enabled = False
            self._vr_store.send_haptic("right", count=2, amp=0.6)

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

            teleop_active_r = self._enabled and r_grip > 0.5
            teleop_active_l = self._enabled and l_grip > 0.5

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
                    self._last_cmd_r,
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
                    self._last_cmd_l,
                )
            else:
                target_l = self._last_cmd_l
                self._anchored_l = False

            grip_l = float(np.clip(1.0 - l_trigger, 0.0, 1.0))
            grip_r = float(np.clip(1.0 - r_trigger, 0.0, 1.0))

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
                  anchor_ee_pos, anchor_ee_rot, last_cmd):
        delta_pos = (vr_pos - anchor_pos) * self._scale
        target_pos = anchor_ee_pos + delta_pos

        q_delta = _quat_mul_xyzw(vr_quat, _quat_inv_xyzw(anchor_quat))
        R_delta = _quat_xyzw_to_R(q_delta)
        target_rot = R_delta @ anchor_ee_rot

        target_pose = np.eye(4, dtype=np.float64)
        target_pose[:3, :3] = target_rot
        target_pose[:3, 3] = target_pos

        init_q = np.zeros(nq, dtype=np.float64)
        init_q[:6] = last_cmd

        converged, q = ik.ik(target_pose, init_q=init_q, damping=1e-3, max_iters=50)
        return q[:6].copy() if converged else last_cmd.copy()

    def _apply_velocity_limit(self, target_l, target_r, max_delta):
        delta_l = target_l - self._last_cmd_l
        delta_l_clipped = np.clip(delta_l, -max_delta, max_delta)
        cmd_l = self._last_cmd_l + delta_l_clipped

        delta_r = target_r - self._last_cmd_r
        delta_r_clipped = np.clip(delta_r, -max_delta, max_delta)
        cmd_r = self._last_cmd_r + delta_r_clipped

        return cmd_l, cmd_r

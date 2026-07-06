"""Delta-based teleoperation controller.

Mirrors the logic from open-a1z-t/teleop.py TeleopBridge:
  - Captures leader and follower zero positions on the first step
  - Sends: follower_init + (leader_pos - leader_init)
  - Clamps per-step spike to avoid sudden jumps
  - Low-pass filters leader velocity for feedforward
"""

import numpy as np
from utils.interfaces import TeleopControllerInterface

_DEFAULT_MAX_STEP_DEG = 10.0
_VEL_ALPHA = 0.4  # low-pass filter coefficient


class DeltaTeleopController(TeleopControllerInterface):
    """Single-arm delta controller. Tracks leader relative to its start pose."""

    def __init__(self, max_step_deg: float = _DEFAULT_MAX_STEP_DEG):
        self._max_step = np.radians(max_step_deg)
        self._leader_init: np.ndarray | None = None
        self._follower_init: np.ndarray | None = None
        self._pos_filtered: np.ndarray | None = None
        self._vel_filtered: np.ndarray = np.zeros(6)

    def compute_command(
        self,
        leader_action: np.ndarray,
        follower_pos: np.ndarray,
        leader_velocity: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray | None]:
        arm_pos = leader_action[:6]
        gripper = leader_action[6:]

        # Spike clamp on arm joints
        if self._pos_filtered is None:
            self._pos_filtered = arm_pos.copy()
        else:
            step = arm_pos - self._pos_filtered
            self._pos_filtered += np.clip(step, -self._max_step, self._max_step)

        # Capture zero on first call
        if self._leader_init is None:
            self._leader_init = self._pos_filtered.copy()
            self._follower_init = follower_pos[:6].copy()
            return np.append(follower_pos[:6], gripper), self._vel_filtered.copy()

        delta = self._pos_filtered - self._leader_init
        arm_cmd = self._follower_init + delta

        # Low-pass filter leader velocity for feedforward
        vel_ff = None
        if leader_velocity is not None:
            vel_meas = leader_velocity[:6]
            self._vel_filtered = _VEL_ALPHA * vel_meas + (1 - _VEL_ALPHA) * self._vel_filtered
            vel_ff = self._vel_filtered.copy()

        return np.append(arm_cmd, gripper), vel_ff


class DualDeltaTeleopController(TeleopControllerInterface):
    """Dual-arm delta controller. Two independent DeltaTeleopControllers."""

    def __init__(self, max_step_deg: float = _DEFAULT_MAX_STEP_DEG):
        self._left = DeltaTeleopController(max_step_deg)
        self._right = DeltaTeleopController(max_step_deg)

    def compute_command(
        self,
        leader_action: np.ndarray,
        follower_pos: np.ndarray,
        leader_velocity: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray | None]:
        vel_l = leader_velocity[:7] if leader_velocity is not None else None
        vel_r = leader_velocity[7:] if leader_velocity is not None else None

        cmd_l, vff_l = self._left.compute_command(leader_action[:7], follower_pos[:7], vel_l)
        cmd_r, vff_r = self._right.compute_command(leader_action[7:], follower_pos[7:], vel_r)

        cmd = np.concatenate([cmd_l, cmd_r])
        vel_ff = np.concatenate([vff_l, vff_r])
        return cmd, vel_ff

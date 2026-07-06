"""Dual-arm composition wrappers.

Combine a left and right single-arm leader/follower into one 14-element
interface, so the rest of the pipeline can treat a dual-arm rig exactly like a
single arm. These wrappers depend only on the abstract interfaces, not on any
concrete hardware class.
"""

import threading

import numpy as np

from utils.interfaces import LeaderArmInterface, FollowerArmInterface


class DualArmLeader(LeaderArmInterface):
    def __init__(self, left: LeaderArmInterface, right: LeaderArmInterface):
        self._left = left
        self._right = right

    def open(self):
        self._left.open()
        self._right.open()

    def read_as_vector(self):
        action_l, vel_l = self._left.read_as_vector()
        action_r, vel_r = self._right.read_as_vector()
        return np.concatenate([action_l, action_r]), np.concatenate([vel_l, vel_r])

    def close(self):
        self._left.close()
        self._right.close()


class DualArmFollower(FollowerArmInterface):
    def __init__(self, left: FollowerArmInterface, right: FollowerArmInterface):
        self._left = left
        self._right = right

    def start(self):
        # Start both arms concurrently so gripper homing runs in parallel (~3s saved).
        err = [None, None]
        def _start(arm, idx):
            try:
                arm.start()
            except Exception as e:
                err[idx] = e

        t_left  = threading.Thread(target=_start, args=(self._left,  0))
        t_right = threading.Thread(target=_start, args=(self._right, 1))
        t_left.start()
        t_right.start()
        t_left.join()
        t_right.join()

        # If either failed, stop whichever succeeded before re-raising.
        if err[0] is not None or err[1] is not None:
            if err[0] is None:
                try: self._left.stop()
                except Exception: pass
            if err[1] is None:
                try: self._right.stop()
                except Exception: pass
            raise RuntimeError(f"Arm start failed — left: {err[0]}  right: {err[1]}")

    def command(self, action: np.ndarray, velocity: np.ndarray | None = None):
        vel_l = velocity[:6] if velocity is not None else None
        vel_r = velocity[6:12] if velocity is not None else None
        self._left.command(action[:7], vel_l)
        self._right.command(action[7:], vel_r)

    def get_joint_pos(self):
        return np.concatenate([self._left.get_joint_pos(), self._right.get_joint_pos()])

    def stop(self):
        # Stop both arms concurrently so disable frames hit the CAN buses simultaneously.
        t_left  = threading.Thread(target=self._left.stop)
        t_right = threading.Thread(target=self._right.stop)
        t_left.start()
        t_right.start()
        t_left.join()
        t_right.join()

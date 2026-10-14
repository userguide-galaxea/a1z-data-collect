import time
import numpy as np
from sdk.a1z.robots.get_robot import get_a1z_robot
from utils.interfaces import FollowerArmInterface

_SOFT_KP = np.array([6.0, 6.0, 6.0, 4.0, 1.0, 1.0])
_CONTROL_HZ = 100
_MIN_FREQ_HZ = 40.0


class A1ZFollowerArm(FollowerArmInterface):
    """Wraps ArmRobot to implement FollowerArmInterface."""

    def __init__(self, can_channel: str, with_gripper: bool = True):
        self._robot = get_a1z_robot(
            can_channel=can_channel,
            gravity_comp_factor=1.0,
            zero_gravity_mode=False,
            with_gripper=with_gripper,
            control_freq_hz=_CONTROL_HZ,
            min_freq_hz=_MIN_FREQ_HZ,
        )

    def start(self) -> None:
        self._robot.start(initial_kp=_SOFT_KP)

    def command(self, action: np.ndarray, velocity: np.ndarray | None = None) -> None:
        # The pinned A1Z SDK supplies its default PD gains when kp/kd are omitted.
        cmd = {
            "pos": action[:6],
            "vel": velocity[:6] if velocity is not None else np.zeros(6),
        }
        self._robot.command_joint_state(cmd)
        if self._robot.gripper is not None and len(action) > 6:
            self._robot.command_gripper(float(np.clip(action[6], 0.0, 1.0)))

    def get_joint_pos(self) -> np.ndarray:
        return self._robot.get_joint_pos()

    def stop(self) -> None:
        self._robot.stop()
        chain = getattr(self._robot, "_motor_chain", None)
        gripper = self._robot.gripper
        for _ in range(2):
            time.sleep(0.015)
            if chain is not None:
                try:
                    chain.disable_all()
                except Exception:
                    pass
            if gripper is not None:
                try:
                    gripper.disable()
                except Exception:
                    pass
        bus = getattr(self._robot, "_bus", None)
        if bus is not None:
            try:
                bus.shutdown()
            except Exception:
                pass

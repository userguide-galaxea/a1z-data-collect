import numpy as np
from leader import DynamixelServoReader, DynamixelBus, load_config
from utils.interfaces import LeaderArmInterface


class A1ZTLeaderArm(LeaderArmInterface):
    """Wraps DynamixelServoReader to implement LeaderArmInterface."""

    def __init__(
        self,
        port: str,
        side: str = "left",
        config_path: str = None,
        shared_bus: "DynamixelBus | None" = None,
    ):
        cfg = load_config(config_path)
        self._reader = DynamixelServoReader(
            port, side=side, config=cfg, shared_bus=shared_bus
        )

    def open(self) -> None:
        self._reader.open()

    def read_as_vector(self) -> tuple[np.ndarray, np.ndarray]:
        arm_pos, grip_norm, vel = self._reader.read()
        action = np.append(arm_pos, 1.0 - grip_norm)
        return action, vel

    def close(self) -> None:
        self._reader.close()

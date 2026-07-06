from abc import ABC, abstractmethod
import numpy as np


class LeaderArmInterface(ABC):
    @abstractmethod
    def open(self) -> None:
        """Initialize hardware connection."""

    @abstractmethod
    def read_as_vector(self) -> tuple[np.ndarray, np.ndarray]:
        """Read current leader state.

        Returns:
            action:   shape (7,) — [j1..j6 rad, gripper_norm 0~1]
            velocity: shape (7,) — [j1..j6 rad/s, gripper_vel rad/s]
        """

    @abstractmethod
    def close(self) -> None:
        """Release hardware connection."""


class FollowerArmInterface(ABC):
    @abstractmethod
    def start(self) -> None:
        """Enable motors and start control loop."""

    @abstractmethod
    def command(self, action: np.ndarray, velocity: np.ndarray | None = None) -> None:
        """Send joint position command.

        Args:
            action:   shape (7,) — [j1..j6 rad, gripper_norm 0~1]
            velocity: shape (6,) — [j1..j6 rad/s] feedforward velocity, optional
        """

    @abstractmethod
    def get_joint_pos(self) -> np.ndarray:
        """Read current follower state.

        Returns:
            shape (7,) — [j1..j6 rad, gripper_norm 0~1]
        """

    @abstractmethod
    def stop(self) -> None:
        """Disable motors and stop control loop."""


class TeleopControllerInterface(ABC):
    """Maps leader readings to follower commands.

    Swap this out to change how the follower tracks the leader —
    e.g. delta control for a physical arm, absolute/cartesian for VR.
    """

    @abstractmethod
    def compute_command(
        self,
        leader_action: np.ndarray,
        follower_pos: np.ndarray,
        leader_velocity: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray | None]:
        """Compute command to send to follower.

        Args:
            leader_action:   shape (7,) or (14,) — leader joint positions
            follower_pos:    shape (7,) or (14,) — current follower positions
            leader_velocity: shape (7,) or (14,) — leader joint velocities, optional

        Returns:
            (action, velocity):
                action:   shape (7,) or (14,) — target positions
                velocity: shape (6,) or (12,) — feedforward velocities, or None
        """


class CameraInterface(ABC):
    @abstractmethod
    def open(self) -> None:
        """Initialize camera."""

    @abstractmethod
    def read(self) -> np.ndarray | None:
        """Return latest frame as HWC uint8 RGB, or None if no new frame.

        Used by the calibration path, which needs decoded pixels.
        """

    @abstractmethod
    def read_jpeg(self) -> bytes | None:
        """Return latest frame as JPEG bytes, or None if no new frame.

        Used by the recording path. Implementations should avoid a needless
        decode/re-encode when the device already delivers JPEG/MJPEG.
        """

    @abstractmethod
    def close(self) -> None:
        """Release camera."""

    def flush(self, n: int = 5) -> None:
        """Drop up to n queued frames so the next read is fresh. Default: no-op."""

"""Smoothly glide the follower to a target joint vector, and return it to home.

Shared by the replay tool and the collection script so Ctrl-C can bring the arm
to a safe home pose (arm joints 0 rad, grippers open) before powering down.
All moves go through the FollowerArmInterface; no hardware specifics here.
"""
import time

import numpy as np

GLIDE_HZ = 50    # step rate for smooth glide moves


def interp_targets(start, target, secs: float, hz: int = GLIDE_HZ) -> list:
    """Linearly interpolate from `start` to `target` over `secs` at `hz`.

    Returns target vectors (gripper column included), excluding the start point
    and ending exactly on `target`. Always at least one step.
    """
    start = np.asarray(start, dtype=np.float32)
    target = np.asarray(target, dtype=np.float32)
    n = max(1, int(secs * hz))
    return [(1.0 - k / n) * start + (k / n) * target for k in range(1, n + 1)]


def home_vector(dim: int) -> np.ndarray:
    """Home pose: arm joints 0 rad, gripper(s) open (norm 1.0 = open).

    dim 7 (single) -> gripper at index 6; dim 14 (dual) -> indices 6 and 13.
    """
    if dim == 7:
        gripper_idx = (6,)
    elif dim == 14:
        gripper_idx = (6, 13)
    else:
        raise ValueError(f"Unexpected dim {dim}, expected 7 (single) or 14 (dual)")
    v = np.zeros(dim, dtype=np.float32)
    for i in gripper_idx:
        v[i] = 1.0
    return v


def glide_to(follower, target, secs: float, hz: int = GLIDE_HZ, sleep_fn=time.sleep) -> None:
    """Smoothly command the follower from its current pose to `target` over `secs`.

    Issues only command()/get_joint_pos(); does not start/stop the follower.
    """
    start = follower.get_joint_pos()
    for t in interp_targets(start, target, secs, hz):
        follower.command(t)
        sleep_fn(1.0 / hz)


def return_to_home(follower, secs: float, hz: int = GLIDE_HZ, sleep_fn=time.sleep) -> None:
    """Glide the follower to home (arm 0, grippers open). dim inferred from state."""
    dim = len(follower.get_joint_pos())
    glide_to(follower, home_vector(dim), secs, hz, sleep_fn)

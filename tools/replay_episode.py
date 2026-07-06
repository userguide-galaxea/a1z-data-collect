#!/usr/bin/env python3
"""Replay a recorded HDF5 episode's joint trajectory on the real follower arm(s).

Usage:
    python tools/replay_episode.py data/two_master_slave/episode_0.hdf5
    python tools/replay_episode.py data/one_master_slave/episode_0.hdf5 \
        --source action --speed 0.5 --can-left can0
"""
import argparse
import os
import sys
import time

import h5py
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from robots.follower_a1z import A1ZFollowerArm
from robots.dual_arm import DualArmFollower
from utils.homing import glide_to, return_to_home

MAX_FRAME_GAP_S = 1.0    # cap per-frame sleep so a bad timestamp gap can't hang


def compute_sleeps(timestamps, speed: float) -> np.ndarray:
    """Per-frame sleep seconds from recorded timestamps, scaled by `speed`.

    Returns an array of length len(timestamps)-1. Negative gaps (non-monotonic
    timestamps) clip to 0; gaps above MAX_FRAME_GAP_S are capped for safety.
    """
    ts = np.asarray(timestamps, dtype=np.float64)
    # Cap the raw recorded gap (guards bad timestamps), then scale by speed so
    # slow playback (speed<1) can still stretch intervals past MAX_FRAME_GAP_S.
    diffs = np.clip(np.diff(ts), 0.0, MAX_FRAME_GAP_S)
    return diffs / float(speed)


def load_episode(path: str, source: str):
    """Read arm/<source> trajectory, follower timestamps, and arm/velocity (None if absent)."""
    if source not in ("state", "action"):
        raise ValueError(f"source must be 'state' or 'action', got {source!r}")
    with h5py.File(path, "r") as f:
        traj = np.asarray(f[f"arm/{source}"][:], dtype=np.float32)
        ts = np.asarray(f["arm/follower_timestamps"][:], dtype=np.float64)
        vel = np.asarray(f["arm/velocity"][:], dtype=np.float32) if "arm/velocity" in f else None
    return traj, ts, vel


def diff_summary(target, actual, recorded):
    """Per-joint amplitude/error stats to localize lost motion. None if empty.

    Spans are per-column (max-min); follow_err = mean|target-actual|,
    max_follow_err = max|target-actual|, reproduce_err = mean|actual-recorded|.
    """
    target, actual, recorded = np.asarray(target), np.asarray(actual), np.asarray(recorded)
    if len(target) == 0:
        return None
    return {
        "target_span": target.max(0) - target.min(0),
        "actual_span": actual.max(0) - actual.min(0),
        "recorded_span": recorded.max(0) - recorded.min(0),
        "follow_err": np.abs(target - actual).mean(0),
        "max_follow_err": np.abs(target - actual).max(0),
        "reproduce_err": np.abs(actual - recorded).mean(0),
    }


def replay(follower, traj, timestamps, speed: float = 1.0,
           move_secs: float = 3.0, velocities=None, sleep_fn=time.sleep,
           recorded=None, diff_every: int = 30, log_fn=print) -> None:
    """Smoothly move to frame 0, then command each trajectory frame on schedule.

    Only issues command()/get_joint_pos(); follower start/stop is the caller's
    responsibility. `sleep_fn` is injectable for testing.

    If `recorded` is given, run the --diff diagnostic: read the follower's actual
    position after each command and compare target/actual/recorded to localize
    where motion amplitude is lost. Off (no get_joint_pos, no logs) when None.
    """
    glide_to(follower, traj[0], move_secs, sleep_fn=sleep_fn)

    sleeps = compute_sleeps(timestamps, speed)
    targets, actuals = [], []
    for i in range(len(traj)):
        if velocities is not None:
            follower.command(traj[i], velocities[i])
        else:
            follower.command(traj[i])
        if recorded is not None:
            actual_i = np.asarray(follower.get_joint_pos(), dtype=np.float64)
            targets.append(np.asarray(traj[i], dtype=np.float64))
            actuals.append(actual_i)
            if i % diff_every == 0:
                fe = np.abs(targets[-1] - actual_i)
                re = np.abs(actual_i - np.asarray(recorded[i], dtype=np.float64))
                log_fn(f"[frame {i:5d}] max|target-actual|={fe.max():.4f} (j{int(fe.argmax())})  "
                       f"max|actual-recorded|={re.max():.4f} (j{int(re.argmax())})")
        if i < len(sleeps):
            sleep_fn(float(sleeps[i]))

    if recorded is not None:
        s = diff_summary(np.asarray(targets), np.asarray(actuals),
                         np.asarray(recorded[:len(targets)], dtype=np.float64))
        if s is not None:
            log_fn("\n   j |  tgt_span  act_span  rec_span | follow_err  max_follow_err reproduce_err")
            log_fn("  ---+------------------------------+----------------------------------------------")
            for j in range(len(s["target_span"])):
                log_fn(f"  {j:2d} | {s['target_span'][j]:9.3f} {s['actual_span'][j]:9.3f} "
                       f"{s['recorded_span'][j]:9.3f} | {s['follow_err'][j]:10.3f} "
                       f"{s['max_follow_err'][j]:15.3f} {s['reproduce_err'][j]:13.3f}")


def make_follower(dim: int, can_left: str, can_right: str):
    """Assemble the follower for a 7D (single) or 14D (dual) trajectory."""
    if dim == 7:
        return A1ZFollowerArm(can_left)
    if dim == 14:
        return DualArmFollower(A1ZFollowerArm(can_left), A1ZFollowerArm(can_right))
    raise ValueError(f"Unexpected joint dim {dim}, expected 7 (single) or 14 (dual)")


def main():
    parser = argparse.ArgumentParser(description="Replay a recorded episode on the follower arm(s).")
    parser.add_argument("episode", help="Path to episode_N.hdf5")
    parser.add_argument("--source", choices=["state", "action"], default="state",
                        help="Which signal to replay (default: state)")
    parser.add_argument("--can-left", default="can0", help="Left/single arm CAN channel")
    parser.add_argument("--can-right", default="can1", help="Right arm CAN channel (dual only)")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Playback speed multiplier (0.5 = half speed)")
    parser.add_argument("--move-secs", type=float, default=3.0,
                        help="Seconds to smoothly move to the first frame")
    parser.add_argument("--diff", action="store_true",
                        help="Print per-frame target/actual/recorded diagnostics")
    parser.add_argument("--use-vel", action="store_true",
                        help="Feed recorded arm/velocity as feedforward (episode must have been collected with use_velocity=true)")
    args = parser.parse_args()

    traj, ts, vel = load_episode(args.episode, args.source)
    dim = traj.shape[1]
    duration = float(ts[-1] - ts[0]) if len(ts) > 1 else 0.0
    recorded = load_episode(args.episode, "state")[0] if args.diff else None

    if args.use_vel:
        if vel is None:
            raise SystemExit("--use-vel set but episode has no arm/velocity (collect with use_velocity=true)")
        if len(vel) != len(traj):
            raise SystemExit(f"--use-vel: velocity frames ({len(vel)}) != trajectory frames ({len(traj)})")

    follower = make_follower(dim, args.can_left, args.can_right)
    print(f"Episode : {args.episode}")
    print(f"Source  : {args.source}   dim={dim} ({'dual' if dim == 14 else 'single'} arm)")
    print(f"Frames  : {len(traj)}   duration={duration:.2f}s   speed={args.speed}x")
    print(f"Frame 0 : {np.round(traj[0], 3)}")

    try:
        follower.start()
        try:
            input("\nPress Enter to start replay (Ctrl-C to cancel)...")
            replay(follower, traj, ts, speed=args.speed, move_secs=args.move_secs,
                   velocities=(vel if args.use_vel else None), recorded=recorded)
            print("Replay finished.")
        except KeyboardInterrupt:
            print("\nInterrupted — homing before exit.")
        return_to_home(follower, args.move_secs)  # home after normal finish or interrupt
        print("Home reached.")
    except KeyboardInterrupt:
        print("\nAborted homing — emergency stop.")
    finally:
        follower.stop()


if __name__ == "__main__":
    main()

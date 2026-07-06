#!/usr/bin/env python3
"""Inspect and visualize an HDF5 episode file.

Usage:
    python tools/inspect_episode.py data/one_master_slave/episode_0.hdf5
    python tools/inspect_episode.py data/two_master_slave/episode_0.hdf5 --no-video
"""

import argparse
import sys
from pathlib import Path

import h5py
import imageio
import numpy as np


def print_arm_info(f: h5py.File) -> None:
    ag = f["arm"]
    ts_leader   = ag["leader_timestamps"][:]
    ts_follower = ag["follower_timestamps"][:]
    n = len(ts_leader)
    duration = ts_leader[-1] - ts_leader[0] if n > 1 else 0.0
    fps = (n - 1) / duration if duration > 0 else 0.0

    print(f"\n{'='*50}")
    print(f"  Arm data")
    print(f"{'='*50}")
    print(f"  samples:   {n}")
    print(f"  duration:  {duration:.2f} s")
    print(f"  rate:      {fps:.1f} Hz")
    print()

    dim = ag["state"].shape[1]  # 7 single-arm / 14 dual-arm
    is_dual = (dim == 14)

    for key, label in [("action", "Leader action"), ("state", "Follower state")]:
        if key not in ag:
            continue
        data = ag[key][:]  # [T, 7] or [T, 14]

        if is_dual:
            for side, sl, name in [("left arm", slice(0, 6), "L"), ("right arm", slice(7, 13), "R")]:
                print(f"  {label} - {side} joints ({data[:, sl].shape}):")
                for j in range(6):
                    col = sl.start + j
                    print(f"    J{j+1}  min={data[:,col].min():8.3f}  "
                          f"max={data[:,col].max():8.3f}  "
                          f"mean={data[:,col].mean():8.3f} rad")
                grip_col = sl.start + 6
                g = data[:, grip_col]
                print(f"    gripper  min={g.min():.3f}  max={g.max():.3f}  mean={g.mean():.3f} (norm)")
                print()
        else:
            print(f"  {label} ({data.shape}):")
            for j in range(6):
                print(f"    J{j+1}  min={data[:,j].min():8.3f}  "
                      f"max={data[:,j].max():8.3f}  "
                      f"mean={data[:,j].mean():8.3f} rad")
            g = data[:, 6]
            print(f"    gripper  min={g.min():.3f}  max={g.max():.3f}  mean={g.mean():.3f} (norm)")
            print()

    lag = ts_follower - ts_leader
    print(f"  leader->follower lag:  mean={lag.mean()*1000:.2f} ms  "
          f"max={lag.max()*1000:.2f} ms  std={lag.std()*1000:.2f} ms")
    print()


def save_video(cam_name: str, frames, timestamps: np.ndarray,
               out_path: Path) -> None:
    import cv2
    n = len(frames)
    if n == 0:
        print(f"  [{cam_name}] 0 frames, skipped")
        return

    duration = timestamps[-1] - timestamps[0] if n > 1 else 1.0
    fps = (n - 1) / duration if duration > 0 else 15.0

    decoded = []
    for f in frames:
        img = cv2.imdecode(np.frombuffer(f, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            decoded.append(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

    if not decoded:
        print(f"  [{cam_name}] decode failed, skipped")
        return

    h, w = decoded[0].shape[:2]
    imageio.mimwrite(str(out_path), decoded, fps=fps, codec="libx264", quality=8)
    print(f"  [{cam_name}] {n} frames, {fps:.1f} fps, {w}x{h} -> {out_path.name}")


def print_camera_info(f: h5py.File, no_video: bool, out_dir: Path,
                      stem: str) -> None:
    import cv2
    if "cameras" not in f:
        print("No camera data")
        return

    cg = f["cameras"]
    print(f"{'='*50}")
    print(f"  Camera data{' (--no-video: reconstruction skipped)' if no_video else ' (video reconstruction)'}")
    print(f"{'='*50}")

    for cam_name in sorted(cg.keys()):
        kg = cg[cam_name]
        if "frames" not in kg:
            print(f"  [{cam_name}] 0 frames (empty group)")
            continue
        frames = kg["frames"][:]
        ts = kg["timestamps"][:]
        n = len(frames)
        duration = ts[-1] - ts[0] if n > 1 else 0.0
        fps = (n - 1) / duration if duration > 0 else 0.0
        if n > 0:
            first = cv2.imdecode(np.frombuffer(frames[0], dtype=np.uint8), cv2.IMREAD_COLOR)
            h, w = first.shape[:2] if first is not None else (0, 0)
        else:
            h, w = 0, 0
        print(f"  [{cam_name}] {n} frames, {fps:.1f} fps, {w}x{h}, {duration:.2f} s")

        if not no_video and n > 0:
            out_path = out_dir / f"{stem}_{cam_name}.mp4"
            save_video(cam_name, frames, ts, out_path)

    print()


def main():
    parser = argparse.ArgumentParser(description="Inspect an HDF5 episode file")
    parser.add_argument("file", help="Path to the HDF5 file")
    parser.add_argument("--no-video", action="store_true", help="Skip video reconstruction")
    args = parser.parse_args()

    path = Path(args.file)
    if not path.exists():
        print(f"File not found: {path}", file=sys.stderr)
        sys.exit(1)

    with h5py.File(path, "r") as f:
        print(f"\nFile: {path.name}  ({path.stat().st_size / 1024 / 1024:.1f} MB)")
        print(f"Task: {f.attrs.get('task', 'unknown')}")

        if "arm" in f:
            print_arm_info(f)
        else:
            print("No arm data")

        print_camera_info(f, args.no_video, path.parent, path.stem)


if __name__ == "__main__":
    main()

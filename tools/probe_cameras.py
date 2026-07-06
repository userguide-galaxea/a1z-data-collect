"""Camera device-index probe script.

Run directly (not via pytest):
    python -m a1z_lerobot.tools.probe_cameras

Tries to open and grab a frame from /dev/video0 ~ /dev/video31 one by one,
saving successful ones to tools/tmp/camera_<N>.jpg and printing the matching
/dev/v4l/by-id stable path, so you can eyeball the views and fill them into the
rollout config.
"""

import glob
import os
import sys

import cv2


def stable_paths_for(dev_path: str) -> list[str]:
    """Return the /dev/v4l/by-id/ stable symlinks pointing at dev_path (possibly several).

    by-id paths are bound to the physical USB port and stay constant across
    reboot/replug, making them suitable for the rollout config's index_or_path
    instead of the drifting /dev/videoN.
    """
    real = os.path.realpath(dev_path)
    found = []
    for link in sorted(glob.glob("/dev/v4l/by-id/*")):
        if os.path.realpath(link) == real:
            found.append(link)
    return found

OUT_DIR = os.path.join(os.path.dirname(__file__), "tmp")
MAX_DEVICE = 32          # scan /dev/video0 ~ /dev/video31
FLUSH_FRAMES = 5         # drop the first few frames to avoid stale buffered images
IMG_WIDTH = 640
IMG_HEIGHT = 480


def probe_cameras(max_device: int = MAX_DEVICE) -> dict[int, str]:
    """Scan all device indices, grab a frame and save an image.

    Returns:
        {device_idx: saved_path} — devices that were captured and their image paths.
    """
    os.makedirs(OUT_DIR, exist_ok=True)
    results: dict[int, str] = {}

    for idx in range(max_device):
        dev_path = f"/dev/video{idx}"
        if not os.path.exists(dev_path):
            continue

        cap = cv2.VideoCapture(dev_path, cv2.CAP_V4L2)
        if not cap.isOpened():
            print(f"  [{idx:2d}] open failed, skipping")
            cap.release()
            continue

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # flush
        for _ in range(FLUSH_FRAMES):
            cap.read()

        ret, frame = cap.read()
        cap.release()

        if not ret or frame is None:
            print(f"  [{idx:2d}] frame read failed, skipping")
            continue

        out_path = os.path.join(OUT_DIR, f"camera_{idx:02d}.jpg")
        cv2.imwrite(out_path, frame)
        print(f"  [{idx:2d}] OK  ->  {out_path}  ({frame.shape[1]}x{frame.shape[0]})")
        for link in stable_paths_for(dev_path):
            print(f"        by-id: {link}")
        results[idx] = out_path

    return results


if __name__ == "__main__":
    print(f"scanning /dev/video0 ~ /dev/video{MAX_DEVICE - 1} ...\n")
    found = probe_cameras()
    print(f"\nfound {len(found)} usable camera(s): {list(found.keys())}")
    if not found:
        print("no usable camera found, please check the device connections.")
        sys.exit(1)
    print(f"\nimages saved to {os.path.abspath(OUT_DIR)}/")
    print("after confirming each view against the images, fill the by-id paths printed above into")
    print("the index_or_path of a1z_lerobot/configs/rollout_a1z.yaml (prefer by-id to avoid device-index drift).")

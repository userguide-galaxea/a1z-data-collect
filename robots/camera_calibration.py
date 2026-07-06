"""Interactive camera calibration.

Displays all detected cameras simultaneously and lets the user assign
each one a name by pressing a key.

Usage:
    mapping = calibrate_cameras(["cam_high", "cam_right_wrist", "cam_left_wrist"])
    # returns {"cam_high": 0, "cam_right_wrist": 2, "cam_left_wrist": 1}
"""

import os
import time
import cv2
import numpy as np


def _open_camera(index: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    time.sleep(0.5)
    for _ in range(3):
        cap.read()
    return cap


def _detect_cameras() -> list[int]:
    """Return one device index per physical USB camera, skipping built-in and metadata nodes."""
    available: list[int] = []
    for i in range(20):
        if not os.path.exists(f"/dev/video{i}"):
            continue
        # index=0 is the primary capture node; index=1 is the metadata/raw node
        index_path = f"/sys/class/video4linux/video{i}/index"
        if os.path.exists(index_path):
            with open(index_path) as f:
                if f.read().strip() != "0":
                    continue
        name_path = f"/sys/class/video4linux/video{i}/name"
        name = ""
        if os.path.exists(name_path):
            with open(name_path) as f:
                name = f.read().strip()
        # Skip built-in / internal cameras
        if any(k in name.lower() for k in ("integrated", "internal", "ir camera", "depth", "webcam")):
            continue
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            available.append(i)
        cap.release()
    return available


def calibrate_cameras(cam_names: list[str]) -> dict[str, int]:
    """Interactively assign camera names to device indices.

    Shows a live window with all detected cameras. For each name,
    prompts the user to click the corresponding camera preview.

    Args:
        cam_names: Ordered list of camera names to assign,
                   e.g. ["cam_high", "cam_right_wrist", "cam_left_wrist"]

    Returns:
        {cam_name: device_index}
    """
    devices = _detect_cameras()
    if len(devices) < len(cam_names):
        raise RuntimeError(
            f"Found {len(devices)} cameras but need {len(cam_names)}: {cam_names}"
        )

    caps = {d: _open_camera(d) for d in devices}

    mapping: dict[str, int] = {}
    assigned: set[int] = set()

    THUMB_W, THUMB_H = 320, 240
    BLANK = np.zeros((THUMB_H, THUMB_W, 3), dtype=np.uint8)

    def make_grid(highlight: int | None = None) -> np.ndarray:
        thumbs = []
        for d in devices:
            ret, frame = caps[d].read()
            frame = cv2.resize(frame, (THUMB_W, THUMB_H)) if ret else BLANK.copy()

            label = f"[{devices.index(d)}] dev={d}"
            if d in assigned:
                name = [k for k, v in mapping.items() if v == d][0]
                label += f" -> {name}"
                cv2.rectangle(frame, (0, 0), (THUMB_W - 1, THUMB_H - 1), (0, 200, 0), 3)
            elif highlight == d:
                cv2.rectangle(frame, (0, 0), (THUMB_W - 1, THUMB_H - 1), (0, 120, 255), 3)

            cv2.putText(frame, label, (6, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            thumbs.append(frame)

        return np.hstack(thumbs)

    window = "Camera Calibration"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    clicked_device: int | None = None

    def on_mouse(event, x, y, flags, param):
        nonlocal clicked_device
        if event == cv2.EVENT_LBUTTONDOWN:
            col = x // THUMB_W
            if col < len(devices):
                clicked_device = devices[col]

    cv2.setMouseCallback(window, on_mouse)

    try:
        for cam_name in cam_names:
            clicked_device = None
            print(f"\nClick the preview for: {cam_name}  (press Q to quit)")

            while True:
                grid = make_grid(highlight=clicked_device)

                remaining = [n for n in cam_names if n not in mapping]
                status = f"Assign: {cam_name}  |  remaining: {remaining}"
                cv2.putText(grid, status, (6, THUMB_H - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 1)

                cv2.imshow(window, grid)
                key = cv2.waitKey(30) & 0xFF

                if key == ord('q'):
                    raise RuntimeError("Calibration cancelled by user.")

                if key in (ord('\r'), ord('\n'), 32):
                    if clicked_device is not None and clicked_device not in assigned:
                        mapping[cam_name] = clicked_device
                        assigned.add(clicked_device)
                        print(f"  {cam_name} -> device {clicked_device}")
                        break
                    else:
                        print("  Please click an unassigned camera first.")
    finally:
        cv2.destroyAllWindows()
        for cap in caps.values():
            cap.release()

    return mapping

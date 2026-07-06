import time
import cv2
import numpy as np
from utils.interfaces import CameraInterface


class OpenCVCamera(CameraInterface):
    """USB camera using cv2.VideoCapture in MJPEG mode."""

    def __init__(self, device: int | str, width: int = 1280, height: int = 720, fps: int = 30):
        self._device = device
        self._width = width
        self._height = height
        self._fps = fps
        self._cap: cv2.VideoCapture | None = None
        self._is_mjpg: bool = False

    def open(self) -> None:
        self._cap = cv2.VideoCapture(self._device, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera: {self._device}")
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self._cap.set(cv2.CAP_PROP_FPS, self._fps)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)
        fourcc = int(self._cap.get(cv2.CAP_PROP_FOURCC))
        self._is_mjpg = (fourcc == cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        if self._is_mjpg:
            self._cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        time.sleep(2.0)
        for _ in range(10):
            self._cap.read()

    def read(self) -> np.ndarray | None:
        """Return decoded RGB frame (used by camera calibration)."""
        ret, buf = self._cap.read()
        if not ret or buf is None:
            return None
        if self._is_mjpg:
            frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        else:
            frame = buf
        if frame is None:
            return None
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def read_jpeg(self) -> bytes | None:
        """Return JPEG bytes (used by CameraCollector)."""
        ret, buf = self._cap.read()
        if not ret or buf is None:
            return None
        if self._is_mjpg:
            return buf.tobytes()
        _, encoded = cv2.imencode('.jpg', buf)
        return encoded.tobytes()

    def flush(self, n: int = 5) -> None:
        """Drop up to n queued frames so the next read is fresh (not a stale buffered frame)."""
        if self._cap is None:
            return
        for _ in range(n):
            self._cap.grab()

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

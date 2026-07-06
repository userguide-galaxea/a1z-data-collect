"""Camera collector — records frames at a fixed frequency, streaming to disk."""

import time
import threading
from utils.clock import now


class CameraCollector:
    """Collect camera frames from one or more cameras at a fixed frequency.

    Uses read_jpeg() to get MJPEG bytes directly from hardware (~300KB vs 6MB
    for raw RGB), minimizing IPC data volume to the writer process.
    """

    def __init__(self, camera_readers: dict, freq: int = 15, dataset=None):
        self._readers = camera_readers
        self._period = 1.0 / freq
        self._dataset = dataset

        self._stop_event = threading.Event()
        self._threads: list = []

    def start(self) -> None:
        self._stop_event.clear()
        self._threads = [
            threading.Thread(target=self._loop, args=(name,), daemon=True)
            for name in self._readers
        ]
        for t in self._threads:
            t.start()

    def stop(self) -> None:
        self._stop_event.set()
        for t in self._threads:
            t.join()

    def _loop(self, cam_name: str) -> None:
        reader = self._readers[cam_name]
        reader.flush()  # drop stale frames buffered while idle (before [S]) so frame 0 is fresh
        while not self._stop_event.is_set():
            t0 = time.monotonic()
            jpeg = reader.read_jpeg()
            if jpeg is not None:
                ts = now()
                if self._dataset is not None:
                    self._dataset.append_camera(cam_name, jpeg, ts)
            elapsed = time.monotonic() - t0
            time.sleep(max(0, self._period - elapsed))

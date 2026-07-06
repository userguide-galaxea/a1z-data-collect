"""Save collected episode data to HDF5 format — streaming write mode.

Arm data is written via multiprocessing.Queue (small per-step payloads).
Camera frames are JPEG-encoded by the camera hardware (MJPG mode), so the
collector only reads bytes (~300KB/frame) and puts them directly into a
Queue.  The writer process stores the bytes as vlen uint8 in HDF5.
No software re-encoding, no shared memory complexity.

File layout (final):
    episode_N.hdf5
    ├── attrs["task"]
    ├── arm/
    │   ├── state                float32 [T, D]
    │   ├── action               float32 [T, D]
    │   ├── action_raw           float32 [T, D]
    │   ├── leader_timestamps    float64 [T]
    │   ├── follower_timestamps  float64 [T]
    │   ├── velocity             float32 [T, D]   (optional, controller output)
    │   └── velocity_raw         float32 [T, D]   (optional, raw leader velocity)
    └── cameras/
        └── <cam_name>/
            ├── frames      vlen uint8 (JPEG bytes per frame)
            └── timestamps  float64 [T]
"""

import os
import time
import multiprocessing as mp
import numpy as np
import h5py
from utils.cfg import DataCollectionCfg

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
_ARM_QUEUE_MAXSIZE = 2000   # arm 30 Hz
_CAM_QUEUE_MAXSIZE = 60     # camera 30 Hz
_ARM_CHUNK_ROWS    = 200
_CAM_CHUNK_ROWS    = 15


def _silent_remove(path: str) -> None:
    try:
        os.remove(path)
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f"[H5Writer] warning: could not remove {path}: {e}")


def _merge_worker(final_path: str, task: str, arm_path: str, cam_paths: dict) -> None:
    """Run in a subprocess so the merge never holds the GIL against the arm control thread."""
    import h5py
    with h5py.File(final_path, "w") as dst:
        dst.attrs["task"] = task
        with h5py.File(arm_path, "r") as src:
            src.copy("arm", dst)
        dst.create_group("cameras")
        for name, p in cam_paths.items():
            with h5py.File(p, "r") as src:
                src.copy(f"cameras/{name}", dst["cameras"])
    _silent_remove(arm_path)
    for p in cam_paths.values():
        _silent_remove(p)


# ---------------------------------------------------------------------------
# Arm writer worker
# ---------------------------------------------------------------------------

def _arm_writer_worker(path: str, task: str, use_velocity: bool, q: mp.Queue) -> None:
    import h5py
    import numpy as np

    f = h5py.File(path, "w")
    f.attrs["task"] = task
    f.create_group("arm")
    ds = {}
    rows = 0

    try:
        while True:
            item = q.get()
            if item is None:
                break

            for key, val_key in [("state", "state"), ("action", "action"),
                                  ("action_raw", "action_raw")]:
                arr = np.asarray(item[val_key], dtype=np.float32)
                D = arr.shape[0]
                if ds.get(key) is None:
                    ds[key] = f["arm"].create_dataset(
                        key, shape=(0, D), maxshape=(None, D),
                        dtype=np.float32, chunks=(_ARM_CHUNK_ROWS, D))
                ds[key].resize(rows + 1, axis=0)
                ds[key][rows] = arr

            for key, val_key in [("leader_timestamps", "leader_timestamp"),
                                  ("follower_timestamps", "follower_timestamp")]:
                val = float(item[val_key])
                if ds.get(key) is None:
                    ds[key] = f["arm"].create_dataset(
                        key, shape=(0,), maxshape=(None,),
                        dtype=np.float64, chunks=(_ARM_CHUNK_ROWS,))
                ds[key].resize(rows + 1, axis=0)
                ds[key][rows] = val

            if use_velocity:
                for key, val_key in [("velocity", "velocity"), ("velocity_raw", "velocity_raw")]:
                    if val_key not in item:
                        continue
                    arr = np.asarray(item[val_key], dtype=np.float32)
                    D = arr.shape[0]
                    if ds.get(key) is None:
                        ds[key] = f["arm"].create_dataset(
                            key, shape=(0, D), maxshape=(None, D),
                            dtype=np.float32, chunks=(_ARM_CHUNK_ROWS, D))
                    ds[key].resize(rows + 1, axis=0)
                    ds[key][rows] = arr

            rows += 1
    finally:
        f.flush()
        f.close()


# ---------------------------------------------------------------------------
# Camera writer worker — receives JPEG bytes, stores as vlen uint8
# ---------------------------------------------------------------------------

def _cam_writer_worker(path: str, cam_name: str, q: mp.Queue) -> None:
    import h5py
    import numpy as np

    f = h5py.File(path, "w")
    f.create_group(f"cameras/{cam_name}")
    grp = f[f"cameras/{cam_name}"]
    ds_frames = None
    ds_ts = None
    rows = 0

    try:
        while True:
            item = q.get()
            if item is None:
                break

            jpeg_bytes = np.frombuffer(item["frame"], dtype=np.uint8)

            if ds_frames is None:
                vlen = h5py.vlen_dtype(np.uint8)
                ds_frames = grp.create_dataset(
                    "frames", shape=(0,), maxshape=(None,),
                    dtype=vlen, chunks=(_CAM_CHUNK_ROWS,))
                ds_ts = grp.create_dataset(
                    "timestamps", shape=(0,), maxshape=(None,),
                    dtype=np.float64, chunks=(_CAM_CHUNK_ROWS,))

            ds_frames.resize(rows + 1, axis=0)
            ds_frames[rows] = jpeg_bytes
            ds_ts.resize(rows + 1, axis=0)
            ds_ts[rows] = item["ts"]
            rows += 1
    finally:
        f.flush()
        f.close()


# ---------------------------------------------------------------------------
# H5Writer
# ---------------------------------------------------------------------------

_spawn = mp.get_context("spawn")


class H5Writer:
    """Streams one episode to HDF5 incrementally — each step/frame is written
    as it arrives into resizable, chunked datasets, so memory stays flat
    regardless of episode length instead of buffering everything in RAM (camera
    JPEGs would otherwise blow up memory on long episodes). Each stream (arm +
    one per camera) runs in its own subprocess so disk I/O never blocks the
    realtime arm thread.
    """

    def __init__(self, path: str, task_description: str,
                 camera_names: list, use_velocity: bool = False):
        self._path = path
        self._task = task_description
        self._cam_names = camera_names
        self._use_velocity = use_velocity

        self._arm_q: _spawn.Queue = None
        self._arm_proc: _spawn.Process = None
        self._cam_qs: dict = {}
        self._cam_procs: dict = {}

    def _arm_path(self) -> str:
        return self._path + ".arm"

    def _cam_path(self, name: str) -> str:
        return self._path + f".cam_{name}"

    def start(self) -> None:
        self._arm_q = _spawn.Queue(maxsize=_ARM_QUEUE_MAXSIZE)
        self._arm_proc = _spawn.Process(
            target=_arm_writer_worker,
            args=(self._arm_path(), self._task, self._use_velocity, self._arm_q),
            daemon=True,
        )
        self._arm_proc.start()

        for name in self._cam_names:
            q = _spawn.Queue(maxsize=_CAM_QUEUE_MAXSIZE)
            proc = _spawn.Process(
                target=_cam_writer_worker,
                args=(self._cam_path(name), name, q),
                daemon=True,
            )
            proc.start()
            self._cam_qs[name] = q
            self._cam_procs[name] = proc

    def put_arm(self, step: dict) -> None:
        if self._arm_q is not None:
            try:
                self._arm_q.put_nowait(step)
            except Exception:
                print("[H5Writer] arm queue full — step dropped (disk too slow?)")

    def put_camera(self, cam_name: str, jpeg_bytes: bytes, timestamp: float) -> None:
        q = self._cam_qs.get(cam_name)
        if q is not None:
            try:
                q.put_nowait({"frame": jpeg_bytes, "ts": timestamp})
            except Exception:
                print(f"[H5Writer] camera '{cam_name}' queue full — frame dropped")

    def finish(self, discard: bool = False):
        if self._arm_q is not None:
            self._arm_q.put(None)
        for q in self._cam_qs.values():
            q.put(None)

        if self._arm_proc is not None:
            self._arm_proc.join()
        for proc in self._cam_procs.values():
            proc.join()

        arm_path = self._arm_path()
        cam_paths = {n: self._cam_path(n) for n in self._cam_names}

        if discard:
            _silent_remove(arm_path)
            for p in cam_paths.values():
                _silent_remove(p)
            return None

        final_path = self._path[:-4] if self._path.endswith(".tmp") else self._path
        # Run merge in a subprocess to avoid holding the GIL against the arm control thread.
        proc = mp.get_context("spawn").Process(
            target=_merge_worker,
            args=(final_path, self._task, arm_path, cam_paths),
            daemon=False,
        )
        proc.start()
        proc.join()
        if proc.exitcode != 0:
            raise RuntimeError(f"[H5Writer] merge subprocess failed (exit {proc.exitcode})")

        return final_path


# ---------------------------------------------------------------------------
# H5Dataset
# ---------------------------------------------------------------------------

class H5Dataset:
    """Facade over a dataset directory: owns episode numbering and opens one
    H5Writer per episode. The collection loop talks only to this class.
    """

    def __init__(self, cfg: DataCollectionCfg) -> None:
        self._cfg = cfg
        dataset_dir = os.path.abspath(cfg.hdf5_cfg.dataset_dir)
        self._save_dir = os.path.join(dataset_dir, cfg.hdf5_cfg.task_name)
        os.makedirs(self._save_dir, exist_ok=True)

        start = cfg.hdf5_cfg.start_episode
        self._episode_idx = self._next_index(0 if start is None else start)
        print(f"[H5Dataset] starting at episode {self._episode_idx}")

        self._writer: H5Writer = None

    def open_episode(self, camera_names: list, use_velocity: bool = False) -> None:
        tmp_path = os.path.join(
            self._save_dir, f"episode_{self._episode_idx}.hdf5.tmp"
        )
        _silent_remove(tmp_path)
        _silent_remove(tmp_path + ".arm")
        for name in camera_names:
            _silent_remove(tmp_path + f".cam_{name}")

        self._writer = H5Writer(
            path=tmp_path,
            task_description=self._cfg.task_description,
            camera_names=camera_names,
            use_velocity=use_velocity,
        )
        self._writer.start()

    def append_arm(self, step: dict) -> None:
        if self._writer is not None:
            self._writer.put_arm(step)

    def append_camera(self, cam_name: str, jpeg_bytes: bytes, timestamp: float) -> None:
        if self._writer is not None:
            self._writer.put_camera(cam_name, jpeg_bytes, timestamp)

    def close_episode(self, discard: bool = False) -> None:
        if self._writer is None:
            return
        t0 = time.time()
        try:
            final = self._writer.finish(discard=discard)
        finally:
            self._writer = None

        if not discard:
            self._episode_idx += 1
            print(f"[H5Dataset] saved {final} ({time.time() - t0:.2f}s flush)")
        else:
            print(f"[H5Dataset] episode {self._episode_idx} discarded")

    def _next_index(self, start: int) -> int:
        for i in range(start, start + 10000):
            if not os.path.exists(os.path.join(self._save_dir, f"episode_{i}.hdf5")):
                return i
        raise RuntimeError("Could not find a free episode index")

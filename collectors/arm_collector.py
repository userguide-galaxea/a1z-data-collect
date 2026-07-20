"""Arm state collector running at high frequency (default 30 Hz).

Runs the full teleoperation + recording loop in one thread:
  read leader → compute command → send to follower → read follower → record
"""

import os
import time
import threading
from typing import Optional
from utils.interfaces import LeaderArmInterface, FollowerArmInterface, TeleopControllerInterface
from utils.clock import now


def _precise_sleep(period: float, t0: float) -> None:
    slack = period - 0.0005
    elapsed = time.monotonic() - t0
    if slack > elapsed:
        time.sleep(slack - elapsed)
    while time.monotonic() - t0 < period:
        pass


class ArmCollector:
    """Teleoperation + recording at a fixed frequency.

    Pass dataset= to enable streaming writes via multiprocess queue.
    """

    def __init__(
        self,
        leader: LeaderArmInterface,
        follower: FollowerArmInterface,
        controller: Optional[TeleopControllerInterface],
        freq: int = 200,
        cpu_affinity: Optional[int] = None,
        realtime_priority: bool = False,
        use_velocity: bool = False,
        dataset=None,
    ):
        self._leader = leader
        self._follower = follower
        self._controller = controller
        self._period = 1.0 / freq
        self._cpu_affinity = cpu_affinity
        self._realtime_priority = realtime_priority
        self._use_velocity = use_velocity
        self._dataset = dataset

        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join()

    def _loop(self) -> None:
        self._apply_os_settings()

        # [DBG] 心跳: 每秒打印一次循环存活 + read_as_vector 耗时, 用于定位"卡住"在哪个阶段
        _hb_frames = 0
        _hb_last = time.monotonic()
        _hb_max_read_ms = 0.0
        _hb_max_iter_ms = 0.0

        while not self._stop_event.is_set():
            t0 = time.monotonic()

            action, vel = self._leader.read_as_vector()
            t_after_read = time.monotonic()
            read_ms = (t_after_read - t0) * 1000.0
            if read_ms > _hb_max_read_ms: _hb_max_read_ms = read_ms

            ts_leader = now()

            if self._controller is not None:
                follower_pos = self._follower.get_joint_pos()
                cmd, vel_ff = self._controller.compute_command(action, follower_pos, vel)
                self._follower.command(cmd, vel_ff if self._use_velocity else None)
            else:
                cmd, vel_ff = action, vel
                # VR 遥操作等模式: controller 为 None, 但仍需将指令发送给从臂。
                self._follower.command(cmd, vel_ff if self._use_velocity else None)

            state = self._follower.get_joint_pos()
            ts_follower = now()

            if self._dataset is not None:
                step = {
                    "state":              state,
                    "action":             cmd,
                    "action_raw":         action,
                    "leader_timestamp":   ts_leader,
                    "follower_timestamp": ts_follower,
                }
                if self._use_velocity:
                    step["velocity"] = vel_ff
                    step["velocity_raw"] = vel
                self._dataset.append_arm(step)

            _precise_sleep(self._period, t0)

            iter_ms = (time.monotonic() - t0) * 1000.0
            if iter_ms > _hb_max_iter_ms: _hb_max_iter_ms = iter_ms
            _hb_frames += 1
            if time.monotonic() - _hb_last >= 1.0:
                print(f"[HB] arm_collector {_hb_frames}it/s "
                      f"read_max={_hb_max_read_ms:.1f}ms iter_max={_hb_max_iter_ms:.1f}ms")
                _hb_frames = 0
                _hb_last = time.monotonic()
                _hb_max_read_ms = 0.0
                _hb_max_iter_ms = 0.0

    def _apply_os_settings(self) -> None:
        if self._cpu_affinity is not None:
            try:
                os.sched_setaffinity(0, {self._cpu_affinity})
            except (AttributeError, OSError) as e:
                print(f"[ArmCollector] cpu_affinity not applied: {e}")

        if self._realtime_priority:
            try:
                os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(99))
            except (AttributeError, OSError) as e:
                print(f"[ArmCollector] realtime_priority not applied: {e}")

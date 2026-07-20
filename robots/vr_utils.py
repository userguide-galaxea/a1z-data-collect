import queue
import threading
import time

import numpy as np


class VRHandData:
    def __init__(self):
        self.pos = np.zeros(3, dtype=np.float64)
        self.quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        self.trigger = 0.0
        self.grip = 0.0
        self.button_lower = False
        self.button_upper = False
        self.stick_click = False
        self.stick_x = 0.0
        self.stick_y = 0.0
        self.timestamp = 0.0


class VRDataStore:
    def __init__(self):
        self._lock = threading.Lock()
        self.left = VRHandData()
        self.right = VRHandData()
        self.head_pos = np.zeros(3, dtype=np.float64)
        self.head_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)

        self._haptic_queue = queue.Queue()

        self._prev = {
            "left": {"lower": False, "upper": False, "grip": 0.0, "stick_click": False},
            "right": {"lower": False, "upper": False, "grip": 0.0, "stick_click": False},
        }
        self._edges = {
            "left": {"lower": False, "upper": False, "grip": False, "stick_click": False},
            "right": {"lower": False, "upper": False, "grip": False, "stick_click": False},
        }

    def update_hand(self, hand, data):
        with self._lock:
            h = getattr(self, hand)
            h.pos = np.array(data["p"], dtype=np.float64)
            h.quat = np.array(data["q"], dtype=np.float64)
            h.trigger = float(data.get("trigger", 0.0))
            h.grip = float(data.get("grip", 0.0))
            h.button_lower = bool(data.get("lower", False))
            h.button_upper = bool(data.get("upper", False))
            h.stick_click = bool(data.get("stick_click", False))
            ax = data.get("ax", [0.0, 0.0])
            h.stick_x = float(ax[0]) if len(ax) > 0 else 0.0
            h.stick_y = float(ax[1]) if len(ax) > 1 else 0.0
            h.timestamp = time.monotonic()

            p = self._prev[hand]
            e = self._edges[hand]
            lower = h.button_lower
            upper = h.button_upper
            grip = h.grip
            sc = h.stick_click
            e["lower"] = lower and not p["lower"]
            e["upper"] = upper and not p["upper"]
            e["grip"] = grip > 0.5 and p["grip"] <= 0.5
            e["stick_click"] = sc and not p["stick_click"]
            p["lower"] = lower
            p["upper"] = upper
            p["grip"] = grip
            p["stick_click"] = sc

    def update_head(self, data):
        with self._lock:
            self.head_pos = np.array(data["p"], dtype=np.float64)
            self.head_quat = np.array(data["q"], dtype=np.float64)

    def is_connected(self, hand, timeout=3.0):
        """True 若该手最近 timeout 秒内有过上报（用作连接存活/掉线检测）。"""
        with self._lock:
            h = getattr(self, hand)
            if h.timestamp == 0.0:
                return False
            return (time.monotonic() - h.timestamp) < timeout

    def get_button_lower_edge(self, hand):
        with self._lock:
            edge = self._edges[hand]["lower"]
            self._edges[hand]["lower"] = False
        return edge

    def get_button_upper_edge(self, hand):
        with self._lock:
            edge = self._edges[hand]["upper"]
            self._edges[hand]["upper"] = False
        return edge

    def get_grip_edge(self, hand):
        with self._lock:
            edge = self._edges[hand]["grip"]
            self._edges[hand]["grip"] = False
        return edge

    def get_stick_click_edge(self, hand):
        with self._lock:
            edge = self._edges[hand]["stick_click"]
            self._edges[hand]["stick_click"] = False
        return edge

    def send_haptic(self, hand, count=1, amp=0.5, duration=150.0):
        self._haptic_queue.put({
            "hand": hand,
            "count": int(count),
            "amp": float(amp),
            "duration": float(duration),
        })


class VREventListener:
    def __init__(self, vr_store, events):
        self._vr_store = vr_store
        self._events = events
        self._running = False
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._poll, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._running = False

    def join(self, timeout=None):
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def _poll(self):
        _dbg_tick = 0
        while self._running:
            if self._vr_store.get_button_lower_edge("left"):
                self._events["start_recording"] = True
                self._vr_store.send_haptic("left", count=2, amp=0.5)
                print("[DBG][VREvt] left-lower(X) edge → start_recording")
            if self._vr_store.get_button_upper_edge("left"):
                self._events["finish_recording"] = True
                self._vr_store.send_haptic("left", count=1, amp=0.5)
                print("[DBG][VREvt] left-upper(Y) edge → finish_recording")
            if self._vr_store.get_stick_click_edge("left"):
                self._vr_store.send_haptic("left", count=1, amp=0.5)
                if self._vr_store.left.stick_x < -0.7:
                    self._events["rerecord"] = True
                    self._events["finish_recording"] = True
                    print("[DBG][VREvt] left-stick(left) → rerecord")
                else:
                    self._events["stop"] = True
                    self._events["finish_recording"] = True
                    print("[DBG][VREvt] left-stick(right) → stop")
            # [DBG] 每 ~2 秒打印一次心跳, 确认监听线程在跑 + 数据在更新
            _dbg_tick += 1
            if _dbg_tick >= 100:
                _dbg_tick = 0
                ls = self._vr_store.left
                rs = self._vr_store.right
                print(f"[DBG][VREvt] alive "
                      f"L lo={int(ls.button_lower)} up={int(ls.button_upper)} ts={ls.timestamp:.2f} "
                      f"R lo={int(rs.button_lower)} up={int(rs.button_upper)} ts={rs.timestamp:.2f} "
                      f"ev={ {k: int(v) for k, v in self._events.items()} }")
            time.sleep(0.02)


def init_vr_event_listener(vr_store, events=None):
    # 若传入 events（通常是键盘监听器已经建好的那组），则复用，使键盘与 VR 按键共享同一组标志。
    if events is None:
        events = {
            "start_recording": False,
            "finish_recording": False,
            "rerecord": False,
            "stop": False,
        }
    listener = VREventListener(vr_store, events)
    listener.start()
    return listener, events

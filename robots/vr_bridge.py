import asyncio
import json
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

import numpy as np
import websockets

_FRONTEND_DIR = Path(__file__).resolve().parent.parent / "webxr" / "frontend"

C = np.array([[0, 0, -1],
              [-1, 0, 0],
              [0, 1, 0]], dtype=np.float64)


def _quat_xyzw_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def _R_to_quat_xyzw(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0])


def convert_pose(p_webxr, q_webxr_xyzw):
    p = C @ np.asarray(p_webxr, dtype=np.float64)
    R = C @ _quat_xyzw_to_R(np.asarray(q_webxr_xyzw, dtype=np.float64)) @ C.T
    q = _R_to_quat_xyzw(R)
    return p, q


class VRBridge:
    def __init__(self, vr_store, ws_port=8001, http_port=8000, ws_bind="0.0.0.0"):
        self._vr_store = vr_store
        self._ws_port = ws_port
        self._http_port = http_port
        self._ws_bind = ws_bind
        self._loop = None
        self._thread = None
        self._httpd = None
        self._stop_event = threading.Event()

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._httpd is not None:
            self._httpd.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        class _QuietHandler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=str(_FRONTEND_DIR), **kwargs)

            def log_message(self, format, *args):
                pass

        self._httpd = HTTPServer(("0.0.0.0", self._http_port), _QuietHandler)
        http_thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        http_thread.start()
        print(f"[VRBridge] HTTP http://0.0.0.0:{self._http_port}")

        try:
            self._loop.run_until_complete(self._ws_serve())
        except Exception:
            pass
        finally:
            self._loop.close()

    async def _ws_serve(self):
        print(f"[VRBridge] WebSocket ws://{self._ws_bind}:{self._ws_port}")
        async with websockets.serve(
            self._ws_handler, self._ws_bind, self._ws_port, max_queue=8
        ):
            while not self._stop_event.is_set():
                await asyncio.sleep(0.1)

    async def _ws_handler(self, ws):
        peer = getattr(ws, "remote_address", "?")
        print(f"[VRBridge] Quest connected: {peer}")
        try:
            async for raw in ws:
                try:
                    data = json.loads(raw)
                    self._handle_frame(data)
                except Exception:
                    pass
        except Exception:
            pass
        finally:
            print(f"[VRBridge] Quest disconnected: {peer}")

    def _handle_frame(self, data):
        head = data.get("head")
        if head:
            p, q = convert_pose(head["p"], head["q"])
            self._vr_store.update_head({"p": p.tolist(), "q": q.tolist()})

        for hand in ("left", "right"):
            rec = data.get(hand)
            if not rec:
                continue
            p, q = convert_pose(rec["p"], rec["q"])
            self._vr_store.update_hand(hand, {
                "p": p.tolist(),
                "q": q.tolist(),
                "trigger": rec.get("trigger", 0.0),
                "grip": rec.get("grip", 0.0),
                "lower": rec.get("lower", False),
                "upper": rec.get("upper", False),
                "stick_click": rec.get("stick_click", False),
                "ax": rec.get("ax", [0.0, 0.0]),
            })

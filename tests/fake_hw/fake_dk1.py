"""Fake TRLC-DK1 arms and OpenCV cameras for exercising LeLab without hardware.

`install(log_path)` patches, in-process:

- the DK1 follower's motor stack (`trlc_dk1_control.DK1Robot`): tracks the
  commanded joint/gripper targets instantly;
- the DK1 leader (`DK1Leader`): reports a deterministic per-port trajectory,
  so left and right arms (and every joint) carry distinguishable signals;
- `lerobot.cameras.opencv.OpenCVCamera`: a capture thread per camera renders
  frames carrying a block code of the camera index and capture time (see
  `encode_frame` / `decode_frame`) plus a human-readable label.

Every bimanual leader action, follower observation (with the code of each
camera frame it returned) and follower command is appended to `log_path` as
JSON lines, which `check_dataset.py` compares against the recorded dataset.
"""

from __future__ import annotations

import json
import math
import threading
import time
from types import SimpleNamespace

import numpy as np

# Frame code layout: GRID_COLS x GRID_ROWS black/white blocks in the top part
# of the frame. Bits: 4-bit camera index, then 28-bit milliseconds since T0.
GRID_COLS, GRID_ROWS = 8, 4
CODE_AREA = 0.6  # fraction of the frame height used by the blocks
CAM_BITS, TIME_BITS = 4, 28
T0 = time.time()

_log_lock = threading.Lock()
_log_file = None


def _log(kind: str, values: dict) -> None:
    if _log_file is None:
        return
    line = json.dumps({"t": time.time(), "kind": kind, "v": values})
    with _log_lock:
        _log_file.write(line + "\n")


# ---------------------------------------------------------------- frame code


def _bits(cam_index: int, t: float) -> list[int]:
    ms = int((t - T0) * 1000) & ((1 << TIME_BITS) - 1)
    code = ((cam_index & ((1 << CAM_BITS) - 1)) << TIME_BITS) | ms
    return [(code >> (31 - i)) & 1 for i in range(32)]


def _cells(height: int, width: int):
    cell_h = int(height * CODE_AREA) // GRID_ROWS
    cell_w = width // GRID_COLS
    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            yield r * GRID_COLS + c, r * cell_h, c * cell_w, cell_h, cell_w


def _background(cam_index: int, height: int, width: int, label: str) -> np.ndarray:
    import cv2

    # A distinct background hue per camera, so feeds are easy to tell apart.
    hue = (cam_index * 47) % 180
    bg = cv2.cvtColor(np.uint8([[[hue, 160, 200]]]), cv2.COLOR_HSV2RGB)[0, 0]
    frame = np.empty((height, width, 3), np.uint8)
    frame[:] = bg
    text_y = int(height * CODE_AREA) + 40
    cv2.putText(frame, label, (12, text_y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 2)
    return frame


def encode_frame(
    cam_index: int, t: float, height: int, width: int, label: str, background: np.ndarray | None = None
) -> np.ndarray:
    import cv2

    if background is None:
        background = _background(cam_index, height, width, label)
    frame = background.copy()
    bits = _bits(cam_index, t)
    for i, y, x, ch, cw in _cells(height, width):
        pad_y, pad_x = ch // 6, cw // 6
        frame[y + pad_y : y + ch - pad_y, x + pad_x : x + cw - pad_x] = 255 if bits[i] else 0
    text_y = int(height * CODE_AREA) + 80
    cv2.putText(frame, f"t={t - T0:9.3f}s", (12, text_y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 2)
    return frame


def decode_frame(frame: np.ndarray) -> tuple[int, float]:
    """(camera index, capture time in seconds since T0) of an `encode_frame` frame."""
    height, width = frame.shape[:2]
    code = 0
    for i, y, x, ch, cw in _cells(height, width):
        centre = frame[y + ch // 3 : y + 2 * ch // 3, x + cw // 3 : x + 2 * cw // 3]
        code |= int(centre.mean() > 127) << (31 - i)
    return code >> TIME_BITS, (code & ((1 << TIME_BITS) - 1)) / 1000.0


# ------------------------------------------------------------------- cameras


def _patch_cameras() -> None:
    from lerobot.cameras.opencv import OpenCVCamera

    def connect(self, warmup: bool = True) -> None:
        cfg = self.config
        self._fake_h = cfg.height or 480
        self._fake_w = cfg.width or 640
        self._fake_fps = cfg.fps or 30
        self._fake_idx = int(cfg.index_or_path)
        self._fake_frame = None
        self._fake_seq = 0
        self._fake_cond = threading.Condition()
        self._fake_running = True
        label = f"fake cam {self._fake_idx}"
        background = _background(self._fake_idx, self._fake_h, self._fake_w, label)

        def run():
            period = 1.0 / self._fake_fps
            next_t = time.perf_counter()
            last_frame_t = None
            while self._fake_running:
                now = time.perf_counter()
                # A capture thread starved for this long would make async_read time out.
                if last_frame_t is not None and now - last_frame_t > 0.05:
                    _log("cam_gap", {"cam_index": self._fake_idx, "gap_ms": (now - last_frame_t) * 1000})
                last_frame_t = now
                frame = encode_frame(
                    self._fake_idx, time.time(), self._fake_h, self._fake_w, label, background
                )
                with self._fake_cond:
                    self._fake_frame = frame
                    self._fake_seq += 1
                    self._fake_cond.notify_all()
                next_t += period
                time.sleep(max(0.0, next_t - time.perf_counter()))

        self._fake_thread = threading.Thread(target=run, daemon=True, name=f"fake-cam-{self._fake_idx}")
        self._fake_thread.start()
        self.async_read(timeout_ms=1000)

    def is_connected(self) -> bool:
        return getattr(self, "_fake_running", False)

    def async_read(self, timeout_ms: float = 200):
        # Like the real camera: wait for a frame newer than the last one read.
        last = getattr(self, "_fake_last_read", 0)
        with self._fake_cond:
            if not self._fake_cond.wait_for(lambda: self._fake_seq > last, timeout_ms / 1000):
                raise TimeoutError(f"fake camera {self._fake_idx}: no new frame in {timeout_ms} ms")
            self._fake_last_read = self._fake_seq
            return self._fake_frame.copy()

    def read_latest(self, max_age_ms: int = 500):
        with self._fake_cond:
            if self._fake_frame is None:
                raise RuntimeError("no frame yet")
            return self._fake_frame.copy()

    def read(self, color_mode=None):
        return read_latest(self)

    def disconnect(self) -> None:
        self._fake_running = False
        if getattr(self, "_fake_thread", None) is not None:
            self._fake_thread.join(timeout=1)

    OpenCVCamera.connect = connect
    OpenCVCamera.is_connected = property(is_connected)
    OpenCVCamera.async_read = async_read
    OpenCVCamera.read_latest = read_latest
    OpenCVCamera.read = read
    OpenCVCamera.disconnect = disconnect


# ---------------------------------------------------------------------- arms


# Per-arm joint centres (rad), inside the DK1 URDF limits even at +/- AMPLITUDE.
_CENTRES = {
    "left": (0.4, 1.0, 1.5, -0.3, 0.3, 0.5),
    "right": (-0.4, 0.8, 1.8, 0.2, -0.3, -0.5),
}
AMPLITUDE = 0.3


def leader_trajectory(port: str, t: float) -> dict[str, float]:
    """Distinct smooth signal per arm (port) and joint."""
    side = 1 if "right" in port.lower() else 0
    centres = _CENTRES["right" if side else "left"]
    out = {}
    for j in range(1, 7):
        freq = 0.1 * j + 0.05 * side
        out[f"joint_{j}.pos"] = centres[j - 1] + AMPLITUDE * math.sin(2 * math.pi * freq * t)
    out["gripper.pos"] = 0.5 + 0.5 * math.sin(2 * math.pi * 0.25 * t + side * math.pi / 2)
    return out


class FakeDK1Robot:
    """Stands in for trlc_dk1_control.DK1Robot."""

    def __init__(self, cfg):
        self.port = cfg.serial_port
        self._q = np.zeros(6)
        self._g = 0.0
        self._motor_chain = SimpleNamespace(is_running=False)

    def connect(self) -> None:
        self._motor_chain.is_running = True

    def disconnect(self) -> None:
        self._motor_chain.is_running = False

    def command_joint_pos(self, q_des: np.ndarray) -> None:
        self._q = np.asarray(q_des, dtype=float).copy()

    def command_gripper(self, normalized_pos: float) -> None:
        self._g = float(normalized_pos)

    def get_joint_state(self) -> dict:
        return {"pos": self._q.copy(), "vel": np.zeros(6), "torque": np.zeros(6)}

    def get_gripper_state(self) -> dict:
        return {"pos": self._g, "torque": 0.0}


def _patch_arms() -> None:
    import trlc_dk1_control
    from lerobot_robot_trlc_dk1 import BiDK1Follower, BiDK1Leader
    from lerobot_robot_trlc_dk1.leader import DK1Leader

    trlc_dk1_control.DK1Robot = FakeDK1Robot

    def leader_connect(self, calibrate: bool = False) -> None:
        self._fake_connected = True

    def leader_disconnect(self) -> None:
        self._fake_connected = False

    DK1Leader.connect = leader_connect
    DK1Leader.disconnect = leader_disconnect
    DK1Leader.is_connected = property(lambda self: getattr(self, "_fake_connected", False))
    DK1Leader.configure = lambda self: None
    DK1Leader.get_action = lambda self: leader_trajectory(self.config.port, time.time() - T0)

    orig_obs = BiDK1Follower.get_observation
    orig_send = BiDK1Follower.send_action
    orig_action = BiDK1Leader.get_action

    def get_observation(self):
        obs = orig_obs(self)
        logged = {}
        for key, value in obs.items():
            if isinstance(value, np.ndarray):
                cam_index, cam_t = decode_frame(value)
                logged[key] = {"cam_index": cam_index, "cam_t": cam_t}
            else:
                logged[key] = float(value)
        _log("obs", logged)
        return obs

    def send_action(self, action):
        _log("cmd", {k: float(v) for k, v in action.items()})
        return orig_send(self, action)

    def get_action(self):
        action = orig_action(self)
        _log("action", {k: float(v) for k, v in action.items()})
        return action

    BiDK1Follower.get_observation = get_observation
    BiDK1Follower.send_action = send_action
    BiDK1Leader.get_action = get_action


def install(log_path: str | None) -> None:
    global _log_file
    if log_path:
        _log_file = open(log_path, "a", buffering=1)  # noqa: SIM115 - lives for the process
        _log("start", {"t0": T0})
    _patch_cameras()
    _patch_arms()

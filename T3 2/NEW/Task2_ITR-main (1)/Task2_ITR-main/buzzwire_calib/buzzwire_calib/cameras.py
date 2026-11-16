"""Camera frame sources (threaded, auto-reconnecting). All expose the same tiny interface."""
from __future__ import annotations
import threading, time
import cv2
import numpy as np


class FrameSource:
    """Holds the most recent frame; `wait_new` blocks for a frame newer than `last_seq`."""
    name = "cam"

    def __init__(self, name: str):
        self.name = name
        self._lock = threading.Condition()
        self._frame = None
        self._seq = 0
        self._stamp = 0.0

    def _push(self, frame):
        with self._lock:
            self._frame, self._seq, self._stamp = frame, self._seq + 1, time.time()
            self._lock.notify_all()

    def latest(self):
        with self._lock:
            return (None if self._frame is None else self._frame.copy()), self._seq, self._stamp

    def age(self) -> float:
        return time.time() - self._stamp if self._stamp else float("inf")

    def wait_new(self, last_seq: int, timeout: float = 1.0):
        end = time.time() + timeout
        with self._lock:
            while self._seq <= last_seq:
                rem = end - time.time()
                if rem <= 0:
                    return None, last_seq, self._stamp
                self._lock.wait(rem)
            return self._frame.copy(), self._seq, self._stamp

    def start(self):
        pass

    def stop(self):
        pass


class OpenCVCamera(FrameSource):
    """source: int index, '/dev/videoN', or any URL/pipeline OpenCV understands."""

    def __init__(self, name, source, width=0, height=0, fps=0, fourcc="MJPG", flush_frames=0):
        super().__init__(name)
        s = str(source).strip()
        self.source = int(s) if s.lstrip("-").isdigit() else s
        self.width, self.height, self.fps, self.fourcc = int(width), int(height), int(fps), fourcc
        self._stop = threading.Event()
        self._thread = None

    def _open(self):
        cap = cv2.VideoCapture(self.source)
        if not cap.isOpened():
            return None
        if self.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        if self.width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.fps:
            cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _run(self):
        cap, fails = None, 0
        while not self._stop.is_set():
            if cap is None:
                cap = self._open()
                if cap is None:
                    time.sleep(1.0)       # camera unplugged / busy -> keep retrying forever
                    continue
                fails = 0
            ok, frame = cap.read()
            if ok and frame is not None:
                fails = 0
                self._push(frame)
            else:
                fails += 1
                if fails > 30:            # USB glitch -> reopen
                    cap.release()
                    cap = None
                time.sleep(0.01)
        if cap is not None:
            cap.release()

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)


class PushSource(FrameSource):
    """Frames are pushed from elsewhere (ROS image callback, or a fake in tests)."""

    def push(self, frame):
        self._push(frame)

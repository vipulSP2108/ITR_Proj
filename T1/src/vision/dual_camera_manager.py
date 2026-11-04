import cv2
import threading
import time
from typing import Tuple, Optional
import numpy as np

class DualCameraManager:
    """
    High-performance multi-threaded dual-camera manager for macOS.
    Captures from Camera 1 (Overhead) and Camera 2 (Bottom/Side) in parallel
    without blocking or USB frame drop.
    """
    def __init__(self, top_cam_idx: int = 0, side_cam_idx: int = 1, width: int = 640, height: int = 480):
        self.top_idx = top_cam_idx
        self.side_idx = side_cam_idx
        self.width = width
        self.height = height

        self.top_cap: Optional[cv2.VideoCapture] = None
        self.side_cap: Optional[cv2.VideoCapture] = None

        self.latest_top_frame: Optional[np.ndarray] = None
        self.latest_side_frame: Optional[np.ndarray] = None

        self.is_running = False
        self.top_thread: Optional[threading.Thread] = None
        self.side_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()

    def start(self) -> Tuple[bool, bool]:
        """Opens video capture devices and starts background worker threads."""
        top_ok = False
        side_ok = False

        self.top_cap = cv2.VideoCapture(self.top_idx)
        if self.top_cap.isOpened():
            self.top_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.top_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            top_ok = True
            print(f"[Camera] Overhead Camera (Index {self.top_idx}) CONNECTED.")

        self.side_cap = cv2.VideoCapture(self.side_idx)
        if self.side_cap.isOpened():
            self.side_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.side_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            side_ok = True
            print(f"[Camera] Side/Bottom Camera (Index {self.side_idx}) CONNECTED.")

        self.is_running = True

        if top_ok:
            self.top_thread = threading.Thread(target=self._capture_worker, args=(self.top_cap, 'top'), daemon=True)
            self.top_thread.start()

        if side_ok:
            self.side_thread = threading.Thread(target=self._capture_worker, args=(self.side_cap, 'side'), daemon=True)
            self.side_thread.start()

        return top_ok, side_ok

    def _capture_worker(self, cap: cv2.VideoCapture, cam_name: str):
        while self.is_running and cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                with self.lock:
                    if cam_name == 'top':
                        self.latest_top_frame = frame
                    else:
                        self.latest_side_frame = frame
            time.sleep(0.01)

    def get_latest_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Returns the most recent (top_frame, side_frame) thread-safely."""
        with self.lock:
            top = self.latest_top_frame.copy() if self.latest_top_frame is not None else None
            side = self.latest_side_frame.copy() if self.latest_side_frame is not None else None
        return top, side

    def stop(self):
        """Stops background capture threads and releases devices."""
        self.is_running = False
        time.sleep(0.1)
        if self.top_cap and self.top_cap.isOpened():
            self.top_cap.release()
        if self.side_cap and self.side_cap.isOpened():
            self.side_cap.release()
        print("[Camera] Dual camera capture stopped.")

if __name__ == "__main__":
    cams = DualCameraManager(top_cam_idx=0, side_cam_idx=1)
    t_ok, s_ok = cams.start()
    print(f"Cameras initialized: Top={t_ok}, Side={s_ok}")
    time.sleep(1.0)
    top_f, side_f = cams.get_latest_frames()
    print(f"Frames captured: Top={top_f is not None}, Side={side_f is not None}")
    cams.stop()


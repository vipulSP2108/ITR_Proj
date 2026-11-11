"""
Camera capture via ffmpeg for macOS when OpenCV VideoCapture lacks permission.

Falls back to OpenCV if ffmpeg is unavailable or capture fails.
The RealSense D435i is accessed by name "Intel" via AVFoundation.
"""

import os
import cv2
import time
import subprocess
import tempfile
import numpy as np
from typing import Optional, Tuple


class CameraCapture:
    """
    Captures frames from the Intel RealSense D435i via ffmpeg on macOS.

    macOS grants camera permission per-app. If the parent app (e.g. Claude.app)
    doesn't have permission, OpenCV VideoCapture fails. ffmpeg from Homebrew
    inherits Terminal.app's permission and works.
    """

    def __init__(self, device_name: str = "Intel", width: int = 640, height: int = 480,
                 use_opencv_fallback: bool = True):
        self.device_name = device_name
        self.width = width
        self.height = height
        self.use_opencv_fallback = use_opencv_fallback
        self._opencv_cap = None
        self._tmp_dir = tempfile.mkdtemp(prefix="cam_")
        self._frame_path = os.path.join(self._tmp_dir, "frame.jpg")
        self._method = None  # "ffmpeg" or "opencv"
        self._detect_method()

    def _detect_method(self):
        """Figure out which capture method works."""
        # Try OpenCV first
        if self.use_opencv_fallback:
            cap = cv2.VideoCapture(1)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    self._opencv_cap = cap
                    self._opencv_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                    self._opencv_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                    self._method = "opencv"
                    return
                cap.release()

        # Try ffmpeg
        try:
            result = subprocess.run(
                ["ffmpeg", "-f", "avfoundation", "-list_devices", "true", "-i", ""],
                capture_output=True, text=True, timeout=5
            )
            if self.device_name.lower() in result.stderr.lower():
                self._method = "ffmpeg"
                return
        except (FileNotFoundError, subprocess.TimeoutExpired):
            pass

        self._method = None

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Capture a single frame. Returns (success, bgr_frame)."""
        if self._method == "opencv" and self._opencv_cap is not None:
            return self._opencv_cap.read()
        elif self._method == "ffmpeg":
            return self._ffmpeg_capture()
        return False, None

    def _ffmpeg_capture(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Capture one frame using ffmpeg."""
        try:
            result = subprocess.run(
                [
                    "ffmpeg", "-y",
                    "-f", "avfoundation",
                    "-framerate", "30",
                    "-video_size", f"{self.width}x{self.height}",
                    "-i", self.device_name,
                    "-frames:v", "1",
                    "-update", "1",
                    self._frame_path,
                ],
                capture_output=True, timeout=10,
            )
            if result.returncode == 0 and os.path.exists(self._frame_path):
                frame = cv2.imread(self._frame_path)
                if frame is not None:
                    return True, frame
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass
        return False, None

    def capture_burst(self, count: int = 5, interval: float = 0.1) -> Optional[np.ndarray]:
        """Capture multiple frames and return the last one (for auto-exposure settling)."""
        frame = None
        for _ in range(count):
            ret, f = self.read()
            if ret:
                frame = f
            time.sleep(interval)
        return frame

    def release(self):
        """Release camera resources."""
        if self._opencv_cap is not None:
            self._opencv_cap.release()
        try:
            os.remove(self._frame_path)
            os.rmdir(self._tmp_dir)
        except OSError:
            pass

    @property
    def is_available(self) -> bool:
        return self._method is not None

    def __str__(self):
        return f"CameraCapture(method={self._method}, device={self.device_name})"

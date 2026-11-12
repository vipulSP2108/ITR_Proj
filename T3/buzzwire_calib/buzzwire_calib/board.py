"""ChArUco board wrapper that works on OpenCV 4.5 (legacy aruco) and >= 4.7 (new API)."""
from __future__ import annotations
from dataclasses import dataclass
import cv2
import numpy as np
from .transforms import rvec_tvec_to_T

_NEW_API = hasattr(cv2.aruco, "CharucoDetector")


@dataclass
class BoardSpec:
    squares_x: int = 7
    squares_y: int = 5
    square_length: float = 0.040   # metres (measure your PRINTED board!)
    marker_length: float = 0.030   # metres
    dictionary: str = "DICT_5X5_100"

    @classmethod
    def from_cfg(cls, d: dict) -> "BoardSpec":
        return cls(int(d["squares_x"]), int(d["squares_y"]), float(d["square_length"]),
                   float(d["marker_length"]), str(d.get("dictionary", "DICT_5X5_100")))


class CharucoBoard:
    def __init__(self, spec: BoardSpec):
        self.spec = spec
        self.dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec.dictionary))
        if _NEW_API:
            self.board = cv2.aruco.CharucoBoard((spec.squares_x, spec.squares_y), spec.square_length,
                                                spec.marker_length, self.dict)
            self.detector = cv2.aruco.CharucoDetector(self.board)
            self._corners3d = np.asarray(self.board.getChessboardCorners(), np.float32).reshape(-1, 3)
        else:  # legacy
            self.board = cv2.aruco.CharucoBoard_create(spec.squares_x, spec.squares_y, spec.square_length,
                                                       spec.marker_length, self.dict)
            self._params = cv2.aruco.DetectorParameters_create()
            self._corners3d = np.asarray(self.board.chessboardCorners, np.float32).reshape(-1, 3)

    @property
    def n_corners(self) -> int:
        return len(self._corners3d)

    def detect(self, img):
        """-> (corners Nx2 float32, ids N int32) or (None, None)."""
        gray = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if _NEW_API:
            corners, ids, _, _ = self.detector.detectBoard(gray)
        else:
            mc, mi, _ = cv2.aruco.detectMarkers(gray, self.dict, parameters=self._params)
            if mi is None or len(mi) == 0:
                return None, None
            n, corners, ids = cv2.aruco.interpolateCornersCharuco(mc, mi, gray, self.board)
            if n == 0:
                return None, None
        if corners is None or ids is None or len(ids) == 0:
            return None, None
        return np.asarray(corners, np.float32).reshape(-1, 2), np.asarray(ids, np.int32).reshape(-1)

    def obj_points(self, ids) -> np.ndarray:
        return self._corners3d[np.asarray(ids, int).reshape(-1)]

    def estimate_pose(self, corners, ids, K, dist, min_corners: int = 8, max_rms_px: float = 1.5):
        """-> (T_cam_board or None, reprojection RMS px). Board frame: origin at board corner, z out of board."""
        if corners is None or len(ids) < min_corners:
            return None, float("inf")
        obj = self.obj_points(ids).astype(np.float64)
        img = np.asarray(corners, np.float64).reshape(-1, 1, 2)
        try:
            ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE)
            if not ok:
                ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                return None, float("inf")
            rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, dist, rvec, tvec)
        except cv2.error:
            return None, float("inf")
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img.reshape(-1, 2)) ** 2, axis=1))))
        if rms > max_rms_px or tvec[2, 0] <= 0:
            return None, rms
        return rvec_tvec_to_T(rvec, tvec), rms

    def draw(self, img, corners, ids):
        if corners is not None and len(corners):
            for c in corners:
                cv2.circle(img, (int(c[0]), int(c[1])), 4, (0, 255, 0), -1)
        return img

    def generate_image(self, width_px: int, height_px: int, margin_px: int = 20):
        if _NEW_API:
            return self.board.generateImage((width_px, height_px), marginSize=margin_px, borderBits=1)
        return self.board.draw((width_px, height_px), marginSize=margin_px, borderBits=1)


def sharpness(img) -> float:
    g = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(g, cv2.CV_64F).var())

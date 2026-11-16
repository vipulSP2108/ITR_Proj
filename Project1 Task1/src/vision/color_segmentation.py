import os
import cv2
import yaml
import numpy as np
from typing import Dict, Tuple, Optional

class ColorSegmenter:
    """
    Robust HSV-based color segmentation with morphological filtering
    and automatic handling of hue wrap-around (e.g. Red).
    """
    def __init__(self, config_path: Optional[str] = None):
        if config_path is None:
            # Default to config/colors.yaml relative to project root
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            config_path = os.path.join(base_dir, "config", "colors.yaml")

        self.config_path = config_path
        self.colors_config = {}
        self.kernel_size = 5
        self.open_iters = 2
        self.close_iters = 2
        self.load_config(config_path)

    def load_config(self, path: str):
        if not os.path.exists(path):
            raise FileNotFoundError(f"Color configuration file not found at: {path}")
        with open(path, "r") as f:
            data = yaml.safe_load(f)
            self.colors_config = data.get("colors", {})
            morph = data.get("morphology", {})
            self.kernel_size = morph.get("kernel_size", 5)
            self.open_iters = morph.get("open_iterations", 2)
            self.close_iters = morph.get("close_iterations", 2)

        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (self.kernel_size, self.kernel_size)
        )

    def get_mask(self, bgr_image: np.ndarray, color_name: str) -> np.ndarray:
        """
        Segments a specific color from a BGR image, returning a binary uint8 mask (0 or 255).
        """
        if color_name not in self.colors_config:
            raise ValueError(f"Color '{color_name}' not defined in config. Available: {list(self.colors_config.keys())}")

        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        cfg = self.colors_config[color_name]

        # Check for dual-range (e.g. red wrap-around across 0/180)
        if "lower1" in cfg and "upper1" in cfg and "lower2" in cfg and "upper2" in cfg:
            l1 = np.array(cfg["lower1"], dtype=np.uint8)
            u1 = np.array(cfg["upper1"], dtype=np.uint8)
            l2 = np.array(cfg["lower2"], dtype=np.uint8)
            u2 = np.array(cfg["upper2"], dtype=np.uint8)
            mask1 = cv2.inRange(hsv, l1, u1)
            mask2 = cv2.inRange(hsv, l2, u2)
            raw_mask = cv2.bitwise_or(mask1, mask2)
        else:
            lower = np.array(cfg["lower"], dtype=np.uint8)
            upper = np.array(cfg["upper"], dtype=np.uint8)
            raw_mask = cv2.inRange(hsv, lower, upper)

        # Morphological operations to suppress noise and fill pinholes
        opened = cv2.morphologyEx(raw_mask, cv2.MORPH_OPEN, self.kernel, iterations=self.open_iters)
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, self.kernel, iterations=self.close_iters)
        return closed

    def segment_all(self, bgr_image: np.ndarray) -> Dict[str, np.ndarray]:
        """
        Returns a dictionary mapping color name to cleaned binary mask.
        """
        return {color: self.get_mask(bgr_image, color) for color in self.colors_config.keys()}


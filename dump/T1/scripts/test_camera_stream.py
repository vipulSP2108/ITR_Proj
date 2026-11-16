#!/usr/bin/env python3
"""
RealSense / USB Camera Capture Test
Tries to open video capture, grab a live frame, and save to device/images/test_camera_capture.jpg.
If macOS blocks access, provides exact steps to enable camera permissions in System Settings.
"""

import sys
import os
import cv2

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "..", "device", "images")
os.makedirs(OUTPUT_DIR, exist_ok=True)
OUT_PATH = os.path.join(OUTPUT_DIR, "test_camera_capture.jpg")

def test_camera():
    print("=" * 60)
    print("      Intel RealSense D435i / USB Camera Capture Test")
    print("=" * 60)

    # Try available camera indices (0, 1, 2)
    found_cam = False
    for cam_idx in [0, 1, 2]:
        print(f"\n[Testing Camera Index {cam_idx}]...")
        cap = cv2.VideoCapture(cam_idx)
        if not cap.isOpened():
            print(f"  ✗ Camera index {cam_idx} could not be opened.")
            continue

        ret, frame = cap.read()
        if ret and frame is not None:
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = cap.get(cv2.CAP_PROP_FPS)
            print(f"  ✓ Camera index {cam_idx} SUCCESS! Resolution: {w}x{h} @ {fps:.1f} FPS")
            cv2.imwrite(OUT_PATH, frame)
            print(f"  ✓ Captured live test frame saved to: {OUT_PATH}")
            cap.release()
            found_cam = True
            break
        else:
            print(f"  ✗ Camera opened but failed to return a frame (check macOS permissions).")
            cap.release()

    if not found_cam:
        print("\n" + "!" * 60)
        print("  macOS Camera Permission Required:")
        print("  1. Open 'System Settings' on your Mac.")
        print("  2. Go to 'Privacy & Security' -> 'Camera'.")
        print("  3. Turn the switch ON for 'Terminal' (or Python / your IDE).")
        print("  4. Rerun this script.")
        print("!" * 60)
    else:
        print("\n✓ Camera stream is working properly!")

if __name__ == "__main__":
    test_camera()


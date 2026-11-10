# `dual_camera_manager.py` — Multi-Threaded Dual Camera Capture

**Module**: `src/vision/dual_camera_manager.py`
**Purpose**: Manages simultaneous capture from overhead and side cameras with thread-safe frame access.

---

## Class: `DualCameraManager`

### Constructor
```python
DualCameraManager(top_cam_idx=0, side_cam_idx=1)
```
- Resolution: 640 × 480 for both cameras
- Background capture threads with `threading.Lock` per camera
- 10ms sleep between reads to prevent CPU saturation

### Architecture
```
Thread 1 (daemon): top camera → lock → self.top_frame
Thread 2 (daemon): side camera → lock → self.side_frame
Main thread: get_top_frame() / get_side_frame() (lock-protected reads)
```

### Key Methods
- `start()` — Opens both VideoCapture devices, starts capture threads
- `stop()` — Signals threads to stop, releases cameras
- `get_top_frame() → np.ndarray` — Thread-safe latest overhead frame
- `get_side_frame() → np.ndarray` — Thread-safe latest side frame

---

## Design Decisions

- **Daemon threads**: Auto-cleanup on program exit. No zombie threads.
- **Per-camera locks**: Top and side cameras don't block each other.
- **10ms inter-read sleep**: Balances freshness vs. CPU load. At 30fps, this gives ~3 fresh frames per 100ms.
- **Side camera**: For secondary verification (grasp monitoring from side angle). Not used in primary perception pipeline.

"""Capture logic (hardware-agnostic so it can be unit-tested with fakes)."""
from __future__ import annotations
import json, sys, time, threading, queue
import cv2
import numpy as np

from .board import CharucoBoard, sharpness
from .session import SessionStore
from .transforms import average_poses


class Collector:
    def __init__(self, cfg: dict, robot, cams: dict, store: SessionStore, board: CharucoBoard, log=print):
        self.cfg, self.cc = cfg, cfg["capture"]
        self.robot, self.cams, self.store, self.board, self.log = robot, cams, store, board, log
        self.lock = threading.Lock()   # one capture at a time

    # ------------------------------------------------------------------ status
    def health(self) -> list[str]:
        """Human readable problems that currently block a capture."""
        problems = []
        if self.robot.age() > self.cc["stale_robot_s"]:
            problems.append(f"ROBOT STATE STALE ({self.robot.age():.1f}s) - is free_drive / heal_server running?")
        else:
            s = self.robot.get()
            if s is None or s.T_base_ee is None:
                problems.append("FK pose unavailable (URDF/KDL not ready)")
        for n, c in self.cams.items():
            if c.age() > self.cc["stale_cam_s"]:
                problems.append(f"CAMERA {n} NO FRAMES ({c.age():.1f}s)")
        return problems

    def status_line(self) -> str:
        st = self.store.state()
        bad = self.health()
        pm = st["per_matrix"].get(st["matrix"], 0)
        s = f"matrix #{st['matrix']} (captures here: {pm}) | total {st['n_captures']}"
        return s + (" | !! " + "; ".join(bad) if bad else " | ready")

    # ------------------------------------------------------------------ capture
    def capture(self, force: bool = False):
        if not self.lock.acquire(blocking=False):
            return False, "capture already in progress"
        try:
            return self._capture(force)
        finally:
            self.lock.release()

    def _capture(self, force):
        cc = self.cc
        probs = self.health()
        if probs and not force:
            return False, "not capturing: " + "; ".join(probs)

        time.sleep(cc["settle_time_s"])            # let free-drive hand-jitter die down
        names = list(self.cams)
        last_seq = {n: self.cams[n].latest()[1] for n in names}
        frames = {n: [] for n in names}
        dets = {n: [] for n in names}
        samples = []
        for _ in range(cc["burst_frames"]):
            for n in names:
                img, seq, _ = self.cams[n].wait_new(last_seq[n], cc["frame_timeout_s"])
                if img is None:
                    return False, f"camera {n} stopped delivering frames - capture aborted (nothing saved)"
                last_seq[n] = seq
                frames[n].append(img)
                dets[n].append(self.board.detect(img))
            s = self.robot.get()
            if s is None or self.robot.age() > cc["stale_robot_s"] * 2:
                return False, "robot state went stale during capture - aborted (nothing saved)"
            samples.append(s)

        # --- robot stillness + averaging
        Q = np.array([s.q for s in samples])
        motion = float(np.max(np.ptp(Q, axis=0)))
        if motion > cc["max_joint_motion_rad"] and not force:
            return False, (f"robot moved during capture ({motion:.4f} rad > {cc['max_joint_motion_rad']}). "
                           "Hold still / let go of the arm and press capture again.")
        Ts = [s.T_base_ee for s in samples if s.T_base_ee is not None]
        if not Ts:
            return False, "no FK pose available - capture aborted"
        T_mean, _, T_stats = average_poses(Ts)
        q_med = np.median(Q, axis=0)

        # --- board visibility gate
        per_cam, ok_flags = {}, []
        for n in names:
            good = [i for i, (c, ids) in enumerate(dets[n]) if c is not None and len(ids) >= cc["min_corners"]]
            per_cam[n] = {"good_frames": len(good), "frames": len(frames[n]),
                          "mean_corners": float(np.mean([len(dets[n][i][1]) for i in good])) if good else 0.0}
            ok_flags.append(len(good) >= cc["min_good_frames"])
        passed = all(ok_flags) if cc["require"] == "all" else any(ok_flags)
        if not passed and not force:
            msg = ", ".join(f"{n}: {per_cam[n]['good_frames']}/{per_cam[n]['frames']} good frames" for n in names)
            return False, f"board not clearly visible ({msg}; need >= {cc['min_good_frames']}) - nothing saved"

        # --- write atomically
        cid, tmp = self.store.begin_capture()
        npz = {}
        for n in names:
            good = [i for i, (c, ids) in enumerate(dets[n]) if c is not None and len(ids) >= cc["min_corners"]]
            pool = good if good else list(range(len(frames[n])))
            pool = sorted(pool, key=lambda i: -sharpness(frames[n][i]))[: 1 + int(cc["extra_frames"])]
            files = []
            for k, i in enumerate(pool):
                fn = f"{n}.png" if k == 0 else f"{n}_x{k}.png"
                cv2.imwrite(str(tmp / fn), frames[n][i])
                files.append(fn)
            per_cam[n]["saved"] = files
            per_cam[n]["image_size"] = [int(frames[n][0].shape[1]), int(frames[n][0].shape[0])]
            for i, (c, ids) in enumerate(dets[n]):
                if c is not None:
                    npz[f"{n}__{i}__corners"], npz[f"{n}__{i}__ids"] = c, ids
        np.savez_compressed(tmp / "detections.npz", **npz)
        meta = {"id": cid, "matrix": self.store.state()["matrix"], "wall_time": time.time(), "forced": bool(force),
                "joint_names": self.cfg["robot"]["joint_names"], "q": q_med.tolist(), "q_ptp_rad": motion,
                "T_base_ee": T_mean.tolist(), "T_stats": T_stats, "cameras": per_cam}
        self.store.commit_capture(cid, tmp, meta)
        cs = ", ".join(f"{n}:{per_cam[n]['good_frames']}/{per_cam[n]['frames']}" for n in names)
        return True, f"saved capture #{cid} on matrix #{meta['matrix']} (good frames {cs}; arm jitter {motion*1000:.2f} mrad)"


# ---------------------------------------------------------------------- keyboard
class KeyReader:
    """Non-blocking single-key reader (cbreak mode); falls back to line input if stdin is not a TTY."""

    def __init__(self):
        self.q: "queue.Queue[str]" = queue.Queue()
        self._old = None
        self._stop = threading.Event()
        self._tty = sys.stdin.isatty()

    def __enter__(self):
        if self._tty:
            import termios, tty
            self._fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        threading.Thread(target=self._run, daemon=True).start()
        return self

    def __exit__(self, *a):
        self._stop.set()
        if self._old is not None:
            import termios
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def _run(self):
        import select
        while not self._stop.is_set():
            try:
                if self._tty:
                    r, _, _ = select.select([sys.stdin], [], [], 0.1)
                    if r:
                        ch = sys.stdin.read(1)
                        if ch:
                            self.q.put(ch)
                else:
                    line = sys.stdin.readline()
                    if not line:
                        return
                    self.q.put(line.strip()[:1] or " ")
            except Exception:
                return

    def get(self):
        try:
            return self.q.get_nowait()
        except queue.Empty:
            return None

#!/usr/bin/env python3
"""End-to-end self-test with NO hardware: simulates a robot, a hand camera, a fixed camera and a ChArUco board,
runs the real Collector (incl. crash/resume/undo/stale handling) and the real calibration, and compares against
ground truth.   python3 scripts/selftest.py"""
import json, shutil, sys, tempfile, threading, time
from pathlib import Path
if "-h" in sys.argv or "--help" in sys.argv:
    print(__doc__)
    sys.exit(0)
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import cv2, numpy as np
from scipy.spatial.transform import Rotation as Rot

from buzzwire_calib.config import load_config
from buzzwire_calib.board import BoardSpec, CharucoBoard
from buzzwire_calib.cameras import PushSource
from buzzwire_calib.collector import Collector
from buzzwire_calib.ros_io import RobotSample
from buzzwire_calib.session import SessionStore
from buzzwire_calib.transforms import make_T, inv_T, pose_diff
from buzzwire_calib import calibration as C
from buzzwire_calib.handeye import solve

rng = np.random.default_rng(7)
W, H, F = 1280, 720, 900.0
K = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1.0]])
DIST = np.zeros(5)


def lookat(pos, target, roll_deg=0.0):
    z = target - pos; z /= np.linalg.norm(z)
    d = np.array([0, 0, -1.0]); y = d - d.dot(z) * z
    if np.linalg.norm(y) < 1e-3:
        y = np.array([0, -1.0, 0]) - 0 * z
    y /= np.linalg.norm(y); x = np.cross(y, z)
    R = np.column_stack([x, y, z]) @ Rot.from_euler("z", roll_deg, degrees=True).as_matrix()
    return make_T(R, pos)


def render(board_img, ppm, T_cb):
    """Render the board seen from a camera with pose T_cb (cam <- board)."""
    S = np.array([[1 / ppm, 0, 0], [0, 1 / ppm, 0], [0, 0, 1]])   # board-image px -> metres
    Hm = K @ np.column_stack([T_cb[:3, 0], T_cb[:3, 1], T_cb[:3, 3]]) @ S
    return cv2.warpPerspective(board_img, Hm, (W, H), borderValue=255)


class FakeRobot:
    def __init__(self): self.T = np.eye(4); self.stale = False
    def age(self): return 99.0 if self.stale else 0.0
    def get(self):
        T = self.T.copy(); T[:3, 3] += rng.normal(0, 0.0002, 3)
        return RobotSample(np.zeros(6), None, T, time.time())


def main():
    tmp = Path(tempfile.mkdtemp(prefix="bwcal_selftest_"))
    cfg = load_config(None)
    cfg["capture"].update(burst_frames=8, settle_time_s=0.02, frame_timeout_s=3.0)
    spec = BoardSpec.from_cfg(cfg["board"])
    board = CharucoBoard(spec)
    ppm = 2500.0   # px per metre of rendered board texture
    bimg = cv2.cvtColor(board.generate_image(int(spec.squares_x * spec.square_length * ppm),
                                             int(spec.squares_y * spec.square_length * ppm), 0), cv2.COLOR_GRAY2BGR)

    # ---------- ground truth: cam0 = HAND (X0=T_ee_cam), cam1 = FIXED (X1=T_base_cam)
    X0 = make_T(Rot.from_euler("xyz", [3, -4, 92], degrees=True).as_matrix(), [0.04, -0.02, 0.09])
    X1 = lookat(np.array([0.9, -0.7, 0.8]), np.array([0.35, 0.05, 0.0]))
    centre = np.array([spec.squares_x, spec.squares_y, 0]) * spec.square_length / 2
    placements = [(0.30, 0.00, 20), (0.45, 0.10, -35), (0.20, 0.15, 60), (0.40, -0.10, 100), (0.30, 0.20, -10)]
    cur = {"T_cb": {}}

    robot = FakeRobot()
    cams = {"cam0": PushSource("cam0"), "cam1": PushSource("cam1")}
    stop = threading.Event()

    def feeder():
        while not stop.is_set():
            for n, im in list(cur.get("img", {}).items()):
                noisy = np.clip(im.astype(np.int16) + rng.normal(0, 2.0, im.shape).astype(np.int16), 0, 255).astype(np.uint8)
                cams[n].push(noisy)
            time.sleep(0.02)
    threading.Thread(target=feeder, daemon=True).start()

    store = SessionStore(tmp / "session_test")
    col = Collector(cfg, robot, cams, store, board, log=print)
    n_cap = 0
    time.sleep(0.5)
    for k, (bx, by, yaw) in enumerate(placements):
        B = make_T(Rot.from_euler("z", yaw, degrees=True).as_matrix() @ Rot.from_euler("x", 180, degrees=True).as_matrix(), [bx, by, 0]) @ make_T(np.eye(3), -centre)
        for j in range(6):
            while True:
                pos = np.array([bx, by, 0]) + np.array([rng.uniform(-.12, .12), rng.uniform(-.12, .12), rng.uniform(.38, .55)])
                Tbc0 = lookat(pos, (B @ np.append(centre, 1))[:3], roll_deg=rng.uniform(-70, 70))
                # add tilt by moving look target
                Tbc0 = lookat(pos, (B @ np.append(centre, 1))[:3] + rng.uniform(-.06, .06, 3) * [1, 1, 0], roll_deg=rng.uniform(-70, 70))
                T_be = Tbc0 @ inv_T(X0)
                T_cb0, T_cb1 = inv_T(Tbc0) @ B, inv_T(X1) @ B
                nz, t1 = T_cb1[:3, 2], T_cb1[:3, 3]      # board z points INTO the board; fixed cam must see the front
                if nz.dot(t1 / np.linalg.norm(t1)) > 0.35 and T_cb0[2, 3] > 0.2: break
            robot.T = T_be
            cur["img"] = {"cam0": render(bimg, ppm, T_cb0), "cam1": render(bimg, ppm, T_cb1)}
            time.sleep(0.1)
            ok, msg = col.capture()
            print(("OK " if ok else "FAIL ") + msg)
            n_cap += int(ok)
        if k == 1:   # ---- simulate a crash + restart in the middle of the session
            with open(store.events_path, "a") as f: f.write('{"type": "capt')     # torn write
            store = SessionStore(tmp / "session_test"); col.store = store
            assert store.state()["matrix"] == 1 and store.state()["n_captures"] == n_cap, store.state()
            print(">>> simulated crash/restart: resumed at matrix", store.state()["matrix"])
        store.next_matrix()
    # ---- edge cases
    robot.stale = True
    ok, msg = col.capture(); assert not ok and "STALE" in msg; print("stale-robot guard OK:", msg[:50]); robot.stale = False
    cur["img"] = {"cam0": np.full((H, W, 3), 128, np.uint8), "cam1": np.full((H, W, 3), 128, np.uint8)}
    ok, msg = col.capture(); assert not ok and "not clearly visible" in msg; print("no-board guard OK")
    undone = store.undo_last(); print("undo ->", undone)
    store.finish(); stop.set()
    st = store.state(); print("\nSTATE:", {k: st[k] for k in ("n_captures", "per_matrix", "finished")})

    # ---------- calibrate
    mounts = {"cam0": "hand", "cam1": "fixed"}
    # dedicated intrinsics run (what capture_intrinsics.py + calibrate_intrinsics.py do on the real robot)
    from buzzwire_calib.intrinsics import calibrate_intrinsics, save_intrinsics
    for c in mounts:
        views = []
        for _ in range(25):
            R = Rot.from_euler("xyz", [rng.uniform(-35, 35), rng.uniform(-35, 35), rng.uniform(-90, 90)], degrees=True).as_matrix()
            ctr = np.array([rng.uniform(-.25, .25), rng.uniform(-.12, .12), rng.uniform(.5, .8)])   # board centre in cam frame
            Tcb = make_T(R, ctr - R @ centre)
            views.append(render(bimg, ppm, Tcb))
        res = calibrate_intrinsics(board, views, min_corners=10, log=lambda *a: None)
        save_intrinsics(res, tmp / "intr" / f"{c}.json")
        print(f"[{c}] dedicated intrinsics: f={res['K'][0][0]:.1f} RMS {res['rms_px']:.3f}px")
    intr = {c: C.get_intrinsics(c, store, board, tmp / "intr", cfg, print) for c in mounts}
    for c in mounts:
        print(f"{c} focal est {intr[c][0][0,0]:.1f} (true {F})")
    obs, bp = C.build_observations(store, cfg, board, intr, print)
    sol = solve(obs, mounts, "world")
    e0 = pose_diff(sol["X"]["cam0"], X0); e1 = pose_diff(sol["X"]["cam1"], X1)
    sd, si = C.direct_stereo(bp, ["cam0", "cam1"]), C.implied_stereo(sol, obs, ["cam0", "cam1"])
    print(f"\nhand cam  T_ee_cam  error: {e0[0]:.3f} deg, {e0[1]*1000:.2f} mm")
    print(f"fixed cam T_base_cam error: {e1[0]:.3f} deg, {e1[1]*1000:.2f} mm")
    print("residuals:", {c: (round(s['trans_rms_mm'], 2), round(s['rot_rms_deg'], 3)) for c, s in sol['stats'].items()})
    rep = C.build_report(sol, obs, sd, si, cfg)
    print("stereo consistency:", rep.get("stereo_consistency"), "| warnings:", rep["warnings"])
    C.write_outputs(tmp / "results", rep, sol, cfg)
    ok = e0[1] < 0.004 and e1[1] < 0.006 and e0[0] < 0.5 and e1[0] < 0.5
    print("\nSELFTEST", "PASSED" if ok else "FAILED", f"(files in {tmp})")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

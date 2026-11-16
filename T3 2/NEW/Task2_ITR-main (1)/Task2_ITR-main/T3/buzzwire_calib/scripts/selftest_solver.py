#!/usr/bin/env python3
"""Fast numeric test of the hand-eye solver for every mounting combination (no images)."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from buzzwire_calib.handeye import Obs, solve
from buzzwire_calib.transforms import make_T, inv_T, pose_diff

rng = np.random.default_rng(1)
def rT(rmax, tmax): return make_T(Rot.from_rotvec(rng.uniform(-rmax, rmax, 3)).as_matrix(), rng.uniform(-tmax, tmax, 3))
def noisy(T, sr=0.002, st=0.0015): return T @ make_T(Rot.from_rotvec(rng.normal(0, sr, 3)).as_matrix(), rng.normal(0, st, 3))

def run(name, board_mount, mounts, n_groups=4, per=6, outliers=2):
    gt_X = {c: (rT(1.5, .1) if m == "hand" else rT(3, 1.0)) for c, m in mounts.items()}
    gt_Y = rT(1.5, .1)
    gt_B = {g: rT(3, 0.8) for g in range(n_groups)}
    obs = []; cap = 0
    for g in range(n_groups):
        for _ in range(per):
            T_be = rT(2.2, 0.6) @ make_T(np.eye(3), [0.5, 0, 0.5])
            B = T_be @ gt_Y if board_mount == "ee" else gt_B[g]
            for c, m in mounts.items():
                Tb_cam = gt_X[c] if m == "fixed" else T_be @ gt_X[c]
                obs.append(Obs(cap, g if board_mount == "world" else 0, c, T_be, noisy(inv_T(Tb_cam) @ B)))
            cap += 1
    for k in rng.choice(len(obs), outliers, replace=False):   # gross outliers
        obs[k].T_cb = obs[k].T_cb @ rT(0.3, 0.05)
    sol = solve(obs, mounts, board_mount, log=lambda *a: None)
    worst = [0, 0]
    for c in mounts:
        r, t = pose_diff(sol["X"][c], gt_X[c]); worst = [max(worst[0], r), max(worst[1], t * 1000)]
    print(f"{name:34s} worst error {worst[0]:.3f} deg {worst[1]:.2f} mm   rejected {sum(not p['used'] for p in sol['per_obs'])}")
    return worst[0] < 0.5 and worst[1] < 5

ok = True
ok &= run("board on gripper, 2 fixed cams", "ee", {"cam0": "fixed", "cam1": "fixed"}, n_groups=1, per=20)
ok &= run("board static, 2 hand cams", "world", {"cam0": "hand", "cam1": "hand"})
ok &= run("board static, hand + fixed", "world", {"cam0": "hand", "cam1": "fixed"})
ok &= run("board static, fixed + hand", "world", {"cam0": "fixed", "cam1": "hand"})
try:
    run("board static, 2 fixed (impossible)", "world", {"cam0": "fixed", "cam1": "fixed"})
    ok = False
except ValueError as e:
    print("impossible setup correctly rejected:", str(e)[:70], "...")
print("SOLVER SELFTEST", "PASSED" if ok else "FAILED"); sys.exit(0 if ok else 1)

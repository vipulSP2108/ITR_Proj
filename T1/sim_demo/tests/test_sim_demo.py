"""
Quick self-checks for the sim demo.   Run:   python tests/test_sim_demo.py     (or: pytest tests)
~1-2 minutes.  Everything here uses simulator ground truth only to SCORE the pipeline.
"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from sim_env import SimEnv            # first: sets the GL backend
import numpy as np
import mujoco

import arm_model as A
from perception import load_vision_cfg, detect_surfaces, detect_holes, detect_green_peg
from calibration import run_calibration, transform_error
from planner import plan_pick_place
from run_pipeline import snapshot_detect

CFG = load_vision_cfg()


def test_fk_matches_mujoco():
    env = SimEnv(); m, d = env.model, env.data
    rng = np.random.default_rng(0); worst = 0
    for _ in range(100):
        q = rng.uniform(A.JOINT_LIMITS[:, 0], A.JOINT_LIMITS[:, 1])
        d.qpos[env.qadr] = q; mujoco.mj_forward(m, d)
        l4 = m.body("link4").id
        tcp = d.xpos[l4] + d.xmat[l4].reshape(3, 3)[:, 0] * A.TOOL
        worst = max(worst, np.linalg.norm(tcp - A.fk(q)["tcp"]),
                    np.linalg.norm(d.geom_xpos[m.geom("tool_marker").id] - A.fk(q)["marker"]))
    assert worst < 1e-6, worst


def test_ik_roundtrip():
    rng = np.random.default_rng(1); n = 0
    for _ in range(500):
        p = [rng.uniform(0.05, 0.3), rng.uniform(-0.2, 0.2), rng.uniform(0.03, 0.15)]
        q = A.ik(*p, np.radians(rng.uniform(0, 60)))
        if q is not None:
            n += 1; assert np.linalg.norm(A.fk(q)["tcp"] - p) < 1e-9
    assert n > 100


def test_calibration_recovers_camera_pose():
    env = SimEnv(); res = run_calibration(env, CFG)
    dt, da = transform_error(res["T_robot_cam"], env.gt_cam_pose())
    assert res["n_used"] >= 12 and dt < 3.0 and da < 0.3, (res["n_used"], dt, da)


def test_all_four_shapes_classified_and_located():
    env = SimEnv(); env.place_peg("circle"); T = env.gt_cam_pose()
    ws = snapshot_detect(env, CFG, T)
    assert [h["shape"] for h in ws["holes"]] == ["circle", "hexagon", "square", "triangle"]
    hx, hy = env.layout.hole_xy("circle")
    c = ws["holes"][0]
    assert np.hypot(c["x"] - hx, c["y"] - hy) < 0.003
    g = env.gt_body_pos("peg_circle"); p = ws["peg"]
    assert np.hypot(p["x"] - g[0], p["y"] - g[1]) < 0.004 and abs(p["height"] - 0.10) < 0.01


def test_planner_picks_tilt_from_reach():
    peg = dict(x=0.16, y=0.07, z_top=0.11, height=0.10)
    far = plan_pick_place(peg, dict(x=0.295, y=-0.115), 0.008)
    near = plan_pick_place(peg, dict(x=0.20, y=-0.05), 0.008)
    assert far is not None and near is not None and far.tilt_deg > near.tilt_deg
    assert plan_pick_place(peg, dict(x=0.55, y=0.0), 0.008) is None        # outside the arm's reach


def test_pick_and_place_physics_multiple_positions():
    for xy in [(0.16, 0.0715), (0.12, -0.08), (0.19, 0.0)]:
        env = SimEnv(peg_xy=xy, depth_noise_m=0.0); env.place_peg("circle"); T = env.gt_cam_pose()
        ws = snapshot_detect(env, CFG, T); hole = [h for h in ws["holes"] if h["shape"] == "circle"][0]
        plan = plan_pick_place(ws["peg"], hole, ws["board_z"]); t = np.radians(plan.tilt_deg)
        env.move_joints(A.HOME_Q, 1.8)
        for w in plan.waypoints:
            cur = A.fk(env.ctrl_q)["tcp"]
            if w.kind == "joint": env.move_joints(w.q, w.duration)
            elif np.linalg.norm(w.p - cur) > 1e-4: env.move_cartesian(cur, w.p, t, w.duration)
            env.settle(0.25)
            if w.gripper == "close": env.set_gripper(True, 1.0)
            if w.gripper == "open": env.set_gripper(False, 0.7); env.settle(0.3)
        env.move_joints(A.SIDE_PARK_Q, 2.0); env.settle(0.8)
        g = env.gt_body_pos("peg_circle"); hx, hy = env.layout.hole_xy("circle")
        assert np.hypot(g[0] - hx, g[1] - hy) < 0.006 and env.gt_body_up_tilt_deg("peg_circle") < 5, xy


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn(); print(f"PASS  {name}")
            except Exception as e:
                fails += 1; print(f"FAIL  {name}: {e!r}")
    sys.exit(1 if fails else 0)

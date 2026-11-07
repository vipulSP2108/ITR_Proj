#!/usr/bin/env python3
"""
Sim demo pipeline for the OpenMANIPULATOR-X peg task (MuJoCo, one circle peg).

  Stage 1  HOME        park the arm in the "L" home pose, record it
  Stage 2  WORKSPACE   (a) move the arm through calibration poses -> camera<->robot transform
                       (b) snapshot with the arm parked aside -> detect mat / board / holes / peg,
                           build the reachability map, choose the grasp strategy
  Stage 3  PICK&PLACE  pick the circle peg and place it in the circle socket, verifying the motion with
                       the mounted camera at ~25 Hz (tracking error, cross-track error, direction)

Everything is written to  <out-root>/demo_<YYYYMMDD_HHMMSS>/  so every run is kept.

    python run_pipeline.py                         # all stages, default random peg position
    python run_pipeline.py --until workspace       # stages 1-2 only
    python run_pipeline.py --peg-xy 0.15 -0.05     # put the peg at (x, y) metres in the robot frame
"""
import os, sys, json, time, shutil, argparse, datetime
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from sim_env import SimEnv, GRIP_CLOSE            # must be imported before mujoco (GL backend)
import numpy as np
import cv2
import yaml

import arm_model as A
import viz
from scene_builder import MAT_TOP, BOARD_TOP, MARKER_R, load_layout, default_peg_xy
from perception import (load_vision_cfg, detect_surfaces, detect_holes, detect_green_peg, detect_marker, color_masks)
from calibration import run_calibration, transform_error
from workspace import reachability_grid, draw_reachability_overlay, TILT_STEPS_DEG
from planner import plan_pick_place, GRASP_DEPTH
from tracker import PathTracker, AbortMotion


class GraspFailed(Exception):
    pass


def jdump(path, obj):
    def d(o):
        if isinstance(o, (np.ndarray,)): return o.tolist()
        if isinstance(o, (np.floating, np.integer)): return o.item()
        if isinstance(o, (np.bool_,)): return bool(o)
        return str(o)
    with open(path, "w") as f:
        json.dump(obj, f, indent=2, default=d)


def say(msg):
    print(msg, flush=True)


# =============================================================================================
def stage_home(env, d, log):
    say("\n[1/3] HOME POSE")
    os.makedirs(d, exist_ok=True)
    env.move_joints(A.HOME_Q, 2.0); env.settle(0.8)
    enc = env.read_joints(); fk = A.fk(enc)
    m = env.model; dd = env.data
    l4 = m.body("link4").id
    tcp_true = dd.xpos[l4] + dd.xmat[l4].reshape(3, 3)[:, 0] * A.TOOL
    err = float(np.linalg.norm(tcp_true - fk["tcp"]) * 1000)
    bgr, _ = env.capture_rgbd()
    cv2.imwrite(os.path.join(d, "home_overhead_camera.png"), bgr)
    cv2.imwrite(os.path.join(d, "home_oblique_view.png"), env.render_view("oblique_view"))
    cv2.imwrite(os.path.join(d, "home_side_view.png"), env.render_view("side_view"))
    info = dict(joint_encoders_rad=enc, expected_L_pose=bool(np.allclose(enc, 0, atol=0.02)),
                tcp_from_arm_dimensions_m=fk["tcp"], tcp_in_simulator_m=tcp_true, fk_vs_sim_mm=err,
                tool_tilt_deg=float(np.degrees(fk["tilt"])))
    jdump(os.path.join(d, "home_pose.json"), info)
    say(f"   arm at home (L pose): encoders {np.round(enc, 3).tolist()}, TCP = {np.round(fk['tcp'], 3).tolist()} m, "
        f"model-vs-sim error {err:.3f} mm")
    log["home"] = info


# =============================================================================================
def stage_calibration(env, cfg, d, log, max_poses):
    say("\n[2a/3] CAMERA <-> ROBOT CALIBRATION (arm moves, camera watches the tool marker)")
    os.makedirs(d, exist_ok=True)
    last = {}

    def on_pose(i, bgr, depth, det, P_fk):
        img = bgr.copy()
        if det is not None:
            cv2.circle(img, (int(det["u"]), int(det["v"])), 12, (0, 255, 0), 2)
            viz.put(img, f"pose {i:02d}: marker found", (10, 22), 0.5, (120, 255, 120))
        else:
            viz.put(img, f"pose {i:02d}: marker NOT found", (10, 22), 0.5, (80, 80, 255))
        cv2.imwrite(os.path.join(d, f"calib_pose_{i:02d}.png"), img)
        last["bgr"] = bgr

    res = run_calibration(env, cfg, on_pose=on_pose, max_poses=max_poses)
    T = res["T_robot_cam"]
    dt, da = transform_error(T, env.gt_cam_pose())
    # reprojection picture: detected marker (green) vs FK marker projected through the estimate (cyan)
    img = last["bgr"].copy()
    for s in res["per_pose"]:
        if not s["ok"]: continue
        cv2.circle(img, (int(s["uv"][0]), int(s["uv"][1])), 6, (0, 255, 0), 1)
        uv = viz.project_robot_pts(env.cam, T, s["P_rob"])[0]
        cv2.drawMarker(img, (int(uv[0]), int(uv[1])), (255, 255, 0), cv2.MARKER_CROSS, 10, 1)
    viz.put(img, "green ring = marker seen by camera | cyan cross = marker predicted from arm dimensions + calibration", (8, 470), 0.4)
    cv2.imwrite(os.path.join(d, "calibration_reprojection.png"), img)
    out = dict(T_robot_cam=T, rms_residual_mm=res["rms_mm"], poses_attempted=res["n_attempted"],
               poses_detected=res["n_detected"], poses_used=res["n_used"],
               mean_reprojection_px=float(np.mean([s["reproj_px"] for s in res["per_pose"] if s["ok"]])),
               SIM_ONLY_error_vs_true_camera_pose=dict(translation_mm=dt, rotation_deg=da),
               per_pose=[{k: v for k, v in s.items() if k in ("i", "ok", "target", "residual_mm", "reproj_px", "used")} for s in res["per_pose"]])
    jdump(os.path.join(d, "calibration.json"), out)
    say(f"   marker detected in {res['n_detected']}/{res['n_attempted']} poses, fit residual {res['rms_mm']:.2f} mm, "
        f"reprojection {out['mean_reprojection_px']:.2f} px")
    say(f"   (sim-only check vs true camera pose: {dt:.2f} mm, {da:.3f} deg)")
    log["calibration"] = out
    return T


# =============================================================================================
def snapshot_detect(env, cfg, T, d=None, tag="snapshot", mat_z=None, board_z=None, save=True):
    """Park the arm aside, grab an RGB-D frame, detect surfaces/holes/peg."""
    env.move_joints(A.SIDE_PARK_Q, 2.0); env.settle(0.6)
    bgr, depth = env.capture_rgbd()
    surf = detect_surfaces(bgr, depth, env.cam, cfg, T)
    mat_z = surf["mat"]["z"] if surf.get("mat") else MAT_TOP
    board_z = surf["board"]["z"] if surf.get("board") else BOARD_TOP
    holes = detect_holes(bgr, depth, env.cam, cfg, T, board_z)
    peg = detect_green_peg(bgr, depth, env.cam, cfg, T, mat_z)
    return dict(bgr=bgr, depth=depth, surfaces=surf, holes=holes, peg=peg, mat_z=mat_z, board_z=board_z)


def stage_workspace(env, cfg, T, d, log):
    say("\n[2b/3] WORKSPACE ANALYSIS (arm parked aside -> clean view)")
    os.makedirs(d, exist_ok=True)
    ws = snapshot_detect(env, cfg, T)
    bgr, depth = ws["bgr"], ws["depth"]
    if ws["peg"] is None:
        raise RuntimeError("circle peg not found in the snapshot - is it on the mat and visible?")
    circle = [h for h in ws["holes"] if h["shape"] == "circle"]
    if not circle:
        raise RuntimeError(f"no circular socket detected; saw shapes {[h['shape'] for h in ws['holes']]}")
    hole = circle[0]; peg = ws["peg"]
    if not peg_upright_ok(peg):
        raise RuntimeError(f"circle peg measured {peg['height']*100:.1f} cm tall (expected ~10): it looks toppled over - stand it upright")

    # ---- images
    cv2.imwrite(os.path.join(d, "01_rgb.png"), bgr)
    dv = np.clip((depth - 0.3) / 0.45, 0, 1); dv = cv2.applyColorMap((255 * (1 - dv)).astype(np.uint8), cv2.COLORMAP_TURBO)
    dv[depth <= 0] = 0
    cv2.imwrite(os.path.join(d, "02_depth.png"), dv)
    masks = color_masks(bgr, cfg); seg = bgr.copy()
    for key, col in (("mat", (80, 200, 80)), ("board", (80, 160, 230)), ("peg_green", (0, 255, 0)), ("red_ring", (0, 0, 255)), ("marker", (255, 0, 255))):
        seg[masks[key] > 0] = (0.45 * seg[masks[key] > 0] + 0.55 * np.array(col)).astype(np.uint8)
    cv2.imwrite(os.path.join(d, "03_segmentation.png"), seg)
    det = bgr.copy()
    for h in ws["holes"]:
        cv2.drawContours(det, [h["contour"]], -1, (0, 255, 255) if h["shape"] == "circle" else (200, 200, 200), 2)
        u, v = h["centroid_px"]
        viz.put(det, f"{h['shape']} ({h['x']*100:.1f},{h['y']*100:.1f})cm", (int(u) - 30, int(v) - 18), 0.38,
                (0, 255, 255) if h["shape"] == "circle" else (210, 210, 210))
    u, v = peg["centroid_px"]
    cv2.circle(det, (int(u), int(v)), int(peg["radius"] * env.cam.fx / 0.5), (0, 255, 0), 2)
    viz.put(det, f"circle peg ({peg['x']*100:.1f},{peg['y']*100:.1f})cm  h={peg['height']*100:.1f}cm", (int(u) - 50, int(v) - 22), 0.4, (120, 255, 120))
    cv2.imwrite(os.path.join(d, "04_detections.png"), det)

    # ---- reachability at the heights that matter
    z_grasp = peg["z_top"] - GRASP_DEPTH
    z_place = ws["board_z"] + (z_grasp - (peg["z_top"] - peg["height"])) + 0.004
    reach_pick = reachability_grid(z_grasp); reach_place = reachability_grid(z_place)
    sm, sb = ws["surfaces"]["mat"], ws["surfaces"]["board"]
    xr = (min(sm["x_range"][0], sb["x_range"][0]), max(sm["x_range"][1], sb["x_range"][1]))
    yr = (min(sm["y_range"][0], sb["y_range"][0]), max(sm["y_range"][1], sb["y_range"][1]))
    ov1, c1 = draw_reachability_overlay(bgr, env.cam, T, *reach_pick, ws["mat_z"], (xr, yr))
    viz.put(ov1, f"TCP height {z_grasp*100:.1f} cm (grasp height)", (10, 470), 0.45)
    ov2, c2 = draw_reachability_overlay(bgr, env.cam, T, *reach_place, ws["board_z"], (xr, yr))
    viz.put(ov2, f"TCP height {z_place*100:.1f} cm (place height)", (10, 470), 0.45)
    cv2.imwrite(os.path.join(d, "05_reachability_grasp_height.png"), ov1)
    cv2.imwrite(os.path.join(d, "06_reachability_place_height.png"), ov2)

    # ---- plan
    plan = plan_pick_place(peg, hole, ws["board_z"])
    viz.topdown_map(os.path.join(d, "07_topdown_map.png"), ws, plan=plan, reach=reach_pick)
    r_peg, r_hole = float(np.hypot(peg["x"], peg["y"])), float(np.hypot(hole["x"], hole["y"]))
    report = dict(
        surfaces={k: {kk: vv for kk, vv in v.items() if kk not in ("mask",)} for k, v in ws["surfaces"].items()},
        holes=[{k: v for k, v in h.items() if k != "contour"} for h in ws["holes"]],
        peg={k: v for k, v in peg.items()}, circle_hole={k: v for k, v in hole.items() if k != "contour"},
        distances_from_base_m=dict(peg=r_peg, circle_hole=r_hole),
        max_vertical_reach_at_grasp_height_m=A.max_vertical_reach(z_grasp),
        reachability_cells_pick_height=c1, reachability_cells_place_height=c2,
        verdict=dict(
            peg_min_tilt_deg=None if plan is None else None,
            chosen_tilt_deg=None if plan is None else plan.tilt_deg,
            feasibility=None if plan is None else plan.feasibility,
            notes=None if plan is None else plan.notes))
    jdump(os.path.join(d, "workspace_report.json"), report)
    if plan is None:
        raise RuntimeError("no feasible pick-and-place plan (peg or socket outside the reachable workspace)")
    say(f"   table plane: mat z={ws['mat_z']*100:.2f} cm, board z={ws['board_z']*100:.2f} cm")
    say(f"   holes found: {[h['shape'] for h in ws['holes']]} (top->bottom)")
    say(f"   circle peg  : x={peg['x']*100:.1f} y={peg['y']*100:.1f} cm, r={r_peg*100:.1f} cm from base, height {peg['height']*100:.1f} cm, "
        f"radius {peg['radius']*1000:.1f} mm")
    say(f"   circle hole : x={hole['x']*100:.1f} y={hole['y']*100:.1f} cm, r={r_hole*100:.1f} cm from base")
    say(f"   vertical top-grasp reach at grasp height = {report['max_vertical_reach_at_grasp_height_m']*100:.1f} cm")
    for row in plan.feasibility:
        if row["tilt_deg"] <= plan.tilt_deg:
            say(f"     tilt {row['tilt_deg']:2d} deg -> {'OK ' if row['feasible'] else 'no '} {row['reason']}")
    say(f"   => chosen constant tool tilt: {plan.tilt_deg} deg" + ("  (" + "; ".join(plan.notes) + ")" if plan.notes else ""))
    log["workspace"] = report
    return ws, plan, hole, peg, (reach_pick,)


# =============================================================================================
SEG_IDX = {"pick_hover": 0, "pick_descend": 1, "grasp_close": 1, "pick_lift": 2, "transit": 3,
           "place_descend": 4, "release_open": 4, "place_retreat": 5, "return_park": 5, "move_to_home": 0}


def held_ok(env):
    return 0.018 < env.read_gripper() < 0.042      # peg is 40 mm: fingers stall at ~30 mm total travel


def save_ckpt(tr, d, idx, name):
    tr.observe()
    cv2.imwrite(os.path.join(d, f"ckpt_{idx:02d}_{name}_overhead.png"), tr.last_frame)
    cv2.imwrite(os.path.join(d, f"ckpt_{idx:02d}_{name}_oblique.png"), tr.last_oblique)
    return tr.last_frame


def execute_plan(env, plan, tr, d, ckpts):
    tr.begin("move_to_home", A.fk(A.HOME_Q)["tcp"], "joint", SEG_IDX["move_to_home"])
    env.move_joints(A.HOME_Q, 2.2); env.settle(0.2)
    tr.end()
    tilt = np.radians(plan.tilt_deg)
    for i, w in enumerate(plan.waypoints):
        tr.begin(w.name, w.p, w.kind, SEG_IDX[w.name])
        cur = A.fk(env.ctrl_q)["tcp"]
        if w.kind == "joint":
            env.move_joints(w.q, w.duration)
        elif np.linalg.norm(w.p - cur) > 1e-4:
            env.move_cartesian(cur, w.p, tilt, w.duration)
        env.settle(0.25)
        if w.gripper == "close":
            env.set_gripper(True, 1.0)
            if not held_ok(env):
                raise GraspFailed(f"finger travel {env.read_gripper()*1000:.1f} mm: closed on nothing (peg is ~30 mm)")
        elif w.gripper == "open":
            env.set_gripper(False, 0.7); env.settle(0.3)
        if w.name in ("pick_lift", "transit") and not held_ok(env):
            raise GraspFailed(f"peg dropped during '{w.name}' (finger travel {env.read_gripper()*1000:.1f} mm)")
        tr.end()
        ckpts.append(save_ckpt(tr, d, i + 1, w.name))
        say(f"   {i+1}/{len(plan.waypoints)} {w.name:14s} reached | last obs: {tr.recs[-1]['status']}, "
            f"track err {tr.recs[-1]['e_track_mm'] if tr.recs[-1]['e_track_mm'] is None else round(tr.recs[-1]['e_track_mm'],1)} mm")


def path_points(plan):
    return [A.fk(A.HOME_Q)["tcp"]] + [plan.waypoints[i].p for i in (0, 1, 3, 4, 5, 7)]


def peg_upright_ok(peg):
    return 0.085 <= peg["height"] <= 0.118        # nominal 100 mm; much lower = lying on its side


def recovery_lift(env, tr, plan, peg_top):
    """Straight up (same tool tilt) until the open fingers are well above the peg top, BEFORE any sideways motion."""
    tilt = np.radians(plan.tilt_deg)
    cur = A.fk(env.ctrl_q)["tcp"]
    z = max(cur[2], peg_top + 0.040)
    target = np.array([cur[0], cur[1], z])
    if A.ik(*target, tilt) is None:
        target = plan.waypoints[0].p
    tr.begin("recovery_lift", target, "line", 1)
    if np.linalg.norm(target - cur) > 1e-3:
        env.move_cartesian(cur, target, tilt, 1.4)
    tr.end()


def stage_pick_place(env, cfg, T, ws, plan, hole, peg, d, log, make_video, max_attempts=2, inject_miss=False):
    say("\n[3/3] PICK & PLACE the circle peg (camera-verified)")
    os.makedirs(d, exist_ok=True)
    if inject_miss:
        env.place_peg("circle", (peg["x"], peg["y"] - 0.09))
        say("   [test] disturbance injected: peg moved 90 mm sideways AFTER the snapshot (simulates a bump) - expect a missed grasp + recovery")
        log["injected_disturbance"] = "peg moved -90 mm in y after snapshot"
    tr = PathTracker(env, cfg, T, ws["mat_z"], path_points(plan),
                     video_path=os.path.join(d, "pick_and_place.mp4") if make_video else None)
    attempts, ckpts = [], []
    outcome, reason = "not_run", ""
    for attempt in range(1, max_attempts + 1):
        say(f"   attempt {attempt}: tilt {plan.tilt_deg} deg, peg ({peg['x']*100:.1f},{peg['y']*100:.1f}) cm -> hole ({hole['x']*100:.1f},{hole['y']*100:.1f}) cm")
        tr.path_pts = path_points(plan); tr.abort_enabled = True
        n0 = len(tr.recs); ckpts.clear()
        outcome, reason = "success", ""
        try:
            execute_plan(env, plan, tr, d, ckpts)
        except GraspFailed as e:
            outcome, reason = "grasp_failed", str(e)
        except AbortMotion as e:
            outcome, reason = "aborted_by_vision_monitor", str(e)
        except RuntimeError as e:
            outcome, reason = "motion_error", str(e)
        attempts.append(dict(attempt=attempt, outcome=outcome, reason=reason, observations=len(tr.recs) - n0))
        say(f"   attempt {attempt} -> {outcome}" + (f": {reason}" if reason else ""))
        if outcome == "success":
            break
        if outcome != "grasp_failed" or attempt == max_attempts:
            say("   no automatic recovery for this failure type: stopping (operator needed)")
            break
        # ---- recovery: open, lift clear, park aside, re-scan, re-localise, re-plan
        tr.abort_enabled = False
        say("   recovery: open gripper, lift clear, re-scan the table, re-plan")
        env.set_gripper(False, 0.6)
        recovery_lift(env, tr, plan, peg["z_top"])
        tr.begin("recovery_rescan", A.fk(A.SIDE_PARK_Q)["tcp"], "joint", 0)
        ws2 = snapshot_detect(env, cfg, T)
        tr.end()
        if ws2["peg"] is None or not peg_upright_ok(ws2["peg"]):
            outcome, reason = "recovery_failed", "peg not visible or not standing upright after re-scan"
            say(f"   {reason}"); attempts[-1]["note"] = reason
            break
        peg = ws2["peg"]
        plan = plan_pick_place(peg, hole, ws["board_z"])
        if plan is None:
            outcome, reason = "recovery_failed", "no feasible plan for the new peg position"
            say(f"   {reason}"); break

    # ---- final verification: park aside, new snapshot, is the peg in the socket?
    tr.abort_enabled = False
    if outcome == "success":
        tr.begin("return_park", A.fk(A.SIDE_PARK_Q)["tcp"], "joint", SEG_IDX["return_park"])
        env.move_joints(A.SIDE_PARK_Q, 2.0); env.settle(0.6)
        tr.end()
    tr.close()
    final = snapshot_detect(env, cfg, T)
    fpeg = final["peg"]
    verdict = dict(outcome=outcome)
    if fpeg is not None:
        err_xy = float(np.hypot(fpeg["x"] - hole["x"], fpeg["y"] - hole["y"]))
        standing = abs(fpeg["z_top"] - (final["board_z"] + peg["height"])) < 0.010
        verdict.update(vision_peg_xy_error_to_socket_mm=err_xy * 1000, vision_peg_standing_on_board=bool(standing),
                       vision_in_socket=bool(err_xy < 0.008 and standing))
    else:
        verdict.update(vision_in_socket=False, note="peg not visible in the final snapshot")
    gp = env.gt_body_pos("peg_circle"); hx, hy = env.layout.hole_xy("circle")      # simulator truth: reporting only
    verdict["SIM_ONLY_true_peg_error_to_socket_mm"] = float(np.hypot(gp[0] - hx, gp[1] - hy) * 1000)
    verdict["SIM_ONLY_true_peg_tilt_deg"] = env.gt_body_up_tilt_deg("peg_circle")
    verdict["success"] = bool(verdict["vision_in_socket"] and verdict["SIM_ONLY_true_peg_tilt_deg"] < 10)
    cv2.imwrite(os.path.join(d, "final_check_overhead.png"), _final_overlay(final, hole, fpeg, env, T, verdict))
    cv2.imwrite(os.path.join(d, "final_check_oblique.png"), env.render_view("oblique_view"))

    # ---- logs / plots
    tr.save_csv(os.path.join(d, "tracking_log.csv"))
    if tr.recs:
        viz.tracking_plot(os.path.join(d, "tracking_plot.png"), tr.recs, tr.seg_bounds)
    executed = [(r["tcp_x"], r["tcp_y"]) for r in tr.recs if r["tcp_x"] is not None and r["seg"] not in ("move_to_home", "return_park", "recovery_rescan", "-")]
    viz.topdown_map(os.path.join(d, "path_topdown.png"), ws, plan=plan, executed=executed, reach=None,
                    title="Planned vs camera-measured TCP path (top view)")
    if ckpts:
        viz.storyboard(os.path.join(d, "storyboard.png"), ckpts[:9])
    log["pick_place"] = dict(attempts=attempts, final=verdict, tracking=tr.summary(), plan_tilt_deg=plan.tilt_deg)
    jdump(os.path.join(d, "pick_place_result.json"), log["pick_place"])
    s = tr.summary()
    if s:
        say(f"   camera tracking: {s['pct_on_track']:.1f}% of frames ON_TRACK, mean error {s['track_err_mean_mm']:.1f} mm "
            f"(p95 {s['track_err_p95_mm']:.1f}, max {s['track_err_max_mm']:.1f}), cross-track max {s['cross_err_max_mm']:.1f} mm, "
            f"direction cos mean {s['dir_cos_mean']:.2f}")
    say(f"   final: peg-to-socket error (vision) {verdict.get('vision_peg_xy_error_to_socket_mm', float('nan')):.1f} mm -> "
        f"{'SUCCESS' if verdict['success'] else 'FAILED'}  (sim truth: {verdict['SIM_ONLY_true_peg_error_to_socket_mm']:.1f} mm, tilt {verdict['SIM_ONLY_true_peg_tilt_deg']:.1f} deg)")


def _final_overlay(final, hole, fpeg, env, T, verdict):
    img = final["bgr"].copy()
    uv = viz.project_robot_pts(env.cam, T, np.array([hole["x"], hole["y"], final["board_z"]]))[0].astype(int)
    cv2.drawMarker(img, tuple(uv), (0, 255, 255), cv2.MARKER_CROSS, 22, 2)
    if fpeg is not None:
        cv2.circle(img, (int(fpeg["centroid_px"][0]), int(fpeg["centroid_px"][1])), 14, (0, 255, 0), 2)
    ok = verdict["success"]
    viz.put(img, f"FINAL CHECK: {'PEG IN SOCKET' if ok else 'NOT IN SOCKET'}", (10, 24), 0.6, (60, 255, 60) if ok else (60, 60, 255), 2)
    viz.put(img, f"vision error to socket: {verdict.get('vision_peg_xy_error_to_socket_mm', float('nan')):.1f} mm (yellow cross = socket)", (10, 46), 0.45)
    return img


# =============================================================================================
def write_summary(run_dir, log, args, elapsed):
    L = ["# Demo run summary", "", f"* run folder: `{os.path.basename(run_dir)}`",
         f"* command line: `{' '.join(sys.argv)}`", f"* wall-clock time: {elapsed:.0f} s", ""]
    h = log.get("home")
    if h:
        L += ["## 1. Home pose", f"* encoders (rad): {np.round(h['joint_encoders_rad'], 3).tolist()}  -> L pose: **{h['expected_L_pose']}**",
              f"* TCP from arm dimensions: {np.round(h['tcp_from_arm_dimensions_m'], 3).tolist()} m (model vs simulator: {h['fk_vs_sim_mm']:.3f} mm)", ""]
    c = log.get("calibration")
    if c:
        e = c["SIM_ONLY_error_vs_true_camera_pose"]
        L += ["## 2a. Camera <-> robot calibration", f"* marker detected in {c['poses_detected']}/{c['poses_attempted']} poses, used {c['poses_used']}",
              f"* fit residual {c['rms_residual_mm']:.2f} mm, reprojection {c['mean_reprojection_px']:.2f} px",
              f"* (simulation-only) error vs the true camera pose: {e['translation_mm']:.2f} mm / {e['rotation_deg']:.3f} deg", ""]
    w = log.get("workspace")
    if w:
        L += ["## 2b. Workspace analysis",
              f"* shapes found on board (top->bottom): {[x['shape'] for x in w['holes']]}",
              f"* circle peg at ({w['peg']['x']*100:.1f}, {w['peg']['y']*100:.1f}) cm, {w['distances_from_base_m']['peg']*100:.1f} cm from the base",
              f"* circle socket at ({w['circle_hole']['x']*100:.1f}, {w['circle_hole']['y']*100:.1f}) cm, {w['distances_from_base_m']['circle_hole']*100:.1f} cm from the base",
              f"* vertical top-grasp reach at grasp height: {w['max_vertical_reach_at_grasp_height_m']*100:.1f} cm",
              f"* **chosen constant tool tilt: {w['verdict']['chosen_tilt_deg']} deg**"]
        for n in (w["verdict"]["notes"] or []):
            L.append(f"  * {n}")
        L.append("")
    p = log.get("pick_place")
    if p:
        f, t = p["final"], p["tracking"]
        L += ["## 3. Pick & place", f"* attempts: {[(a['attempt'], a['outcome']) for a in p['attempts']]}",
              f"* **result: {'SUCCESS' if f['success'] else 'FAILED'}** - vision error peg->socket {f.get('vision_peg_xy_error_to_socket_mm', float('nan')):.1f} mm"]
        if t:
            L += [f"* camera-verified tracking: {t['pct_on_track']:.1f}% ON_TRACK, mean {t['track_err_mean_mm']:.1f} mm, p95 {t['track_err_p95_mm']:.1f} mm, max {t['track_err_max_mm']:.1f} mm",
                  f"* cross-track error: mean {t['cross_err_mean_mm']:.1f} mm, max {t['cross_err_max_mm']:.1f} mm; motion direction vs plan (cosine) mean {t['dir_cos_mean']:.2f}",
                  f"* marker visible in {t['marker_seen_pct']:.0f}% of frames"]
        L.append("")
    L += ["## Files", "* `01_home_pose/`, `02_calibration/`, `03_workspace_analysis/`, `04_pick_and_place/` - images, JSON, CSV, plots, video",
          "* `run_log.json` - everything in machine-readable form", "* `config_snapshot/` - the exact layout/vision settings used"]
    with open(os.path.join(run_dir, "summary.md"), "w") as f:
        f.write("\n".join(L) + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--until", choices=["home", "workspace", "pick"], default="pick")
    ap.add_argument("--peg-xy", type=float, nargs=2, metavar=("X", "Y"), help="peg position in the robot frame (m)")
    ap.add_argument("--seed", type=int, default=7, help="seed for the random peg position")
    ap.add_argument("--out-root", default=os.path.join(HERE, "..", "demo_runs"))
    ap.add_argument("--calib-poses", type=int, default=16)
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--depth-noise-mm", type=float, default=1.5, help="RealSense-like depth noise at 0.6 m")
    ap.add_argument("--inject-miss", action="store_true", help="test recovery: move the peg after the snapshot so the first grasp misses")
    ap.add_argument("--layout", default=None, help="alternative sim_layout.yaml")
    args = ap.parse_args()

    t0 = time.time()
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    base = os.path.abspath(os.path.join(args.out_root, f"demo_{stamp}"))
    run_dir, k = base, 1
    while os.path.exists(run_dir):                 # never overwrite an earlier run
        k += 1; run_dir = f"{base}_{k}"
    os.makedirs(run_dir)
    cfg = load_vision_cfg()
    layout = load_layout(args.layout) if args.layout else load_layout()
    snap = os.path.join(run_dir, "config_snapshot"); os.makedirs(snap, exist_ok=True)
    yaml.safe_dump(layout, open(os.path.join(snap, "sim_layout.yaml"), "w"))
    yaml.safe_dump(cfg, open(os.path.join(snap, "vision.yaml"), "w"))
    peg_xy = tuple(args.peg_xy) if args.peg_xy else default_peg_xy(args.seed)
    say(f"run folder: {run_dir}\npeg placed (ground truth, hidden from the pipeline) at x={peg_xy[0]:.3f} y={peg_xy[1]:.3f} m")

    env = SimEnv(layout_cfg=layout, peg_xy=peg_xy, depth_noise_m=args.depth_noise_mm / 1000.0)
    open(os.path.join(snap, "scene.xml"), "w").write(env.xml)
    log = dict(peg_xy_true=peg_xy, args=vars(args))
    try:
        stage_home(env, os.path.join(run_dir, "01_home_pose"), log)
        if args.until != "home":
            T = stage_calibration(env, cfg, os.path.join(run_dir, "02_calibration"), log, args.calib_poses)
            env.place_peg("circle")        # the instructor now puts the peg on the mat (position unknown to the pipeline)
            say("   >> circle peg placed on the mat (table was clear during calibration)")
            ws, plan, hole, peg, _ = stage_workspace(env, cfg, T, os.path.join(run_dir, "03_workspace_analysis"), log)
            if args.until == "pick":
                stage_pick_place(env, cfg, T, ws, plan, hole, peg, os.path.join(run_dir, "04_pick_and_place"), log, not args.no_video, inject_miss=args.inject_miss)
    finally:
        jdump(os.path.join(run_dir, "run_log.json"), log)
        write_summary(run_dir, log, args, time.time() - t0)
    say(f"\ndone in {time.time()-t0:.0f} s  ->  {run_dir}\n   open summary.md first")
    return run_dir


if __name__ == "__main__":
    main()

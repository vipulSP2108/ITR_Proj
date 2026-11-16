"""
Vision-based path verification.

At ~25 Hz the mounted camera looks at the arm.  The coloured tool marker is detected, lifted to 3-D with
the depth image, moved into the robot frame with the calibrated transform and converted to a TCP estimate.
Every number reported (tracking error, cross-track error, direction cosine, progress) is computed from
those camera measurements against the planned path - nothing is hard-coded or read from the simulator.
"""
import csv
import numpy as np
import cv2

import arm_model as A
from perception import detect_marker, detect_peg, detect_green_peg, to_robot
from scene_builder import MARKER_R
import viz

WARN_MM = 10.0            # base tolerance (mm); grows with commanded speed to allow for servo lag
ABORT_MM = 40.0
LAG_S = 0.12              # expected servo lag: tolerance += LAG_S * commanded speed
ABORT_CONSEC = 4


class AbortMotion(Exception):
    pass


class PathTracker:
    def __init__(self, env, cfg, T_robot_cam, mat_z, path_pts, video_path=None, obs_every=2, fps=25.0, shape="circle"):
        self.env, self.cfg, self.T, self.mat_z = env, cfg, T_robot_cam, mat_z
        self.shape = shape
        self.path_pts = path_pts
        self.every, self.k = obs_every, 0
        self.seg = None
        self.recs = []
        self.seg_bounds = []
        self.prev_est = None; self.prev_plan = None; self.prev_goal_dist = []
        self.bad = 0
        self.enabled = True
        self.abort_enabled = True
        self.last_frame = None; self.last_oblique = None
        self.writer = None
        if video_path:
            self.writer = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (1280, 480))
        env.frame_hooks.append(self)

    # ---------------------------------------------------------------------------
    def begin(self, name, p_goal, kind, seg_index):
        p_start = A.fk(self.env.read_joints())["tcp"]
        self.seg = dict(name=name, start=p_start, goal=np.asarray(p_goal, float), kind=kind, idx=seg_index)
        self.seg_bounds.append((name, self.env.t, None))
        self.prev_est = None; self.prev_plan = None; self.prev_goal_dist = []

    def end(self):
        if self.seg_bounds and self.seg_bounds[-1][2] is None:
            n, t0, _ = self.seg_bounds[-1]
            self.seg_bounds[-1] = (n, t0, self.env.t)

    # ---------------------------------------------------------------------------
    def __call__(self, env):
        if not self.enabled:
            return
        self.k += 1
        if self.k % self.every:
            return
        self.observe()

    def observe(self):
        env = self.env
        bgr, depth = env.capture_rgbd()
        obl = env.render_view("oblique_view")
        enc = env.read_joints(); fk = A.fk(enc)
        planned = A.fk(env.ctrl_q)["tcp"]
        det = detect_marker(bgr, depth, env.cam, self.cfg, MARKER_R)
        peg = detect_peg(bgr, depth, env.cam, self.cfg, self.T, self.mat_z, shape=self.shape)
        rec = dict(t=env.t, seg=self.seg["name"] if self.seg else "-", marker_seen=det is not None,
                   e_track_mm=None, e_cross_mm=None, progress=None, dir_cos=None, closing=None,
                   status="MARKER_LOST", peg_visible=peg is not None, peg_x=None, peg_y=None, peg_ztop=None,
                   tcp_x=None, tcp_y=None, tcp_z=None, plan_x=planned[0], plan_y=planned[1], plan_z=planned[2])
        est = None
        if self.prev_plan is not None:
            rec["_v"] = float(np.linalg.norm(planned - self.prev_plan) / (self.every * 0.02) * 1000.0)   # mm/s
        else:
            rec["_v"] = None
        if det is not None:
            marker_r = to_robot(det["P_cam"][None], self.T)[0]
            est = marker_r + (fk["tcp"] - fk["marker"])          # marker (vision) + marker->TCP offset (encoders)
            rec.update(tcp_x=est[0], tcp_y=est[1], tcp_z=est[2])
            rec["e_track_mm"] = float(np.linalg.norm(est - planned) * 1000)
            s = self.seg
            if s is not None:
                a, b = s["start"], s["goal"]; ab = b - a; L = np.linalg.norm(ab)
                if s["kind"] == "line" and L > 5e-3:
                    u = ab / L; w = est - a
                    prog = float(np.clip(w @ u / L, -0.2, 1.2)); cross = np.linalg.norm(w - (w @ u) * u)
                    rec["progress"] = prog; rec["e_cross_mm"] = float(cross * 1000)
                gd = float(np.linalg.norm(est - b)); self.prev_goal_dist.append(gd)
                if len(self.prev_goal_dist) >= 4:
                    rec["closing"] = bool(self.prev_goal_dist[-1] <= self.prev_goal_dist[-4] + 0.002)
            if self.prev_est is not None and self.prev_plan is not None:
                dm, dp = est - self.prev_est, planned - self.prev_plan
                if np.linalg.norm(dp) > 0.0015 and np.linalg.norm(dm) > 0.0007:
                    rec["dir_cos"] = float(dm @ dp / (np.linalg.norm(dm) * np.linalg.norm(dp)))
            self.prev_est = est
            v_cmd = 0.0 if rec.get("_v") is None else rec["_v"]
            tol = WARN_MM + LAG_S * v_cmd
            bad = rec["e_track_mm"] > tol or (rec["dir_cos"] is not None and rec["dir_cos"] < 0.5)
            rec["status"] = "DEVIATING" if bad else "ON_TRACK"
            rec["tol_mm"] = tol
            self.bad = self.bad + 1 if rec["e_track_mm"] > ABORT_MM + LAG_S * v_cmd else 0
            if self.bad >= ABORT_CONSEC and self.abort_enabled:
                rec["status"] = "ABORT"
        if peg is not None:
            rec.update(peg_x=peg["x"], peg_y=peg["y"], peg_ztop=peg["z_top"])
        self.prev_plan = planned.copy()
        rec["cmd_speed_mm_s"] = rec.pop("_v")
        self.recs.append(rec)
        frame = self.annotate(bgr, rec, est, planned, peg)
        self.last_frame, self.last_oblique = frame, obl
        if self.writer is not None:
            viz.put(obl, "oblique view (human reference, not used by the pipeline)", (10, 20), 0.45, (220, 220, 220))
            self.writer.write(np.hstack([frame, obl]))
        if rec["status"] == "ABORT" and self.abort_enabled:
            raise AbortMotion(f"tracking error {rec['e_track_mm']:.1f} mm for {ABORT_CONSEC} consecutive frames")

    def annotate(self, bgr, rec, est, planned, peg):
        img = bgr.copy(); cam = self.env.cam
        cur = self.seg["idx"] if self.seg else None
        viz.draw_path_overlay(img, cam, self.T, self.path_pts, cur)
        up = viz.project_robot_pts(cam, self.T, planned)[0].astype(int)
        cv2.circle(img, tuple(up), 5, (0, 255, 255), 1, cv2.LINE_AA)           # commanded TCP (yellow ring)
        if est is not None:
            ue = viz.project_robot_pts(cam, self.T, est)[0].astype(int)
            cv2.drawMarker(img, tuple(ue), (255, 255, 0), cv2.MARKER_CROSS, 12, 2)   # camera-estimated TCP (cyan cross)
            cv2.line(img, tuple(up), tuple(ue), (60, 60, 255), 1)
        if peg is not None:
            cv2.circle(img, (int(peg["centroid_px"][0]), int(peg["centroid_px"][1])), int(max(6, peg["radius"] * cam.fx / 0.5)), (0, 255, 0), 1, cv2.LINE_AA)
        col = viz.status_color(rec["status"])
        cv2.rectangle(img, (6, 6), (330, 112), (20, 20, 20), -1)
        cv2.rectangle(img, (6, 6), (330, 112), col, 1)
        f = lambda v, fmt: "n/a" if v is None else fmt % v
        viz.put(img, f"segment: {rec['seg']}   t={rec['t']:.1f}s", (12, 24), 0.45)
        viz.put(img, f"status: {rec['status']}", (12, 44), 0.5, col, 1)
        viz.put(img, f"track err {f(rec['e_track_mm'], '%.1f')} mm | cross {f(rec['e_cross_mm'], '%.1f')} mm", (12, 64), 0.42)
        viz.put(img, f"dir cos {f(rec['dir_cos'], '%.2f')} | closing goal: {rec['closing']}", (12, 82), 0.42)
        viz.put(img, f"peg: {'seen' if rec['peg_visible'] else 'occluded/not seen'}", (12, 100), 0.42, (120, 255, 120))
        return img

    # ---------------------------------------------------------------------------
    def close(self):
        self.enabled = False
        if self.writer is not None:
            self.writer.release(); self.writer = None

    def save_csv(self, path):
        if not self.recs:
            return
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(self.recs[0].keys()))
            w.writeheader(); w.writerows(self.recs)

    def summary(self):
        r = [x for x in self.recs if x["e_track_mm"] is not None]
        if not r:
            return {}
        e = np.array([x["e_track_mm"] for x in r])
        c = np.array([x["e_cross_mm"] for x in r if x["e_cross_mm"] is not None])
        d = np.array([x["dir_cos"] for x in r if x["dir_cos"] is not None])
        st = [x["status"] for x in self.recs]
        return dict(n_obs=len(self.recs), marker_seen_pct=100.0 * len(r) / len(self.recs),
                    track_err_mean_mm=float(e.mean()), track_err_p95_mm=float(np.percentile(e, 95)), track_err_max_mm=float(e.max()),
                    cross_err_mean_mm=float(c.mean()) if len(c) else None, cross_err_max_mm=float(c.max()) if len(c) else None,
                    dir_cos_mean=float(d.mean()) if len(d) else None, dir_cos_frac_ok=float((d > 0.7).mean()) if len(d) else None,
                    pct_on_track=100.0 * st.count("ON_TRACK") / len(st), n_deviating=st.count("DEVIATING"),
                    n_marker_lost=st.count("MARKER_LOST"))

"""Drawing helpers: camera overlays, top-down map, tracking plots, storyboard."""
import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon

FONT = cv2.FONT_HERSHEY_SIMPLEX


def put(img, text, org, scale=0.45, color=(255, 255, 255), thick=1):
    cv2.putText(img, text, org, FONT, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
    cv2.putText(img, text, org, FONT, scale, color, thick, cv2.LINE_AA)


def project_robot_pts(env_cam, T_robot_cam, P):
    Tinv = np.linalg.inv(T_robot_cam)
    P = np.atleast_2d(P)
    pc = (Tinv[:3, :3] @ P.T).T + Tinv[:3, 3]
    return env_cam.project(pc)


def draw_path_overlay(img, cam, T, path_pts, current_idx=None):
    uv = project_robot_pts(cam, T, np.array(path_pts)).astype(int)
    for i in range(len(uv) - 1):
        col = (0, 255, 255) if (current_idx is not None and i == current_idx) else (0, 170, 170)
        cv2.line(img, tuple(uv[i]), tuple(uv[i + 1]), col, 2 if col == (0, 255, 255) else 1, cv2.LINE_AA)
    for p in uv:
        cv2.circle(img, tuple(p), 3, (0, 200, 255), -1)


def status_color(status):
    return {"ON_TRACK": (60, 220, 60), "DEVIATING": (40, 140, 255), "MARKER_LOST": (180, 180, 180),
            "ABORT": (50, 50, 255)}.get(status, (255, 255, 255))


def topdown_map(path, ws, plan=None, executed=None, reach=None, title="Workspace (robot frame, top view)"):
    """ws: workspace dict (surfaces/holes/peg). +X to the LEFT so it matches the overhead camera view."""
    fig, ax = plt.subplots(figsize=(8.2, 6.4))
    ax.set_aspect("equal"); ax.invert_xaxis()
    if reach is not None:
        xs, ys, G = reach
        for tilt, col, lab in ((0, "#7ddc7d", "vertical grasp reach"), (30, "#f2e55c", "tilt <= 30 deg reach"), (60, "#ffb066", "tilt <= 60 deg reach")):
            Z = np.where(np.isnan(G), 0, (G <= tilt).astype(float))
            ax.contourf(xs, ys, Z, levels=[0.5, 1.5], colors=[col], alpha=0.35)
            ax.plot([], [], color=col, lw=6, alpha=0.6, label=lab)
    for key, col in (("mat", "#2fb585"), ("board", "#e9b27d")):
        s = ws["surfaces"].get(key)
        if s:
            (x0, x1), (y0, y1) = s["x_range"], s["y_range"]
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec=col, lw=2, label=key))
    for h in ws["holes"]:
        ax.plot(h["x"], h["y"], "o", color="#cc2b2b" if h["shape"] == "circle" else "#a05a5a", ms=9 if h["shape"] == "circle" else 6, mfc="none")
        ax.annotate(h["shape"], (h["x"], h["y"]), textcoords="offset points", xytext=(6, 5), fontsize=8)
    if ws.get("peg"):
        p = ws["peg"]
        ax.add_patch(Circle((p["x"], p["y"]), p["radius"], color="#27b52f", alpha=0.8))
        ax.annotate("circle peg", (p["x"], p["y"]), textcoords="offset points", xytext=(8, -12), fontsize=8)
    ax.plot(0, 0, "ks", ms=12); ax.annotate("robot base", (0, 0), textcoords="offset points", xytext=(6, 8), fontsize=8)
    if plan is not None:
        pts = np.array([w.p for w in plan.waypoints])
        ax.plot(pts[:, 0], pts[:, 1], "-o", color="#2255cc", lw=1.5, ms=3, label=f"planned TCP path (tilt {plan.tilt_deg:.0f} deg)")
    if executed is not None and len(executed) > 1:
        e = np.array(executed)
        ax.plot(e[:, 0], e[:, 1], "-", color="#e8590c", lw=1.2, label="TCP measured by camera")
    ax.set_xlabel("X forward [m]  (board is on the left, like the camera image)"); ax.set_ylabel("Y [m]")
    ax.set_title(title); ax.grid(alpha=0.25); ax.legend(loc="upper right", fontsize=7)
    ax.set_xlim(0.45, -0.12); ax.set_ylim(0.25, -0.25)   # +Y down like the image
    fig.tight_layout(); fig.savefig(path, dpi=130); plt.close(fig)


def tracking_plot(path, recs, seg_bounds):
    t = np.array([r["t"] for r in recs])
    fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    et = np.array([r["e_track_mm"] if r["e_track_mm"] is not None else np.nan for r in recs])
    ec = np.array([r["e_cross_mm"] if r["e_cross_mm"] is not None else np.nan for r in recs])
    dc = np.array([r["dir_cos"] if r["dir_cos"] is not None else np.nan for r in recs])
    tol = np.array([r.get("tol_mm") if r.get("tol_mm") is not None else np.nan for r in recs])
    axs[0].plot(t, et, color="#e8590c", label="error"); axs[0].plot(t, tol, color="gray", ls="--", lw=0.9, label="allowed (10 mm + servo lag x speed)")
    axs[0].legend(fontsize=7, loc="upper right"); axs[0].set_ylabel("tracking error [mm]\n(camera TCP vs commanded)")
    axs[1].plot(t, ec, color="#2255cc"); axs[1].set_ylabel("cross-track error [mm]\n(distance to planned line)")
    axs[2].plot(t, dc, color="#2b9348"); axs[2].set_ylabel("direction cosine\n(motion vs plan)"); axs[2].set_ylim(-1.1, 1.1)
    axs[2].axhline(0.7, color="gray", ls="--", lw=0.8)
    for name, t0, t1 in seg_bounds:
        for a in axs:
            a.axvline(t0, color="k", alpha=0.15, lw=0.8)
        axs[0].text(t0, axs[0].get_ylim()[1] * 0.92, name, rotation=90, fontsize=6, va="top")
    axs[2].set_xlabel("simulation time [s]")
    fig.suptitle("Vision-based path verification"); fig.tight_layout(); fig.savefig(path, dpi=120); plt.close(fig)


def storyboard(path, images, cols=3, size=(426, 320)):
    imgs = [cv2.resize(im, size) for im in images]
    while len(imgs) % cols:
        imgs.append(np.zeros_like(imgs[0]))
    rows = [np.hstack(imgs[i:i + cols]) for i in range(0, len(imgs), cols)]
    cv2.imwrite(path, np.vstack(rows))

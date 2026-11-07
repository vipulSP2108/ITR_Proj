"""
MuJoCo simulation environment: arm control, mounted-camera RGB-D capture, ground-truth accessors.

Ground-truth accessors (gt_*) exist ONLY for validating the pipeline (calibration error, final
placement check).  The perception / planning code never calls them.
"""
import os, sys, time
if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")      # headless Linux; macOS uses its native context
import numpy as np
import cv2
import mujoco

import arm_model as A
from scene_builder import build_scene_xml, load_layout, Layout, MAT_TOP, BOARD_TOP, default_peg_xy

CTRL_DT = 0.02          # control period (s); physics runs at 0.002 s
GRIP_OPEN = 0.0
PEG_PARK_XY = (-0.35, -0.40)   # on the table, outside the camera view
GRIP_CLOSE = 0.025      # commanded finger travel (stalls on the peg -> grip force)


class Camera:
    """Pinhole intrinsics (OpenCV convention: x right, y down, z forward)."""
    def __init__(self, width, height, fovy_deg):
        self.w, self.h = width, height
        self.fy = (height / 2.0) / np.tan(np.radians(fovy_deg) / 2.0)
        self.fx = self.fy
        self.cx, self.cy = (width - 1) / 2.0, (height - 1) / 2.0
        self.K = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.0]])

    def deproject(self, u, v, z):
        """pixel(s) + depth(m) -> camera-frame 3-D point(s) (N,3)."""
        u, v, z = np.atleast_1d(u).astype(float), np.atleast_1d(v).astype(float), np.atleast_1d(z).astype(float)
        return np.stack([(u - self.cx) * z / self.fx, (v - self.cy) * z / self.fy, z], axis=1)

    def project(self, P):
        """camera-frame points (N,3) -> pixel (N,2)."""
        P = np.atleast_2d(P)
        return np.stack([P[:, 0] * self.fx / P[:, 2] + self.cx, P[:, 1] * self.fy / P[:, 2] + self.cy], axis=1)


class SimEnv:
    def __init__(self, layout_cfg=None, pegs=("circle",), peg_xy=None, seed=7,
                 depth_noise_m=0.0015, rng_seed=0, park_peg=True,
                 viewer=False, pace_fps=60.0):
        self.layout_cfg = layout_cfg or load_layout()
        self.layout = Layout(self.layout_cfg)
        self.xml = build_scene_xml(self.layout_cfg, pegs=pegs, peg_xy=peg_xy, seed=seed)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        c = self.layout_cfg["camera"]
        self.cam = Camera(c["width"], c["height"], c["fovy_deg"])
        self.depth_noise = depth_noise_m
        self.rng = np.random.default_rng(rng_seed)
        self.pegs = list(pegs)
        self.peg_target_xy = (peg_xy if peg_xy is not None else default_peg_xy(seed))
        self.peg_h = self.layout_cfg['peg_height_m']
        self.park_peg = park_peg

        m = self.model
        self.qadr = [m.joint(f"joint{i}").qposadr[0] for i in range(1, 5)]
        self.gadr = [m.joint("grip_left").qposadr[0], m.joint("grip_right").qposadr[0]]
        self._sub = int(round(CTRL_DT / m.opt.timestep))
        self.rgb_r = mujoco.Renderer(m, self.cam.h, self.cam.w)
        self.dep_r = mujoco.Renderer(m, self.cam.h, self.cam.w)
        self.dep_r.enable_depth_rendering()
        self.ctrl_q = np.zeros(4)
        self.ctrl_g = GRIP_OPEN
        self.t = 0.0
        self.frame_hooks = []        # callables(env) invoked once per control step
        self.viewer = None
        self.opencv_viewer = False
        self.pace_fps = pace_fps
        self._step_count = 0
        self.reset()
        if viewer:
            self.open_viewer()

    def open_viewer(self):
        """Launches live MuJoCo passive 3D viewer, or falls back to OpenCV HUD."""
        try:
            import mujoco.viewer
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.cam.lookat[:] = [0.15, 0.0, 0.05]
            self.viewer.cam.distance = 0.70
            self.viewer.cam.elevation = -28.0
            self.viewer.cam.azimuth = 135.0
            print("  [MuJoCo Viewer] Interactive 3D viewer opened.")
        except Exception as exc:
            print(f"  [MuJoCo Viewer] Native viewer unavailable ({exc}); activating OpenCV HUD window.")
            self.opencv_viewer = True
            cv2.namedWindow("MuJoCo Live Simulation", cv2.WINDOW_AUTOSIZE)

    def close_viewer(self, linger_s: float = 0.0):
        if linger_s > 0 and (self.viewer is not None or self.opencv_viewer):
            t_end = time.time() + linger_s
            while time.time() < t_end:
                if self.viewer is not None and self.viewer.is_running():
                    self.viewer.sync()
                    time.sleep(0.02)
                elif self.opencv_viewer:
                    cv2.waitKey(20)
                else:
                    break
        if self.viewer is not None:
            try:
                self.viewer.close()
            except Exception:
                pass
            self.viewer = None
        if self.opencv_viewer:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
            self.opencv_viewer = False

    def _sync_opencv_viewer(self):
        self._step_count += 1
        if self._step_count % 2 != 0:
            return
        bgr, _ = self.capture_rgbd()
        obl = self.render_view("oblique_view")
        cv2.putText(obl, "3D Oblique View", (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (240, 240, 240), 1)
        cv2.putText(bgr, "Mounted Camera View", (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 255, 120), 1)
        combo = np.hstack([bgr, obl])
        cv2.imshow("MuJoCo Live Simulation", combo)
        cv2.waitKey(1)

    # ------------------------------------------------------------------ basic control
    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.ctrl_q = A.HOME_Q.copy(); self.ctrl_g = GRIP_OPEN
        self.data.qpos[self.qadr] = self.ctrl_q
        self._apply_ctrl()
        mujoco.mj_forward(self.model, self.data)
        if self.park_peg:
            self.place_peg("circle", PEG_PARK_XY, z0=0.0)       # table is clear until the 'instructor' places the peg
        for _ in range(100):
            mujoco.mj_step(self.model, self.data)
        self.t = 0.0

    def place_peg(self, name="circle", xy=None, z0=MAT_TOP):
        """Teleport a peg upright onto the surface at xy (the 'instructor puts the peg on the mat')."""
        xy = self.peg_target_xy if xy is None else xy
        j = self.model.joint(f"jnt_peg_{name}")
        a, v = j.qposadr[0], j.dofadr[0]
        self.data.qpos[a:a + 3] = [xy[0], xy[1], z0 + self.peg_h / 2 + 0.0005]
        self.data.qpos[a + 3:a + 7] = [1, 0, 0, 0]
        self.data.qvel[v:v + 6] = 0
        mujoco.mj_forward(self.model, self.data)
        for _ in range(150):
            mujoco.mj_step(self.model, self.data)
        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()

    def _apply_ctrl(self):
        self.data.ctrl[0:4] = self.ctrl_q
        self.data.ctrl[4] = -self.ctrl_g
        self.data.ctrl[5] = +self.ctrl_g

    def step(self, n_ctrl=1):
        for _ in range(n_ctrl):
            self._apply_ctrl()
            for _ in range(self._sub):
                mujoco.mj_step(self.model, self.data)
            self.t += CTRL_DT
            for h in self.frame_hooks:
                h(self)
            if self.viewer is not None and self.viewer.is_running():
                self.viewer.sync()
                if self.pace_fps > 0:
                    time.sleep(1.0 / self.pace_fps)
            elif self.opencv_viewer:
                self._sync_opencv_viewer()

    def read_joints(self):
        """Joint encoder readout (what the real Dynamixels would report)."""
        return self.data.qpos[self.qadr].copy()

    def read_gripper(self):
        return float(self.data.qpos[self.gadr[1]] - self.data.qpos[self.gadr[0]])   # finger travel sum (m)

    def move_joints(self, q_target, duration=1.5):
        q0 = self.ctrl_q.copy(); q1 = np.asarray(q_target, float)
        n = max(2, int(duration / CTRL_DT))
        for i in range(1, n + 1):
            s = i / n; a = 10 * s**3 - 15 * s**4 + 6 * s**5          # minimum-jerk
            self.ctrl_q = q0 + a * (q1 - q0)
            self.step()

    def move_cartesian(self, p0, p1, tilt, duration=1.5, q_start=None):
        """Straight TCP line p0->p1 (robot frame), constant tool tilt, IK at every control step."""
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        n = max(2, int(duration / CTRL_DT))
        prev = self.ctrl_q.copy() if q_start is None else q_start
        for i in range(1, n + 1):
            s = i / n; a = 10 * s**3 - 15 * s**4 + 6 * s**5
            p = p0 + a * (p1 - p0)
            q = A.ik(*p, tilt=tilt)
            if q is None:
                raise RuntimeError(f"IK failed on Cartesian line at {p}, tilt {np.degrees(tilt):.0f} deg")
            if np.max(np.abs(q - prev)) > 0.25:
                raise RuntimeError("IK branch jump on Cartesian line")
            self.ctrl_q = q; prev = q
            self.step()

    def set_gripper(self, close: bool, duration=0.8):
        self.ctrl_g = GRIP_CLOSE if close else GRIP_OPEN
        self.step(int(duration / CTRL_DT))

    def settle(self, seconds=0.5):
        self.step(int(seconds / CTRL_DT))

    # ------------------------------------------------------------------ mounted camera
    def capture_rgbd(self, camera="overhead_realsense"):
        """Returns (bgr uint8 HxWx3, depth float32 HxW in metres, with RealSense-like noise)."""
        self.rgb_r.update_scene(self.data, camera=camera)
        rgb = self.rgb_r.render()
        self.dep_r.update_scene(self.data, camera=camera)
        depth = self.dep_r.render().astype(np.float32)
        depth[depth > 5.0] = 0.0
        if self.depth_noise > 0:
            noise = self.rng.normal(0.0, self.depth_noise, depth.shape).astype(np.float32) * (depth / 0.6) ** 2
            depth = np.where(depth > 0, depth + noise, 0).astype(np.float32)
        return rgb[:, :, ::-1].copy(), depth

    def render_view(self, camera):
        self.rgb_r.update_scene(self.data, camera=camera)
        return self.rgb_r.render()[:, :, ::-1].copy()

    # ------------------------------------------------------------------ ground truth (validation only)
    def gt_cam_pose(self, camera="overhead_realsense"):
        """4x4 robot<-camera(OpenCV) transform from the simulator (for validating calibration only)."""
        cid = self.model.camera(camera).id
        R_mj = self.data.cam_xmat[cid].reshape(3, 3)
        R = R_mj @ np.diag([1.0, -1.0, -1.0])
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = self.data.cam_xpos[cid]
        return T

    def gt_body_pos(self, name):
        return self.data.xpos[self.model.body(name).id].copy()

    def gt_body_up_tilt_deg(self, name):
        R = self.data.xmat[self.model.body(name).id].reshape(3, 3)
        return float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))

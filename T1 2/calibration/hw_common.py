"""Shared hardware helpers for the calibration scripts (camera via ffmpeg + gentle arm moves).

Camera note (macOS): OpenCV VideoCapture is blocked by camera permissions, ffmpeg is not.
All scripts must therefore be started from a real terminal (Terminal.app / IDE terminal).
"""
import os, sys, time, subprocess
import cv2
import numpy as np

T1_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if T1_DIR not in sys.path:
    sys.path.insert(0, T1_DIR)

from src.control.dynamixel_wrapper import OpenManipulatorController  # noqa: E402

CAM_DEVICE = "Intel"          # AVFoundation name fragment of the RealSense RGB stream
CAM_W, CAM_H, CAM_FPS = 1280, 720, 15
ARM_IDS = [11, 12, 13, 14]
GRIPPER_ID = 15


class LiveCamera:
    """Persistent ffmpeg process streaming raw BGR frames; read() returns the newest frame."""

    def __init__(self, device=CAM_DEVICE, width=CAM_W, height=CAM_H, fps=CAM_FPS):
        import threading
        self.w, self.h = width, height
        self._frame = None
        self._count = 0
        self._lock = threading.Lock()
        # The frame rates the RealSense offers depend on the USB link (USB 3: 30/15 fps, USB 2: only
        # 6 fps at 1280x720). The picture geometry is the same, so take the fastest one that works.
        device = self._colour_device(device)
        self.fps = None
        for rate in dict.fromkeys((30, fps, 10, 6)):
            cmd = ["ffmpeg", "-nostdin", "-loglevel", "error", "-f", "avfoundation", "-pixel_format", "uyvy422",
                   "-fflags", "nobuffer", "-framerate", str(rate), "-video_size", f"{width}x{height}", "-i", device,
                   "-f", "rawvideo", "-pix_fmt", "bgr24", "-"]
            self._count = 0
            self._proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                          stderr=subprocess.DEVNULL, bufsize=width * height * 3)
            self._alive = True
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()
            t0 = time.time()
            while self._count == 0 and self._proc.poll() is None and time.time() - t0 < 6:
                time.sleep(0.1)
            if self._count > 0:
                self.fps = rate
                break
            self._alive = False
            self._proc.kill()
            self._thread.join(timeout=1)
        if self.fps is None:
            raise RuntimeError(f"Camera '{device}' produced no frames at {width}x{height}. Run from a terminal with "
                               f"camera permission and check the RealSense USB cable (a USB 3 port gives more frame rates).")
        if self.fps < 15:
            print(f"[Info] camera runs at only {self.fps} fps (slow USB link) - works, just a little slower")
        # The RealSense stream changes exposure/crop during its first seconds: wait until
        # about 4 s of video have arrived and two frames 0.5 s apart look the same.
        t0, prev = time.time(), None
        while time.time() - t0 < 15:
            time.sleep(0.5)
            if self._count < 4 * self.fps:
                continue
            cur = self._frame[::8, ::8].astype(np.int16)
            if prev is not None and np.abs(cur - prev).mean() < 2.0:
                break
            prev = cur

    @staticmethod
    def _colour_device(name):
        """Exact AVFoundation name of the RealSense COLOUR stream.

        The camera shows up as several devices ("... RGB", "... Depth"). On a slow (USB 2) link macOS
        may list only the Depth/infra-red one, whose grey dotted picture is useless for finding pegs.
        """
        out = subprocess.run(["ffmpeg", "-hide_banner", "-f", "avfoundation", "-list_devices", "true", "-i", ""],
                             capture_output=True, text=True).stderr
        video = out.split("AVFoundation audio devices")[0]
        names = [ln.split("] ", 2)[-1].strip() for ln in video.splitlines() if "] [" in ln]
        mine = [n for n in names if name.lower() in n.lower()]
        # the product is called "... Depth Camera 435i <stream>": only the LAST word tells the stream
        colour = ([n for n in mine if n.split()[-1].lower() == "rgb"]
                  or [n for n in mine if n.split()[-1].lower() not in ("depth", "infrared")])
        if colour:
            return colour[0]
        raise RuntimeError(
            "The RealSense colour camera is not available"
            + (f" - macOS only lists: {mine}. " if mine else " - no RealSense found at all. ")
            + "Unplug the camera and plug it into a USB 3 port with its USB 3 cable (directly into the Mac or the same hub "
              "socket as before), wait 5 s and run again. Check with:  ffmpeg -f avfoundation -list_devices true -i \"\"")

    def _loop(self):
        n = self.w * self.h * 3
        buf = bytearray(n)
        view = memoryview(buf)
        out = self._proc.stdout
        while self._alive:
            got = 0
            while got < n:
                k = out.readinto(view[got:])
                if not k:
                    self._alive = False
                    return
                got += k
            frame = np.frombuffer(buf, np.uint8).reshape(self.h, self.w, 3).copy()
            with self._lock:
                self._frame = frame
                self._count += 1
                self._stamp = time.time()

    def read(self, fresh=True, timeout=3.0):
        """Return a BGR frame. fresh=True waits for 2 frames captured after the call."""
        if fresh:
            start = self._count
            t0 = time.time()
            while self._count < start + 2 and time.time() - t0 < timeout and self._alive:
                time.sleep(0.01)
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def read_avg(self, n=4):
        """Average n fresh frames (reduces sensor noise for frame differencing)."""
        acc = None
        for _ in range(n):
            f = self.read().astype(np.float32)
            acc = f if acc is None else acc + f
        return (acc / n).astype(np.uint8)

    def release(self):
        self._alive = False
        try:
            self._proc.kill()
        except Exception:
            pass


def connect_arm(profile_vel=40, profile_acc=10):
    arm = OpenManipulatorController()
    if not arm.connect():
        raise RuntimeError("Cannot open U2D2 serial port")
    for jid in ARM_IDS + [GRIPPER_ID]:
        ok, _ = arm.ping_motor(jid)
        if not ok:
            arm.disconnect()
            raise RuntimeError(f"Motor {jid} does not answer - check 12V power. Run calibration/00_check_arm.py")
    arm.initialize_all_joints(profile_vel=profile_vel, profile_acc=profile_acc)
    setup_gripper(arm)
    # Base servo is multi-turn: after a power cycle / hand move its counter can sit one turn off.
    # Work relative to the nearest whole turn so the base never unwinds more than half a turn.
    p11 = arm.read_position(11)
    arm.base_turn = int(round(p11 / 4096.0)) * 4096
    if arm.base_turn:
        print(f"[Info] base counter offset by {arm.base_turn} counts (handled automatically)")
    return arm


def read_joints(arm, retries=3):
    """Raw counts of joints 11-14 (base is signed multi-turn)."""
    out = []
    for jid in ARM_IDS:
        p = None
        for _ in range(retries):
            p = arm.read_position(jid)
            if p is not None:
                break
            time.sleep(0.02)
        if p is None:
            raise RuntimeError(f"Cannot read joint {jid}")
        out.append(p - getattr(arm, "base_turn", 0) if jid == 11 else p)
    return out


def move_joints(arm, target4, max_step=25, dt=0.04, settle=0.6, tol=12):
    """Linear joint-space interpolation from the present position to target4 (raw counts, joints 11-14).

    Speed is bounded by max_step counts per dt (25/0.04 s ~ 55 deg/s max on the largest-moving joint).
    Returns the final measured raw counts.
    """
    cur = read_joints(arm)
    target4 = [int(round(t)) for t in target4]
    target4[0] = max(-1990, min(1990, target4[0]))
    for i, (lo, hi) in enumerate(((1024, 3072), (1024, 3072), (900, 3300)), start=1):
        target4[i] = max(lo, min(hi, target4[i]))
    n = max(1, int(np.ceil(max(abs(t - c) for t, c in zip(target4, cur)) / max_step)))
    for s in range(1, n + 1):
        a = s / n
        a = a * a * (3 - 2 * a)                       # smoothstep ease in/out
        for jid, c, t in zip(ARM_IDS, cur, target4):
            g = int(round(c + (t - c) * a))
            if jid == 11:
                g = (g + getattr(arm, "base_turn", 0)) & 0xFFFFFFFF
                arm.packet_handler.write4ByteTxRx(arm.port_handler, 11, 116, g)
            else:
                arm.write_position(jid, g)
        time.sleep(dt)
    t0 = time.time()
    while time.time() - t0 < 4.0:
        now = read_joints(arm)
        if max(abs(a - b) for a, b in zip(now, target4)) <= tol:
            break
        time.sleep(0.05)
    time.sleep(settle)
    return read_joints(arm)


# ---------------------------------------------------------------- gripper ----
# Gripper (rack): fully OPEN near 3250 counts, fingers touch (CLOSED, stall) near 1385.
# Measured 2026-11-10 with fresh camera frames. It is driven in current-based position
# mode (5) so that closing on a peg stalls gently instead of overloading the servo.
GRIP_OPEN = 3250
GRIP_CLOSED = 1420            # just before the fingers touch
GRIP_SQUEEZE = 1250           # target when grasping: stalls on the object with limited force
GRIP_CURRENT = 160            # x2.69 mA  (~430 mA) grip force limit


def setup_gripper(arm, current=GRIP_CURRENT):
    pk, ph = arm.packet_handler, arm.port_handler
    pk.write1ByteTxRx(ph, GRIPPER_ID, 64, 0)
    pk.write1ByteTxRx(ph, GRIPPER_ID, 11, 5)
    pk.write2ByteTxRx(ph, GRIPPER_ID, 102, int(current))
    pk.write4ByteTxRx(ph, GRIPPER_ID, 108, 30)
    pk.write4ByteTxRx(ph, GRIPPER_ID, 112, 120)
    pk.write1ByteTxRx(ph, GRIPPER_ID, 64, 1)


def gripper_pos(arm):
    v = arm.packet_handler.read4ByteTxRx(arm.port_handler, GRIPPER_ID, 132)[0]
    return v - 0x100000000 if v >= 0x80000000 else v


def gripper_goto(arm, raw, timeout=2.5):
    """Command the gripper (mode 5). Returns final position - if it is far from `raw` the fingers stalled on something."""
    arm.packet_handler.write4ByteTxRx(arm.port_handler, GRIPPER_ID, 116, int(raw) & 0xFFFFFFFF)
    t0, last, still = time.time(), None, 0
    while time.time() - t0 < timeout:
        time.sleep(0.1)
        p = gripper_pos(arm)
        if abs(p - raw) < 15:
            break
        still = still + 1 if (last is not None and abs(p - last) < 4) else 0
        if still >= 3:
            break
        last = p
    return gripper_pos(arm)


# ------------------------------------------------------- calibrated motion ----
def load_calib():
    import yaml
    with open(os.path.join(T1_DIR, "config", "camera_robot_calib.yaml")) as f:
        return yaml.safe_load(f)


def ik_cmd(x, y, z, down_deg=90.0, tool_len=None, calib=None, min_down_deg=30.0):
    """Raw joint command that puts a tool point at robot (x, y, z) mm.

    Tries down_deg first and tilts the tool towards horizontal in 5 deg steps if out of reach.
    Subtracts the joint zero/sag offsets found by the calibration. Returns (raw4, used_down_deg)
    or (None, None).
    """
    from src.planning import omx_kinematics as K
    import math
    tool_len = K.L4 if tool_len is None else tool_len
    off = (calib or {}).get("joint_offsets_rad", [0.0, 0.0, 0.0])
    d = down_deg
    while d >= min_down_deg - 1e-6:
        a = K.ik(x, y, z, math.radians(d), tool_len)
        if a is not None:
            return K.angles_to_raw([a[0], a[1] - off[0], a[2] - off[1], a[3] - off[2]]), d
        d -= 5.0
    return None, None

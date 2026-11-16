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


# ------------------------------------------------------------------ motion ----
# The OpenMANIPULATOR servos run with a TIME-BASED profile (Drive Mode bit 2): Profile Velocity
# (addr 112) is the duration of a move in ms and Profile Acceleration (108) the ramp time in ms.
# Smooth motion = ONE goal per move for all four joints in one packet (sync write), with a
# duration long enough for the biggest joint and ramps at both ends. Paths (lines, carry
# routes) are streamed: a smooth minimum-jerk reference sampled at 50 Hz with the servo
# profile switched off, so the joints follow one continuous motion instead of start-stop steps.
ADDR_DRIVE_MODE, ADDR_PROFILE_ACC, ADDR_GOAL, ADDR_PRESENT = 10, 108, 116, 132
COUNTS_PER_S_PER_VEL_UNIT = 0.229 * 4096 / 60.0          # velocity-based profile units (if ever used)
COUNTS_PER_S2_PER_ACC_UNIT = 214.577 * 4096 / 3600.0
LIMITS = {12: (1024, 3072), 13: (1024, 3072), 14: (900, 3300)}


def _safety():
    import yaml
    with open(os.path.join(T1_DIR, "config", "robot_safety.yaml")) as f:
        c = yaml.safe_load(f)
    k = 4096.0 / 360.0
    fold = [CENTER_COUNT + int(round(a * k)) for a in c["fold_angles_deg"]]
    return {"base_min": int(round(c["base_min_deg"] * k)), "base_max": int(round(c["base_max_deg"] * k)),
            "big_turn": int(round(c["big_turn_deg"] * k)), "fold": fold, "scan_base": int(round(c["scan_base_deg"] * k)),
            "base_min_deg": c["base_min_deg"], "base_max_deg": c["base_max_deg"]}


CENTER_COUNT = 2048
SAFE = _safety()
FOLD_RAW = SAFE["fold"]                                  # shoulder, elbow, wrist counts of the folded-up arm
SCAN_RAW = [SAFE["scan_base"]] + FOLD_RAW                 # scan pose: folded, turned away from the camera stand
ADDR_POSITION_I_GAIN = 82
POSITION_I_GAIN = 100  # factory default 0 lets shoulder/elbow hang 1-3 deg low under gravity (7-10 mm at the tip); 100 removes it within ~1 s
SYNC_GAP_S = 0.005    # sync writes get no reply: the servos drop a second packet sent less than ~3 ms after the first


def connect_arm(profile_vel=None, profile_acc=None):
    from dynamixel_sdk import GroupSyncWrite, GroupSyncRead
    arm = OpenManipulatorController()
    if not arm.connect():
        raise RuntimeError("Cannot open U2D2 serial port")
    for jid in ARM_IDS + [GRIPPER_ID]:
        ok, _ = arm.ping_motor(jid)
        if not ok:
            arm.disconnect()
            raise RuntimeError(f"Motor {jid} does not answer - check 12V power. Run calibration/00_check_arm.py")
    pk, ph = arm.packet_handler, arm.port_handler
    arm.time_based = all(pk.read1ByteTxRx(ph, j, ADDR_DRIVE_MODE)[0] & 4 for j in ARM_IDS)
    arm.initialize_all_joints(profile_vel=1500 if arm.time_based else 30, profile_acc=500 if arm.time_based else 8)
    for j in (12, 13, 14):                                      # integral gain: removes the gravity sag
        pk.write2ByteTxRx(ph, j, ADDR_POSITION_I_GAIN, POSITION_I_GAIN)
    arm.gw_goal = GroupSyncWrite(ph, pk, ADDR_GOAL, 4)
    arm.gw_prof = GroupSyncWrite(ph, pk, ADDR_PROFILE_ACC, 8)       # acceleration (108) + velocity (112)
    arm.gr_pos = GroupSyncRead(ph, pk, ADDR_PRESENT, 4)
    for j in ARM_IDS:
        arm.gr_pos.addParam(j)
    # Base servo is multi-turn: after a power cycle / hand move its counter can sit one turn off.
    # Work relative to the nearest whole turn so the base never unwinds more than half a turn.
    p11 = arm.read_position(11)
    arm.base_turn = int(round(p11 / 4096.0)) * 4096
    if arm.base_turn:
        print(f"[Info] base counter offset by {arm.base_turn} counts (handled automatically)")
    setup_gripper(arm)
    return arm


def _le32(v):
    v = int(v) & 0xFFFFFFFF
    return [v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF]


def _clamp(target4):
    t = [int(round(v)) for v in target4]
    t[0] = max(SAFE["base_min"], min(SAFE["base_max"], t[0]))   # never towards the camera stand
    for i, j in enumerate(ARM_IDS[1:], start=1):
        lo, hi = LIMITS[j]
        t[i] = max(lo, min(hi, t[i]))
    return t


def sync_goal(arm, target4):
    """All four joint goals in ONE packet, so the joints start together."""
    arm.gw_goal.clearParam()
    for j, g in zip(ARM_IDS, target4):
        arm.gw_goal.addParam(j, _le32(g + (arm.base_turn if j == 11 else 0)))
    arm.gw_goal.txPacket()
    time.sleep(SYNC_GAP_S)


def set_profiles(arm, acc, vel):
    """acc / vel: one value for all joints, or a list per joint (time-based: ms, velocity-based: units)."""
    acc = acc if isinstance(acc, (list, tuple)) else [acc] * 4
    vel = vel if isinstance(vel, (list, tuple)) else [vel] * 4
    arm.gw_prof.clearParam()
    for j, a, v in zip(ARM_IDS, acc, vel):
        arm.gw_prof.addParam(j, _le32(a) + _le32(v))
    arm.gw_prof.txPacket()
    time.sleep(SYNC_GAP_S)


def read_joints(arm, retries=3):
    """Raw counts of joints 11-14 (base relative to its whole-turn offset)."""
    from dynamixel_sdk import COMM_SUCCESS
    for _ in range(retries):
        if arm.gr_pos.txRxPacket() == COMM_SUCCESS and all(arm.gr_pos.isAvailable(j, ADDR_PRESENT, 4) for j in ARM_IDS):
            out = []
            for j in ARM_IDS:
                v = arm.gr_pos.getData(j, ADDR_PRESENT, 4)
                v = v - 0x100000000 if v >= 0x80000000 else v
                out.append(v - arm.base_turn if j == 11 else v)
            return out
        time.sleep(0.02)
    raise RuntimeError("Cannot read the joint positions (check the 12 V supply and the cable)")


def _wait_reached(arm, target4, tol, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        now = read_joints(arm)
        if max(abs(a - b) for a, b in zip(now, target4)) <= tol:
            return now
        time.sleep(0.03)
    return read_joints(arm)


def move_joints(arm, target4, max_step=25, dt=0.04, settle=0.6, tol=12):
    """Smooth, synchronised point-to-point move to target4 (raw counts, joints 11-14).

    Peak speed of the biggest-moving joint ~ max_step/dt counts/s (25/0.04 = 625 counts/s = 55 deg/s).
    All joints start and finish together with gentle acceleration and braking. A big base turn is
    always done with the arm folded up first (config/robot_safety.yaml). Returns the final counts.
    """
    cur = read_joints(arm)
    target4 = _clamp(target4)
    if abs(target4[0] - cur[0]) > SAFE["big_turn"]:
        if max(abs(a - b) for a, b in zip(cur[1:], FOLD_RAW)) > 40:
            _move_direct(arm, [cur[0]] + FOLD_RAW, max_step, dt, 0.2, tol)          # 1) fold up where it is
        _move_direct(arm, [target4[0]] + FOLD_RAW, max_step, dt, 0.2, tol)          # 2) turn, folded
        cur = read_joints(arm)
    return _move_direct(arm, target4, max_step, dt, settle, tol)                  # 3) unfold to the target


def _move_direct(arm, target4, max_step, dt, settle, tol):
    cur = read_joints(arm)
    target4 = _clamp(target4)
    d = [abs(t - c) for t, c in zip(target4, cur)]
    speed = max_step / dt
    T = max(0.5, 1.5 * max(d) / speed)                    # trapezoid with 1/3 ramps -> peak = 1.5 x average
    if arm.time_based:
        set_profiles(arm, int(T * 1000 / 3), int(T * 1000))
    else:
        vel = [max(1, int(dj / (T * 2 / 3) / COUNTS_PER_S_PER_VEL_UNIT)) for dj in d]
        acc = [max(1, int(dj / (T * 2 / 3) / (T / 3) / COUNTS_PER_S2_PER_ACC_UNIT)) for dj in d]
        set_profiles(arm, acc, vel)
    sync_goal(arm, target4)
    time.sleep(T * 0.9)
    _wait_reached(arm, target4, tol, timeout=1.5 + T * 0.3)
    time.sleep(settle)
    return read_joints(arm)


def stream_path(arm, waypoints, speed=300.0, settle=0.4, tol=12):
    """One continuous, smooth motion through a list of raw-count waypoints (joints 11-14).

    The waypoints are joined by straight joint-space segments; progress along them follows a
    minimum-jerk time law (gentle start, gentle stop, no stops in between) at 50 Hz.
    speed = average speed of the biggest-moving joint in counts/s.
    """
    cur = read_joints(arm)
    pts = [np.array(cur, float)] + [np.array(_clamp(w), float) for w in waypoints]
    seg = [float(np.max(np.abs(b - a))) for a, b in zip(pts, pts[1:])]
    total = sum(seg)
    if total < 1:
        return cur
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    T = max(0.6, 1.875 * total / speed)                   # min-jerk peak speed is 1.875 x average
    set_profiles(arm, 0, 0)                               # profile off: the joints follow the 50 Hz reference
    try:
        t0, dtc = time.time(), 0.02
        while True:
            t = time.time() - t0
            tau = min(1.0, t / T)
            s = total * (10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5)
            k = min(len(seg) - 1, int(np.searchsorted(cum, s, side="right") - 1))
            f = 0.0 if seg[k] == 0 else (s - cum[k]) / seg[k]
            sync_goal(arm, [int(round(v)) for v in pts[k] + (pts[k + 1] - pts[k]) * min(1.0, max(0.0, f))])
            if tau >= 1.0:
                break
            time.sleep(max(0.0, dtc - ((time.time() - t0) - t)))
    finally:
        set_profiles(arm, 500, 1500) if arm.time_based else set_profiles(arm, 8, 30)
    target = [int(round(v)) for v in pts[-1]]
    _wait_reached(arm, target, tol, timeout=1.5)
    time.sleep(settle)
    return read_joints(arm)


# ---------------------------------------------------------------- gripper ----
# The gripper is a CRANK: starting at its end stop the fingers are closed, they open as the motor
# turns away from the stop, are widest about half way, and close again towards the other end stop
# (measured 2026-11-12 with calibration/04_gripper_sweep.py: closed at the stop 3270, widest at
# ~1520-1770, closed again near 0). The motor's turn counter restarts at power-up and whenever the
# mode is written, so at every start the gripper is driven gently to the end stop ("homing") and
# all numbers below are counted from it ("nominal counts": the end stop is 3300).
# Only the half between the end stop (closed) and the widest point is used.
GRIP_OPEN_STOP = 3300         # nominal count of the end stop the homing finds (fingers closed)
GRIP_WIDE = 1650              # widest opening
GRIP_OPEN = 1700              # "open": just short of the widest point
GRIP_CLOSED = 3290            # fingers fully together at the end stop (3250 still left a visible gap)
GRIP_SQUEEZE = 3550           # grasp target past the stop: the fingers stall on the peg with limited force
GRIP_MAX_GAP_MM = 70.0        # finger gap at the widest point (approximate; crank: gap ~ sin of the turn)
GRIP_HELD_LIMIT = 3240        # stopping short of this when squeezing = something is between the fingers
GRIP_CURRENT = 160            # x2.69 mA  (~430 mA) grip force limit
GRIP_HOME_CURRENT = 90        # gentler limit while homing
# Direction of the gripper motor (Drive Mode bit 0). Kept fixed so the homing always finds the same
# end stop; written at every start-up because other programs (ROBOTIS tools, ROS) may change it.
GRIPPER_REVERSE = True


def grip_counts_for_gap(gap_mm):
    """Nominal gripper count that opens the fingers to gap_mm (crank model)."""
    import math
    g = max(0.0, min(1.0, gap_mm / GRIP_MAX_GAP_MM))
    return int(round(GRIP_OPEN_STOP - (GRIP_OPEN_STOP - GRIP_WIDE) * (2 / math.pi) * math.asin(g)))


def gap_for_grip_counts(c):
    """Approximate finger gap (mm) at nominal count c."""
    import math
    t = (GRIP_OPEN_STOP - c) / (GRIP_OPEN_STOP - GRIP_WIDE)
    return GRIP_MAX_GAP_MM * math.sin(max(0.0, min(1.0, t)) * math.pi / 2)


def gripper_holding(c):
    """True if the fingers stopped on something when squeezing."""
    return c < GRIP_HELD_LIMIT


def _gripper_raw(arm):
    v = arm.packet_handler.read4ByteTxRx(arm.port_handler, GRIPPER_ID, 132)[0]
    return v - 0x100000000 if v >= 0x80000000 else v


def setup_gripper(arm, current=GRIP_CURRENT):
    pk, ph = arm.packet_handler, arm.port_handler
    pk.write1ByteTxRx(ph, GRIPPER_ID, 64, 0)
    dm = pk.read1ByteTxRx(ph, GRIPPER_ID, ADDR_DRIVE_MODE)[0]
    want = (dm & ~1) | (1 if GRIPPER_REVERSE else 0)          # set the direction explicitly every time
    if want != dm:
        pk.write1ByteTxRx(ph, GRIPPER_ID, ADDR_DRIVE_MODE, want)
        print(f"[Info] gripper direction setting corrected (drive mode {dm} -> {want})")
    pk.write1ByteTxRx(ph, GRIPPER_ID, 11, 5)
    tb = want & 4
    pk.write4ByteTxRx(ph, GRIPPER_ID, 108, 150 if tb else 30)
    pk.write4ByteTxRx(ph, GRIPPER_ID, 112, 600 if tb else 120)
    pk.write2ByteTxRx(ph, GRIPPER_ID, 102, GRIP_HOME_CURRENT)
    pk.write1ByteTxRx(ph, GRIPPER_ID, 64, 1)
    # homing: drive towards the opening end stop until the fingers stop moving
    arm.grip_offset = 0
    start = _gripper_raw(arm)
    pk.write4ByteTxRx(ph, GRIPPER_ID, 116, (start + 4200) & 0xFFFFFFFF)     # > full travel (~3400): always reaches the stop
    t0, last, still = time.time(), start, 0
    while time.time() - t0 < 9.0:
        time.sleep(0.1)
        p = _gripper_raw(arm)
        still = still + 1 if abs(p - last) < 4 else 0
        last = p
        if still >= 4:
            break
    stop = _gripper_raw(arm)
    arm.grip_offset = stop - GRIP_OPEN_STOP
    pk.write2ByteTxRx(ph, GRIPPER_ID, 102, int(current))
    gripper_goto(arm, GRIP_OPEN)
    print(f"[Info] gripper homed: end stop at raw {stop} (offset {arm.grip_offset:+d} counts)")


def gripper_pos(arm):
    """Gripper position in NOMINAL counts (see above)."""
    return _gripper_raw(arm) - getattr(arm, "grip_offset", 0)


def gripper_goto(arm, raw, timeout=2.5):
    """Command the gripper (nominal counts, mode 5). Returns the final position - if it is far from `raw`
    the fingers stalled on something."""
    off = getattr(arm, "grip_offset", 0)
    arm.packet_handler.write4ByteTxRx(arm.port_handler, GRIPPER_ID, 116, int(raw + off) & 0xFFFFFFFF)
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
def calib_path():
    """config/camera_robot_calib.yaml, or the quick-calibration file when CALIB=quick is set."""
    name = "camera_robot_calib_quick.yaml" if os.environ.get("CALIB") == "quick" else "camera_robot_calib.yaml"
    return os.path.join(T1_DIR, "config", name)


def load_calib():
    import yaml
    with open(calib_path()) as f:
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
    lat = float((calib or {}).get("tool_lateral_mm", 0.0))
    if lat:
        # the tool sits `lat` mm to the left of the arm's plane: turn the base so the TOOL lands on (x, y)
        R = math.hypot(x, y)
        if R <= abs(lat) + 1.0:
            return None, None
        r = math.sqrt(R * R - lat * lat)
        a1 = math.atan2(y, x) - math.atan2(lat, r)
        x, y = r * math.cos(a1), r * math.sin(a1)
    if not (SAFE["base_min_deg"] <= math.degrees(math.atan2(y, x)) <= SAFE["base_max_deg"]):
        return None, None                                  # towards the camera stand: not allowed
    d = down_deg
    while d >= min_down_deg - 1e-6:
        a = K.ik(x, y, z, math.radians(d), tool_len)
        if a is not None:
            return K.angles_to_raw([a[0], a[1] - off[0], a[2] - off[1], a[3] - off[2]]), d
        d -= 5.0
    return None, None

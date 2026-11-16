#!/usr/bin/env python3
"""No-hardware test of the 6-DOF controller (teleop + backtrack + auto-tangent + safety) on a simulated 6-axis arm."""
import sys
from pathlib import Path
if "-h" in sys.argv or "--help" in sys.argv:
    print(__doc__)
    sys.exit(0)
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from buzzwire_calib.teleop_core import PoseController, Limits, numeric_jacobian

from buzzwire_calib.simarm import fk, jac
from buzzwire_calib.simarm import Q_HOME as Q0

def run(ctrl, q, secs, twist=(0,)*6, dt=0.01, backtrack=False, stuck=False, hook=None):
    qmax = 0
    for k in range(int(secs/dt)):
        t = twist(k*dt) if callable(twist) else twist
        qd, info = ctrl.step(q, np.array(t, float), dt, backtrack=backtrack)
        assert np.all(np.isfinite(qd))
        qmax = max(qmax, np.abs(qd).max())
        if not stuck:
            q = q + qd*dt
    return q, info, qmax

def mk(**kw):
    lim = Limits(**{k: v for k, v in kw.items() if k in Limits.__dataclass_fields__})
    c = PoseController(fk, jac, lim, tool_axis=kw.get("tool_axis", (0,0,1)), plane_normal=(0,1,0), auto_tangent=kw.get("auto", False))
    c.sync(Q0); return c

ok = True
def check(name, cond, detail=""):
    global ok; ok &= bool(cond); print(("PASS " if cond else "FAIL ") + name, detail)

# 1 translation along +Y at 20 mm/s for 3 s, orientation must be held
c = mk(); p0 = fk(Q0)[:3,3]; R0 = fk(Q0)[:3,:3]
q, info, _ = run(c, Q0.copy(), 3.0, (0, .02, 0, 0, 0, 0))
d = fk(q)[:3,3] - p0; rot = np.degrees(np.linalg.norm(Rot.from_matrix(fk(q)[:3,:3] @ R0.T).as_rotvec()))
check("translate +Y 20mm/s x3s", abs(d[1]-0.06) < 0.004 and abs(d[0]) < 0.002 and abs(d[2]) < 0.002, f"moved {np.round(d*1000,1)} mm")
check("orientation held while translating", rot < 0.5, f"drift {rot:.3f} deg")

# 2 pitch about base Y (wire-plane normal) 15 deg/s x 2 s, TCP must stay put
c = mk(); q, info, _ = run(c, Q0.copy(), 2.0, (0,0,0, 0, np.radians(15), 0))
Rrel = Rot.from_matrix(fk(q)[:3,:3] @ R0.T).as_rotvec(); dp = np.linalg.norm(fk(q)[:3,3]-p0)
check("pitch 15deg/s x2s about base Y", abs(np.degrees(Rrel[1]) - (30 - np.degrees(np.radians(15)**2/(2*0.6)))) < 1.0 and dp < 0.003, f"pitch {np.degrees(Rrel[1]):.1f} deg, TCP drift {dp*1000:.2f} mm")

# 3 all 6 DOF at once
c = mk(); q, info, _ = run(c, Q0.copy(), 2.0, (.01, .01, .01, .1, .1, .1))
check("combined 6-DOF twist is stable", np.isfinite(q).all() and info["err_lin"] < 0.012, f"err {info['err_lin']*1000:.1f} mm")

# 4 workspace box
c = mk(box=[p0[0]-0.5, p0[0]+0.03, -1, 1, -1, 1]); q, info, _ = run(c, Q0.copy(), 3.0, (.04, 0, 0, 0, 0, 0))
check("workspace box stops motion", fk(q)[0,3] <= p0[0] + 0.0305, f"x travelled {(fk(q)[0,3]-p0[0])*1000:.1f} mm (limit 30)")

# 5 backtrack retraces the path
c = mk(); q, _, _ = run(c, Q0.copy(), 2.0, (0, .03, .02, 0, 0, 0)); far = np.linalg.norm(fk(q)[:3,3]-p0)
q, info, _ = run(c, q, 6.0, (0, .03, 0, 0, 0, 0), backtrack=True)
check("backtrack retraces to start", np.linalg.norm(fk(q)[:3,3]-p0) < 0.004, f"went {far*1000:.0f} mm out, ended {np.linalg.norm(fk(q)[:3,3]-p0)*1000:.1f} mm from start")

# 6 auto-tangent: move diagonally in the X-Z plane, tool axis should align with travel direction
tool_axis = (0, 0, 1)
c = mk(auto=True, tool_axis=tool_axis)
v = np.array([.015, 0, .015]); q, info, _ = run(c, Q0.copy(), 6.0, (*v, 0, 0, 0))
a = fk(q)[:3,:3] @ np.array(tool_axis); a[1] = 0; a /= np.linalg.norm(a); dd = v/np.linalg.norm(v)
ang = np.degrees(np.arccos(np.clip(abs(a @ dd), -1, 1)))
check("auto-tangent aligns tool axis with direction of travel", ang < 6, f"residual {ang:.1f} deg")

# 7 arm refuses to move (stuck/collision): leash keeps T_des close and lag_s grows
c = mk(); q, info, _ = run(c, Q0.copy(), 3.0, (.05, 0, 0, 0, 0, 0), stuck=True)
lead = np.linalg.norm(c.T_des[:3,3] - fk(Q0)[:3,3])
check("stuck arm: desired pose stays on the leash, lag detected", lead <= 0.0125 and info["lag_s"] > 2.0, f"lead {lead*1000:.1f} mm, lag {info['lag_s']:.1f}s")

# 8 near-singular: stretch the arm fully, push outward; nothing blows up, joint speed respected
Qs = np.array([0.0, -1.0, 0.02, -0.5, 1.57, 0.0]); c = mk(); c.sync(Qs)
q, info, qmax = run(c, Qs.copy(), 4.0, (.05, 0, 0, 0, 0, 0))
check("near singularity stays finite and within joint speed limit", qmax <= 0.3501, f"max joint vel {qmax:.3f} rad/s, sigma_min {info['sigma_min']:.3f}")

# 9 acceleration limit: step command must ramp (no instant velocity jump)
c = mk(); qd, info = c.step(Q0, np.array([.04,0,0,0,0,0]), 0.01)
check("acceleration limited (no jerk on key press)", np.linalg.norm(c.twist[:3]) <= 0.10*0.01 + 1e-9, f"v after 10ms = {np.linalg.norm(c.twist[:3])*1000:.2f} mm/s")

print("\nTELEOP CORE SELFTEST", "PASSED" if ok else "FAILED"); sys.exit(0 if ok else 1)

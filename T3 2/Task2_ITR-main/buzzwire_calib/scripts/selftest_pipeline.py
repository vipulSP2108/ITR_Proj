#!/usr/bin/env python3
"""No-hardware end-to-end test: synthetic wire-like segments -> merge_segments.py -> replay_trajectory.py --sim."""
import json, shutil, subprocess, sys
from pathlib import Path
if "-h" in sys.argv or "--help" in sys.argv:
    print(__doc__)
    sys.exit(0)
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from buzzwire_calib.simarm import fk, Q_HOME
from buzzwire_calib.trajectory import SegmentWriter, Waypoint

sess = Path("/tmp/pipeline_test_session"); shutil.rmtree(sess, ignore_errors=True)
T0 = fk(Q_HOME); p0 = T0[:3, 3]
x = np.linspace(0, 0.30, 600)
z = 0.02 * np.sin(2 * np.pi * x / 0.12)                     # ~18 mm radius bends, like the wire loops
pts = np.stack([p0[0] + x, np.full_like(x, p0[1]), p0[2] + z], 1)
tan = np.gradient(pts, axis=0); pitch = np.arctan2(tan[:, 2], tan[:, 0])
quats = [(Rot.from_euler("y", -p) * Rot.from_matrix(T0[:3, :3])).as_quat() for p in pitch]   # tool pitches with the wire
pieces = [(0, 220), (200, 420), (400, 600)]                  # 3 teaching segments with overlaps
for sid, (a, b) in enumerate(pieces):
    with SegmentWriter(sess / "segments", sid, T0) as w:
        for i in range(a, b):
            w.write(Waypoint(i * 0.05, *pts[i], *quats[i], *Q_HOME, contact=(sid == 1 and i == 300)))
def run(args):
    r = subprocess.run([sys.executable, *args], capture_output=True, text=True, timeout=240, cwd=ROOT)
    return r.stdout + r.stderr
print(run(["scripts/merge_segments.py", "--session", str(sess)]).split("MERGE COMPLETE")[-1][:260])
for speed in ("0.4", "1.0"):
    out = run(["scripts/replay_trajectory.py", "--sim", "--session", str(sess), "--speed", speed, "--yes"])
    line = [l for l in out.replace("\r", "\n").split("\n") if "DONE" in l or "ABORT" in l]
    print(f"speed {speed}:", line[-1] if line else out[-300:])
    st = json.loads((sess / "replay_state.json").read_text())
    ok = st.get("done") and st["max_err_mm"] < 6
    print("  ->", "PASS" if ok else "FAIL", f"(done={st.get('done')}, worst err {st.get('max_err_mm', -1):.2f} mm)")
    if not ok: sys.exit(1)
print("PIPELINE SELFTEST PASSED")

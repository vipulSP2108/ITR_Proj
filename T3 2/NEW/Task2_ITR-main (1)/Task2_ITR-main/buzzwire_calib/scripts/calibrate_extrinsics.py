#!/usr/bin/env python3
"""Hand-eye + camera-to-camera calibration from a collected session (no robot / ROS needed)."""
import argparse, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from buzzwire_calib.config import load_config
from buzzwire_calib.board import BoardSpec, CharucoBoard
from buzzwire_calib.session import SessionStore, find_latest_session
from buzzwire_calib import calibration as C
from buzzwire_calib.handeye import solve


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", default=None, help="session dir (default: latest in data/)")
    ap.add_argument("--config", default=None, help="default: <session>/config_used.yaml")
    ap.add_argument("--intrinsics-dir", default="results/intrinsics")
    ap.add_argument("--out", default=None)
    ap.add_argument("--no-reject", action="store_true")
    ap.add_argument("--exclude", type=int, nargs="*", default=[], help="capture ids to ignore")
    a = ap.parse_args()

    sdir = Path(a.session) if a.session else find_latest_session("data")
    if sdir is None:
        sys.exit("no session found")
    store = SessionStore(sdir)
    cfg = load_config(a.config or (sdir / "config_used.yaml" if (sdir / "config_used.yaml").exists() else None))
    board = CharucoBoard(BoardSpec.from_cfg(cfg["board"]))
    cams = list(cfg["cameras"])
    mounts = {c: cfg["cameras"][c]["mount"] for c in cams}
    bm = cfg["calibration"]["board_mount"]
    print(f"Session {sdir.name}: {store.state()['n_captures']} captures, matrices {sorted(store.state()['per_matrix'])}")
    print(f"Assumed setup: board_mount={bm}, camera mounts={mounts}   (edit in the config if wrong!)\n")

    intr = {c: C.get_intrinsics(c, store, board, Path(a.intrinsics_dir), cfg, print) for c in cams}
    obs, bp = C.build_observations(store, cfg, board, intr, print)
    obs = [o for o in obs if o.cap not in a.exclude]
    bp = {k: v for k, v in bp.items() if k[0] not in a.exclude}
    print(f"\n{len(obs)} usable camera observations. Solving...")
    sol = solve(obs, mounts, bm, reject=not a.no_reject)
    sd, si = C.direct_stereo(bp, cams), C.implied_stereo(sol, obs, cams)
    rep = C.build_report(sol, obs, sd, si, cfg)
    out = Path(a.out) if a.out else Path(cfg["paths"]["results_root"]) / sdir.name
    C.write_outputs(out, rep, sol, cfg)

    print("\n================ RESULT ================")
    for c, s in sol["stats"].items():
        print(f"{c} ({mounts[c]}): {s['n_used']} obs, residual {s['trans_rms_mm']:.2f} mm / {s['rot_rms_deg']:.3f} deg RMS")
    for c, d in rep["cameras"].items():
        k = "T_base_cam" if "T_base_cam" in d else "T_ee_cam"
        print(f"{c} {k}: xyz={[round(v,4) for v in d[k]['xyz_m']]} quat_xyzw={[round(v,4) for v in d[k]['quat_xyzw']]}")
    if "stereo_from_model" in rep:
        print(f"stereo {rep['stereo_from_model']['pair']} (from model): xyz={[round(v,4) for v in rep['stereo_from_model']['xyz_m']]}")
    if "stereo_consistency" in rep:
        print(f"model vs direct stereo agreement: {rep['stereo_consistency']['rot_deg']:.2f} deg / {rep['stereo_consistency']['trans_mm']:.1f} mm")
    for w in rep["warnings"]:
        print("WARNING:", w)
    bad = [p for p in sol["per_obs"] if not p["used"]]
    if bad:
        print("Rejected observations (re-capture these poses if you can):",
              sorted({(p['cap'], p['cam']) for p in bad}))
    print(f"\nSaved to {out}/ (extrinsics.json, per_observation_residuals.csv, publish_tf.sh)")


if __name__ == "__main__":
    main()

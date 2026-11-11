#!/usr/bin/env python3
"""
Step 1: Hardware test — capture camera, read joints, scan workspace.
Run this from YOUR terminal (not Claude) so macOS camera permission works:

    cd "T1 2" && python hw_test_step1.py
"""
import os, sys, cv2, time, math, json
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.control.dynamixel_wrapper import OpenManipulatorController
from src.planning.kinematics import OpenManipulatorKinematics
from src.vision.workspace_vision import WorkspaceVision

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hw_test_output")
os.makedirs(OUTPUT_DIR, exist_ok=True)

def main():
    print("=" * 60)
    print("  HARDWARE TEST STEP 1")
    print("=" * 60)

    # ─── 1. Arm ──────────────────────────────────────────────
    print("\n[1] Connecting to arm...")
    arm = OpenManipulatorController()
    if not arm.connect():
        print("FAILED: No arm connection")
        return
    arm.initialize_all_joints(profile_vel=50, profile_acc=15)

    joint_data = {}
    for jid in [11, 12, 13, 14, 15]:
        pos = arm.read_position(jid)
        deg = arm.read_position_deg(jid)
        load = arm.read_load(jid)
        joint_data[jid] = {"raw": pos, "deg": round(deg, 1) if deg else None, "load": load}
        print(f"  Joint {jid}: raw={pos}  deg={deg:.1f}°  load={load}")

    kin = OpenManipulatorKinematics()
    raw4 = [joint_data[11]["raw"], joint_data[12]["raw"], joint_data[13]["raw"], joint_data[14]["raw"]]
    rads = kin.raw_to_radians(raw4)
    fk = kin.forward_kinematics(*rads)
    print(f"\n  FK → TCP at ({fk[0]:.1f}, {fk[1]:.1f}, {fk[2]:.1f}) mm")
    print(f"  Horizontal reach: {math.sqrt(fk[0]**2 + fk[1]**2):.1f} mm")

    # ─── 2. Go HOME then SIDE PARK ──────────────────────────
    print("\n[2] Moving to HOME position [0, 2048, 2048, 2048, 3130]...")
    home = [0, 2048, 2048, 2048, 3130]
    for i, jid in enumerate([11, 12, 13, 14, 15]):
        arm.write_position(jid, home[i])
    time.sleep(3.0)

    # Read positions after home
    print("  Positions after HOME:")
    for jid in [11, 12, 13, 14, 15]:
        pos = arm.read_position(jid)
        print(f"    Joint {jid}: {pos}")

    print("\n[3] Moving to SIDE PARK [3072, 2048, 2048, 2048, 3130]...")
    side = [3072, 2048, 2048, 2048, 3130]
    # Smooth move: just joint 11 first, then others
    arm.write_position(11, 3072)
    time.sleep(2.5)

    # ─── 3. Camera ───────────────────────────────────────────
    print("\n[4] Opening camera...")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("  FAILED: Camera not available!")
        print("  Grant camera permission in System Settings > Privacy > Camera")
        arm.disconnect()
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    time.sleep(1.5)  # auto-exposure

    # Capture several frames
    for _ in range(15):
        ret, frame = cap.read()
        time.sleep(0.05)

    if not ret or frame is None:
        print("  FAILED: No frame captured!")
        cap.release()
        arm.disconnect()
        return

    raw_path = os.path.join(OUTPUT_DIR, "raw_frame.jpg")
    cv2.imwrite(raw_path, frame)
    print(f"  Saved raw frame: {raw_path}")
    print(f"  Frame size: {frame.shape[1]}x{frame.shape[0]}")

    # ─── 4. Vision Scan ──────────────────────────────────────
    print("\n[5] Running workspace scan...")
    config_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config')
    vision = WorkspaceVision(config_dir)

    scan = vision.full_scan(frame)

    print(f"\n  RESULTS:")
    print(f"  Pegs found: {len(scan.pegs)}")
    for p in scan.pegs:
        x, y = p.robot_xy
        r = math.sqrt(x**2 + y**2)
        print(f"    {p.shape:10s} ({p.color:6s})  pixel=({p.pixel_uv[0]:.0f},{p.pixel_uv[1]:.0f})  robot=({x:.1f},{y:.1f})  r={r:.0f}mm  yaw={p.yaw_deg:.1f}°  conf={p.confidence:.2f}")

    print(f"\n  Board ROI: {scan.board_roi}")
    print(f"  Sockets found: {len(scan.sockets)}")
    for s in scan.sockets:
        x, y = s.robot_xy
        print(f"    [{s.shape:10s}]  pixel=({s.pixel_uv[0]:.0f},{s.pixel_uv[1]:.0f})  robot=({x:.1f},{y:.1f})  yaw={s.yaw_deg:.1f}°")

    print(f"\n  Assignments: {list(scan.assignments.keys())}")
    for shape, (peg, sock) in scan.assignments.items():
        print(f"    {shape}: peg({peg.robot_xy[0]:.1f},{peg.robot_xy[1]:.1f}) → socket({sock.robot_xy[0]:.1f},{sock.robot_xy[1]:.1f})")

    # Save annotated frame
    if scan.annotated_frame is not None:
        ann_path = os.path.join(OUTPUT_DIR, "annotated_scan.jpg")
        cv2.imwrite(ann_path, scan.annotated_frame)
        print(f"\n  Saved annotated scan: {ann_path}")

    # ─── 5. Save summary ────────────────────────────────────
    summary = {
        "joints": {str(k): v for k, v in joint_data.items()},
        "fk_tcp": {"x": round(fk[0], 1), "y": round(fk[1], 1), "z": round(fk[2], 1)},
        "pegs": [{"shape": p.shape, "color": p.color, "pixel": list(p.pixel_uv), "robot": list(p.robot_xy), "yaw": p.yaw_deg} for p in scan.pegs],
        "sockets": [{"shape": s.shape, "pixel": list(s.pixel_uv), "robot": list(s.robot_xy), "yaw": s.yaw_deg} for s in scan.sockets],
        "board_roi": list(scan.board_roi) if scan.board_roi else None,
        "assignments": {k: {"peg_robot": list(v[0].robot_xy), "socket_robot": list(v[1].robot_xy)} for k, v in scan.assignments.items()},
        "vision_config": {"scale": vision.scale, "base_u": vision.base_u, "base_v": vision.base_v, "homography": vision.homography is not None},
    }
    summary_path = os.path.join(OUTPUT_DIR, "scan_summary.json")
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"  Saved summary: {summary_path}")

    # ─── 6. Show live window ────────────────────────────────
    print("\n[6] Showing live camera window (press 'q' to quit, 's' to re-scan)...")
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Draw base marker and reach circles
        bu, bv = int(vision.base_u), int(vision.base_v)
        cv2.circle(frame, (bu, bv), 5, (0, 0, 255), -1)
        for r_mm, label in [(215, "V"), (290, "A")]:
            r_px = int(r_mm / vision.scale)
            cv2.circle(frame, (bu, bv), r_px, (60, 60, 60), 1)

        # Draw detected pegs
        for p in scan.pegs:
            u, v = int(p.pixel_uv[0]), int(p.pixel_uv[1])
            cv2.circle(frame, (u, v), 8, (0, 255, 0), 2)
            cv2.putText(frame, p.shape, (u+10, v-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        # Draw sockets
        for s in scan.sockets:
            u, v = int(s.pixel_uv[0]), int(s.pixel_uv[1])
            cv2.circle(frame, (u, v), 8, (200, 200, 200), 2)
            cv2.putText(frame, f"[{s.shape}]", (u+10, v-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        cv2.putText(frame, "q=quit  s=re-scan", (10, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow("HW Test Step 1", frame)

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            print("  Re-scanning...")
            for _ in range(5):
                ret, frame = cap.read()
            if ret:
                scan = vision.full_scan(frame)
                print(f"  Pegs: {[p.shape for p in scan.pegs]}, Sockets: {[s.shape for s in scan.sockets]}")

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()

    # Return to home
    print("\n[7] Returning to HOME...")
    arm.write_position(11, 0)
    time.sleep(2.0)
    arm.disconnect()
    print("Done!")

if __name__ == "__main__":
    main()

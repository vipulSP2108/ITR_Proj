#!/usr/bin/env python3
"""Step 0: ping all 5 motors, print position / hardware-error / voltage. Moves nothing.

    cd "Project1 Task1" && python calibration/00_check_arm.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.control.dynamixel_wrapper import OpenManipulatorController, JOINT_IDS
from dynamixel_sdk import COMM_SUCCESS

ADDR_HW_ERROR, ADDR_VOLTAGE, ADDR_TORQUE, ADDR_MODE = 70, 144, 64, 11

def main():
    arm = OpenManipulatorController()
    if not arm.connect():
        print("FAIL: cannot open U2D2 port"); return 1
    ph, pk = arm.port_handler, arm.packet_handler
    ok_all = True
    for jid, name in JOINT_IDS.items():
        ok, model = arm.ping_motor(jid)
        if not ok:
            print(f"  ID {jid} {name}: NO RESPONSE  (check 12V power supply / cable)")
            ok_all = False
            continue
        pos = arm.read_position(jid)
        err, r1, _ = pk.read1ByteTxRx(ph, jid, ADDR_HW_ERROR)
        volt, r2, _ = pk.read2ByteTxRx(ph, jid, ADDR_VOLTAGE)
        tq, _, _ = pk.read1ByteTxRx(ph, jid, ADDR_TORQUE)
        mode, _, _ = pk.read1ByteTxRx(ph, jid, ADDR_MODE)
        flag = "" if err == 0 else "  <-- HARDWARE ERROR (power-cycle the arm)"
        if err != 0: ok_all = False
        print(f"  ID {jid} {name}: pos={pos} mode={mode} torque={tq} volt={volt/10:.1f}V hw_err={err:#04x}{flag}")
    arm.disconnect()
    print("ARM OK" if ok_all else "ARM NOT OK")
    return 0 if ok_all else 1

if __name__ == "__main__":
    sys.exit(main())

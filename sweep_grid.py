#!/usr/bin/env python3
"""
Launcher for OpenMANIPULATOR-X 12x12 Batch Grid Sweep (x: 0..11, y: 0..11).
Delegates directly to T1 2/sim_demo/batch_grid_sweep.py.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TARGET = os.path.join(HERE, "T1 2", "sim_demo", "batch_grid_sweep.py")

if __name__ == "__main__":
    if not os.path.exists(TARGET):
        print(f"Error: Target script not found at {TARGET}")
        sys.exit(1)
    # Execute batch_grid_sweep.py with all forwarded arguments
    cmd = [sys.executable, TARGET] + sys.argv[1:]
    os.execv(sys.executable, cmd)

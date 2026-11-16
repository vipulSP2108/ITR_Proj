import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pandas as pd
import json
import shutil
from pathlib import Path

# Paths
base_dir = Path('/Users/aai/Desktop/Task2/taken/G9/Task2_ITR-main/buzzwire_calib')
s_new = base_dir / 'data' / 'session_20261115_122231'
s_old = base_dir / 'data' / 'session_20261115_113441'

# Load new raw
df_raw = pd.read_csv(s_new / 'merged_trajectory.csv')
p_raw = df_raw[['x','y','z']].values * 1000

# Load calibrated
df_cal = pd.read_csv(s_new / 'calibrated_wire_LR.csv')
p_cal = df_cal[['x','y','z']].values * 1000

# Load metrics
with open(s_new / 'calibration_metrics.json') as f:
    metrics = json.load(f)

# Compute SVD plane
p_mean = np.mean(p_raw, axis=0)
p_raw_cent = p_raw - p_mean
u, s, vt = np.linalg.svd(p_raw_cent)
e_len = vt[0]
if e_len[1] < 0:
    e_len = -e_len
e_hgt = vt[1]
if e_hgt[2] < 0:
    e_hgt = -e_hgt
e_norm = np.cross(e_len, e_hgt)

L_raw = np.dot(p_raw_cent, e_len)
H_raw = np.dot(p_raw_cent, e_hgt)
N_raw = np.dot(p_raw_cent, e_norm)

p_cal_cent = p_cal - p_mean
L_cal = np.dot(p_cal_cent, e_len)
H_cal = np.dot(p_cal_cent, e_hgt)

s_cal = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(p_cal, axis=0), axis=1))])
s_raw = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(p_raw, axis=0), axis=1))])

# Create 4-panel figure
plt.style.use('dark_background')
fig = plt.figure(figsize=(18, 12), dpi=150)
fig.patch.set_facecolor('#0b0d13')

# 1. 3D Perspective Plot
ax1 = fig.add_subplot(2, 2, 1, projection='3d')
ax1.set_facecolor('#0b0d13')
ax1.plot(p_raw[:,0], p_raw[:,1], p_raw[:,2], color='#f59e0b', alpha=0.8, linewidth=1.5, label='Raw Recorded Waypoints (Tremor + Hunting)')
ax1.plot(p_cal[:,0], p_cal[:,1], p_cal[:,2], color='#10b981', linewidth=2.8, label='Calibrated & Averaged Fitted Profile')

# Plot touch point at index 87
ax1.scatter(p_raw[87,0], p_raw[87,1], p_raw[87,2], color='#ef4444', s=120, marker='*', zorder=10, label='Touch Event (idx 87, descent into V1)')

# Start / Goal
ax1.scatter(p_cal[0,0], p_cal[0,1], p_cal[0,2], color='#3b8bff', s=80, marker='o', label='Start Post (L)')
ax1.scatter(p_cal[-1,0], p_cal[-1,1], p_cal[-1,2], color='#f43f5e', s=80, marker='s', label='Goal Post (R)')

ax1.set_title('3D Trajectory in Robot Base Frame {B}', fontsize=12, fontweight='bold', pad=12, color='#eef1f8')
ax1.set_xlabel('X (Depth mm)', color='#9097aa')
ax1.set_ylabel('Y (Length mm)', color='#9097aa')
ax1.set_zlabel('Z (Height mm)', color='#9097aa')
ax1.legend(loc='upper right', fontsize=8, facecolor='#141721', edgecolor='#232838')
ax1.view_init(elev=24, azim=135)

# 2. 2D In-Plane Wire Profile
ax2 = fig.add_subplot(2, 2, 2)
ax2.set_facecolor('#141721')
ax2.plot(L_raw, H_raw, color='#f59e0b', alpha=0.7, linewidth=1.2, linestyle='--', label='Raw Jog Waypoints')
ax2.plot(L_cal, H_cal, color='#10b981', linewidth=2.5, label='Fitted Wire Centerline (B-Spline C2)')

# Clearance tube envelope (+/- 9mm)
ax2.fill_between(L_cal, H_cal - 9.0, H_cal + 9.0, color='#06b6d4', alpha=0.15, label='9.0mm Safe Clearance Tube')

# Mark touch
ax2.scatter(L_raw[87], H_raw[87], color='#ef4444', s=120, marker='*', zorder=10, label='Wire Contact (Collapsing pitch)')

# Extrema labels
extrema_indices = [0, 107, 195, 272, 370, 470, 563, 659, 721]
extrema_labels = ['Start L', 'V1', 'P1', 'V2', 'P2', 'V3', 'P3', 'V4', 'Goal R']
for idx, lab in zip(extrema_indices, extrema_labels):
    offset_y = 10 if 'P' in lab or 'Goal' in lab or 'Start' in lab else -16
    ax2.annotate(lab, (L_raw[idx], H_raw[idx]), textcoords='offset points', xytext=(0, offset_y),
                 ha='center', fontsize=8, fontweight='bold', color='#a5f3fc')

ax2.set_title('In-Plane Wire Profile: Recorded Waypoints vs Fitted Centerline', fontsize=12, fontweight='bold', color='#eef1f8')
ax2.set_xlabel('Wire Longitudinal Axis (mm)', color='#9097aa')
ax2.set_ylabel('Wire Vertical Axis (mm)', color='#9097aa')
ax2.grid(True, linestyle=':', alpha=0.3, color='#333b4f')
ax2.legend(loc='lower center', ncol=2, fontsize=8, facecolor='#141721', edgecolor='#232838')

# 3. Out of plane hunting
ax3 = fig.add_subplot(2, 2, 3)
ax3.set_facecolor('#141721')
ax3.plot(s_raw, N_raw, color='#f59e0b', linewidth=1.5, label='Raw Out-of-Plane Hunting (std: 3.25mm, max: 9.81mm)')
ax3.axhline(0, color='#10b981', linewidth=2.5, label='Calibrated Planar Centerline (0.00mm deviation)')
ax3.axhline(10.5, color='#ef4444', linestyle=':', alpha=0.7, label='+10.5mm Radial Clearance Boundary')
ax3.axhline(-10.5, color='#ef4444', linestyle=':', alpha=0.7, label='-10.5mm Radial Clearance Boundary')
ax3.fill_between(s_raw, -10.5, 10.5, color='#10b981', alpha=0.06)

ax3.set_title('Lateral Out-of-Plane Hunting: Human Tremor vs SVD Calibration', fontsize=12, fontweight='bold', color='#eef1f8')
ax3.set_xlabel('Path Arc Length (mm)', color='#9097aa')
ax3.set_ylabel('Off-Plane Deviation (mm)', color='#9097aa')
ax3.set_ylim(-14, 14)
ax3.grid(True, linestyle=':', alpha=0.3, color='#333b4f')
ax3.legend(loc='upper right', fontsize=8, facecolor='#141721', edgecolor='#232838')

# 4. Multi-Constraint Speed Profiles
ax4 = fig.add_subplot(2, 2, 4)
ax4.set_facecolor('#141721')
profiles = metrics.get('profiles', {})
colors = {
    'Fast Profile (Competition)': '#f43f5e',
    'Balanced Profile (Standard)': '#3b8bff',
    'Show Profile (Presentation)': '#10b981'
}
for name, p_data in profiles.items():
    v_prof = p_data.get('speeds_mmps', [])
    if v_prof:
        s_axis = np.linspace(0, s_cal[-1], len(v_prof))
        dur = p_data['duration_s']
        v_max = p_data['v_max_mmps']
        ax4.plot(s_axis, v_prof, color=colors.get(name, '#fff'), linewidth=2.0, label=f'{name} ({dur}s, v_max: {v_max}mm/s)')

# Raw crawl
ax4.axhline(1.0, color='#f59e0b', linestyle='--', linewidth=1.5, label='Actual Recorded Teleop Speed (~1.0 mm/s)')

# Burst points from old session seg_0000
ax4.scatter([180, 220, 250, 280], [22.3, 26.0, 24.2, 22.6], color='#a855f7', marker='^', s=50, zorder=5, label='Old Run Straight Burst Speeds (20-26 mm/s)')

ax4.set_title('Multi-Constraint Time-Optimal Speed Profiles v(s)', fontsize=12, fontweight='bold', color='#eef1f8')
ax4.set_xlabel('Path Arc Length (mm)', color='#9097aa')
ax4.set_ylabel('Traversing Velocity (mm/s)', color='#9097aa')
ax4.set_ylim(0, 55)
ax4.grid(True, linestyle=':', alpha=0.3, color='#333b4f')
ax4.legend(loc='upper right', fontsize=8, facecolor='#141721', edgecolor='#232838')

plt.suptitle('Buzz Wire Trajectory: Recorded Waypoints vs Calibrated & Averaged Fitted Profiles', fontsize=15, fontweight='bold', color='#ffffff', y=0.98)
plt.tight_layout(rect=[0, 0.03, 1, 0.96])

out_png = Path('/Users/aai/Desktop/Task2/taken/buzz_wire_waypoints_recorded_fit_profile.png')
plt.savefig(out_png, facecolor=fig.get_facecolor(), edgecolor='none')
print(f'Saved image to: {out_png}')

# Copy to brain artifact folder
dest = Path('/Users/aai/.gemini/antigravity/brain/4fc16566-e7dd-4b99-9df4-e9230b93a9a1/buzz_wire_waypoints_recorded_fit_profile.png')
shutil.copy(out_png, dest)
print(f'Copied to artifact directory: {dest}')

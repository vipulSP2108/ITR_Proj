#!/usr/bin/env python3
"""
Generates the comprehensive 3D Buzz Wire Calibration, Averaging & Simulation Dashboard.
Outputs: /Users/aai/Desktop/Task2/taken/buzz_wire_3d_calibration_and_averaging_dashboard.html
"""

import json
import numpy as np
import pandas as pd
from pathlib import Path

# Paths
base_dir = Path("/Users/aai/Desktop/Task2/taken/G9/Task2_ITR-main/buzzwire_calib")
data_dir = base_dir / "data" / "session_20261115_122231"
out_html = Path("/Users/aai/Desktop/Task2/taken/buzz_wire_3d_calibration_and_averaging_dashboard.html")

# 1. Load Data
df_raw = pd.read_csv(data_dir / "merged_trajectory.csv")
p_raw = df_raw[['x','y','z']].values * 1000.0  # in mm

df_std = pd.read_csv(data_dir / "calibrated_wire_LR.csv")
p_std = df_std[['x','y','z']].values * 1000.0  # in mm

df_sm = pd.read_csv(data_dir / "calibrated_wire_LR_smooth.csv")
p_sm = df_sm[['x','y','z']].values * 1000.0  # in mm

# Load metrics
with open(data_dir / "calibration_metrics.json") as f:
    metrics = json.load(f)

# Calculate out-of-plane hunting for raw
p_mean = np.mean(p_raw, axis=0)
p_raw_cent = p_raw - p_mean
u, s, vt = np.linalg.svd(p_raw_cent)
e_norm = vt[2]
if e_norm[0] < 0: e_norm = -e_norm
off_plane_raw = np.dot(p_raw_cent, e_norm).tolist()

# Subsample points for web rendering
sub_p_raw = np.round(p_raw[::2], 2).tolist()
sub_off_raw = np.round(off_plane_raw[::2], 2).tolist()
sub_p_std = np.round(p_std[::2], 2).tolist()
sub_p_sm = np.round(p_sm[::2], 2).tolist()

# Extrema coordinates in base frame
extrema_indices = [0, 107, 195, 272, 370, 470, 563, 659, 721]
extrema_names = ['Start L', 'Valley 1', 'Peak 1', 'Valley 2', 'Peak 2', 'Valley 3', 'Peak 3', 'Valley 4', 'Goal R']
extrema_points = [
    {"name": name, "pos": np.round(p_raw[min(idx, len(p_raw)-1)], 2).tolist()}
    for name, idx in zip(extrema_names, extrema_indices)
]

# Touch event location
touch_pt = np.round(p_raw[87], 2).tolist()

# HTML template
html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>Buzz Wire Trajectory: Calibration, Averaging & 3D Simulation</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@400;500;600;700&family=Inter:wght@400;500;600&family=JetBrains+Mono:wght@400;500&display=swap">
  <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
  <style>
    :root {{
      --bg: #0b0d13; --bg-card: #141721; --bg-card-hover: #1c2130; --border: #232838;
      --fg: #eef1f8; --fg-sec: #9097aa; --fg-muted: #575e74;
      --accent: #3b8bff; --smooth: #10b981; --std: #38bdf8; --raw: #f59e0b; --danger: #ef4444;
      --tube: rgba(6, 182, 212, 0.25); --wire: #d4a373;
    }}
    * {{ box-sizing: border-box; margin: 0; padding: 0; }}
    body {{ background: var(--bg); color: var(--fg); font-family: 'Inter', sans-serif; padding: 20px; line-height: 1.5; }}
    h1, h2, h3 {{ font-family: 'Space Grotesk', sans-serif; }}
    .mono {{ font-family: 'JetBrains Mono', monospace; }}
    .container {{ max-width: 1300px; margin: 0 auto; display: flex; flex-direction: column; gap: 20px; }}
    .header {{ display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 10px; border-bottom: 1px solid var(--border); padding-bottom: 16px; }}
    .badge {{ font-size: 0.75rem; font-weight: 600; text-transform: uppercase; letter-spacing: 0.05em; padding: 4px 10px; border-radius: 6px; }}
    .badge-smooth {{ background: rgba(16, 185, 129, 0.15); color: var(--smooth); border: 1px solid rgba(16, 185, 129, 0.3); }}
    .badge-std {{ background: rgba(56, 189, 248, 0.15); color: var(--std); border: 1px solid rgba(56, 189, 248, 0.3); }}
    .badge-raw {{ background: rgba(245, 158, 11, 0.15); color: var(--raw); border: 1px solid rgba(245, 158, 11, 0.3); }}
    .badge-touch {{ background: rgba(239, 68, 68, 0.15); color: var(--danger); border: 1px solid rgba(239, 68, 68, 0.3); }}

    .grid-stats {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(190px, 1fr)); gap: 12px; }}
    .stat-card {{ background: var(--bg-card); border: 1px solid var(--border); border-radius: 10px; padding: 14px 16px; }}
    .stat-label {{ font-size: 0.7rem; color: var(--fg-muted); text-transform: uppercase; font-weight: 600; letter-spacing: 0.05em; margin-bottom: 4px; }}
    .stat-val {{ font-size: 1.4rem; font-weight: 700; font-family: 'Space Grotesk', sans-serif; }}
    .stat-sub {{ font-size: 0.75rem; color: var(--fg-sec); margin-top: 2px; }}

    .viewport-card {{ background: var(--bg-card); border: 1px solid var(--border); border-radius: 12px; overflow: hidden; position: relative; }}
    #viewport3d {{ width: 100%; height: 580px; display: block; }}
    .viewport-overlay {{ position: absolute; top: 16px; left: 16px; background: rgba(20, 23, 33, 0.85); backdrop-filter: blur(8px); border: 1px solid var(--border); border-radius: 8px; padding: 12px 16px; max-width: 380px; pointer-events: auto; z-index: 10; }}
    .viewport-controls {{ position: absolute; bottom: 16px; left: 16px; display: flex; gap: 8px; flex-wrap: wrap; z-index: 10; }}
    .btn {{ background: #1c2130; color: var(--fg); border: 1px solid var(--border); padding: 8px 14px; border-radius: 6px; font-size: 0.8rem; font-weight: 500; cursor: pointer; transition: all 0.15s; }}
    .btn:hover {{ background: #283046; border-color: var(--accent); }}
    .btn.active {{ background: rgba(59, 139, 255, 0.2); border-color: var(--accent); color: #93c5fd; }}
    .btn.btn-smooth.active {{ background: rgba(16, 185, 129, 0.2); border-color: var(--smooth); color: #6ee7b7; }}
    .btn.btn-std.active {{ background: rgba(56, 189, 248, 0.2); border-color: var(--std); color: #7dd3fc; }}
    .btn.btn-raw.active {{ background: rgba(245, 158, 11, 0.2); border-color: var(--raw); color: #fcd34d; }}

    .grid-charts {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(450px, 1fr)); gap: 16px; }}
    .chart-card {{ background: var(--bg-card); border: 1px solid var(--border); border-radius: 10px; padding: 16px; }}
    .chart-title {{ font-size: 0.95rem; font-weight: 600; margin-bottom: 8px; display: flex; justify-content: space-between; }}
    svg {{ width: 100%; height: 220px; overflow: visible; }}
  </style>
</head>
<body>

<div class="container">

  <!-- HEADER -->
  <div class="header">
    <div>
      <h1>Buzz Wire Trajectory Calibration & Averaging</h1>
      <p style="color: var(--fg-sec); font-size: 0.85rem;">Addverb HEAL Robot | SVD Planar Fit | Contact Culling | Multi-Stage Smoothing (-smooth)</p>
    </div>
    <div style="display: flex; gap: 8px;">
      <span class="badge badge-smooth">CAD-Grade Smooth: Active</span>
      <span class="badge badge-std">Standard Mode: Preserved</span>
      <span class="badge badge-raw">Raw Human Jog</span>
      <span class="badge badge-touch">Contact Culled @ Idx 87</span>
    </div>
  </div>

  <!-- STATS TILES -->
  <div class="grid-stats">
    <div class="stat-card">
      <div class="stat-label">Raw Human Path</div>
      <div class="stat-val" style="color: var(--raw);">1,248.5 <span style="font-size: 0.8rem; font-weight: 400;">mm</span></div>
      <div class="stat-sub">722 pts, lateral tremor ±9.8 mm</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">Standard Calibrated</div>
      <div class="stat-val" style="color: var(--std);">1,229.6 <span style="font-size: 0.8rem; font-weight: 400;">mm</span></div>
      <div class="stat-sub">615 pts, basic SVD planar projection</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">Smooth CAD-Grade (-smooth)</div>
      <div class="stat-val" style="color: var(--smooth);">1,096.3 <span style="font-size: 0.8rem; font-weight: 400;">mm</span></div>
      <div class="stat-sub">548 pts, 0.1% match to CAD wire (1097.8 mm)</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">Out-of-Plane Error</div>
      <div class="stat-val" style="color: var(--smooth);">0.00 <span style="font-size: 0.8rem; font-weight: 400;">mm</span></div>
      <div class="stat-sub">Eliminated from human std: 3.25 mm</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">Traverse Durations</div>
      <div class="stat-val" style="color: var(--accent);">42.6 s <span style="font-size: 0.8rem; font-weight: 400;">to</span> 69.0 s</div>
      <div class="stat-sub">Show (69s) | Balanced (55s) | Fast (43s)</div>
    </div>
  </div>

  <!-- 3D VIEWPORT -->
  <div class="viewport-card">
    <div class="viewport-overlay">
      <h3 style="font-size: 1rem; margin-bottom: 6px;">3D Trajectory & Clearance Inspection</h3>
      <p style="font-size: 0.8rem; color: var(--fg-sec); margin-bottom: 8px;">
        Use left mouse to orbit, right mouse to pan, scroll wheel to zoom. Toggle layers below to compare what the simulator expects vs. what the human operator demonstrated vs. the CAD-grade smoothed trajectory.
      </p>
      <div style="font-size: 0.75rem; color: var(--fg-muted);" class="mono">
        Plane Normal: [0.9824, 0.1823, -0.0405]<br>
        Touch Culled: p = [144.1, 316.3, 475.9] mm
      </div>
    </div>

    <div id="viewport3d"></div>

    <div class="viewport-controls">
      <button class="btn btn-smooth active" id="btnSmooth">● Smooth Trajectory (-smooth)</button>
      <button class="btn btn-std active" id="btnStd">● Standard Calibrated (Earlier)</button>
      <button class="btn btn-raw active" id="btnRaw">● Raw Demonstrated (Orange)</button>
      <button class="btn active" id="btnTube">Clearance Corridor (9 mm)</button>
      <button class="btn active" id="btnWire">Copper Wire Model</button>
      <button class="btn active" id="btnTouch">Touch Event Sphere</button>
      <button class="btn" id="btnReset">Reset View</button>
    </div>
  </div>

  <!-- 2D CHARTS -->
  <div class="grid-charts">
    <div class="chart-card">
      <div class="chart-title">
        <span>Out-of-Plane Lateral Hunting (Tremor vs 0.00 mm Plane)</span>
        <span class="mono" style="font-size: 0.75rem; color: var(--fg-sec);">Threshold: ±10.5 mm</span>
      </div>
      <svg id="chartHunting" viewBox="0 0 540 220"></svg>
      <div style="display: flex; gap: 16px; margin-top: 8px; font-size: 0.75rem;">
        <span style="color: var(--raw);">■ Raw Demonstrated Hunting (±9.8 mm peak)</span>
        <span style="color: var(--smooth);">■ Calibrated Planar (0.00 mm deviation)</span>
        <span style="color: var(--danger);">-- Collision Boundary (±10.5 mm air gap)</span>
      </div>
    </div>

    <div class="chart-card">
      <div class="chart-title">
        <span>Curvature-Aware Speed Profiles (Cartesian mm/s)</span>
        <span class="mono" style="font-size: 0.75rem; color: var(--fg-sec);">Cap: 60.0 mm/s</span>
      </div>
      <svg id="chartSpeed" viewBox="0 0 540 220"></svg>
      <div style="display: flex; gap: 16px; margin-top: 8px; font-size: 0.75rem;">
        <span style="color: #f43f5e;">■ Fast Profile (42.6 s)</span>
        <span style="color: #3b8bff;">■ Balanced Profile (54.7 s)</span>
        <span style="color: #10b981;">■ Show Profile (69.0 s)</span>
        <span style="color: var(--raw); stroke-dasharray: 2,2;">- - Raw Jog Speed (~1.0 mm/s)</span>
      </div>
    </div>
  </div>

</div>

<script>
// Data injected from Python
const RAW_P = {json.dumps(sub_p_raw)};
const STD_P = {json.dumps(sub_p_std)};
const SM_P = {json.dumps(sub_p_sm)};
const RAW_HUNT = {json.dumps(sub_off_raw)};
const TOUCH_PT = {json.dumps(touch_pt)};
const EXTREMA = {json.dumps(extrema_points)};

// -------------------------------------------------------------
// THREE.JS 3D SCENE SETUP
// -------------------------------------------------------------
const container = document.getElementById('viewport3d');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0d13);

const camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 1, 5000);
const renderer = new THREE.WebGLRenderer({{ antialias: true }});
renderer.setSize(container.clientWidth, container.clientHeight);
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
container.appendChild(renderer.domElement);

const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.05;

// Lighting
scene.add(new THREE.AmbientLight(0xffffff, 0.7));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(500, 1000, 500);
scene.add(dirLight);

// Groups for toggling
const groupSmooth = new THREE.Group();
const groupStd = new THREE.Group();
const groupRaw = new THREE.Group();
const groupWire = new THREE.Group();
const groupTube = new THREE.Group();
const groupTouch = new THREE.Group();
const groupExtrema = new THREE.Group();

scene.add(groupSmooth);
scene.add(groupStd);
scene.add(groupRaw);
scene.add(groupWire);
scene.add(groupTube);
scene.add(groupTouch);
scene.add(groupExtrema);

// Centering target
const center = new THREE.Vector3(130, 400, 520);

// 1. Smooth Trajectory (Emerald Green, CAD-Grade)
const smCurvePts = SM_P.map(p => new THREE.Vector3(p[0], p[1], p[2]));
const smGeom = new THREE.BufferGeometry().setFromPoints(smCurvePts);
const smMat = new THREE.LineBasicMaterial({{ color: 0x10b981, linewidth: 4 }});
groupSmooth.add(new THREE.Line(smGeom, smMat));

// 2. Standard Calibrated Trajectory (Sky Blue)
const stdCurvePts = STD_P.map(p => new THREE.Vector3(p[0], p[1], p[2]));
const stdGeom = new THREE.BufferGeometry().setFromPoints(stdCurvePts);
const stdMat = new THREE.LineBasicMaterial({{ color: 0x38bdf8, linewidth: 2 }});
groupStd.add(new THREE.Line(stdGeom, stdMat));

// 3. Copper Wire Tube (Built along Smooth Trajectory)
const smCurve = new THREE.CatmullRomCurve3(smCurvePts);
const wireGeom = new THREE.TubeGeometry(smCurve, 300, 2.0, 12, false);
const wireMat = new THREE.MeshStandardMaterial({{ color: 0xd4a373, metalness: 0.8, roughness: 0.3 }});
groupWire.add(new THREE.Mesh(wireGeom, wireMat));

// 4. Cyan Clearance Envelope (9 mm radial cylinder)
const tubeGeom = new THREE.TubeGeometry(smCurve, 250, 9.0, 16, false);
const tubeMat = new THREE.MeshStandardMaterial({{
  color: 0x06b6d4,
  transparent: true,
  opacity: 0.22,
  wireframe: false,
  depthWrite: false
}});
groupTube.add(new THREE.Mesh(tubeGeom, tubeMat));

// 5. Raw Human Demonstrated Jog (Orange, Jagged)
const rawCurvePts = RAW_P.map(p => new THREE.Vector3(p[0], p[1], p[2]));
const rawGeom = new THREE.BufferGeometry().setFromPoints(rawCurvePts);
const rawMat = new THREE.LineBasicMaterial({{ color: 0xf59e0b, linewidth: 1.5 }});
groupRaw.add(new THREE.Line(rawGeom, rawMat));

// 6. Touch Event Sphere (Culled contact point at Idx 87)
const touchSphereGeom = new THREE.SphereGeometry(6.0, 16, 16);
const touchSphereMat = new THREE.MeshBasicMaterial({{ color: 0xef4444 }});
const touchSphere = new THREE.Mesh(touchSphereGeom, touchSphereMat);
touchSphere.position.set(TOUCH_PT[0], TOUCH_PT[1], TOUCH_PT[2]);
groupTouch.add(touchSphere);

// 7. Base Plate Reference Grid
const grid = new THREE.GridHelper(500, 20, 0x232838, 0x181c28);
grid.position.set(130, 400, 400);
grid.rotation.x = Math.PI / 2;
scene.add(grid);

// Camera default placement
function resetView() {{
  camera.position.set(130, -100, 750);
  camera.lookAt(center);
  controls.target.copy(center);
  controls.update();
}}
resetView();

// Animation loop
function animate() {{
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}}
animate();

// Button handlers
function toggleBtn(id, group, className) {{
  const b = document.getElementById(id);
  b.addEventListener('click', () => {{
    group.visible = !group.visible;
    b.classList.toggle('active', group.visible);
  }});
}}
toggleBtn('btnSmooth', groupSmooth);
toggleBtn('btnStd', groupStd);
toggleBtn('btnRaw', groupRaw);
toggleBtn('btnTube', groupTube);
toggleBtn('btnWire', groupWire);
toggleBtn('btnTouch', groupTouch);
document.getElementById('btnReset').addEventListener('click', resetView);

window.addEventListener('resize', () => {{
  camera.aspect = container.clientWidth / container.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(container.clientWidth, container.clientHeight);
}});

// -------------------------------------------------------------
// 2D CHARTS GENERATION (SVG)
// -------------------------------------------------------------
(function() {{
  const svg = document.getElementById('chartHunting');
  const w = 540, h = 220, p = {{ l: 45, r: 15, t: 15, b: 30 }};
  const cw = w - p.l - p.r, ch = h - p.t - p.b;
  const n = RAW_HUNT.length;
  const maxErr = 12.0;

  const sx = i => p.l + (i / (n - 1)) * cw;
  const sy = v => p.t + ch/2 - (v / maxErr) * (ch / 2);

  // Zero line
  const l0 = document.createElementNS('http://www.w3.org/2000/svg', 'line');
  l0.setAttribute('x1', p.l); l0.setAttribute('x2', w - p.r);
  l0.setAttribute('y1', sy(0)); l0.setAttribute('y2', sy(0));
  l0.setAttribute('stroke', '#333b4f');
  svg.appendChild(l0);

  // Clearance threshold lines (danger boundary)
  [-10.5, 10.5].forEach(thresh => {{
    const lt = document.createElementNS('http://www.w3.org/2000/svg', 'line');
    lt.setAttribute('x1', p.l); lt.setAttribute('x2', w - p.r);
    lt.setAttribute('y1', sy(thresh)); lt.setAttribute('y2', sy(thresh));
    lt.setAttribute('stroke', 'rgba(239, 68, 68, 0.4)');
    lt.setAttribute('stroke-dasharray', '4,4');
    svg.appendChild(lt);
  }});

  // Raw tremor curve
  let pts = RAW_HUNT.map((v, i) => `${{sx(i)}},${{sy(v)}}`).join(' ');
  const polyRaw = document.createElementNS('http://www.w3.org/2000/svg', 'polyline');
  polyRaw.setAttribute('points', pts);
  polyRaw.setAttribute('fill', 'none');
  polyRaw.setAttribute('stroke', '#f59e0b');
  polyRaw.setAttribute('stroke-width', '1.8');
  svg.appendChild(polyRaw);

  // Calibrated zero line
  const polyCal = document.createElementNS('http://www.w3.org/2000/svg', 'line');
  polyCal.setAttribute('x1', p.l); polyCal.setAttribute('x2', w - p.r);
  polyCal.setAttribute('y1', sy(0)); polyCal.setAttribute('y2', sy(0));
  polyCal.setAttribute('stroke', '#10b981');
  polyCal.setAttribute('stroke-width', '2.5');
  svg.appendChild(polyCal);

  // Y Axis labels
  [-10, 0, 10].forEach(val => {{
    const t = document.createElementNS('http://www.w3.org/2000/svg', 'text');
    t.setAttribute('x', p.l - 8); t.setAttribute('y', sy(val) + 4);
    t.setAttribute('fill', '#9097aa'); t.setAttribute('font-size', '9');
    t.setAttribute('text-anchor', 'end');
    t.textContent = val + 'mm';
    svg.appendChild(t);
  }});
}})();

(function() {{
  const svg = document.getElementById('chartSpeed');
  const w = 540, h = 220, p = {{ l: 40, r: 15, t: 15, b: 30 }};
  const cw = w - p.l - p.r, ch = h - p.t - p.b;
  const maxV = 55.0;

  const sx = frac => p.l + frac * cw;
  const sy = v => p.t + ch - (v / maxV) * ch;

  // Grid lines
  for (let v = 10; v <= 50; v += 10) {{
    const l = document.createElementNS('http://www.w3.org/2000/svg', 'line');
    l.setAttribute('x1', p.l); l.setAttribute('x2', w - p.r);
    l.setAttribute('y1', sy(v)); l.setAttribute('y2', sy(v));
    l.setAttribute('stroke', '#1e2433');
    svg.appendChild(l);
    
    const t = document.createElementNS('http://www.w3.org/2000/svg', 'text');
    t.setAttribute('x', p.l - 6); t.setAttribute('y', sy(v) + 4);
    t.setAttribute('fill', '#575e74'); t.setAttribute('font-size', '9');
    t.setAttribute('text-anchor', 'end');
    t.textContent = v;
    svg.appendChild(t);
  }}

  // Profiles data
  const profiles = {json.dumps(metrics.get("profiles", {}))};
  const colors = {{
    'Fast Profile (Competition)': '#f43f5e',
    'Balanced Profile (Standard)': '#3b8bff',
    'Show Profile (Presentation)': '#10b981'
  }};

  for (const [name, prof] of Object.entries(profiles)) {{
    const vList = prof.speeds_mmps;
    if (!vList) continue;
    const pts = vList.map((v, i) => `${{sx(i / (vList.length - 1))}},${{sy(v)}}`).join(' ');
    const line = document.createElementNS('http://www.w3.org/2000/svg', 'polyline');
    line.setAttribute('points', pts);
    line.setAttribute('fill', 'none');
    line.setAttribute('stroke', colors[name] || '#ffffff');
    line.setAttribute('stroke-width', '2');
    svg.appendChild(line);
  }}

  // Raw crawl reference (1.0 mm/s)
  const lRaw = document.createElementNS('http://www.w3.org/2000/svg', 'line');
  lRaw.setAttribute('x1', p.l); lRaw.setAttribute('x2', w - p.r);
  lRaw.setAttribute('y1', sy(1.0)); lRaw.setAttribute('y2', sy(1.0));
  lRaw.setAttribute('stroke', '#f59e0b');
  lRaw.setAttribute('stroke-dasharray', '3,3');
  svg.appendChild(lRaw);
}})();
</script>

</body>
</html>
"""

out_html.write_text(html_content)
print(f"Successfully generated 3D Interactive Dashboard: {out_html}")

# `perception.py` — Simulation Perception Pipeline

**Module**: `sim_demo/perception.py`
**Purpose**: MuJoCo-specific perception using known color masks, RANSAC plane fitting, and ray-plane intersection.

---

## Color Masks (MuJoCo Object Colors)

| Mask Name | Object |
|-----------|--------|
| `peg_green` | Circle peg |
| `peg_blue` | Square peg |
| `peg_orange` | Hexagon peg (orange in sim, red in hardware) |
| `peg_yellow` | Triangle peg |
| `mat` | Green working mat |
| `board` | Wooden board |
| `marker` | Calibration marker (colored ball on wrist) |
| `red_ring` | Socket indicator rings |

---

## Key Algorithms

### RANSAC Plane Fitting
Fits a plane to the board surface from depth points:
1. Random sample 3 points → compute plane normal
2. Count inliers within distance threshold
3. Repeat N iterations → best plane

Used to establish the board plane for accurate peg height measurement.

### Ray-Plane Intersection
Projects pixel rays through the camera model onto the fitted board plane:
```
ray_origin = camera_position
ray_dir = deproject(u, v, 1.0)  → normalized direction
t = (plane_point - ray_origin) · plane_normal / (ray_dir · plane_normal)
intersection = ray_origin + t × ray_dir
```

### Sphere Center Fitting
For the calibration marker (colored ball), fits a sphere to visible depth points to find the ball center. More accurate than simple centroid for a curved surface.

### Depth-Sliced Peg Detection
Same concept as hardware `depth_slice_top_face()`:
1. Mask pixels by color
2. Find minimum depth (top surface)
3. Slice to 20mm thickness
4. Extract yaw from remaining contour shape

---

## Sim vs. Hardware Perception

| Aspect | Simulation | Hardware |
|--------|-----------|----------|
| Color source | Known MuJoCo RGBA | HSV thresholds on camera image |
| Plane fitting | RANSAC on depth | Assumed flat from ruler measurement |
| Noise | Injected Gaussian + dropout | Real RealSense noise |
| Marker tracking | Sphere fitting on colored ball | N/A (uses gripper flick) |
| Socket detection | Color mask (red rings) | Otsu threshold on grayscale |

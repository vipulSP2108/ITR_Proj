# `dynamic_workspace_perceiver.py` — Self-Assessing Vision for Movable Board

**Module**: `src/vision/dynamic_workspace_perceiver.py`
**Purpose**: Autonomously detects movable wooden board, classifies all cutouts, locates peg on mat, and computes 3D robot coordinates — all from a single overhead frame.

---

## Physical Ground Truths (Ruler-Measured)

| Item | Dimension |
|------|-----------|
| Camera height | 470 mm above table |
| Focal length | 344 px (RealSense D435, ~86° FOV) |
| Scale | 1.366 mm/px (470 ÷ 344) |
| Green foam mat | 300 × 300 mm |
| Wooden board length | 300 mm (movable!) |
| Circle cutout | ⌀50 mm (radius 25 mm, ~18.3 px) |
| Hexagon cutout | 47 mm across flats |
| Square cutout | 50 × 50 mm |
| Triangle cutout | 43 mm height |
| Cutout spacing | 25 mm gap (73.5 mm center-to-center) |

---

## Class: `DynamicWorkspacePerceiver`

### Constructor
```python
DynamicWorkspacePerceiver(config_path="config/workspace_dimensions.yaml")
```
Loads camera parameters, robot base pixel estimate (u=485, v=215), manipulation heights.

### `pixel_to_robot_3d(u, v, z_rob=None) → (X, Y, Z) mm`

Orthogonal overhead projection (simplified pinhole for near-vertical camera):
```
effective_scale = (H_cam - z_rob) / f_px
X_rob = (u_base - u) × effective_scale    # rightward in image = +X (toward board)
Y_rob = (v - v_base) × effective_scale     # downward in image = +Y (toward mat)
```

**Height correction**: Objects above table (z_rob > 0) appear larger because they're closer to camera. The `(H_cam - z_rob)` factor compensates.

### `detect_board_and_cutouts(gray) → (list[dict], circle_socket)`

**3-Pass Detection Strategy**:

| Pass | Method | Target |
|------|--------|--------|
| 1 | Otsu INV threshold | Dark cutouts (wood darker than board) |
| 2 | Otsu Normal threshold | Bright cutouts (metallic reflection) |
| 3 | Hough Circles (r=15–24px) | Circular cutout specifically |

**Spatial Binning** (board ROI: u∈[150,330], v∈[40,460]):

| Bin | v Range | Shape | Default (u, v) |
|-----|---------|-------|-----------------|
| 1 | 100–180 | Circle | (244.0, 148.2) |
| 2 | 180–235 | Hexagon | (244.3, 206.2) |
| 3 | 235–290 | Square | (242.6, 262.8) |
| 4 | 290–360 | Triangle | (242.4, 316.4) |

Candidates filtered to board column (u∈[200,280]). Best candidate per bin chosen by proximity to u=244 (board centerline). Falls back to default positions if bin is empty.

### `detect_peg_on_mat(gray) → dict or None`

1. Search ROI: mat interior (u∈[300,405], v∈[100,360])
2. **11×11 median filter** to suppress RealSense IR projector dot pattern
3. Hough Circles: dp=1.0, minDist=30, param1=40, param2=15, r=14–24px
4. Verify: interior intensity > 90 (peg top face is brighter than mat)
5. Return best scoring circle: {u, v, radius_px, intensity}

### `assess(frame) → dict`

Complete pipeline:
1. `detect_board_and_cutouts()` → cutout positions
2. `detect_peg_on_mat()` → peg position
3. `pixel_to_robot_3d()` for pick and place coordinates
4. Annotate diagnostic image with HUD overlay
5. Returns: `{board_cutouts, target_socket, peg, pick_3d, place_3d, scale_mm_per_px, annotated_frame}`

---

## Design Decisions

- **Movable board**: Board is NOT fixed to the table. Pipeline must detect dynamically every run.
- **3-pass detection**: Single threshold fails when lighting changes board appearance (dark wood vs. metallic reflection). Three complementary methods ensure robust detection.
- **Median filter for IR dots**: RealSense D435's IR projector creates visible dot pattern on smooth surfaces. 11×11 median filter is the minimum size to suppress 3–5px dots.
- **Spatial bins over shape classification**: Board cutout order is fixed by physical design (circle→hexagon→square→triangle from top to bottom). v-coordinate binning is more reliable than classifying partially-visible cutouts.
- **Fallback positions**: If detection fails temporarily (occlusion by arm), known-good defaults prevent total failure.

---

## Dependency Chain
```
Single BGR frame → detect_board_and_cutouts() → cutout positions
                 → detect_peg_on_mat() → peg position
                 → pixel_to_robot_3d() → robot coordinates
                 → LiveHardwareSorter.run_vision_perception()
```

# `color_segmentation.py` — HSV Color Segmentation

**Module**: `src/vision/color_segmentation.py`
**Purpose**: Segment objects by color from BGR camera frames using HSV thresholds with morphological cleanup.

---

## Class: `ColorSegmenter`

### Constructor
```python
ColorSegmenter(config_path="config/colors.yaml")
```
Loads HSV ranges and morphological parameters from YAML. Supports dual-range hue wrap-around for red (hue crosses 0°/180° boundary).

### Key Methods

#### `get_mask(bgr_image, color_name) → np.ndarray`
1. Converts BGR → HSV
2. Applies `cv2.inRange()` with loaded thresholds
3. For **red**: combines two hue ranges (0–12 and 170–179) with bitwise OR
4. Morphological open (5×5 elliptical kernel, 2 iterations) — removes small noise
5. Morphological close (5×5 elliptical kernel, 2 iterations) — fills small holes
6. Returns: binary mask (0 or 255)

#### `segment_all(bgr_image) → dict[str, np.ndarray]`
Runs `get_mask()` for every color defined in config. Returns dict mapping color name to mask.

---

## HSV Thresholds (from `config/colors.yaml`)

| Color | Hue Range | Sat Range | Val Range | Peg Shape |
|-------|-----------|-----------|-----------|-----------|
| Yellow | 16–32 | 80–255 | 80–255 | Triangle |
| Blue | 95–130 | 100–255 | 50–255 | Square |
| Red | 0–12 + 170–179 | 85–255 | 90–255 | Hexagon |
| Green | 38–80 | 65–255 | 65–255 | Circle |

---

## Morphology Parameters

| Parameter | Value |
|-----------|-------|
| Kernel shape | Elliptical |
| Kernel size | 5×5 |
| Open iterations | 2 |
| Close iterations | 2 |

---

## Design Decisions

- **Dual-range red**: Red hue wraps around 0°/180° in OpenCV HSV. Two `inRange` calls with bitwise OR solve this cleanly.
- **Elliptical kernel**: Better preserves curved peg contours vs. rectangular kernel.
- **Open before close**: Removes speckle noise first, then fills internal gaps for solid contour extraction downstream.

---

## Dependency Chain
```
colors.yaml → ColorSegmenter → SingleCameraPipeline.process_scene()
                              → DynamicWorkspacePerceiver (indirect, uses grayscale)
```

# `colors.yaml` — HSV Color Thresholds & Morphology Settings

**File**: `config/colors.yaml`
**Purpose**: Central color configuration for peg detection. Tuned for overhead RealSense D435i under lab lighting.

---

## Color Definitions

### Yellow (Triangle Peg)
| Channel | Low | High |
|---------|-----|------|
| Hue | 16 | 32 |
| Saturation | 80 | 255 |
| Value | 80 | 255 |

### Blue (Square Peg)
| Channel | Low | High |
|---------|-----|------|
| Hue | 95 | 130 |
| Saturation | 100 | 255 |
| Value | 50 | 255 |

### Red (Hexagon Peg) — Dual Range
**Range 1** (low hue):
| Channel | Low | High |
|---------|-----|------|
| Hue | 0 | 12 |
| Saturation | 85 | 255 |
| Value | 90 | 255 |

**Range 2** (high hue, wrapping around 180):
| Channel | Low | High |
|---------|-----|------|
| Hue | 170 | 179 |
| Saturation | 85 | 255 |
| Value | 90 | 255 |

### Green (Circle Peg)
| Channel | Low | High |
|---------|-----|------|
| Hue | 38 | 80 |
| Saturation | 65 | 255 |
| Value | 65 | 255 |

---

## Morphology Settings
- **Kernel size**: 5×5 (elliptical)
- **Open iterations**: 2
- **Close iterations**: 2

---

## Peg-Color-Shape Mapping

| Color | Shape | Socket Assignment |
|-------|-------|-------------------|
| Green | Circle | Green socket |
| Blue | Square | Blue socket |
| Yellow | Triangle | Yellow socket |
| Red/Orange | Hexagon | Red socket |

---

## Tuning Notes
- Yellow's low saturation (80) accommodates dim lighting conditions
- Blue's low value (50) handles shadows on the blue peg
- Green's wide hue range (38–80) overlaps with the green foam mat — the mat is excluded by depth slicing (peg top face is higher than mat)
- Red dual-range is required because hue 0° and 180° are adjacent in HSV but numerically distant in OpenCV (0–179 range)

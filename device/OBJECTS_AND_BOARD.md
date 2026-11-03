# Peg Objects & Socket Board Specifications

Physical dimensions, color values, and geometric features of the task objects.

Visual reference image saved at: [`device/images/actual_pegs_and_board.jpg`](file:///Users/aai/Desktop/ITR_Proj/device/images/actual_pegs_and_board.jpg)

---

## 1. Peg Objects (Sorted by Appearance)

| Peg Shape | Physical Color | Material | Approximate Dimensions | HSV Range (OpenCV) |
| :--- | :--- | :--- | :--- | :--- |
| **Triangle** | **Bright Yellow** | Wood / Plastic | ~4.5 cm side, ~10 cm tall | H: [16, 32], S: [80, 255], V: [80, 255] |
| **Square** | **Deep Blue** | Wood / Plastic | ~4.5 cm width, ~10 cm tall | H: [95, 130], S: [100, 255], V: [50, 255] |
| **Hexagon** | **Orange / Salmon Red** | Wood / Plastic | ~4.5 cm across, ~10 cm tall| H: [0, 12] & [170, 179], S: [85, 255], V: [90, 255] |
| **Circle** | **Green** | Wood / Plastic | ~4.5 cm dia, ~10 cm tall | H: [38, 80], S: [65, 255], V: [65, 255] |

---

## 2. Socket Board

* **Material**: Laser-cut Medium-Density Fibreboard (MDF / Plywood).
* **Base Surface**: Light brown wood grain.
* **Socket Internal Cutout Lining**: Laser-cut red internal boundary edges.
* **Socket Arrangement (Single Strip, Top to Bottom)**:
  1. **Position 1 (Top)**: Triangle Cutout
  2. **Position 2**: Square Cutout
  3. **Position 3**: Hexagon Cutout
  4. **Position 4 (Bottom)**: Circular Cutout

---

## 3. Grasping & Tolerances

* **Grasp Type**: Top-down parallel pinch grasp ($Z$-axis approach).
* **Pinch Width**: Gripper opens to $\approx 60\text{ mm}$ (clearance for ~45 mm pegs).
* **Insertion Clearance**: ~1 to 2 mm nominal socket margin.
* **Pre-Grasp Offset**: Approach from $+60\text{ mm}$ to $+80\text{ mm}$ vertically above peg.


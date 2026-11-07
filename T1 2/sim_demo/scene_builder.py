"""
Generates the MuJoCo scene (MJCF) for the peg-sorting workspace from config/sim_layout.yaml.

World frame == robot frame: origin at the OpenMANIPULATOR-X base on the table surface (z = 0 is the
table top), +X toward the shape board, +Y toward image-bottom of the mounted camera, +Z up.
"""
import math, os
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
LAYOUT_PATH = os.path.join(HERE, "config", "sim_layout.yaml")

MAT_TOP = 0.010
BOARD_TOP = 0.008
MARKER_R = 0.009

PEG_COLORS = {"circle": "mat_green_peg", "hexagon": "mat_orange_peg",
              "square": "mat_blue_peg", "triangle": "mat_yellow_peg"}


def load_layout(path: str = LAYOUT_PATH) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


class Layout:
    """Pixel -> metre conversion + derived geometry."""
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.s = cfg["pixel_scale_m"]
        self.bx, self.by = cfg["base_px"]

    def px2m(self, px, py):
        return ((self.bx - px) * self.s, (py - self.by) * self.s)

    def box(self, d):
        x_a, y_a = self.px2m(d["x0"], d["y0"]); x_b, y_b = self.px2m(d["x1"], d["y1"])
        xlo, xhi = sorted([x_a, x_b]); ylo, yhi = sorted([y_a, y_b])
        return (xlo + xhi) / 2, (ylo + yhi) / 2, (xhi - xlo) / 2, (yhi - ylo) / 2   # cx, cy, hx, hy

    def hole_xy(self, name):
        return self.px2m(*self.cfg["holes_px"][name])


def _prism_vertices(kind, r, h):
    if kind == "triangle":     # apex toward -X (toward the robot)
        pts = [(-r, 0), (0.5 * r, 0.866 * r), (0.5 * r, -0.866 * r)]
    else:                      # hexagon, vertices on +-Y
        pts = [(r * math.cos(math.radians(90 + 60 * k)), r * math.sin(math.radians(90 + 60 * k))) for k in range(6)]
    return " ".join(f"{x:.5f} {y:.5f} {z:.5f}" for z in (-h, h) for x, y in pts)


def look_at_xyaxes(eye, target, up=(0, 0, 1)):
    eye, target, up = map(lambda v: np.asarray(v, float), (eye, target, up))
    zc = eye - target; zc /= np.linalg.norm(zc)
    xc = np.cross(up, zc); xc /= np.linalg.norm(xc)
    yc = np.cross(zc, xc)
    return " ".join(f"{v:.5f}" for v in (*xc, *yc))


def default_peg_xy(seed: int = 7):
    rng = np.random.default_rng(seed)
    return float(rng.uniform(0.11, 0.19)), float(rng.uniform(-0.09, 0.09))


def build_scene_xml(layout_cfg=None, pegs=("circle",), peg_xy=None, seed=7) -> str:
    cfg = layout_cfg or load_layout()
    L = Layout(cfg)
    pr, ph, pm = cfg["peg_radius_m"], cfg["peg_height_m"], cfg["peg_mass_kg"]

    mcx, mcy, mhx, mhy = L.box(cfg["mat_px"])
    bcx, bcy, bhx, bhy = L.box(cfg["board_px"])
    table_edge_x = (L.bx - cfg["table_edge_px_x"]) * L.s
    rail = cfg["rail_px"]
    rail_x0, rail_x1 = (L.bx - rail["x1"]) * L.s, (L.bx - rail["x0"]) * L.s
    rail_y = (rail["y"] - L.by) * L.s
    cam = cfg["camera"]
    cam_x, cam_y = L.px2m(*cam["centre_px"])

    # ---- holes: red outline + dark interior (visual decals) -------------------------------------
    meshes, decals = [], []
    zr, zd = BOARD_TOP + 0.0002, BOARD_TOP + 0.0005
    for name, rr in cfg["hole_size_m"].items():
        hx, hy = L.hole_xy(name)
        rd = rr - 0.003
        if name == "circle":
            decals.append(f'<geom name="hole_{name}_outline" type="cylinder" size="{rr} 0.0002" pos="{hx:.4f} {hy:.4f} {zr}" material="mat_socket_red" contype="0" conaffinity="0"/>')
            decals.append(f'<geom name="hole_{name}_interior" type="cylinder" size="{rd} 0.0002" pos="{hx:.4f} {hy:.4f} {zd}" material="mat_hole" contype="0" conaffinity="0"/>')
        elif name == "square":
            decals.append(f'<geom name="hole_{name}_outline" type="box" size="{rr} {rr} 0.0002" pos="{hx:.4f} {hy:.4f} {zr}" material="mat_socket_red" contype="0" conaffinity="0"/>')
            decals.append(f'<geom name="hole_{name}_interior" type="box" size="{rd} {rd} 0.0002" pos="{hx:.4f} {hy:.4f} {zd}" material="mat_hole" contype="0" conaffinity="0"/>')
        else:
            meshes.append(f'<mesh name="m_{name}_out" vertex="{_prism_vertices(name, rr, 0.0002)}"/>')
            meshes.append(f'<mesh name="m_{name}_in" vertex="{_prism_vertices(name, rd, 0.0002)}"/>')
            decals.append(f'<geom name="hole_{name}_outline" type="mesh" mesh="m_{name}_out" pos="{hx:.4f} {hy:.4f} {zr}" material="mat_socket_red" contype="0" conaffinity="0"/>')
            decals.append(f'<geom name="hole_{name}_interior" type="mesh" mesh="m_{name}_in" pos="{hx:.4f} {hy:.4f} {zd}" material="mat_hole" contype="0" conaffinity="0"/>')

    # ---- pegs -------------------------------------------------------------------------------------
    peg_bodies = []
    if peg_xy is None:
        peg_xy = default_peg_xy(seed)
    default_slots = {"circle": peg_xy, "hexagon": (0.13, -0.075), "square": (0.13, 0.075), "triangle": (0.19, 0.0)}
    for name in pegs:
        px, py = peg_xy if name == "circle" else default_slots[name]
        mat = PEG_COLORS[name]
        fr = 'friction="1.5 0.02 0.002" condim="4" solref="0.004 1" solimp="0.95 0.99 0.001"'
        if name == "circle":
            g = f'<geom name="peg_{name}_geom" type="cylinder" size="{pr} {ph/2}" material="{mat}" mass="{pm}" {fr}/>'
        elif name == "square":
            g = f'<geom name="peg_{name}_geom" type="box" size="{pr} {pr} {ph/2}" material="{mat}" mass="{pm}" {fr}/>'
        else:
            meshes.append(f'<mesh name="m_{name}_peg" vertex="{_prism_vertices(name, pr*1.1, ph/2)}"/>')
            g = f'<geom name="peg_{name}_geom" type="mesh" mesh="m_{name}_peg" material="{mat}" mass="{pm}" {fr}/>'
        peg_bodies.append(f'''    <body name="peg_{name}" pos="{px:.4f} {py:.4f} {MAT_TOP + ph/2 + 0.0005:.4f}">
      <freejoint name="jnt_peg_{name}"/>
      {g}
    </body>''')

    # ---- cameras ----------------------------------------------------------------------------------
    cam_pos = (cam_x, cam_y, cam["height_above_table_m"])
    obl = look_at_xyaxes((-0.30, -0.42, 0.34), (0.15, 0.0, 0.05))
    side = look_at_xyaxes((0.16, -0.62, 0.17), (0.16, 0.0, 0.10))

    xml = f'''<mujoco model="openmanipulator_x_peg_sorting_sim_demo">
  <compiler angle="radian" coordinate="local"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="implicitfast" solver="Newton" iterations="50" noslip_iterations="3"/>
  <visual>
    <headlight ambient="0.25 0.25 0.25" diffuse="0.35 0.35 0.35" specular="0.0 0.0 0.0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global offwidth="1280" offheight="720"/>
    <quality shadowsize="2048"/>
  </visual>
  <asset>
    <material name="mat_table" rgba="0.30 0.32 0.35 1" specular="0.1"/>
    <material name="mat_green_pad" rgba="0.20 0.70 0.50 1" specular="0.0"/>
    <material name="mat_board" rgba="0.85 0.65 0.45 1" specular="0.0"/>
    <material name="mat_robot_dark" rgba="0.18 0.18 0.20 1"/>
    <material name="mat_robot_white" rgba="0.88 0.88 0.90 1"/>
    <material name="mat_marker" rgba="1.0 0.0 1.0 1" specular="0"/>
    <material name="mat_blue_peg" rgba="0.10 0.35 0.85 1"/>
    <material name="mat_yellow_peg" rgba="0.95 0.85 0.10 1"/>
    <material name="mat_orange_peg" rgba="0.95 0.40 0.10 1"/>
    <material name="mat_green_peg" rgba="0.20 0.80 0.25 1"/>
    <material name="mat_socket_red" rgba="0.85 0.15 0.15 1"/>
    <material name="mat_hole" rgba="0.10 0.10 0.10 1"/>
    <material name="mat_aluminum" rgba="0.80 0.82 0.85 1"/>
    {chr(10).join("    " + m for m in meshes).strip()}
  </asset>
  <default>
    <geom condim="3" friction="1 0.02 0.002"/>
  </default>

  <worldbody>
    <light directional="true" pos="0 0 3" dir="0 0 -1" diffuse="0.45 0.45 0.45" specular="0 0 0"/>
    <light directional="false" pos="0.5 -0.5 1.5" diffuse="0.15 0.15 0.15" specular="0 0 0"/>

    <geom name="floor" type="plane" size="3 3 0.1" pos="0 0 -0.74" rgba="0.35 0.35 0.37 1"/>
    <!-- table: top surface at z = 0, ends at X = {table_edge_x:.3f} (board overhangs it) -->
    <geom name="table" type="box" size="{(table_edge_x + 0.6)/2:.4f} 0.51 0.02" pos="{(table_edge_x - 0.6)/2:.4f} {rail_y + 0.02 - 0.51:.4f} -0.02" material="mat_table"/>
    <geom name="rail" type="box" size="{(rail_x1-rail_x0)/2:.4f} 0.02 0.02" pos="{(rail_x0+rail_x1)/2:.4f} {rail_y:.4f} 0.02" material="mat_aluminum" contype="0" conaffinity="0"/>
    <geom name="green_mat" type="box" size="{mhx:.4f} {mhy:.4f} {MAT_TOP/2:.4f}" pos="{mcx:.4f} {mcy:.4f} {MAT_TOP/2:.4f}" material="mat_green_pad"/>
    <geom name="socket_board" type="box" size="{bhx:.4f} {bhy:.4f} {BOARD_TOP/2:.4f}" pos="{bcx:.4f} {bcy:.4f} {BOARD_TOP/2:.4f}" material="mat_board"/>
    {chr(10).join("    " + d for d in decals).strip()}

    <camera name="overhead_realsense" pos="{cam_x:.4f} {cam_y:.4f} {cam_pos[2]:.4f}" xyaxes="-1 0 0 0 -1 0" fovy="{cam['fovy_deg']}" mode="fixed"/>
    <camera name="oblique_view" pos="-0.30 -0.42 0.34" xyaxes="{obl}" fovy="50" mode="fixed"/>
    <camera name="side_view" pos="0.16 -0.62 0.17" xyaxes="{side}" fovy="45" mode="fixed"/>

{chr(10).join(peg_bodies)}

    <!-- ============ ROBOTIS OpenMANIPULATOR-X (published dimensions) ============ -->
    <body name="robot_base" pos="0 0 0">
      <geom type="cylinder" size="0.05 0.015" pos="0 0 0.015" material="mat_robot_dark"/>
      <geom type="box" size="0.035 0.035 0.02" pos="0 0 0.050" material="mat_robot_dark"/>
      <body name="link1" pos="0 0 0.077" gravcomp="1">
        <joint name="joint1" type="hinge" axis="0 0 1" range="-3.14159 3.14159" damping="0.5" armature="0.01"/>
        <geom type="cylinder" size="0.028 0.018" material="mat_robot_dark" contype="0" conaffinity="0"/>
        <body name="link2" gravcomp="1">
          <joint name="joint2" type="hinge" axis="0 1 0" range="-1.7907 1.5708" damping="0.5" armature="0.01"/>
          <geom type="capsule" size="0.020" fromto="0 0 0 0.024 0 0.128" material="mat_robot_white"/>
          <geom type="box" size="0.018 0.024 0.015" pos="0.024 0 0.128" material="mat_robot_dark"/>
          <body name="link3" pos="0.024 0 0.128" gravcomp="1">
            <joint name="joint3" type="hinge" axis="0 1 0" range="-0.9425 1.3823" damping="0.5" armature="0.01"/>
            <geom type="capsule" size="0.018" fromto="0 0 0 0.124 0 0" material="mat_robot_white"/>
            <geom type="box" size="0.016 0.024 0.018" pos="0.124 0 0" material="mat_robot_dark"/>
            <body name="link4" pos="0.124 0 0" gravcomp="1">
              <joint name="joint4" type="hinge" axis="0 1 0" range="-1.7907 2.0420" damping="0.5" armature="0.01"/>
              <geom type="box" size="0.018 0.022 0.018" pos="0.012 0 0" material="mat_robot_dark"/>
              <!-- colored tool marker on a short mast (detected by the camera for calibration/tracking) -->
              <geom name="marker_mast" type="capsule" size="0.0025" fromto="0 0 0 0 0 0.030" material="mat_robot_dark" contype="0" conaffinity="0"/>
              <geom name="tool_marker" type="sphere" size="{MARKER_R}" pos="0 0 0.030" material="mat_marker" contype="0" conaffinity="0"/>
              <body name="gripper_base" pos="0.045 0 0" gravcomp="1">
                <geom type="box" size="0.025 0.050 0.016" material="mat_robot_dark"/>
                <body name="finger_left" pos="0.025 0.040 0" gravcomp="1">
                  <joint name="grip_left" type="slide" axis="0 1 0" range="-0.025 0" damping="2.0" armature="0.002"/>
                  <geom name="finger_left_pad" type="box" size="0.028 0.005 0.014" pos="0.028 0 0" material="mat_robot_white" friction="2.0 0.05 0.002" condim="4" solref="0.004 1" solimp="0.95 0.99 0.001"/>
                </body>
                <body name="finger_right" pos="0.025 -0.040 0" gravcomp="1">
                  <joint name="grip_right" type="slide" axis="0 1 0" range="0 0.025" damping="2.0" armature="0.002"/>
                  <geom name="finger_right_pad" type="box" size="0.028 0.005 0.014" pos="0.028 0 0" material="mat_robot_white" friction="2.0 0.05 0.002" condim="4" solref="0.004 1" solimp="0.95 0.99 0.001"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <contact>
    <exclude body1="robot_base" body2="link1"/>
    <exclude body1="link1" body2="link2"/>
    <exclude body1="robot_base" body2="link2"/>
    <exclude body1="robot_base" body2="link3"/>
    <exclude body1="link2" body2="link3"/>
    <exclude body1="link3" body2="link4"/>
    <exclude body1="link4" body2="gripper_base"/>
    <exclude body1="gripper_base" body2="finger_left"/>
    <exclude body1="gripper_base" body2="finger_right"/>
  </contact>

  <actuator>
    <position name="act_joint1" joint="joint1" kp="200" kv="14" forcerange="-8 8"/>
    <position name="act_joint2" joint="joint2" kp="300" kv="20" forcerange="-8 8"/>
    <position name="act_joint3" joint="joint3" kp="250" kv="16" forcerange="-8 8"/>
    <position name="act_joint4" joint="joint4" kp="200" kv="12" forcerange="-6 6"/>
    <position name="act_grip_left"  joint="grip_left"  kp="400" kv="6" forcerange="-12 12"/>
    <position name="act_grip_right" joint="grip_right" kp="400" kv="6" forcerange="-12 12"/>
  </actuator>
</mujoco>
'''
    return xml


if __name__ == "__main__":
    import mujoco
    m = mujoco.MjModel.from_xml_string(build_scene_xml())
    print("scene compiled: bodies", m.nbody, "geoms", m.ngeom)

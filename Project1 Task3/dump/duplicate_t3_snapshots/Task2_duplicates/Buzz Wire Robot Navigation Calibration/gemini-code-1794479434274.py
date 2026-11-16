import json
import math
import os
import shutil
import zipfile
import numpy as np

# 1. Define exact calibrated keypoints in Ground Frame {G} (units: mm, radians, degrees)
# Origin O = (0, 0, 0) at inner face of Left MDF Post, top surface of base plate (Z=0), wire plane (Y=0)

keypoints = [
    {
        "id": "S_L",
        "name": "Left Post Entry (Start)",
        "type": "endpoint",
        "position_mm": {"x": 0.0, "y": 0.0, "z": 145.0},
        "wire_outer_top_z_mm": 147.0,
        "wire_outer_bottom_z_mm": 143.0,
        "bend_radius_centerline_mm": None,
        "bend_radius_inner_mm": None,
        "bend_radius_outer_mm": None,
        "arc_center_mm": None,
        "safe_robot_entry_x_mm": 8.0,
        "notes": "Wire emerges horizontally from inner face of Left MDF post at X=0.0 mm; robot must start loop at X >= 8.0 mm to clear post thickness and wand collar."
    },
    {
        "id": "V_1",
        "name": "Valley 1 (Bottom Trough 1)",
        "type": "valley",
        "position_mm": {"x": 31.5, "y": 0.0, "z": 38.0},
        "wire_outer_top_z_mm": 40.0,
        "wire_outer_bottom_z_mm": 36.0,
        "bend_radius_centerline_mm": 13.75,
        "bend_radius_inner_mm": 11.75,
        "bend_radius_outer_mm": 15.75,
        "arc_center_mm": {"x": 31.5, "y": 0.0, "z": 51.75},
        "leg_to_leg_center_span_mm": 27.5,
        "leg_to_leg_inner_gap_mm": 23.5,
        "notes": "Tightest U-bend in the track; requires smooth end-effector pitch rotation from -77 deg through 0 deg to +77 deg."
    },
    {
        "id": "P_1",
        "name": "Peak 1 (Top Crest 1)",
        "type": "peak",
        "position_mm": {"x": 60.0, "y": 0.0, "z": 145.0},
        "wire_outer_top_z_mm": 147.0,
        "wire_outer_bottom_z_mm": 143.0,
        "bend_radius_centerline_mm": 14.25,
        "bend_radius_inner_mm": 12.25,
        "bend_radius_outer_mm": 16.25,
        "arc_center_mm": {"x": 60.0, "y": 0.0, "z": 130.75},
        "leg_to_leg_center_span_mm": 28.5,
        "leg_to_leg_inner_gap_mm": 24.5,
        "notes": "First upper crest; matches left post entry height."
    },
    {
        "id": "V_2",
        "name": "Valley 2 (Bottom Trough 2)",
        "type": "valley",
        "position_mm": {"x": 88.5, "y": 0.0, "z": 34.0},
        "wire_outer_top_z_mm": 36.0,
        "wire_outer_bottom_z_mm": 32.0,
        "bend_radius_centerline_mm": 14.50,
        "bend_radius_inner_mm": 12.50,
        "bend_radius_outer_mm": 16.50,
        "arc_center_mm": {"x": 88.5, "y": 0.0, "z": 48.50},
        "leg_to_leg_center_span_mm": 29.0,
        "leg_to_leg_inner_gap_mm": 25.0,
        "notes": "Second lower trough."
    },
    {
        "id": "P_2",
        "name": "Peak 2 (Tallest Central Crest)",
        "type": "peak",
        "position_mm": {"x": 118.5, "y": 0.0, "z": 157.0},
        "wire_outer_top_z_mm": 159.0,
        "wire_outer_bottom_z_mm": 155.0,
        "bend_radius_centerline_mm": 15.50,
        "bend_radius_inner_mm": 13.50,
        "bend_radius_outer_mm": 17.50,
        "arc_center_mm": {"x": 118.5, "y": 0.0, "z": 141.50},
        "leg_to_leg_center_span_mm": 31.0,
        "leg_to_leg_inner_gap_mm": 27.0,
        "notes": "Global Z maximum of the wire track (157.0 mm centerline, 159.0 mm outer top), rising 5 mm above the MDF post tops (154.0 mm)."
    },
    {
        "id": "V_3",
        "name": "Valley 3 (Lowest Trough / Rocker Switch Zone)",
        "type": "valley",
        "position_mm": {"x": 154.0, "y": 0.0, "z": 25.5},
        "wire_outer_top_z_mm": 27.5,
        "wire_outer_bottom_z_mm": 23.5,
        "bend_radius_centerline_mm": 19.75,
        "bend_radius_inner_mm": 17.75,
        "bend_radius_outer_mm": 21.75,
        "arc_center_mm": {"x": 154.0, "y": 0.0, "z": 45.25},
        "leg_to_leg_center_span_mm": 39.5,
        "leg_to_leg_inner_gap_mm": 35.5,
        "notes": "Global Z minimum of the wire track; bottom outer loop edge descends to Z ~ 11.0 mm, clearing the red rocker switch (Z_max = 6.5 mm at X = 157-173 mm) by ~4.5 mm."
    },
    {
        "id": "P_3",
        "name": "Peak 3 (Top Crest 3)",
        "type": "peak",
        "position_mm": {"x": 190.0, "y": 0.0, "z": 152.0},
        "wire_outer_top_z_mm": 154.0,
        "wire_outer_bottom_z_mm": 150.0,
        "bend_radius_centerline_mm": 16.50,
        "bend_radius_inner_mm": 14.50,
        "bend_radius_outer_mm": 18.50,
        "arc_center_mm": {"x": 190.0, "y": 0.0, "z": 135.50},
        "leg_to_leg_center_span_mm": 33.0,
        "leg_to_leg_inner_gap_mm": 29.0,
        "notes": "Third upper crest."
    },
    {
        "id": "V_4",
        "name": "Valley 4 (Buzzer Trough)",
        "type": "valley",
        "position_mm": {"x": 226.0, "y": 0.0, "z": 33.0},
        "wire_outer_top_z_mm": 35.0,
        "wire_outer_bottom_z_mm": 31.0,
        "bend_radius_centerline_mm": 19.50,
        "bend_radius_inner_mm": 17.50,
        "bend_radius_outer_mm": 21.50,
        "arc_center_mm": {"x": 226.0, "y": 0.0, "z": 52.50},
        "leg_to_leg_center_span_mm": 39.0,
        "leg_to_leg_inner_gap_mm": 35.0,
        "notes": "Fourth lower trough, positioned just past the piezo buzzer (X = 218 mm)."
    },
    {
        "id": "S_R",
        "name": "Right Post Exit (Goal)",
        "type": "endpoint",
        "position_mm": {"x": 280.0, "y": 0.0, "z": 144.0},
        "wire_outer_top_z_mm": 146.0,
        "wire_outer_bottom_z_mm": 142.0,
        "bend_radius_centerline_mm": None,
        "bend_radius_inner_mm": None,
        "bend_radius_outer_mm": None,
        "arc_center_mm": None,
        "safe_robot_exit_x_mm": 272.0,
        "notes": "Wire terminates into inner face of Right MDF post at X=280.0 mm; robot stops trajectory at X <= 272.0 mm to prevent post collision."
    }
]

# 2. Mid-height leg cross-sections measured directly against horizontal ruler (Image 14)
mid_height_legs = [
    {"leg_id": "L1", "segment": "S_L -> V_1", "direction": "descending", "x_centerline_mm": 18.0, "x_left_edge_mm": 16.0, "x_right_edge_mm": 20.0, "z_reference_mm": 82.0, "slope_angle_deg": -76.5},
    {"leg_id": "L2", "segment": "V_1 -> P_1", "direction": "ascending",  "x_centerline_mm": 45.5, "x_left_edge_mm": 43.5, "x_right_edge_mm": 47.5, "z_reference_mm": 82.0, "slope_angle_deg": 77.2},
    {"leg_id": "L3", "segment": "P_1 -> V_2", "direction": "descending", "x_centerline_mm": 74.0, "x_left_edge_mm": 72.0, "x_right_edge_mm": 76.0, "z_reference_mm": 82.0, "slope_angle_deg": -77.6},
    {"leg_id": "L4", "segment": "V_2 -> P_2", "direction": "ascending",  "x_centerline_mm": 103.0, "x_left_edge_mm": 101.0, "x_right_edge_mm": 105.0, "z_reference_mm": 82.0, "slope_angle_deg": 78.1},
    {"leg_id": "L5", "segment": "P_2 -> V_3", "direction": "descending", "x_centerline_mm": 134.0, "x_left_edge_mm": 132.0, "x_right_edge_mm": 136.0, "z_reference_mm": 82.0, "slope_angle_deg": -76.8},
    {"leg_id": "L6", "segment": "V_3 -> P_3", "direction": "ascending",  "x_centerline_mm": 173.5, "x_left_edge_mm": 171.5, "x_right_edge_mm": 175.5, "z_reference_mm": 82.0, "slope_angle_deg": 76.1},
    {"leg_id": "L7", "segment": "P_3 -> V_4", "direction": "descending", "x_centerline_mm": 206.5, "x_left_edge_mm": 204.5, "x_right_edge_mm": 208.5, "z_reference_mm": 82.0, "slope_angle_deg": -75.2},
    {"leg_id": "L8", "segment": "V_4 -> S_R", "direction": "ascending",  "x_centerline_mm": 245.5, "x_left_edge_mm": 243.5, "x_right_edge_mm": 247.5, "z_reference_mm": 82.0, "slope_angle_deg": 64.1}
]

# 3. Generate continuous geometric centerline path (straight tangent segments + circular U-bend arcs)
# Build exact piecewise C1-continuous 2D curve in (X, Z) plane
def build_c1_wire_path(kps, step_mm=2.0):
    # Extract centers and radii for V_1..V_4, P_1..P_3
    # Start point S_L = (0, 145), End point S_R = (280, 144)
    nodes = [
        {"type": "point", "p": np.array([0.0, 145.0])},
        {"type": "valley", "c": np.array([31.5, 51.75]), "r": 13.75},
        {"type": "peak",   "c": np.array([60.0, 130.75]), "r": 14.25},
        {"type": "valley", "c": np.array([88.5, 48.50]), "r": 14.50},
        {"type": "peak",   "c": np.array([118.5, 141.50]), "r": 15.50},
        {"type": "valley", "c": np.array([154.0, 45.25]), "r": 19.75},
        {"type": "peak",   "c": np.array([190.0, 135.50]), "r": 16.50},
        {"type": "valley", "c": np.array([226.0, 52.50]), "r": 19.50},
        {"type": "point", "p": np.array([280.0, 144.0])}
    ]

    # Compute exact internal bitangents between consecutive circles (and point-to-circle tangents at ends)
    segments = []
    # Point S_L to Valley V_1 (entering from left, i.e., tangent point on left side of V_1)
    p0 = nodes[0]["p"]
    c1, r1 = nodes[1]["c"], nodes[1]["r"]
    v = c1 - p0
    d = np.linalg.norm(v)
    ang = math.atan2(v[1], v[0])
    # Left side of valley means normal points toward (-sin, cos) -> alpha = ang - pi/2 + asin(r1/d)
    # Check: for valley, we enter at angle near pi (left) and sweep counterclockwise to ~0 (right)
    phi = math.asin(r1 / d)
    n_in = np.array([-math.cos(ang + phi - math.pi/2), -math.sin(ang + phi - math.pi/2)])
    # Simpler exact parameterization: normal angle theta_in on circle 1
    # Line tangent to circle (c1, r1) with outward unit normal u = (cos th, sin th) satisfies (p0 - c1).u = r1
    # Since p0 - c1 = (-31.5, 93.25), angle(p0 - c1) = 1.8966 rad (108.66 deg)
    # cos(th - ang_p0) = r1 / d => th = ang_p0 + acos(r1 / d) (around 108.66 + 81.98 = 190.64 deg -> left side!)
    ang_p0 = math.atan2(p0[1] - c1[1], p0[0] - c1[0])
    th1_in = ang_p0 + math.acos(r1 / d)
    t1_in = c1 + r1 * np.array([math.cos(th1_in), math.sin(th1_in)])
    segments.append({"kind": "line", "p_start": p0, "p_end": t1_in})

    prev_th_in = th1_in

    for i in range(1, len(nodes) - 2):
        curr = nodes[i]
        nxt = nodes[i + 1]
        cA, rA = curr["c"], curr["r"]
        cB, rB = nxt["c"], nxt["r"]
        # Internal bitangent between circle A and circle B:
        # Outward unit normal u = (cos th, sin th) on A has outward normal -u on B
        # (cB - cA) . u = rA + rB
        diff = cB - cA
        dist = np.linalg.norm(diff)
        ang_AB = math.atan2(diff[1], diff[0])
        delta = math.acos((rA + rB) / dist)
        if curr["type"] == "valley":
            # Exiting valley on its right side -> th_out near 0 rad (-0.2 to +0.2 rad)
            thA_out = ang_AB - delta
            # Arc on valley sweeps counterclockwise from prev_th_in (around pi) through 3pi/2 to thA_out + 2pi
            th_start = prev_th_in
            th_end = thA_out + 2 * math.pi
            while th_end <= th_start:
                th_end += 2 * math.pi
            segments.append({"kind": "arc", "c": cA, "r": rA, "th_start": th_start, "th_end": th_end})
            # On next peak B, normal is -u, so thB_in = thA_out + pi (around pi, left side of peak!)
            thB_in = (thA_out + math.pi) % (2 * math.pi)
        else:
            # curr is peak: exiting peak on its right side -> th_out near 0 rad
            thA_out = ang_AB + delta
            # Arc on peak sweeps clockwise from prev_th_in (around pi) through pi/2 to thA_out (near 0)
            th_start = prev_th_in
            th_end = thA_out
            while th_end >= th_start:
                th_end -= 2 * math.pi
            segments.append({"kind": "arc", "c": cA, "r": rA, "th_start": th_start, "th_end": th_end})
            thB_in = (thA_out + math.pi) % (2 * math.pi)

        pA_out = cA + rA * np.array([math.cos(thA_out), math.sin(thA_out)])
        pB_in = cB + rB * np.array([math.cos(thB_in), math.sin(thB_in)])
        segments.append({"kind": "line", "p_start": pA_out, "p_end": pB_in})
        prev_th_in = thB_in

    # Final arc on Valley V_4 and line to S_R
    cLast, rLast = nodes[-2]["c"], nodes[-2]["r"]
    pEnd = nodes[-1]["p"]
    ang_pEnd = math.atan2(pEnd[1] - cLast[1], pEnd[0] - cLast[0])
    dEnd = np.linalg.norm(pEnd - cLast)
    thLast_out = ang_pEnd - math.acos(rLast / dEnd)
    th_start = prev_th_in
    th_end = thLast_out + 2 * math.pi
    while th_end <= th_start:
        th_end += 2 * math.pi
    segments.append({"kind": "arc", "c": cLast, "r": rLast, "th_start": th_start, "th_end": th_end})
    pLast_out = cLast + rLast * np.array([math.cos(thLast_out), math.sin(thLast_out)])
    segments.append({"kind": "line", "p_start": pLast_out, "p_end": pEnd})

    # Sample densely at uniform arc-length intervals (step_mm)
    waypoints = []
    arc_length = 0.0
    wp_idx = 0

    for seg in segments:
        if seg["kind"] == "line":
            pA, pB = seg["p_start"], seg["p_end"]
            L = np.linalg.norm(pB - pA)
            n_steps = max(1, int(round(L / step_mm)))
            tangent_2d = (pB - pA) / L
            for k in range(n_steps):
                frac = k / n_steps
                pt = pA + frac * (pB - pA)
                s_val = arc_length + frac * L
                waypoints.append((wp_idx, s_val, pt, tangent_2d, "linear_segment"))
                wp_idx += 1
            arc_length += L
        else:
            c, r = seg["c"], seg["r"]
            th0, th1 = seg["th_start"], seg["th_end"]
            dth = th1 - th0
            L = abs(dth) * r
            n_steps = max(2, int(round(L / step_mm)))
            sgn = 1.0 if dth >= 0 else -1.0
            for k in range(n_steps):
                frac = k / n_steps
                th = th0 + frac * dth
                pt = c + r * np.array([math.cos(th), math.sin(th)])
                tangent_2d = sgn * np.array([-math.sin(th), math.cos(th)])
                s_val = arc_length + frac * L
                waypoints.append((wp_idx, s_val, pt, tangent_2d, "bend_arc"))
                wp_idx += 1
            arc_length += L

    # Add final endpoint S_R
    last_tan = segments[-1]["p_end"] - segments[-1]["p_start"]
    last_tan = last_tan / np.linalg.norm(last_tan)
    waypoints.append((wp_idx, arc_length, nodes[-1]["p"], last_tan, "endpoint"))

    # Format into structured waypoint dictionaries with 3D frames & quaternions
    formatted_wps = []
    for idx, s_val, pt, tan2d, seg_type in waypoints:
        tx, tz = float(tan2d[0]), float(tan2d[1])
        # Wire tangent in 3D: T = [tx, 0, tz]
        # Loop plane normal should align with T so wire passes perpendicularly through loop center!
        # Let Tool Frame {T} have:
        #   Z_tool = T = [tx, 0, tz] (loop axis / wire feed direction)
        #   Y_tool = [0, 1, 0] (aligned with ground +Y)
        #   X_tool = Y_tool x Z_tool = [tz, 0, -tx]
        # This corresponds to a pure pitch rotation around Y-axis by angle theta_y = atan2(tx, tz)
        pitch_rad = math.atan2(tx, tz)
        pitch_deg = math.degrees(pitch_rad)
        # Quaternion [qx, qy, qz, qw] for rotation by pitch_rad around Y-axis:
        qy = math.sin(pitch_rad / 2.0)
        qw = math.cos(pitch_rad / 2.0)

        in_safe_span = (8.0 <= pt[0] <= 272.0)
        formatted_wps.append({
            "index": idx,
            "arc_length_mm": round(s_val, 2),
            "segment_type": seg_type,
            "active_navigation_zone": in_safe_span,
            "position_mm": {
                "x": round(float(pt[0]), 2),
                "y": 0.0,
                "z": round(float(pt[1]), 2)
            },
            "unit_tangent_vector": {
                "tx": round(tx, 5),
                "ty": 0.0,
                "tz": round(tz, 5)
            },
            "tool_orientation": {
                "pitch_y_deg": round(pitch_deg, 2),
                "pitch_y_rad": round(pitch_rad, 5),
                "euler_xyz_deg": [0.0, round(pitch_deg, 2), 0.0],
                "quaternion_xyzw": [0.0, round(qy, 6), 0.0, round(qw, 6)]
            }
        })
    return round(arc_length, 2), formatted_wps

total_wire_length_mm, dense_waypoints = build_c1_wire_path(keypoints, step_mm=5.0)
print(f"Total calibrated wire path length: {total_wire_length_mm} mm across {len(dense_waypoints)} waypoints.")

calibration_data = {
    "schema_version": "1.0.0",
    "project": "Autonomous Buzz Wire Navigation — Ground Frame & Path Calibration",
    "calibration_date": "2026-11-12",
    "units": {
        "linear": "millimeters (mm)",
        "angular": "degrees (deg) and radians (rad)",
        "coordinate_system": "Right-handed 3D Cartesian"
    },
    "ground_reference_frame": {
        "frame_id": "G",
        "origin_definition": {
            "x_zero": "Inner face of the Left vertical MDF post (plain post without red wire terminal)",
            "y_zero": "Vertical symmetry plane containing the copper wire centerline",
            "z_zero": "Top horizontal surface of the wooden base plate"
        },
        "axes": {
            "+X": "Horizontal along the wire span from Left post (X = 0.0 mm) toward Right post (X = 280.0 mm)",
            "+Y": "Horizontal across the base board depth, perpendicular to the wire plane",
            "+Z": "Vertical upward normal to the wooden base plate"
        },
        "robot_base_transform_template": {
            "description": "Homogeneous transformation T_Base_to_G (4x4 matrix) mapping Ground Frame {G} points P_G into Robot Base Frame {B}: P_B = T_Base_to_G * P_G. Populate translation [tx, ty, tz] and rotation R after touching 3 reference points on the base board (Origin O, Right Post Base [280, 0, 0], and Front-Left Base Corner).",
            "matrix_4x4": [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]
            ],
            "calibration_fiducials_in_G_mm": [
                {"name": "Origin_O_Left_Post_Inner_Base", "x": 0.0, "y": 0.0, "z": 0.0},
                {"name": "Right_Post_Inner_Base", "x": 280.0, "y": 0.0, "z": 0.0},
                {"name": "Left_Post_Top_Inner_Edge", "x": 0.0, "y": 0.0, "z": 154.0},
                {"name": "Right_Post_Top_Inner_Edge", "x": 280.0, "y": 0.0, "z": 154.0}
            ]
        }
    },
    "physical_hardware_metrics": {
        "base_board": {
            "length_x_mm": 330.0,
            "x_range_in_G_mm": [-25.0, 305.0],
            "width_y_mm": 110.0,
            "y_range_in_G_mm": [-55.0, 55.0],
            "top_surface_z_mm": 0.0
        },
        "support_posts": {
            "material": "Laser-cut MDF",
            "inner_post_to_post_span_mm": 280.0,
            "outer_post_to_post_span_mm": 288.0,
            "post_thickness_x_mm": 4.0,
            "post_width_y_mm": 35.0,
            "post_height_above_base_z_mm": 154.0,
            "left_post_x_bounds_mm": [-4.0, 0.0],
            "right_post_x_bounds_mm": [280.0, 284.0]
        },
        "copper_track_wire": {
            "wire_diameter_mm": 4.0,
            "wire_radius_mm": 2.0,
            "total_centerline_arc_length_mm": total_wire_length_mm,
            "num_peaks": 3,
            "num_valleys": 4,
            "z_min_centerline_mm": 25.5,
            "z_max_centerline_mm": 157.0
        },
        "end_effector_loop": {
            "shape": "Slightly elliptical hand-formed copper ring",
            "major_inner_diameter_mm": 24.0,
            "minor_inner_diameter_mm": 21.0,
            "mean_inner_diameter_mm": 22.5,
            "loop_wire_gauge_diameter_mm": 4.0,
            "major_outer_diameter_mm": 32.0,
            "minor_outer_diameter_mm": 29.0,
            "effective_governing_inner_radius_mm": 10.5
        },
        "clearance_and_gap_analysis": {
            "nominal_radial_air_gap_minor_axis_mm": 8.5,
            "nominal_radial_air_gap_major_axis_mm": 10.0,
            "formula": "radial_air_gap = (D_loop_inner - d_wire) / 2",
            "minimum_leg_to_leg_inner_gap_mm": 23.5,
            "maximum_leg_to_leg_inner_gap_mm": 35.5,
            "minimum_base_clearance_at_V3_mm": 4.5,
            "critical_collision_constraints": {
                "max_lateral_y_error_mm": 6.5,
                "max_vertical_z_error_mm": 6.5,
                "max_uncompensated_tilt_angle_deg": 45.0,
                "rationale": "Because loop minor inner radius (10.5 mm) minus wire radius (2.0 mm) yields 8.5 mm nominal clearance, keeping the loop strictly vertical (pitch = 0 deg) on 77-degree wire legs shrinks projected horizontal clearance to 10.5*cos(77 deg) - 2.0 = 0.36 mm. Continuous pitch alignment with the wire tangent is mandatory."
            }
        },
        "board_mounted_obstacles": [
            {
                "name": "Red Rocker Power Switch",
                "x_bounds_mm": [157.0, 173.0],
                "y_bounds_mm": [-26.0, -10.0],
                "z_bounds_mm": [0.0, 6.5],
                "hazard_note": "Located directly beneath Valley V_3 (X=154.0 mm, Z_center=25.5 mm). Bottom outer edge of loop reaches Z ~ 11.0 mm, leaving 4.5 mm vertical clearance above switch."
            },
            {
                "name": "Piezo Buzzer",
                "x_bounds_mm": [212.0, 224.0],
                "y_bounds_mm": [-24.0, -12.0],
                "z_bounds_mm": [0.0, 5.5],
                "hazard_note": "Offset in -Y from the wire plane (Y = 0); safe as long as tool handle stays on +Y or neutral side."
            }
        ]
    },
    "keypoints": keypoints,
    "mid_height_leg_measurements": mid_height_legs,
    "navigation_trajectory": {
        "sampling_interval_mm": 5.0,
        "waypoint_count": len(dense_waypoints),
        "recommended_linear_speed_mm_per_s": {
            "straight_segments": 25.0,
            "u_bend_arcs": 12.0
        },
        "tool_frame_convention": {
            "Z_tool": "Aligned with wire unit tangent vector T = (tx, 0, tz) in direction of travel from S_L to S_R",
            "Y_tool": "Aligned with Ground Frame +Y axis (0, 1, 0)",
            "X_tool": "Completes right-handed frame: X_tool = Y_tool x Z_tool = (tz, 0, -tx)"
        },
        "waypoints": dense_waypoints
    },
    "source_image_calibration_log": [
        {"image_index": 1,  "filename": "01_front_right_overview_ruler.png", "measurement_role": "Valley V_4 height reference and overall board layout"},
        {"image_index": 2,  "filename": "02_full_front_elevation_peak2_ruler.png", "measurement_role": "Full front elevation, Peak P_2 max height (159 mm outer top), Valley V_3 clearance over red switch"},
        {"image_index": 3,  "filename": "03_peak3_height_ruler.png", "measurement_role": "Peak P_3 height (154 mm outer top) and wire gauge thickness"},
        {"image_index": 4,  "filename": "04_valley3_switch_clearance_ruler.png", "measurement_role": "Valley V_3 lowest trough height (23.5 mm outer bottom) and switch height"},
        {"image_index": 5,  "filename": "05_valley4_right_post_ruler.png", "measurement_role": "Valley V_4 height (31 mm outer bottom) and Right post entry height (146 mm outer top)"},
        {"image_index": 6,  "filename": "06_valley2_peak2_ruler.png", "measurement_role": "Valley V_2 height (32 mm outer bottom) and Peak P_2 verification"},
        {"image_index": 7,  "filename": "07_center_span_ruler.png", "measurement_role": "Central wave symmetry check between P_2, V_3, and P_3"},
        {"image_index": 8,  "filename": "08_left_post_valley1_ruler.png", "measurement_role": "Left post entry height (143 mm outer bottom) and Valley V_1 height (36 mm outer bottom)"},
        {"image_index": 9,  "filename": "09_top_full_span_ruler.png", "measurement_role": "Top-level horizontal span check across all peaks and posts"},
        {"image_index": 10, "filename": "10_peak1_valley2_ruler.png", "measurement_role": "Peak P_1 height (147 mm outer top)"},
        {"image_index": 11, "filename": "11_baseplate_horizontal_ruler.png", "measurement_role": "Exact inner post-to-post span (280.0 mm), base board X-bounds (-25 to 305 mm), and switch/buzzer X-coordinates"},
        {"image_index": 12, "filename": "12_loop_major_diameter_ruler.png", "measurement_role": "Loop major inner diameter (24.0 mm), outer diameter (32.0 mm), and loop wire diameter (4.0 mm)"},
        {"image_index": 13, "filename": "13_loop_minor_diameter_ruler.png", "measurement_role": "Loop minor inner diameter (21.0 mm) and outer diameter (29.0 mm)"},
        {"image_index": 14, "filename": "14_midheight_leg_spacing_ruler.png", "measurement_role": "Exact X-coordinates of all 8 wire legs (L1-L8) at mid-height (Z ~ 82 mm) and exact leg-to-leg inner gaps"}
    ]
}

json_filename = "buzz_wire_calibration.json"
with open(json_filename, "w", encoding="utf-8") as f:
    json.dump(calibration_data, f, indent=2)

# 4. Write comprehensive Markdown documentation explaining every element of the JSON for the robot & engineering team
md_content = """# Autonomous Buzz Wire Navigation — Calibration Data & Robot Control Specification

## 1. Purpose & System Overview

This document explains the complete geometric, kinematic, and safety calibration dataset stored in **`buzz_wire_calibration.json`**. The dataset enables a robotic manipulator holding a conductive loop wand to traverse a 3D copper buzz-wire track from start (`S_L`) to finish (`S_R`) without making electrical contact.

All numerical values in `buzz_wire_calibration.json` were calibrated directly from **14 multi-angle millimeter-scale ruler photographs** (`images/01` through `images/14`) covering:
- **Exact inner post-to-post span** (`280.0 mm`) measured flush along the base plate (`Image 11`).
- **Mid-height leg-to-leg X-coordinates** (`L1` through `L8`) and inner U-bend clearances (`Image 14`).
- **Peak and valley vertical heights** (`Z`) relative to the wooden base plate (`Images 1–10`).
- **End-effector loop elliptical inner/outer diameters** (`24.0 mm × 21.0 mm` inner; `4.0 mm` wire gauge) (`Images 12–13`).

---

## 2. Ground Reference Frame `{G}` Definition

Every coordinate in `buzz_wire_calibration.json` is expressed in millimeters (`mm`) in a right-handed Cartesian frame **`{G}`** rigidly attached to the wooden game base:

| Axis / Point | Physical Definition on Hardware |
| :--- | :--- |
| **Origin `O = (0, 0, 0)`** | Intersection of the **inner face of the Left MDF post** (`X = 0`), the **vertical symmetry plane of the copper wire** (`Y = 0`), and the **top surface of the wooden base plate** (`Z = 0`). |
| **`+X` Axis** | Horizontal along the wire span from the Left post (`X = 0.0 mm`) toward the Right post (`X = 280.0 mm`). |
| **`+Y` Axis** | Horizontal across the depth of the wooden base board, perpendicular to the wire plane (`Y ∈ [-55.0, +55.0] mm`). |
| **`+Z` Axis** | Vertical upward normal to the top surface of the wooden base plate. |

### Registering `{G}` to the Robot Base Frame `{B}`
Before executing the trajectory, touch off at least three non-collinear fiducial points listed under `ground_reference_frame.robot_base_transform_template.calibration_fiducials_in_G_mm`:
1. **`Origin_O_Left_Post_Inner_Base`**: `(0.0, 0.0, 0.0)`
2. **`Right_Post_Inner_Base`**: `(280.0, 0.0, 0.0)`
3. **`Left_Post_Top_Inner_Edge`**: `(0.0, 0.0, 154.0)`

Compute the homogeneous rigid-body transform $\mathbf{T}_{B \leftarrow G} \in SE(3)$ and store it in `matrix_4x4`. Every waypoint $\mathbf{p}_G = [x, y, z, 1]^T$ maps to robot base coordinates via $\mathbf{p}_B = \mathbf{T}_{B \leftarrow G}\,\mathbf{p}_G$.

---

## 3. Calibrated Keypoints (`keypoints` Array)

The copper track consists of **2 post endpoints**, **4 valleys (`V_1`–`V_4`)**, and **3 peaks (`P_1`–`P_3`)**. All `Z` values in `position_mm` represent the **true wire centerline** (adjusted by $\pm 2.0\text{ mm}$ from measured outer edges using the verified $d_w = 4.0\text{ mm}$ wire diameter):

| ID | Feature | `X` (mm) | `Y` (mm) | `Z_center` (mm) | Bend Radius `R_center` (mm) | Arc Center `(X, Z)` (mm) | Leg Inner Gap (mm) |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **`S_L`** | Left Post Entry | `0.0` | `0.0` | `145.0` | — | — | — |
| **`V_1`** | Valley 1 | `31.5` | `0.0` | `38.0` | `13.75` | `(31.5, 51.75)` | `23.5` |
| **`P_1`** | Peak 1 | `60.0` | `0.0` | `145.0` | `14.25` | `(60.0, 130.75)` | `24.5` |
| **`V_2`** | Valley 2 | `88.5` | `0.0` | `34.0` | `14.50` | `(88.5, 48.50)` | `25.0` |
| **`P_2`** | Peak 2 (Tallest) | `118.5` | `0.0` | `157.0` | `15.50` | `(118.5, 141.50)` | `27.0` |
| **`V_3`** | Valley 3 (Lowest) | `154.0` | `0.0` | `25.5` | `19.75` | `(154.0, 45.25)` | `35.5` |
| **`P_3`** | Peak 3 | `190.0` | `0.0` | `152.0` | `16.50` | `(190.0, 135.50)` | `29.0` |
| **`V_4`** | Valley 4 | `226.0` | `0.0` | `33.0` | `19.50` | `(226.0, 52.50)` | `35.0` |
| **`S_R`** | Right Post Exit | `280.0` | `0.0` | `144.0` | — | — | — |

### Key Geometric Observations
- **Asymmetric Pitch Distribution**: The left half of the track (`S_L` through `P_2`) has tighter horizontal spacing (`28.5–30.0 mm` half-periods and `23.5–27.0 mm` inner leg gaps), whereas the right half (`P_2` through `V_4`) widens (`35.5–36.0 mm` half-periods and `29.0–35.5 mm` inner leg gaps).
- **Final Ascent (`V_4 -> S_R`)**: The final segment rises over a longer `54.0 mm` horizontal span (`64.1°` inclination vs. `75°–78°` on legs `L1`–`L7`).
- **Safe Start/Stop Buffer**: Although the wire spans `X ∈ [0.0, 280.0] mm`, the robot must begin at **`X = 8.0 mm`** and terminate at **`X = 272.0 mm`** (`active_navigation_zone: true`) so the `4.0 mm` thick loop ring never collides with the MDF support posts.

---

## 4. Loop Geometry, Radial Air Gap & Collision Mechanics

Direct close-up measurements (`Images 12–13`) revealed that the hand-bent copper loop is **slightly elliptical**:
- **Major Inner Diameter**: `24.0 mm` (`R_major = 12.0 mm`)
- **Minor Inner Diameter**: `21.0 mm` (`R_minor = 10.5 mm`)
- **Loop Wire Gauge Diameter**: `4.0 mm` (`29.0 mm × 32.0 mm` outer dimensions)
- **Track Wire Diameter**: `4.0 mm` (`R_wire = 2.0 mm`)

### Governing Radial Clearance ($\delta_r$)
Using the conservative minor inner radius (`10.5 mm`) to guarantee collision-free operation regardless of loop roll angle:
$$\\delta_r = R_{\\text{loop,inner,minor}} - R_{\\text{wire}} = 10.5\\text{ mm} - 2.0\\text{ mm} = 8.5\\text{ mm}$$

### Why Continuous Tool Pitch Alignment Is Mandatory
If the robot holds the loop plane strictly vertical ($\theta_y = 0^\circ$) while traversing a steep $77^\circ$ leg (`L1`–`L7`), the effective horizontal aperture shrinks by $\cos(77^\circ) \approx 0.225$:
$$\\delta_{x,\\text{vertical}} = 10.5\\cos(77^\\circ) - 2.0 = 2.36 - 2.0 = 0.36\\text{ mm}$$
An uncompensated vertical loop has virtually **zero margin** (`0.36 mm`) and will trigger the buzzer. By pitching the end-effector around the `Y`-axis so the loop axis (`Z_tool`) tracks the wire tangent $\hat{\mathbf{t}} = (t_x, 0, t_z)$, the full **`8.5 mm` radial air gap** is preserved along the entire `690.13 mm` path.

### Base-Plate Obstacle Clearances
- **Red Rocker Switch (`X ∈ [157, 173] mm`, `Z_max = 6.5 mm`)**: Sits directly under Valley `V_3` (`Z_center = 25.5 mm`). At the bottom of `V_3`, the lowest outer edge of the loop reaches $Z = 25.5 - 10.5 - 4.0 = 11.0\text{ mm}$, clearing the rocker switch by **`4.5 mm`**.
- **Piezo Buzzer (`X ∈ [212, 224] mm`, `Y ∈ [-24, -12] mm`)**: Offset in `-Y` from the wire plane (`Y = 0`), posing no interference provided the robot wand approaches from `+Y` or neutral.

---

## 5. JSON Schema Reference (`navigation_trajectory`)

The `navigation_trajectory.waypoints` array contains **139 $C^1$-continuous waypoints** sampled every `5.0 mm` along the exact analytical centerline (straight tangent segments joined seamlessly to circular U-bend arcs):

- **`index`**: Sequential waypoint ID (`0` to `138`).
- **`arc_length_mm`**: Cumulative distance $s$ along the wire from `S_L` (`0.0 mm` to `690.13 mm`).
- **`segment_type`**: Either `"linear_segment"`, `"bend_arc"`, or `"endpoint"`.
- **`active_navigation_zone`**: Boolean (`true` when `8.0 <= X <= 272.0 mm`); the robot should only traverse waypoints flagged `true`.
- **`position_mm`**: Target 3D coordinates `{x, y, z}` of the **center of the loop opening** in Frame `{G}`.
- **`unit_tangent_vector`**: Unit vector `{tx, ty, tz}` tangent to the wire in the direction of travel (`S_L -> S_R`).
- **`tool_orientation`**:
  - `pitch_y_deg` / `pitch_y_rad`: Pitch angle $\theta_y = \operatorname{atan2}(t_x, t_z)$ around Ground `+Y`.
  - `euler_xyz_deg`: `[0.0, pitch_y_deg, 0.0]`.
  - `quaternion_xyzw`: `[0.0, qy, 0.0, qw]` defining Tool Frame `{T}` where `Z_tool = T` (wire pass-through axis), `Y_tool = [0, 1, 0]`, and `X_tool = Y_tool × Z_tool`.

---

## 6. Archive Manifest (`buzz_wire_robot_calibration_package.zip`)

- `buzz_wire_calibration.json` — Complete machine-readable calibration & waypoint dataset.
- `buzz_wire_calibration_guide.md` — This technical documentation file.
- `images/01_front_right_overview_ruler.png` … `images/14_midheight_leg_spacing_ruler.png` — All 14 reference photographs indexed in `source_image_calibration_log`.
"""

md_filename = "buzz_wire_calibration_guide.md"
with open(md_filename, "w", encoding="utf-8") as f:
    f.write(md_content)

# 5. Copy all 14 images with descriptive indexed names into a staging folder and create the ZIP archive
os.makedirs("package_staging/images", exist_ok=True)
shutil.copy2(json_filename, f"package_staging/{json_filename}")
shutil.copy2(md_filename, f"package_staging/{md_filename}")

image_mapping = [
    ("input_file_0.png",  "01_front_right_overview_ruler.png"),
    ("input_file_1.png",  "02_full_front_elevation_peak2_ruler.png"),
    ("input_file_2.png",  "03_peak3_height_ruler.png"),
    ("input_file_3.png",  "04_valley3_switch_clearance_ruler.png"),
    ("input_file_4.png",  "05_valley4_right_post_ruler.png"),
    ("input_file_5.png",  "06_valley2_peak2_ruler.png"),
    ("input_file_6.png",  "07_center_span_ruler.png"),
    ("input_file_7.png",  "08_left_post_valley1_ruler.png"),
    ("input_file_8.png",  "09_top_full_span_ruler.png"),
    ("input_file_9.png",  "10_peak1_valley2_ruler.png"),
    ("input_file_10.png", "11_baseplate_horizontal_ruler.png"),
    ("input_file_11.png", "12_loop_major_diameter_ruler.png"),
    ("input_file_12.png", "13_loop_minor_diameter_ruler.png"),
    ("input_file_13.png", "14_midheight_leg_spacing_ruler.png"),
]

for src, dst in image_mapping:
    shutil.copy2(src, f"package_staging/images/{dst}")

zip_filename = "buzz_wire_robot_calibration_package.zip"
with zipfile.ZipFile(zip_filename, "w", zipfile.ZIP_DEFLATED) as zf:
    for root, dirs, files in os.walk("package_staging"):
        for file in sorted(files):
            full_path = os.path.join(root, file)
            rel_path = os.path.relpath(full_path, "package_staging")
            zf.write(full_path, rel_path)

print("Generated files:")
print(" - JSON size:", os.path.getsize(json_filename), "bytes")
print(" - MD size:", os.path.getsize(md_filename), "bytes")
print(" - ZIP size:", os.path.getsize(zip_filename), "bytes")
with zipfile.ZipFile(zip_filename, "r") as zf:
    print("ZIP contents:", zf.namelist())
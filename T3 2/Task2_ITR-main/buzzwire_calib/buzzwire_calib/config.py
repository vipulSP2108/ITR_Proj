from __future__ import annotations
import copy
from pathlib import Path
import yaml

DEFAULTS = {
    "board": {"squares_x": 7, "squares_y": 5, "square_length": 0.040, "marker_length": 0.030,
              "dictionary": "DICT_5X5_100"},
    "robot": {"base_link": "base_link", "tip_link": "end-effector",
              "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
              "joint_state_topic": "/joint_states", "robot_description_topic": "/robot_description",
              "ee_pose_topic": "", "velocity_command_topic": "/velocity_controller/commands"},
    "cameras": {
        "cam0": {"source": "0", "mount": "hand", "width": 1280, "height": 720, "fps": 30, "fourcc": "MJPG"},
        "cam1": {"source": "2", "mount": "fixed", "width": 1280, "height": 720, "fps": 30, "fourcc": "MJPG"},
    },
    "calibration": {"board_mount": "world"},
    "capture": {"burst_frames": 12, "frame_timeout_s": 2.0, "settle_time_s": 0.4, "max_joint_motion_rad": 0.004,
                "stale_robot_s": 0.5, "stale_cam_s": 1.0, "min_corners": 8, "min_good_frames": 4,
                "require": "all", "extra_frames": 2, "preview": True},
    "keys": {"capture": " ", "next_matrix": "n", "finish": "f", "undo": "u", "quit": "q", "status": "s"},
    "paths": {"data_root": "data", "results_root": "results"},
    "gripper": {"enabled": True, "workspace_path": "", "service_name": "/gripper_controller/command",
                "service_type": "addverb_cobot_msgs/srv/Gripper", "open_position": 1.0, "close_position": 0.0,
                "force": 100.0, "initial_state": "open"},
    "teleop": {"jog_frame_yaw_deg": 0.0, "lin_speeds_mmps": [2, 5, 10, 20, 40],
               "ang_speeds_degps": [3, 6, 12, 20, 30], "default_level": 2,
               "lin_accel": 0.10, "ang_accel": 0.6, "key_backend": "terminal", "key_hold_s": 0.12,
               "record_hz": 20.0, "workspace_box_m": None, "max_joint_vel": 0.35,
               "lead_lin_m": 0.012, "lead_ang_rad": 0.12, "kp_lin": 4.0, "kp_ang": 3.0,
               "tool_axis": [0, 0, 1], "plane_normal_base": [0, 1, 0], "auto_tangent": False,
               "joint_limits_rad": None, "joint_limit_margin_rad": 0.05},
    "wire": {"expected_length_m": 1.0},   # approx. centreline length of the 3D wire; refine after first full merge
    "replay": {"v_max_mps": 0.06, "a_max_mps2": 0.15, "a_lat_mps2": 0.10, "w_max_radps": 0.5,
               "max_joint_vel": 0.5, "approach_speed_mps": 0.03, "max_approach_m": 0.30},
}


def _merge(base, over):
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict) and k != "cameras":
            out[k] = _merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def load_config(path=None) -> dict:
    if path is None:
        return copy.deepcopy(DEFAULTS)
    with open(path) as f:
        return _merge(DEFAULTS, yaml.safe_load(f) or {})


def save_config(cfg: dict, path):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)

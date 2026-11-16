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

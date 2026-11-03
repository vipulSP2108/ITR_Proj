import numpy as np
from typing import Tuple, Optional, List

# OpenMANIPULATOR-X Physical Link Dimensions (in millimeters)
L1 = 77.0   # Base to Shoulder pitch axis
L2 = 130.0  # Shoulder pitch axis to Elbow pitch axis
L3 = 124.0  # Elbow pitch axis to Wrist pitch axis
L4 = 126.0  # Wrist pitch axis to Gripper tool center point (TCP)

# DYNAMIXEL XM430 Resolution (4096 counts = 360 degrees = 2*pi radians)
COUNTS_PER_RAD = 4096.0 / (2.0 * np.pi)
RAD_PER_COUNT  = (2.0 * np.pi) / 4096.0
CENTER_COUNT   = 2048

class OpenManipulatorKinematics:
    """
    Analytical Forward and Inverse Kinematics for ROBOTIS OpenMANIPULATOR-X.
    Uses closed-form trigonometry for instantaneous, zero-latency computation.
    
    Coordinate System (Robot Base Frame):
      • +X: Forward
      • +Y: Left
      • +Z: Upward
      • Origin (0,0,0): Center of robot mounting base on table
    """
    def __init__(self, l1=L1, l2=L2, l3=L3, l4=L4):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4

    @staticmethod
    def raw_to_radians(raw_counts: List[int]) -> List[float]:
        """Converts raw encoder counts (0 ~ 4095) to joint radians (centered at 2048 = 0 rad)."""
        # Joint 1: standard
        # Joint 2: relative to vertical zero
        # Joint 3: relative to collinear zero
        # Joint 4: relative to collinear zero
        return [(c - CENTER_COUNT) * RAD_PER_COUNT for c in raw_counts]

    @staticmethod
    def radians_to_raw(radians: List[float], base_wrap_count: int = 4096) -> List[int]:
        """Converts joint radians to raw DYNAMIXEL encoder counts."""
        # For base joint (joint 1), add to base_wrap_count to preserve multi-turn alignment
        q1_raw = int(round(radians[0] * COUNTS_PER_RAD)) + base_wrap_count
        q2_raw = int(round(radians[1] * COUNTS_PER_RAD)) + CENTER_COUNT
        q3_raw = int(round(radians[2] * COUNTS_PER_RAD)) + CENTER_COUNT
        q4_raw = int(round(radians[3] * COUNTS_PER_RAD)) + CENTER_COUNT
        return [q1_raw, q2_raw, q3_raw, q4_raw]

    def forward_kinematics(self, q1: float, q2: float, q3: float, q4: float) -> Tuple[float, float, float, float]:
        """
        Calculates end-effector (gripper center) Cartesian pose from joint angles (in radians).
        Returns: (x, y, z, pitch_rad) in millimeters and radians.
        """
        th2 = np.pi / 2.0 - q2
        th3 = -q3
        th4 = -q4
        pitch = th2 + th3 + th4

        r = self.l2 * np.cos(th2) + self.l3 * np.cos(th2 + th3) + self.l4 * np.cos(pitch)
        z = self.l1 + self.l2 * np.sin(th2) + self.l3 * np.sin(th2 + th3) + self.l4 * np.sin(pitch)

        x = r * np.cos(q1)
        y = r * np.sin(q1)

        return (float(x), float(y), float(z), float(pitch))

    def inverse_kinematics(self, target_x: float, target_y: float, target_z: float,
                           pitch_rad: float = -np.pi / 2.0) -> Optional[Tuple[float, float, float, float]]:
        """
        Calculates joint angles (q1, q2, q3, q4) in radians to place gripper TCP at (target_x, target_y, target_z)
        with desired gripper pitch angle (default: -pi/2 = -90 deg, pointing vertically downward for top-grasp).
        
        Returns: (q1, q2, q3, q4) in radians, or None if target is unreachable.
        """
        # 1. Base yaw (q1)
        q1 = np.arctan2(target_y, target_x)
        r_target = np.sqrt(target_x**2 + target_y**2)

        # 2. Wrist center position (subtract tool length L4 along pitch direction)
        r_wrist = r_target - self.l4 * np.cos(pitch_rad)
        z_wrist = target_z - self.l1 - self.l4 * np.sin(pitch_rad)

        # 3. Check reachability of wrist point
        d_sq = r_wrist**2 + z_wrist**2
        d = np.sqrt(d_sq)
        if d > (self.l2 + self.l3) or d < abs(self.l2 - self.l3) or d == 0:
            return None # Unreachable target

        # 4. Law of Cosines for Elbow
        cos_th3 = (d_sq - self.l2**2 - self.l3**2) / (2.0 * self.l2 * self.l3)
        if abs(cos_th3) > 1.0:
            return None
            
        th3 = np.arctan2(-np.sqrt(max(0.0, 1.0 - cos_th3**2)), cos_th3)
        th2 = np.arctan2(z_wrist, r_wrist) - np.arctan2(self.l3 * np.sin(th3), self.l2 + self.l3 * np.cos(th3))
        th4 = pitch_rad - th2 - th3

        # Convert planar link angles to DYNAMIXEL motor joint offsets
        q2 = np.pi / 2.0 - th2
        q3 = -th3
        q4 = -th4

        # Enforce physical joint limits of OpenMANIPULATOR-X
        # Joint 2 (Shoulder): [-90 deg, +90 deg]
        if abs(q2) > (np.pi / 2.0 + 0.05):
            return None
        # Joint 3 (Elbow): [-100 deg, +100 deg]
        if abs(q3) > (np.radians(105.0)):
            return None
        # Joint 4 (Wrist): [-105 deg, +105 deg]
        if abs(q4) > (np.radians(110.0)):
            return None

        return (float(q1), float(q2), float(q3), float(q4))

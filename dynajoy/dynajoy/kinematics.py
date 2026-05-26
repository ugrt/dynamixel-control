"""
Kinematics for Dynamixel Robot Arm
====================================
Pure-math library — no file I/O, no ROS imports.
All geometry lives in ArmConfig; the ROS node owns parameter loading.

Usage
-----
  from dynajoy.kinematics import ArmConfig, ArmKinematics

  cfg = ArmConfig(L1=437.0, L2=437.0, L3=220.0, base_height=123.0)
  kin = ArmKinematics(cfg)

  x, y, phi, T = kin.forward_kinematics(theta1, theta2, theta3)
  result       = kin.inverse_kinematics(x_target, y_target)

Arm structure
-------------
  ┌──────────────┬──────────────────────────────────────────────┐
  │  Joint       │  Actuator(s)                                 │
  ├──────────────┼──────────────────────────────────────────────┤
  │  Shoulder    │  SHOULDER1 (ID 1) + SHOULDER2 (ID 2)         │
  │  Elbow       │  ELBOW (ID 3)                                │
  │  Wrist       │  WRIST1 (ID 5) + WRIST2 (ID 4)              │
  └──────────────┴──────────────────────────────────────────────┘

Coordinate frame (world origin = frame surface)
  X → horizontal reach away from base
  Y ↑ vertical height above frame

Physical dimensions (from design diagram)
  L1 = 437 mm   shoulder pivot → elbow pivot
  L2 = 437 mm   elbow pivot   → wrist pivot
  L3 = 220 mm   wrist pivot   → gripper tip
  base_height = 123 mm  frame → shoulder pivot

Relationship to ABB IRB 120 analysis
  dh_matrix()          identical formula to ABB dh()
  inverse_kinematics() same 2R law-of-cosines as ABB ik_2d_planar(),
                       extended to 3 joints via wrist decoupling
"""
from __future__ import annotations
from dataclasses import dataclass, field
import math
import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# Configuration dataclass  (populated by the ROS node from ROS params)
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class ArmConfig:
    """
    All geometric and encoder constants for the arm.

    Default values match the physical design diagram.
    The ROS node reads these from robot_params.yaml via the ROS 2
    parameter server and passes them here — no file I/O in this module.
    """
    # ── Link lengths (mm) ────────────────────────────────────────────────────
    L1: float = 437.0          # shoulder pivot → elbow pivot
    L2: float = 437.0          # elbow pivot   → wrist pivot
    L3: float = 220.0          # wrist pivot   → gripper tip
    base_height: float = 123.0  # frame surface → shoulder pivot

    # ── Joint limits (rad) ───────────────────────────────────────────────────
    shoulder_min: float = -math.pi
    shoulder_max: float =  math.pi
    elbow_min:    float = -math.pi / 2   # restricted to avoid self-collision
    elbow_max:    float =  math.pi
    wrist_min:    float = -math.pi
    wrist_max:    float =  math.pi

    # ── Encoder (XL430-W250 / compatible) ────────────────────────────────────
    counts_per_rev:   int   = 4096
    position_neutral: int   = 1750
    min_position:     int   = 0
    max_position:     int   = 3500

    # ── IK control ───────────────────────────────────────────────────────────
    ik_step_mm: float = 5.0    # mm per callback at full stick deflection

    @property
    def counts_per_rad(self) -> float:
        return self.counts_per_rev / (2.0 * math.pi)

    @property
    def joint_limits(self) -> list[tuple[float, float]]:
        return [
            (self.shoulder_min, self.shoulder_max),
            (self.elbow_min,    self.elbow_max),
            (self.wrist_min,    self.wrist_max),
        ]


# ─────────────────────────────────────────────────────────────────────────────
# DH matrix  (pure function — no config dependency)
# ─────────────────────────────────────────────────────────────────────────────
def dh_matrix(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """
    Standard Denavit-Hartenberg transformation matrix.

    Identical to the formulation used in the ABB IRB 120 analysis:
        T = [[cos θ, -sin θ·cos α,  sin θ·sin α,  a·cos θ],
             [sin θ,  cos θ·cos α, -cos θ·sin α,  a·sin θ],
             [0,      sin α,        cos α,         d      ],
             [0,      0,            0,             1      ]]
    """
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct,  -st * ca,   st * sa,  a * ct],
        [st,   ct * ca,  -ct * sa,  a * st],
        [ 0,   sa,        ca,       d     ],
        [ 0,   0,         0,        1     ],
    ], dtype=float)


# ─────────────────────────────────────────────────────────────────────────────
# ArmKinematics  — all math methods, config-aware
# ─────────────────────────────────────────────────────────────────────────────
class ArmKinematics:
    """
    FK, IK, and encoder conversion for the 3R planar Dynamixel arm.

    Instantiate once in the ROS node after reading parameters:
        self.kin = ArmKinematics(cfg)
    """

    def __init__(self, cfg: ArmConfig):
        self.cfg = cfg

    # ── Forward kinematics ────────────────────────────────────────────────────
    def forward_kinematics(
        self,
        theta1: float,
        theta2: float,
        theta3: float,
    ) -> tuple[float, float, float, np.ndarray]:
        """
        End-effector pose in the world frame.

        DH table (planar ⇒ d = 0, alpha = 0 for every joint):
            Joint 1  [θ₁, 0, L1, 0]
            Joint 2  [θ₂, 0, L2, 0]
            Joint 3  [θ₃, 0, L3, 0]

        T = T₁ · T₂ · T₃  (same accumulation as ABB analysis)
        y_world = y_shoulder_frame + base_height

        Returns:
            (x, y, phi, T)
            x, y : end-effector position in world frame (mm)
            phi  : orientation = θ₁+θ₂+θ₃ (rad)
            T    : 4×4 homogeneous transform (shoulder frame)
        """
        cfg = self.cfg
        dh_table = [
            (theta1, 0.0, cfg.L1, 0.0),
            (theta2, 0.0, cfg.L2, 0.0),
            (theta3, 0.0, cfg.L3, 0.0),
        ]
        T = np.eye(4)
        for row in dh_table:
            T = T @ dh_matrix(*row)

        x   = T[0, 3]
        y   = T[1, 3] + cfg.base_height
        phi = theta1 + theta2 + theta3
        return x, y, phi, T

    # ── All joint positions (for visualisation / debugging) ───────────────────
    def all_joint_positions(
        self,
        theta1: float,
        theta2: float,
        theta3: float,
    ) -> np.ndarray:
        """
        World-frame (x, y) of every joint origin.

        Mirrors compute_all_joint_positions() from the ABB analysis.

        Returns:
            (4, 2) array — [shoulder, elbow, wrist, tip]  (mm)
        """
        cfg = self.cfg
        dh_table = [
            (theta1, 0.0, cfg.L1, 0.0),
            (theta2, 0.0, cfg.L2, 0.0),
            (theta3, 0.0, cfg.L3, 0.0),
        ]
        positions = [[0.0, cfg.base_height]]   # shoulder pivot in world frame
        T_accum = np.eye(4)
        for row in dh_table:
            T_accum = T_accum @ dh_matrix(*row)
            positions.append([T_accum[0, 3], T_accum[1, 3] + cfg.base_height])
        return np.array(positions)

    # ── Inverse kinematics ────────────────────────────────────────────────────
    def inverse_kinematics(
        self,
        x_target: float,
        y_target: float,
        phi: float = 0.0,
        elbow_up: bool = True,
    ) -> tuple[float, float, float] | None:
        """
        2-D planar IK using wrist-decoupling + law of cosines.

        Algorithm (adapted from ABB ik_2d_planar):
          1. Fix end-effector orientation to phi.
          2. Back-project to wrist-pivot (world frame → shoulder frame):
                 wx = x_target − L3·cos(phi)
                 wy = (y_target − base_height) − L3·sin(phi)
          3. Solve 2R sub-problem with law of cosines (same as ABB):
                 cos θ₂ = (r² − L1² − L2²) / (2·L1·L2)
                 θ₁     = atan2(wy,wx) − atan2(L2·sin θ₂, L1+L2·cos θ₂)
          4. Recover θ₃ = phi − θ₁ − θ₂.

        Args:
            x_target, y_target : world-frame target (mm)
            phi                : desired end-effector orientation (rad)
            elbow_up           : True → positive θ₂

        Returns:
            (theta1, theta2, theta3) rad, or None if unreachable / out of limits
        """
        cfg = self.cfg

        # Convert world frame → shoulder frame (subtract base height)
        ys = y_target - cfg.base_height

        # Wrist pivot position in shoulder frame
        wx = x_target - cfg.L3 * math.cos(phi)
        wy = ys        - cfg.L3 * math.sin(phi)
        r  = math.hypot(wx, wy)

        # Reachability check (mirrors ABB warning)
        if r > (cfg.L1 + cfg.L2) or r < abs(cfg.L1 - cfg.L2):
            return None

        # 2R IK — law of cosines (same as ABB ik_2d_planar)
        cos_t2 = (r**2 - cfg.L1**2 - cfg.L2**2) / (2.0 * cfg.L1 * cfg.L2)
        cos_t2 = max(-1.0, min(1.0, cos_t2))
        theta2 = math.acos(cos_t2) if elbow_up else -math.acos(cos_t2)

        theta1 = math.atan2(wy, wx) - math.atan2(
            cfg.L2 * math.sin(theta2),
            cfg.L1 + cfg.L2 * math.cos(theta2),
        )

        theta3 = phi - theta1 - theta2

        # Joint-limit check
        for ang, (lo, hi) in zip((theta1, theta2, theta3), cfg.joint_limits):
            wrapped = (ang + math.pi) % (2.0 * math.pi) - math.pi
            if not (lo <= wrapped <= hi):
                return None

        return theta1, theta2, theta3

    # ── Validation (FK round-trip, mirrors ABB validate_ik_solution) ──────────
    def validate_ik(
        self,
        theta1: float,
        theta2: float,
        theta3: float,
        x_target: float,
        y_target: float,
    ) -> float:
        """
        Position error (mm) of an IK solution.  Mirrors ABB validate_ik_solution().
        Both target and FK result are in the world frame.
        """
        x_fk, y_fk, _, _ = self.forward_kinematics(theta1, theta2, theta3)
        return math.hypot(x_target - x_fk, y_target - y_fk)

    # ── Encoder conversion ────────────────────────────────────────────────────
    def angle_to_position(self, angle_rad: float) -> int:
        cfg = self.cfg
        pos = int(round(cfg.position_neutral + angle_rad * cfg.counts_per_rad))
        return max(cfg.min_position, min(cfg.max_position, pos))

    def position_to_angle(self, position: int) -> float:
        cfg = self.cfg
        return (int(position) - cfg.position_neutral) / cfg.counts_per_rad

    def joint_angles_to_positions(
        self, theta1: float, theta2: float, theta3: float
    ) -> tuple[int, int, int]:
        """(shoulder, elbow, wrist) angles → Dynamixel encoder counts."""
        return (
            self.angle_to_position(theta1),
            self.angle_to_position(theta2),
            self.angle_to_position(theta3),
        )

    def positions_to_joint_angles(
        self, pos_shoulder: int, pos_elbow: int, pos_wrist: int
    ) -> tuple[float, float, float]:
        """Dynamixel encoder counts → (shoulder, elbow, wrist) angles (rad)."""
        return (
            self.position_to_angle(pos_shoulder),
            self.position_to_angle(pos_elbow),
            self.position_to_angle(pos_wrist),
        )

    # ── Workspace sampling (mirrors ABB workspace analysis) ───────────────────
    def sample_workspace(self, n_samples: int = 500, seed: int = 42) -> np.ndarray:
        """
        Random FK samples → end-effector positions.  Mirrors ABB workspace block.

        Returns:
            (n_samples, 2) array of [x, y] world-frame positions (mm)
        """
        cfg = self.cfg
        rng = np.random.default_rng(seed)
        limits = cfg.joint_limits
        points = []
        for _ in range(n_samples):
            t1 = rng.uniform(*limits[0])
            t2 = rng.uniform(*limits[1])
            t3 = rng.uniform(*limits[2])
            x, y, _, _ = self.forward_kinematics(t1, t2, t3)
            points.append([x, y])
        return np.array(points)

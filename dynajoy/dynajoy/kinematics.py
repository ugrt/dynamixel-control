"""
Kinematics for 6-DOF Dynamixel / ODrive Robot Arm
===================================================
Pure-math library — no file I/O, no ROS imports.
All geometry lives in ArmConfig; the ROS node owns parameter loading.

Usage
-----
::

    from dynajoy.kinematics import ArmConfig, ArmKinematics

    cfg = ArmConfig(L1=437.0, L2=437.0, L3=220.0, base_height=123.0)
    kin = ArmKinematics(cfg)

    # 3-D forward kinematics
    x, y, z, phi = kin.forward_kinematics_3d(theta0, theta1, theta2, theta3)

    # 3-D inverse kinematics
    result = kin.inverse_kinematics_3d(x, y, z, phi_arm=0.0)

Arm structure
-------------
  ┌──────────────────┬──────────┬────────────────────────────┐
  │  Joint           │  Driver  │  Actuator                  │
  ├──────────────────┼──────────┼────────────────────────────┤
  │  Base rotation   │  ODrive  │  S1  (CAN node 0)          │
  │  Shoulder        │  ODrive  │  S1  (CAN node 1)          │
  │  Elbow           │  ODrive  │  S1  (CAN node 2)          │
  │  Wrist pitch     │  Dynamixel│  XM540-W270-R             │
  │  Wrist spin      │  Dynamixel│  XM540-W270-R             │
  │  Gripper         │  Dynamixel│  XM540-W150-R             │
  └──────────────────┴──────────┴────────────────────────────┘

Coordinate frame (world origin = frame surface centre)
  X  →  forward (arm points here when θ₀ = 0)
  Y  →  left
  Z  ↑  vertical

Joint angle convention
  θ₀  Base rotation around Z.  0 = arm extends along +X.
  θ₁  Shoulder pitch.          0 = arm horizontal.  + = upward.
  θ₂  Elbow.                   0 = straight.         + = elbow folds up.
  θ₃  Wrist pitch (in-plane).  0 = wrist straight.   + = pitch up.
  θ₄  Wrist spin (roll).       0 = neutral.
  φ   Gripper.                 0 = fully open.        + = closing.

Physical dimensions (from design diagram)
  L1 = 437 mm   shoulder pivot → elbow pivot
  L2 = 437 mm   elbow pivot   → wrist pivot
  L3 = 220 mm   wrist pivot   → gripper tip
  base_height = 123 mm  frame surface → shoulder pivot
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# ArmConfig  (populated by the ROS node from ROS params)
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class ArmConfig:
    """
    All geometric and encoder constants for the 6-DOF arm.

    ODrive joints (base, shoulder, elbow) operate in radians natively.
    Dynamixel joints (wrist_pitch, wrist_spin, gripper) need count ↔ rad
    conversion via the dxl_* encoder fields.
    """

    # ── Link lengths (mm) ────────────────────────────────────────────────────
    L1: float = 437.0           # shoulder pivot → elbow pivot
    L2: float = 437.0           # elbow pivot   → wrist pivot
    L3: float = 220.0           # wrist pivot   → gripper tip
    base_height: float = 123.0  # frame surface → shoulder pivot

    # ── ODrive joint limits (rad) ────────────────────────────────────────────
    base_min:     float = -math.pi
    base_max:     float =  math.pi
    shoulder_min: float = -math.pi / 2   # −90°: prevents going below horizontal
    shoulder_max: float =  math.pi       #  180°
    elbow_min:    float = -math.pi       # −180°
    elbow_max:    float =  math.pi       #  180°

    # ── Dynamixel joint limits (rad) ─────────────────────────────────────────
    wrist_pitch_min: float = -math.pi / 2   # −90°
    wrist_pitch_max: float =  math.pi / 2   #  90°
    wrist_spin_min:  float = -math.pi       # −180°
    wrist_spin_max:  float =  math.pi       #  180°
    gripper_min:     float =  0.0           # fully open
    gripper_max:     float =  math.pi / 2   # fully closed (~90°)

    # ── Dynamixel XM540 encoder ──────────────────────────────────────────────
    # Applies to wrist_pitch, wrist_spin, and gripper (all XM540 series).
    # XM540 single-turn mode: 0–4095 counts, 4096 counts per revolution.
    dxl_counts_per_rev:   int = 4096
    dxl_position_neutral: int = 2048   # count at 0 rad (centre of range)
    dxl_min_position:     int =    0
    dxl_max_position:     int = 4095

    # ── IK / teleoperation behaviour ─────────────────────────────────────────
    ik_step_mm:  float = 5.0    # mm per callback at full stick deflection
    ik_step_rad: float = 0.05   # rad per callback for wrist / base adjustments
    vel_step_rad: float = 0.03  # rad per callback for ODrive velocity mode

    # ── Computed helpers ─────────────────────────────────────────────────────
    @property
    def dxl_counts_per_rad(self) -> float:
        return self.dxl_counts_per_rev / (2.0 * math.pi)

    @property
    def kinematic_joint_limits(self) -> list[tuple[float, float]]:
        """
        Limits for the 4 position-controlled kinematic joints:
        base, shoulder, elbow, wrist_pitch.
        (wrist_spin and gripper are independent and not part of 3D IK.)
        """
        return [
            (self.base_min,        self.base_max),
            (self.shoulder_min,    self.shoulder_max),
            (self.elbow_min,       self.elbow_max),
            (self.wrist_pitch_min, self.wrist_pitch_max),
        ]


# ─────────────────────────────────────────────────────────────────────────────
# DH matrix  (pure function — no config dependency)
# ─────────────────────────────────────────────────────────────────────────────
def dh_matrix(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """
    Standard Denavit-Hartenberg transformation matrix.

    T = [[cos θ, −sin θ·cos α,  sin θ·sin α,  a·cos θ],
         [sin θ,  cos θ·cos α, −cos θ·sin α,  a·sin θ],
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
# ArmKinematics
# ─────────────────────────────────────────────────────────────────────────────
class ArmKinematics:
    """
    FK, IK, encoder conversion, and workspace sampling for the 6-DOF arm.

    Instantiate once in the ROS node after reading parameters::

        cfg      = ArmConfig(...)
        self.kin = ArmKinematics(cfg)
    """

    def __init__(self, cfg: ArmConfig):
        self.cfg = cfg

    # ── 3-D Forward kinematics ────────────────────────────────────────────────

    def forward_kinematics_3d(
        self,
        theta0: float,
        theta1: float,
        theta2: float,
        theta3: float,
        theta4: float = 0.0,
    ) -> tuple[float, float, float, float]:
        """
        End-effector position in the world frame.

        The arm operates in a vertical plane whose direction is determined by
        the base rotation θ₀.  Within that plane the kinematics reduce to a
        standard 3R planar chain (shoulder + elbow + wrist-pitch).

        Reach in the arm plane::

            r   = L1·cos θ₁ + L2·cos(θ₁+θ₂) + L3·cos(θ₁+θ₂+θ₃)
            z_s = L1·sin θ₁ + L2·sin(θ₁+θ₂) + L3·sin(θ₁+θ₂+θ₃)

        World frame::

            x       = r · cos θ₀
            y       = r · sin θ₀
            z       = base_height + z_s
            phi_arm = θ₁ + θ₂ + θ₃   (wrist orientation in arm plane)

        Args:
            theta0 : base rotation (rad)
            theta1 : shoulder pitch (rad)
            theta2 : elbow (rad)
            theta3 : wrist pitch (rad)
            theta4 : wrist spin (rad) — does not affect EE position

        Returns:
            (x, y, z, phi_arm)  — position in mm, phi_arm in rad
        """
        cfg = self.cfg
        phi12  = theta1 + theta2
        phi123 = phi12 + theta3

        reach = (
            cfg.L1 * math.cos(theta1)
            + cfg.L2 * math.cos(phi12)
            + cfg.L3 * math.cos(phi123)
        )
        z = (
            cfg.base_height
            + cfg.L1 * math.sin(theta1)
            + cfg.L2 * math.sin(phi12)
            + cfg.L3 * math.sin(phi123)
        )
        x = reach * math.cos(theta0)
        y = reach * math.sin(theta0)

        return x, y, z, phi123

    # ── 3-D Inverse kinematics ────────────────────────────────────────────────

    def inverse_kinematics_3d(
        self,
        x_target: float,
        y_target: float,
        z_target: float,
        phi_arm: float = 0.0,
        elbow_up: bool = True,
    ) -> tuple[float, float, float, float] | None:
        """
        3-D IK via base-decoupling + wrist-decoupling + law of cosines.

        Algorithm:
          1. θ₀ = atan2(y, x)               — base points at target
          2. r   = √(x² + y²)               — horizontal reach
          3. Wrist-decouple in arm plane
             (φ_arm = desired wrist orientation in the arm plane)::

               wx = r   − L3·cos(φ_arm)
               wz = (z − base_height) − L3·sin(φ_arm)

          4. 2R IK for (wx, wz) via law of cosines::

               cos θ₂ = (rw² − L1² − L2²) / (2·L1·L2)
               θ₁     = atan2(wz, wx) − atan2(L2·sin θ₂, L1+L2·cos θ₂)

          5. θ₃ = φ_arm − θ₁ − θ₂

        Args:
            x_target, y_target, z_target : world-frame target (mm)
            phi_arm                       : desired arm-plane wrist orientation (rad)
                                            0 = wrist horizontal, + = pitch up
            elbow_up                      : True → positive θ₂ (elbow above line)

        Returns:
            (θ₀, θ₁, θ₂, θ₃) in radians, or None if unreachable / out of limits.
        """
        cfg = self.cfg

        # Step 1 — base rotation
        theta0 = math.atan2(y_target, x_target)

        # Step 2 — horizontal reach and arm-plane height
        r   = math.hypot(x_target, y_target)
        z_s = z_target - cfg.base_height   # height above shoulder pivot

        # Step 3 — wrist decoupling
        wx = r   - cfg.L3 * math.cos(phi_arm)
        wz = z_s - cfg.L3 * math.sin(phi_arm)
        rw = math.hypot(wx, wz)

        # Reachability check for the 2R sub-chain
        max_reach = cfg.L1 + cfg.L2
        min_reach = abs(cfg.L1 - cfg.L2)
        if rw > max_reach or rw < min_reach:
            return None

        # Step 4 — law of cosines
        cos_t2 = (rw**2 - cfg.L1**2 - cfg.L2**2) / (2.0 * cfg.L1 * cfg.L2)
        cos_t2 = max(-1.0, min(1.0, cos_t2))   # clamp for numerical safety
        theta2 = math.acos(cos_t2) if elbow_up else -math.acos(cos_t2)

        theta1 = math.atan2(wz, wx) - math.atan2(
            cfg.L2 * math.sin(theta2),
            cfg.L1 + cfg.L2 * math.cos(theta2),
        )

        # Step 5 — wrist pitch
        theta3 = phi_arm - theta1 - theta2

        # Joint-limit check (base, shoulder, elbow, wrist_pitch)
        angles = (theta0, theta1, theta2, theta3)
        for ang, (lo, hi) in zip(angles, cfg.kinematic_joint_limits):
            wrapped = (ang + math.pi) % (2.0 * math.pi) - math.pi
            if not (lo <= wrapped <= hi):
                return None

        return theta0, theta1, theta2, theta3

    # ── 3-D joint positions (for visualisation / debugging) ──────────────────

    def all_joint_positions_3d(
        self,
        theta0: float,
        theta1: float,
        theta2: float,
        theta3: float,
    ) -> np.ndarray:
        """
        World-frame (x, y, z) of every joint origin.

        Returns
        -------
        (5, 3) ndarray — rows: [base, shoulder, elbow, wrist, tip]  (mm)
        """
        cfg = self.cfg
        c0, s0 = math.cos(theta0), math.sin(theta0)

        def to_world(reach: float, height: float) -> list:
            return [reach * c0, reach * s0, height]

        pts = [
            [0.0, 0.0, 0.0],                       # base  (frame surface)
            [0.0, 0.0, cfg.base_height],            # shoulder pivot
        ]

        phi12  = theta1 + theta2
        phi123 = phi12  + theta3

        r1 = cfg.L1 * math.cos(theta1)
        z1 = cfg.base_height + cfg.L1 * math.sin(theta1)
        pts.append(to_world(r1, z1))               # elbow

        r2 = r1 + cfg.L2 * math.cos(phi12)
        z2 = z1 + cfg.L2 * math.sin(phi12)
        pts.append(to_world(r2, z2))               # wrist

        r3 = r2 + cfg.L3 * math.cos(phi123)
        z3 = z2 + cfg.L3 * math.sin(phi123)
        pts.append(to_world(r3, z3))               # gripper tip

        return np.array(pts)

    # ── IK validation ─────────────────────────────────────────────────────────

    def validate_ik_3d(
        self,
        theta0: float,
        theta1: float,
        theta2: float,
        theta3: float,
        x_target: float,
        y_target: float,
        z_target: float,
    ) -> float:
        """3-D Euclidean position error (mm) of an IK solution."""
        x, y, z, _ = self.forward_kinematics_3d(theta0, theta1, theta2, theta3)
        return math.sqrt(
            (x_target - x) ** 2 + (y_target - y) ** 2 + (z_target - z) ** 2
        )

    # ── Dynamixel encoder conversion ──────────────────────────────────────────

    def dxl_rad_to_count(self, angle_rad: float) -> int:
        """
        Convert radians to XM540 encoder counts, clamped to config limits.

        0 rad → dxl_position_neutral (2048)
        +π rad → 2048 + 2048 = 4096 (clamped to 4095)
        −π rad → 2048 − 2048 = 0
        """
        cfg = self.cfg
        count = int(round(cfg.dxl_position_neutral + angle_rad * cfg.dxl_counts_per_rad))
        return max(cfg.dxl_min_position, min(cfg.dxl_max_position, count))

    def dxl_count_to_rad(self, count: int) -> float:
        """Convert XM540 encoder counts to radians."""
        cfg = self.cfg
        return (int(count) - cfg.dxl_position_neutral) / cfg.dxl_counts_per_rad

    def dxl_angles_to_counts(
        self,
        wrist_pitch: float,
        wrist_spin: float,
        gripper: float,
    ) -> tuple[int, int, int]:
        """Convert the three Dynamixel joint angles (rad) → encoder counts."""
        return (
            self.dxl_rad_to_count(wrist_pitch),
            self.dxl_rad_to_count(wrist_spin),
            self.dxl_rad_to_count(gripper),
        )

    def dxl_counts_to_angles(
        self,
        count_wp: int,
        count_ws: int,
        count_gr: int,
    ) -> tuple[float, float, float]:
        """Convert Dynamixel counts → (wrist_pitch, wrist_spin, gripper) rad."""
        return (
            self.dxl_count_to_rad(count_wp),
            self.dxl_count_to_rad(count_ws),
            self.dxl_count_to_rad(count_gr),
        )

    # ── Workspace sampling ────────────────────────────────────────────────────

    def sample_workspace_3d(
        self, n_samples: int = 2000, seed: int = 42
    ) -> np.ndarray:
        """
        Random FK samples → end-effector positions in 3-D world frame.

        Returns (n_samples, 3) array of [x, y, z] (mm).
        """
        cfg = self.cfg
        rng = np.random.default_rng(seed)
        limits = cfg.kinematic_joint_limits
        points: list[list[float]] = []
        for _ in range(n_samples):
            t0 = rng.uniform(*limits[0])
            t1 = rng.uniform(*limits[1])
            t2 = rng.uniform(*limits[2])
            t3 = rng.uniform(*limits[3])
            x, y, z, _ = self.forward_kinematics_3d(t0, t1, t2, t3)
            points.append([x, y, z])
        return np.array(points)

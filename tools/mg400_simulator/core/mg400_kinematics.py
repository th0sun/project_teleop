#!/usr/bin/env python3
"""
MG400 Kinematics - Forward and Inverse Kinematics
Using exact constants from teleop_logic/utils/kinematics.py
"""

import numpy as np
from typing import Optional

# ── MG400 Link Constants (mm) ────────────────────────────────────────────────
LINK1 = np.array([43.0,   0.0,   0.0])   # shoulder horizontal offset
LINK2 = np.array([ 0.0,   0.0, 175.0])   # upper arm (vertical when j2=0)
LINK3 = np.array([175.0,  0.0,   0.0])   # forearm   (horizontal when j3=0)
LINK4 = np.array([66.0,   0.0, -57.0])   # wrist → end-effector

BASE_HEIGHT = 109.0  # mm – approximate height of j1 axis above base plate

# ── Joint limits (degrees) ────────────────────────────────────────────────────
JOINT_LIMITS = [
    (-160.0, 160.0),   # J1  – base yaw
    ( -25.0,  85.0),   # J2  – shoulder pitch
    ( -25.0,  105.0),  # J3  – elbow pitch
    (-360.0, 360.0),   # J4  – wrist yaw
]

ELBOW_ANGLE_LIMIT = (-60.0, 60.0) # (J3 - J2) constraint

EE_RADIUS   = 18.0   # mm – visual EE sphere radius for hit-testing
JOINT_RADIUS = 10.0  # mm – visual joint sphere radius


# ── Rotation helpers ─────────────────────────────────────────────────────────

def rot_y(vec: np.ndarray, angle_deg: float) -> np.ndarray:
    a = np.deg2rad(angle_deg)
    c, s = np.cos(a), np.sin(a)
    R = np.array([[ c, 0, s],
                  [ 0, 1, 0],
                  [-s, 0, c]])
    return R @ vec


def rot_z(vec: np.ndarray, angle_deg: float) -> np.ndarray:
    a = np.deg2rad(angle_deg)
    c, s = np.cos(a), np.sin(a)
    R = np.array([[c, -s, 0],
                  [s,  c, 0],
                  [0,  0, 1]])
    return R @ vec


# ── Forward Kinematics ────────────────────────────────────────────────────────

def forward_kinematics(joints_deg) -> np.ndarray:
    """
    Compute EE pose from 4 active joint angles (degrees).
    Returns np.array([px, py, pz, rx, ry, rz]) in mm.
    Matches teleop_logic/utils/kinematics.py exactly.
    """
    j1, j2, j3, j4 = (float(joints_deg[i]) for i in range(4))
    pos = LINK1 + rot_y(LINK2, j2) + rot_y(LINK3, j3) + LINK4
    px, py, pz = rot_z(pos, j1)
    rx = j1 + j4
    return np.array([px, py, pz, rx, 0.0, 0.0])


# ── Inverse Kinematics (analytical) ──────────────────────────────────────────

def inverse_kinematics(px: float, py: float, pz: float,
                       j4_deg: float = 0.0,
                       prefer_elbow: str = 'up',
                       current_joints: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
    """
    Analytical IK for MG400.

    Args:
        px, py, pz : target EE position in mm (MG400 world frame)
        j4_deg     : desired wrist rotation (degrees)
        prefer_elbow: 'up' or 'down' – selects the two 2-link solutions
        current_joints: used to unwrap J1 and prevent violent 360 jumps

    Returns:
        np.array([j1, j2, j3, j4]) in degrees, clamped to joint limits.
        None if target is unreachable.
    """
    L = 175.0   # both arm link lengths

    # ── J1: base rotation ────────────────────────────────────────────────────
    j1_rad = np.arctan2(py, px)
    j1     = np.degrees(j1_rad)

    if current_joints is not None:
        curr_j1 = current_joints[0]
        # Unwrap J1 to be as close to current J1 as possible
        diff = (j1 - curr_j1 + 180) % 360 - 180
        
        # If the target is physically across the origin (e.g. crossing -X axis),
        # prevent massive >90 deg jumps in a single frame to stop IK warping
        if abs(diff) > 100.0:
            return None
            
        j1 = curr_j1 + diff
        j1_rad = np.radians(j1)

    # ── Undo J1 rotation → arm-local X-Z frame ───────────────────────────────
    c, s  = np.cos(j1_rad), np.sin(j1_rad)
    x_loc = c * px + s * py   # arm-plane X (horizontal reach)
    z_loc = pz                  # vertical (same)

    # Effective target for the LINK2+LINK3 two-link chain
    eff_x = x_loc - LINK1[0] - LINK4[0]   # x_loc - 43 - 66  = x_loc - 109
    eff_z = z_loc - LINK1[2] - LINK4[2]   # z_loc - 0  - (-57) = z_loc + 57

    # Normalise
    ex = eff_x / L
    ez = eff_z / L
    R  = np.sqrt(ex**2 + ez**2)

    if R > 1.99:
        return None   # out of reach

    # ── J2: shoulder pitch ───────────────────────────────────────────────────
    # From dot-product identity: ex*sin(j2) + ez*cos(j2) = R²/2
    phi = np.arctan2(ex, ez)                # coeff angle
    val = np.clip(R / 2, -1.0, 1.0)
    acos_val = np.arccos(val)

    if prefer_elbow == 'up':
        j2_rad = phi - acos_val
    else:
        j2_rad = phi + acos_val

    j2 = np.degrees(j2_rad)

    # ── J3: elbow pitch ──────────────────────────────────────────────────────
    cos_j3 = ex - np.sin(j2_rad)
    sin_j3 = np.cos(j2_rad) - ez
    j3_rad = np.arctan2(sin_j3, cos_j3)
    j3     = np.degrees(j3_rad)

    result = np.array([j1, j2, j3, j4_deg])

    # Clamp to joint limits and enforce elbow constraints
    result = clamp_joints(result)

    return result


def clamp_joints(joints_deg) -> np.ndarray:
    result = np.array(joints_deg, dtype=float)
    
    # Check elbow constraint first, adjust J3 if needed before full clamp
    j2, j3 = result[1], result[2]
    diff = j3 - j2
    
    if diff < ELBOW_ANGLE_LIMIT[0]:
        result[2] = j2 + ELBOW_ANGLE_LIMIT[0]
    elif diff > ELBOW_ANGLE_LIMIT[1]:
        result[2] = j2 + ELBOW_ANGLE_LIMIT[1]
        
    for i, (lo, hi) in enumerate(JOINT_LIMITS):
        result[i] = np.clip(result[i], lo, hi)
        
    return result

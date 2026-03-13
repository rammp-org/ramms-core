"""
Unit conversion and coordinate-system helpers for URDF ↔ Unreal Engine.

URDF: meters, radians, kg, kg·m²  —  right-handed Z-up
UE5:  centimeters, degrees, kg, kg·cm²  —  left-handed Z-up (Y negated)
"""

from __future__ import annotations

import math
from typing import Tuple


# ---------------------------------------------------------------------------
# Scalar conversions
# ---------------------------------------------------------------------------

def m_to_cm(m: float) -> float:
    return m * 100.0


def cm_to_m(cm: float) -> float:
    return cm / 100.0


def rad_to_deg(rad: float) -> float:
    return math.degrees(rad)


def deg_to_rad(deg: float) -> float:
    return math.radians(deg)


def kgm2_to_kgcm2(kgm2: float) -> float:
    """Convert inertia from kg·m² to kg·cm²."""
    return kgm2 * 10000.0


def kgcm2_to_kgm2(kgcm2: float) -> float:
    """Convert inertia from kg·cm² to kg·m²."""
    return kgcm2 / 10000.0


# ---------------------------------------------------------------------------
# Coordinate transforms  (URDF ↔ UE)
#
# URDF is right-handed Z-up.  UE is left-handed Z-up.
# The standard conversion negates the Y axis:
#   UE.X =  URDF.X
#   UE.Y = -URDF.Y
#   UE.Z =  URDF.Z
# ---------------------------------------------------------------------------

def urdf_pos_to_ue(xyz: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Convert URDF position (meters) to UE position (cm), with handedness flip."""
    return (m_to_cm(xyz[0]), -m_to_cm(xyz[1]), m_to_cm(xyz[2]))


def ue_pos_to_urdf(xyz: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Convert UE position (cm) to URDF position (meters), with handedness flip."""
    return (cm_to_m(xyz[0]), -cm_to_m(xyz[1]), cm_to_m(xyz[2]))


def urdf_axis_to_ue(axis: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Convert URDF axis direction to UE (negate Y for handedness)."""
    return (axis[0], -axis[1], axis[2])


def ue_axis_to_urdf(axis: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Convert UE axis direction to URDF (negate Y for handedness)."""
    return (axis[0], -axis[1], axis[2])


def urdf_rpy_to_ue_deg(rpy: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """
    Convert URDF roll-pitch-yaw (radians, right-handed) to UE rotation (degrees, left-handed).

    URDF RPY: extrinsic X-Y-Z = intrinsic Z-Y'-X''
    UE rotator: Roll=X, Pitch=Y, Yaw=Z (degrees)
    With Y negated: Roll stays, Pitch negates, Yaw negates.
    """
    roll_deg = rad_to_deg(rpy[0])
    pitch_deg = -rad_to_deg(rpy[1])
    yaw_deg = -rad_to_deg(rpy[2])
    return (roll_deg, pitch_deg, yaw_deg)


def ue_deg_to_urdf_rpy(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Tuple[float, float, float]:
    """Convert UE rotation (degrees, left-handed) to URDF RPY (radians, right-handed)."""
    roll = deg_to_rad(roll_deg)
    pitch = -deg_to_rad(pitch_deg)
    yaw = -deg_to_rad(yaw_deg)
    return (roll, pitch, yaw)


def rotation_matrix_to_rpy(
    x_axis: Tuple[float, float, float],
    y_axis: Tuple[float, float, float],
    z_axis: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """Extract URDF RPY (roll, pitch, yaw) from a rotation matrix defined by its column axes.

    The rotation matrix R = [x_axis | y_axis | z_axis] (columns) maps from
    the child/joint frame to the parent frame.  URDF convention:
    R = Rz(yaw) · Ry(pitch) · Rx(roll).

    The axes must already be in the URDF (right-handed) coordinate system.
    """
    # R[row][col] with columns = x_axis, y_axis, z_axis
    # R[2][0] = x_axis[2] = -sin(pitch)
    sp = -x_axis[2]
    if abs(sp) >= 1.0 - 1e-12:
        # Gimbal lock
        pitch = math.copysign(math.pi / 2.0, sp)
        roll = 0.0
        yaw = math.atan2(-y_axis[0], y_axis[1])
    else:
        pitch = math.asin(max(-1.0, min(1.0, sp)))
        roll = math.atan2(y_axis[2], z_axis[2])   # atan2(R[2][1], R[2][2])
        yaw = math.atan2(x_axis[1], x_axis[0])    # atan2(R[1][0], R[0][0])
    return (roll, pitch, yaw)


# ---------------------------------------------------------------------------
# Quaternion utilities  (q = (w, x, y, z))
# ---------------------------------------------------------------------------

def quat_conjugate(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    """Return the conjugate (= inverse for unit quaternions)."""
    return (q[0], -q[1], -q[2], -q[3])


def quat_multiply(
    q1: Tuple[float, float, float, float],
    q2: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """Hamilton product q1 * q2."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )


def quat_rotate_vector(
    q: Tuple[float, float, float, float],
    v: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """Rotate vector *v* by quaternion *q*: v' = q·v·q*."""
    vq = (0.0, v[0], v[1], v[2])
    return quat_multiply(quat_multiply(q, vq), quat_conjugate(q))[1:]


def quat_to_rotation_matrix(
    q: Tuple[float, float, float, float],
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
    """Convert quaternion (w, x, y, z) to column-axis rotation matrix (col_x, col_y, col_z)."""
    w, x, y, z = q
    col_x = (1 - 2*(y*y + z*z), 2*(x*y + w*z), 2*(x*z - w*y))
    col_y = (2*(x*y - w*z), 1 - 2*(x*x + z*z), 2*(y*z + w*x))
    col_z = (2*(x*z + w*y), 2*(y*z - w*x), 1 - 2*(x*x + y*y))
    return col_x, col_y, col_z


def ue_quat_to_urdf_quat(
    q_ue: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """Convert a UE quaternion (w, x, y, z) to a URDF-space quaternion.

    Under the UE→URDF basis change (Y-axis flip for handedness),
    the rotation axis (ax, ay, az) becomes (ax, -ay, az) and the
    angle negates for the X and Z components, giving:
        q_urdf = (w, -x, y, -z)
    """
    return (q_ue[0], -q_ue[1], q_ue[2], -q_ue[3])


def ue_quat_to_urdf_rpy(
    q_ue: Tuple[float, float, float, float],
) -> Tuple[float, float, float]:
    """Convert a UE quaternion (w, x, y, z) to URDF Roll-Pitch-Yaw.

    Applies the UE→URDF basis change then extracts RPY from the
    resulting rotation matrix.
    """
    q_urdf = ue_quat_to_urdf_quat(q_ue)
    col_x, col_y, col_z = quat_to_rotation_matrix(q_urdf)
    return rotation_matrix_to_rpy(col_x, col_y, col_z)


def rpy_to_quat(
    roll: float, pitch: float, yaw: float,
) -> Tuple[float, float, float, float]:
    """Convert URDF RPY (extrinsic XYZ) to quaternion (w, x, y, z).

    R = Rz(yaw) · Ry(pitch) · Rx(roll)  →  q = qz · qy · qx
    """
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr,
    )


def urdf_inertia_to_ue(ixx, ixy, ixz, iyy, iyz, izz):
    """
    Convert URDF inertia tensor (kg·m²) to UE (kg·cm²) with handedness.

    Y negation flips sign of off-diagonal terms involving Y:
      UE.Ixy = -URDF.Ixy,  UE.Iyz = -URDF.Iyz
    Diagonal terms and Ixz are unchanged (after unit conversion).
    """
    s = 10000.0  # kg·m² → kg·cm²
    return (ixx * s, -ixy * s, ixz * s, iyy * s, -iyz * s, izz * s)


def ue_inertia_to_urdf(ixx, ixy, ixz, iyy, iyz, izz):
    """Convert UE inertia tensor (kg·cm²) to URDF (kg·m²) with handedness."""
    s = 1.0 / 10000.0
    return (ixx * s, -ixy * s, ixz * s, iyy * s, -iyz * s, izz * s)


# ---------------------------------------------------------------------------
# Joint type mapping
# ---------------------------------------------------------------------------

# URDF → UE constraint type mapping
URDF_JOINT_TO_UE = {
    "revolute":   "angular",     # single-axis rotation with limits
    "continuous": "angular",     # single-axis rotation, no limits
    "prismatic":  "linear",      # single-axis translation with limits
    "fixed":      "fixed",       # fully locked
    "floating":   "free",        # 6-DOF (not typically used in physics assets)
    "planar":     "planar",      # 2D translation + 1 rotation
}

# UE → URDF joint type (for export — assumes revolute by default)
UE_CONSTRAINT_TO_URDF = {
    "angular":  "revolute",
    "linear":   "prismatic",
    "fixed":    "fixed",
    "free":     "floating",
}


def classify_ue_constraint(twist_motion: str, swing1_motion: str, swing2_motion: str,
                           linear_x_motion: str, linear_y_motion: str, linear_z_motion: str):
    """
    Classify a UE physics constraint into a URDF joint type based on DOF motion types.

    Angular motion values: "ACM_Free", "ACM_Limited", "ACM_Locked"
    Linear motion values: "LCM_Free", "LCM_Limited", "LCM_Locked"

    Returns (urdf_joint_type, active_axes) where active_axes lists the
    non-locked DOF names (e.g. ["swing1"], ["twist", "swing2"]).
    """
    angular_limited = []
    angular_free = []
    for name, motion in [("twist", twist_motion), ("swing1", swing1_motion),
                         ("swing2", swing2_motion)]:
        if motion == "ACM_Limited":
            angular_limited.append(name)
        elif motion == "ACM_Free":
            angular_free.append(name)

    linear_limited = []
    linear_free = []
    for name, motion in [("x", linear_x_motion), ("y", linear_y_motion),
                         ("z", linear_z_motion)]:
        if motion == "LCM_Limited":
            linear_limited.append(name)
        elif motion == "LCM_Free":
            linear_free.append(name)

    # Combine: limited DOFs listed first (they carry limit info)
    all_angular = angular_limited + angular_free
    all_linear = linear_limited + linear_free
    all_dofs = all_angular + all_linear

    if not all_dofs:
        return ("fixed", [])

    if len(all_dofs) >= 6:
        return ("floating", all_dofs)

    # Single angular DOF → revolute (has limits) or continuous (unlimited)
    if len(all_angular) == 1 and not all_linear:
        if angular_limited:
            return ("revolute", all_angular)
        return ("continuous", all_angular)

    # Single linear DOF → prismatic
    if len(all_linear) == 1 and not all_angular:
        return ("prismatic", all_linear)

    # Multiple angular DOFs only
    if all_angular and not all_linear:
        if angular_limited:
            return ("revolute", all_angular)
        return ("continuous", all_angular)

    # Multiple linear DOFs only
    if all_linear and not all_angular:
        return ("prismatic", all_linear)

    # Mix of angular + linear
    return ("floating", all_dofs)


# ---------------------------------------------------------------------------
# Formatting helpers (for XML output)
# ---------------------------------------------------------------------------

def fmt_vec3(v: Tuple[float, ...], precision: int = 8) -> str:
    """Format a 3-tuple as a space-separated string."""
    return f"{v[0]:.{precision}g} {v[1]:.{precision}g} {v[2]:.{precision}g}"


def fmt_float(f: float, precision: int = 8) -> str:
    return f"{f:.{precision}g}"

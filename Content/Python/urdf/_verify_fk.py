"""
Verify URDF FK chain using Kinova Gen3 probe data.
Runs standalone (no UE required).
"""
import math
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from urdf.urdf_utils import (
    quat_conjugate, quat_multiply, quat_rotate_vector,
    quat_to_rotation_matrix, ue_quat_to_urdf_rpy, rotation_matrix_to_rpy,
)

# Kinova Gen3 WORLD transforms from probe (cm, quat as w,x,y,z)
BONES = [
    ("base_link", (0, 0, 0),          (0, 0, 0.7071, -0.7071)),
    ("link_0",    (0, 0, 15.88),      (0.7071, 0, -0.7071, 0)),
    ("link_1",    (0, 1.175, 28.48),  (0, 0.7071, 0, 0.7071)),
    ("link_2",    (0, 1.175, 49.5175),(-0.7071, 0, -0.7071, 0)),
    ("link_3",    (0, 2.45, 70.555),  (0, -0.7071, 0, -0.7071)),
    ("link_4",    (0, 2.45, 91.64),   (0.7071, 0, -0.7071, 0)),
    ("link_5",    (0, 2.4675, 101.99),(0, -0.7071, 0, 0.7071)),
    ("link_6",    (0, 2.485, 112.8325),(1, 0, 0, 0)),
]

# Official Kinova gen3 lite URDF approximate values for comparison
OFFICIAL_JOINTS = [
    # (name, xyz_m, rpy_rad) - approximate
    ("joint_1", (0, 0, 0.15643), (3.14159, 0, 0)),
    ("joint_2", (0, 0.00540, 0.12838), (-1.5708, -1.5708, 0)),
    ("joint_3", (0, -0.00999, 0.21038), (3.14159, 0, 0)),
    ("joint_4", (0, 0.00640, 0.21038), (-1.5708, 1.5708, 0)),
    ("joint_5", (0, -0.00550, 0.20843), (3.14159, 0, 0)),
    ("joint_6", (0, 0, 0.10588),        (-1.5708, -1.5708, 0)),
    ("joint_ee",(0, 0, 0.06150),        (3.14159, 0, 0)),
]


def rpy_to_matrix(r, p, y):
    """RPY (extrinsic XYZ) to 3x3 rotation matrix."""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr],
    ]


def mat_vec(R, v):
    return [sum(R[i][j]*v[j] for j in range(3)) for i in range(3)]


def mat_mul(A, B):
    return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def compute_bone_based_origins():
    """Compute URDF joint origins from bone WORLD transforms (our algorithm)."""
    origins = []
    for i in range(1, len(BONES)):
        parent_name, pp, pq = BONES[i-1]
        child_name, cp, cq = BONES[i]

        delta = (cp[0]-pp[0], cp[1]-pp[1], cp[2]-pp[2])
        pq_inv = quat_conjugate(pq)
        pos_in_parent = quat_rotate_vector(pq_inv, delta)

        urdf_pos = (pos_in_parent[0]/100.0, -pos_in_parent[1]/100.0, pos_in_parent[2]/100.0)

        rel_quat = quat_multiply(pq_inv, cq)
        urdf_rpy = ue_quat_to_urdf_rpy(rel_quat)

        eps = 1e-10
        urdf_pos = tuple(0.0 if abs(v) < eps else v for v in urdf_pos)
        urdf_rpy = tuple(0.0 if abs(v) < eps else v for v in urdf_rpy)

        origins.append((child_name, urdf_pos, urdf_rpy))
    return origins


def forward_kinematics(origins):
    """Propagate the URDF FK chain and return world positions for each link."""
    positions = [(0.0, 0.0, 0.0)]  # root at origin
    cur_pos = [0.0, 0.0, 0.0]
    cur_rot = [[1,0,0],[0,1,0],[0,0,1]]

    for name, xyz, rpy in origins:
        rotated = mat_vec(cur_rot, list(xyz))
        new_pos = [cur_pos[j] + rotated[j] for j in range(3)]
        R_joint = rpy_to_matrix(*rpy)
        new_rot = mat_mul(cur_rot, R_joint)
        cur_pos = new_pos
        cur_rot = new_rot
        positions.append(tuple(cur_pos))

    return positions


def expected_urdf_positions():
    """Compute expected URDF-world positions (bone positions relative to root bone, in URDF coords)."""
    root_pos = BONES[0][1]
    root_quat = BONES[0][2]
    root_inv = quat_conjugate(root_quat)

    positions = [(0.0, 0.0, 0.0)]  # root at origin
    for i in range(1, len(BONES)):
        bp = BONES[i][1]
        delta = (bp[0]-root_pos[0], bp[1]-root_pos[1], bp[2]-root_pos[2])
        pos_in_root = quat_rotate_vector(root_inv, delta)
        urdf = (pos_in_root[0]/100.0, -pos_in_root[1]/100.0, pos_in_root[2]/100.0)
        positions.append(urdf)
    return positions


def main():
    print("=" * 80)
    print("URDF FK Verification with Kinova Gen3 Probe Data")
    print("=" * 80)

    # Step 1: Compute joint origins
    origins = compute_bone_based_origins()
    print("\n--- Computed URDF joint origins (bone-based) ---")
    print(f"{'Joint':<12s}  {'xyz (m)':<40s}  {'rpy (rad)':<40s}")
    for name, xyz, rpy in origins:
        print(f"{name:<12s}  ({xyz[0]:10.6f}, {xyz[1]:10.6f}, {xyz[2]:10.6f})  "
              f"({rpy[0]:8.4f}, {rpy[1]:8.4f}, {rpy[2]:8.4f})")

    # Step 2: Forward kinematics
    fk_positions = forward_kinematics(origins)
    expected = expected_urdf_positions()

    print("\n--- FK chain vs expected (bone WORLD relative to root, URDF coords) ---")
    print(f"{'Link':<12s}  {'FK position':<40s}  {'Expected position':<40s}  {'Error (m)'}")
    max_err = 0.0
    for i, (name, _, _) in enumerate([(BONES[0][0], None, None)] + origins):
        if i == 0:
            name = BONES[0][0]
        fk = fk_positions[i]
        ex = expected[i]
        err = math.sqrt(sum((fk[j]-ex[j])**2 for j in range(3)))
        max_err = max(max_err, err)
        print(f"{name:<12s}  ({fk[0]:10.6f}, {fk[1]:10.6f}, {fk[2]:10.6f})  "
              f"({ex[0]:10.6f}, {ex[1]:10.6f}, {ex[2]:10.6f})  {err:.10f}")

    if max_err < 1e-6:
        print(f"\n  FK chain is CONSISTENT (max error: {max_err:.2e}m)")
    else:
        print(f"\n  FK chain has ERRORS (max error: {max_err:.2e}m)")

    # Step 3: Compare with UE world positions (in URDF convention directly)
    print("\n--- UE WORLD positions converted directly to URDF world ---")
    print("(This shows what positions we'd expect if root frame = UE world frame)")
    print(f"{'Link':<12s}  {'Direct URDF pos':<40s}  {'FK pos (bone-frame-based)':<40s}")
    for i in range(len(BONES)):
        bp = BONES[i][1]
        direct_urdf = (bp[0]/100.0, -bp[1]/100.0, bp[2]/100.0)
        fk = fk_positions[i]
        print(f"{BONES[i][0]:<12s}  ({direct_urdf[0]:10.6f}, {direct_urdf[1]:10.6f}, {direct_urdf[2]:10.6f})  "
              f"({fk[0]:10.6f}, {fk[1]:10.6f}, {fk[2]:10.6f})")

    # Step 4: Show what the root bone's rotation does
    print("\n--- Root bone analysis ---")
    root_quat = BONES[0][2]
    print(f"Root bone WORLD quaternion (w,x,y,z): {root_quat}")
    col_x, col_y, col_z = quat_to_rotation_matrix(root_quat)
    print(f"Root bone X axis in UE world: ({col_x[0]:.4f}, {col_x[1]:.4f}, {col_x[2]:.4f})")
    print(f"Root bone Y axis in UE world: ({col_y[0]:.4f}, {col_y[1]:.4f}, {col_y[2]:.4f})")
    print(f"Root bone Z axis in UE world: ({col_z[0]:.4f}, {col_z[1]:.4f}, {col_z[2]:.4f})")

    # Convert to URDF frame meaning
    print(f"\nURDF base_link frame axes in UE world (with Y-flip):")
    print(f"  URDF X =  bone X = UE ({col_x[0]:.4f}, {col_x[1]:.4f}, {col_x[2]:.4f})")
    print(f"  URDF Y = -bone Y = UE ({-col_y[0]:.4f}, {-col_y[1]:.4f}, {-col_y[2]:.4f})")
    print(f"  URDF Z =  bone Z = UE ({col_z[0]:.4f}, {col_z[1]:.4f}, {col_z[2]:.4f})")

    root_is_identity = all(abs(v) < 0.01 for v in [
        root_quat[0]-1, root_quat[1], root_quat[2], root_quat[3]])
    if not root_is_identity:
        print("\n  WARNING: Root bone has non-identity rotation!")
        print("  The URDF base frame does NOT align with standard Z-up world frame.")
        print("  URDF Z-up axis corresponds to UE direction "
              f"({col_z[0]:.4f}, {col_z[1]:.4f}, {col_z[2]:.4f})")
        print("  This means the robot appears ROTATED when loaded in a standard URDF viewer.")

    # Step 5: Compare with official Kinova URDF
    print("\n--- Comparison with official Kinova Gen3 URDF (approximate) ---")
    print(f"{'Joint':<12s}  {'Our xyz':<35s}  {'Official xyz':<35s}  {'xyz diff'}")
    for i, (name, xyz, rpy) in enumerate(origins):
        if i < len(OFFICIAL_JOINTS):
            oname, oxyz, orpy = OFFICIAL_JOINTS[i]
            diff = math.sqrt(sum((xyz[j]-oxyz[j])**2 for j in range(3)))
            print(f"{name:<12s}  ({xyz[0]:8.5f}, {xyz[1]:8.5f}, {xyz[2]:8.5f})  "
                  f"({oxyz[0]:8.5f}, {oxyz[1]:8.5f}, {oxyz[2]:8.5f})  {diff:.5f}m")


if __name__ == "__main__":
    main()

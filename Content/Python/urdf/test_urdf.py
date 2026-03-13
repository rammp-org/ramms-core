"""
Tests for the URDF parser and utility modules.
These run standalone (no UE editor required).

Usage:
    cd C:/Users/waemf/data/Ramms/py
    python -m pytest urdf/test_urdf.py -v
    # or simply:
    python urdf/test_urdf.py
"""

from __future__ import annotations

import math
import os
import sys
import tempfile

# Ensure the py/ directory is on the path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from urdf.urdf_parser import URDFRobot
from urdf.urdf_utils import (
    m_to_cm, cm_to_m, rad_to_deg, deg_to_rad,
    urdf_pos_to_ue, ue_pos_to_urdf,
    urdf_axis_to_ue, urdf_inertia_to_ue, ue_inertia_to_urdf,
    classify_ue_constraint, rotation_matrix_to_rpy,
    quat_conjugate, quat_multiply, quat_rotate_vector,
    quat_to_rotation_matrix, ue_quat_to_urdf_quat, ue_quat_to_urdf_rpy,
    rpy_to_quat,
)
from urdf.name_mapping import NameMapping


# ===================================================================
# Sample URDF for testing
# ===================================================================

SAMPLE_URDF = """\
<?xml version="1.0" ?>
<robot name="test_robot">
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <mass value="5.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.03"/>
    </inertial>
    <collision>
      <geometry>
        <box size="0.2 0.3 0.1"/>
      </geometry>
    </collision>
  </link>

  <link name="shoulder_link">
    <inertial>
      <mass value="2.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.003"/>
    </inertial>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <link name="elbow_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0005"/>
    </inertial>
    <collision>
      <geometry>
        <sphere radius="0.04"/>
      </geometry>
    </collision>
  </link>

  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="39.0" velocity="1.3963"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <joint name="elbow_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="elbow_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.41" upper="2.41" effort="9.0" velocity="1.2218"/>
  </joint>
</robot>
"""


# ===================================================================
# Parser tests
# ===================================================================

def test_parse_basic():
    robot = URDFRobot.from_string(SAMPLE_URDF)
    assert robot.name == "test_robot"
    assert len(robot.links) == 3
    assert len(robot.joints) == 2


def test_parse_links():
    robot = URDFRobot.from_string(SAMPLE_URDF)
    base = robot.get_link("base_link")
    assert base is not None
    assert base.inertial is not None
    assert base.inertial.mass == 5.0
    assert abs(base.inertial.inertia.ixx - 0.01) < 1e-6
    assert abs(base.inertial.inertia.iyy - 0.02) < 1e-6
    assert len(base.collisions) == 1
    assert base.collisions[0].geometry.box is not None
    assert abs(base.collisions[0].geometry.box.size[0] - 0.2) < 1e-6


def test_parse_joints():
    robot = URDFRobot.from_string(SAMPLE_URDF)
    shoulder = robot.get_joint("shoulder_joint")
    assert shoulder is not None
    assert shoulder.joint_type == "revolute"
    assert shoulder.parent_link == "base_link"
    assert shoulder.child_link == "shoulder_link"
    assert shoulder.axis == (0.0, 0.0, 1.0)
    assert shoulder.limit is not None
    assert abs(shoulder.limit.effort - 39.0) < 1e-6
    assert shoulder.dynamics is not None
    assert abs(shoulder.dynamics.damping - 0.5) < 1e-6


def test_root_link():
    robot = URDFRobot.from_string(SAMPLE_URDF)
    root = robot.get_root_link()
    assert root is not None
    assert root.name == "base_link"


def test_tree_navigation():
    robot = URDFRobot.from_string(SAMPLE_URDF)
    children = robot.get_child_joints("base_link")
    assert len(children) == 1
    assert children[0].name == "shoulder_joint"

    parent_j = robot.get_parent_joint("elbow_link")
    assert parent_j is not None
    assert parent_j.name == "elbow_joint"


def test_round_trip_file():
    """Parse → write to file → parse again → verify."""
    robot = URDFRobot.from_string(SAMPLE_URDF)

    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
        from urdf.urdf_parser import robot_to_xml_string
        f.write(robot_to_xml_string(robot))
        tmp_path = f.name

    try:
        robot2 = URDFRobot.from_file(tmp_path)
        assert robot2.name == robot.name
        assert len(robot2.links) == len(robot.links)
        assert len(robot2.joints) == len(robot.joints)
        assert robot2.get_joint("shoulder_joint").joint_type == "revolute"
    finally:
        os.unlink(tmp_path)


# ===================================================================
# Utils tests
# ===================================================================

def test_unit_conversions():
    assert m_to_cm(1.0) == 100.0
    assert cm_to_m(100.0) == 1.0
    assert abs(rad_to_deg(math.pi) - 180.0) < 1e-6
    assert abs(deg_to_rad(180.0) - math.pi) < 1e-6


def test_position_conversion():
    # 1m in URDF → 100cm in UE, Y negated
    ue = urdf_pos_to_ue((1.0, 2.0, 3.0))
    assert abs(ue[0] - 100.0) < 1e-6
    assert abs(ue[1] - (-200.0)) < 1e-6
    assert abs(ue[2] - 300.0) < 1e-6

    # Round-trip
    back = ue_pos_to_urdf(ue)
    assert abs(back[0] - 1.0) < 1e-6
    assert abs(back[1] - 2.0) < 1e-6
    assert abs(back[2] - 3.0) < 1e-6


def test_axis_conversion():
    ue_axis = urdf_axis_to_ue((0, 1, 0))
    assert ue_axis == (0, -1, 0)


def test_inertia_conversion():
    ue = urdf_inertia_to_ue(0.01, 0.002, 0.003, 0.02, 0.004, 0.03)
    # Diagonal: multiplied by 10000
    assert abs(ue[0] - 100.0) < 1e-6  # ixx
    assert abs(ue[3] - 200.0) < 1e-6  # iyy
    assert abs(ue[5] - 300.0) < 1e-6  # izz
    # Off-diagonal with Y: negated
    assert abs(ue[1] - (-20.0)) < 1e-6  # ixy
    assert abs(ue[4] - (-40.0)) < 1e-6  # iyz

    # Round-trip
    back = ue_inertia_to_urdf(*ue)
    assert abs(back[0] - 0.01) < 1e-8
    assert abs(back[1] - 0.002) < 1e-8


def test_classify_constraint():
    # All locked = fixed
    jtype, axes = classify_ue_constraint(
        "ACM_Locked", "ACM_Locked", "ACM_Locked",
        "LCM_Locked", "LCM_Locked", "LCM_Locked")
    assert jtype == "fixed"

    # Only twist limited = revolute
    jtype, axes = classify_ue_constraint(
        "ACM_Limited", "ACM_Locked", "ACM_Locked",
        "LCM_Locked", "LCM_Locked", "LCM_Locked")
    assert jtype == "revolute"
    assert "twist" in axes

    # Only twist free = continuous
    jtype, axes = classify_ue_constraint(
        "ACM_Free", "ACM_Locked", "ACM_Locked",
        "LCM_Locked", "LCM_Locked", "LCM_Locked")
    assert jtype == "continuous"
    assert "twist" in axes

    # Only swing1 free = continuous
    jtype, axes = classify_ue_constraint(
        "ACM_Locked", "ACM_Free", "ACM_Locked",
        "LCM_Locked", "LCM_Locked", "LCM_Locked")
    assert jtype == "continuous"
    assert "swing1" in axes

    # Only linear X limited = prismatic
    jtype, axes = classify_ue_constraint(
        "ACM_Locked", "ACM_Locked", "ACM_Locked",
        "LCM_Limited", "LCM_Locked", "LCM_Locked")
    assert jtype == "prismatic"

    # Swing1 limited + twist/swing2 locked = revolute
    jtype, axes = classify_ue_constraint(
        "ACM_Locked", "ACM_Limited", "ACM_Locked",
        "LCM_Locked", "LCM_Locked", "LCM_Locked")
    assert jtype == "revolute"
    assert "swing1" in axes


def test_rotation_matrix_to_rpy():
    # Identity matrix → (0, 0, 0)
    rpy = rotation_matrix_to_rpy((1, 0, 0), (0, 1, 0), (0, 0, 1))
    assert all(abs(v) < 1e-10 for v in rpy)

    # 90° yaw (rotation about Z)
    rpy = rotation_matrix_to_rpy((0, 1, 0), (-1, 0, 0), (0, 0, 1))
    assert abs(rpy[0]) < 1e-10          # roll ≈ 0
    assert abs(rpy[1]) < 1e-10          # pitch ≈ 0
    assert abs(rpy[2] - math.pi/2) < 1e-6  # yaw ≈ π/2

    # 90° pitch (rotation about Y)
    rpy = rotation_matrix_to_rpy((0, 0, -1), (0, 1, 0), (1, 0, 0))
    assert abs(rpy[0]) < 1e-10          # roll ≈ 0
    assert abs(rpy[1] - math.pi/2) < 1e-6  # pitch ≈ π/2
    assert abs(rpy[2]) < 1e-10          # yaw ≈ 0

    # 90° roll (rotation about X)
    rpy = rotation_matrix_to_rpy((1, 0, 0), (0, 0, 1), (0, -1, 0))
    assert abs(rpy[0] - math.pi/2) < 1e-6  # roll ≈ π/2
    assert abs(rpy[1]) < 1e-10          # pitch ≈ 0
    assert abs(rpy[2]) < 1e-10          # yaw ≈ 0

def test_exact_match():
    mapping = NameMapping.auto_match(
        ["base_link", "shoulder_link"],
        ["base_link", "shoulder_link"],
    )
    assert mapping.get_bone("base_link") == "base_link"
    assert mapping.get_bone("shoulder_link") == "shoulder_link"
    assert len(mapping.unmatched_links) == 0


def test_case_insensitive_match():
    mapping = NameMapping.auto_match(
        ["Base_Link", "Shoulder_Link"],
        ["base_link", "shoulder_link"],
    )
    assert mapping.get_bone("Base_Link") == "base_link"


def test_normalized_match():
    mapping = NameMapping.auto_match(
        ["base_link", "shoulder"],
        ["base", "shoulder_link"],
    )
    assert mapping.get_bone("base_link") == "base"
    assert mapping.get_bone("shoulder") == "shoulder_link"


def test_override_file():
    """Test JSON override loading."""
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
        import json
        json.dump({
            "links": {"urdf_name": "ue_bone"},
            "joints": {"urdf_joint": "ue_constraint"}
        }, f)
        tmp_path = f.name

    try:
        mapping = NameMapping.auto_match(
            ["urdf_name", "other"],
            ["ue_bone", "other"],
            override_file=tmp_path,
        )
        assert mapping.get_bone("urdf_name") == "ue_bone"
        assert mapping.joint_to_constraint.get("urdf_joint") == "ue_constraint"
    finally:
        os.unlink(tmp_path)


def test_save_load_mapping():
    """Test save/load round-trip."""
    mapping = NameMapping.auto_match(["a", "b"], ["a", "b"])
    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
        tmp_path = f.name

    try:
        mapping.save(tmp_path)
        loaded = NameMapping.from_file(tmp_path)
        assert loaded.get_bone("a") == "a"
        assert loaded.get_bone("b") == "b"
    finally:
        os.unlink(tmp_path)


# ===================================================================
# Quaternion utility tests
# ===================================================================

def test_quat_conjugate():
    q = (0.7071, 0.1, 0.2, 0.3)
    qc = quat_conjugate(q)
    assert qc == (0.7071, -0.1, -0.2, -0.3)

    # Conjugate of identity is identity
    assert quat_conjugate((1, 0, 0, 0)) == (1, 0, 0, 0)


def test_quat_multiply():
    # Identity * q = q
    q = (math.cos(math.pi/4), 0.0, math.sin(math.pi/4), 0.0)
    result = quat_multiply((1, 0, 0, 0), q)
    for a, b in zip(result, q):
        assert abs(a - b) < 1e-10

    # q * q_inv = identity
    q_inv = quat_conjugate(q)
    result = quat_multiply(q, q_inv)
    assert abs(result[0] - 1.0) < 1e-6
    for i in range(1, 4):
        assert abs(result[i]) < 1e-6

    # 90° around Z twice = 180° around Z
    qz90 = (math.cos(math.pi/4), 0, 0, math.sin(math.pi/4))
    qz180 = quat_multiply(qz90, qz90)
    # 180° around Z: (cos(90°), 0, 0, sin(90°)) = (0, 0, 0, 1)
    assert abs(qz180[0]) < 1e-6
    assert abs(qz180[3] - 1.0) < 1e-6


def test_quat_rotate_vector():
    # Identity rotation: vector unchanged
    v = (1.0, 2.0, 3.0)
    result = quat_rotate_vector((1, 0, 0, 0), v)
    for a, b in zip(result, v):
        assert abs(a - b) < 1e-10

    # 90° around Z: X → Y
    qz90 = (math.cos(math.pi/4), 0, 0, math.sin(math.pi/4))
    result = quat_rotate_vector(qz90, (1, 0, 0))
    assert abs(result[0]) < 1e-6
    assert abs(result[1] - 1.0) < 1e-6
    assert abs(result[2]) < 1e-6

    # 90° around X: Y → Z
    qx90 = (math.cos(math.pi/4), math.sin(math.pi/4), 0, 0)
    result = quat_rotate_vector(qx90, (0, 1, 0))
    assert abs(result[0]) < 1e-6
    assert abs(result[1]) < 1e-6
    assert abs(result[2] - 1.0) < 1e-6


def test_quat_to_rotation_matrix():
    # Identity quaternion → identity matrix
    cx, cy, cz = quat_to_rotation_matrix((1, 0, 0, 0))
    assert abs(cx[0] - 1) < 1e-10 and abs(cx[1]) < 1e-10 and abs(cx[2]) < 1e-10
    assert abs(cy[0]) < 1e-10 and abs(cy[1] - 1) < 1e-10 and abs(cy[2]) < 1e-10
    assert abs(cz[0]) < 1e-10 and abs(cz[1]) < 1e-10 and abs(cz[2] - 1) < 1e-10

    # 90° around Z: X→Y, Y→-X
    qz90 = (math.cos(math.pi/4), 0, 0, math.sin(math.pi/4))
    cx, cy, cz = quat_to_rotation_matrix(qz90)
    assert abs(cx[0]) < 1e-6 and abs(cx[1] - 1.0) < 1e-6
    assert abs(cy[0] - (-1.0)) < 1e-6 and abs(cy[1]) < 1e-6


def test_ue_quat_to_urdf_rpy():
    # Identity UE rotation → zero RPY
    rpy = ue_quat_to_urdf_rpy((1, 0, 0, 0))
    assert all(abs(v) < 1e-10 for v in rpy)

    # 90° around UE Z → -90° around URDF Z (handedness flip on X/Z components)
    qz90_ue = (math.cos(math.pi/4), 0, 0, math.sin(math.pi/4))
    rpy = ue_quat_to_urdf_rpy(qz90_ue)
    assert abs(rpy[0]) < 1e-6          # roll ≈ 0
    assert abs(rpy[1]) < 1e-6          # pitch ≈ 0
    assert abs(rpy[2] - (-math.pi/2)) < 1e-6  # yaw ≈ -π/2

    # 90° around UE Y → 90° around URDF Y (Y rotation preserved)
    qy90_ue = (math.cos(math.pi/4), 0, math.sin(math.pi/4), 0)
    rpy = ue_quat_to_urdf_rpy(qy90_ue)
    assert abs(rpy[0]) < 1e-6
    assert abs(rpy[1] - math.pi/2) < 1e-6
    assert abs(rpy[2]) < 1e-6

    # 180° rotation (q and -q same rotation): quat=(-1,0,0,0) → identity
    rpy = ue_quat_to_urdf_rpy((-1, 0, 0, 0))
    assert all(abs(v) < 1e-10 for v in rpy)


def test_rpy_to_quat():
    """rpy_to_quat should roundtrip with ue_quat_to_urdf_rpy for URDF rotations."""
    # Identity
    q = rpy_to_quat(0, 0, 0)
    assert abs(q[0] - 1) < 1e-10 and all(abs(q[i]) < 1e-10 for i in range(1, 4))

    # 90° roll
    q = rpy_to_quat(math.pi/2, 0, 0)
    cx, cy, cz = quat_to_rotation_matrix(q)
    rpy = rotation_matrix_to_rpy(cx, cy, cz)
    assert abs(rpy[0] - math.pi/2) < 1e-10
    assert abs(rpy[1]) < 1e-10
    assert abs(rpy[2]) < 1e-10

    # 90° pitch
    q = rpy_to_quat(0, math.pi/2, 0)
    cx, cy, cz = quat_to_rotation_matrix(q)
    rpy = rotation_matrix_to_rpy(cx, cy, cz)
    assert abs(rpy[0]) < 1e-6
    assert abs(rpy[1] - math.pi/2) < 1e-6
    assert abs(rpy[2]) < 1e-6

    # 90° yaw
    q = rpy_to_quat(0, 0, math.pi/2)
    cx, cy, cz = quat_to_rotation_matrix(q)
    rpy = rotation_matrix_to_rpy(cx, cy, cz)
    assert abs(rpy[0]) < 1e-10
    assert abs(rpy[1]) < 1e-10
    assert abs(rpy[2] - math.pi/2) < 1e-10

    # Composition: Rz(pi/4) * Rx(pi/3) via quaternions should match
    q1 = rpy_to_quat(math.pi/3, 0, 0)
    q2 = rpy_to_quat(0, 0, math.pi/4)
    q_composed = quat_multiply(q2, q1)  # Rz * Rx
    # Verify it's a valid unit quaternion
    mag = math.sqrt(sum(c**2 for c in q_composed))
    assert abs(mag - 1.0) < 1e-10


def test_world_aligned_joint_origin():
    """World-aligned joint origins should be direct UE world-space offsets."""
    # Kinova: base_link at (0,0,0) with non-identity rotation,
    # link_0 at (0,0,15.88)
    # World-aligned origin = ue_pos_to_urdf((0, 0, 15.88)) = (0, 0, 0.1588)
    delta = (0.0, 0.0, 15.88)  # child_world - parent_world (cm)
    urdf_pos = ue_pos_to_urdf(delta)
    assert abs(urdf_pos[0]) < 1e-10
    assert abs(urdf_pos[1]) < 1e-10
    assert abs(urdf_pos[2] - 0.1588) < 1e-6


def test_bone_rotation_collision():
    """Bone rotation should properly transform collision centers to world frame."""
    from urdf.urdf_utils import ue_quat_to_urdf_quat

    # A bone rotated 90° about UE Z: quat = (cos45, 0, 0, sin45)
    bone_q = (math.cos(math.pi/4), 0, 0, math.sin(math.pi/4))

    # Collision center at (10, 0, 0) in bone-local UE space
    center_local = (10.0, 0.0, 0.0)
    center_world = quat_rotate_vector(bone_q, center_local)
    # After 90° Z rotation: (10,0,0) -> (0,10,0) in UE world
    assert abs(center_world[0]) < 1e-6
    assert abs(center_world[1] - 10.0) < 1e-6
    assert abs(center_world[2]) < 1e-6

    # Convert to URDF: (0, -10, 0) cm -> (0, -0.1, 0) m
    urdf_pos = ue_pos_to_urdf(center_world)
    assert abs(urdf_pos[0]) < 1e-8
    assert abs(urdf_pos[1] - (-0.1)) < 1e-8
    assert abs(urdf_pos[2]) < 1e-8


# ===================================================================
# Main
# ===================================================================

if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    passed = 0
    failed = 0
    for test_fn in tests:
        try:
            test_fn()
            print(f"  PASS: {test_fn.__name__}")
            passed += 1
        except Exception as e:
            print(f"  FAIL: {test_fn.__name__}: {e}")
            failed += 1
    print(f"\n{passed} passed, {failed} failed out of {passed + failed} tests")
    sys.exit(1 if failed else 0)

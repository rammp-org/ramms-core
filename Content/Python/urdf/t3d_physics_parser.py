"""
Parser for UE T3D PhysicsAsset export format.

Extracts body setups (bone names, collision geometry, mass) and
constraint templates (bone pairs, joint limits, drive settings) from
the text produced by unreal.AssetExportTask on a PhysicsAsset.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


# ===================================================================
# Data structures
# ===================================================================

@dataclass
class T3DVector:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class T3DBoxElem:
    center: T3DVector = field(default_factory=T3DVector)
    x: float = 0.0  # full extent in X
    y: float = 0.0
    z: float = 0.0
    rotation: Optional[T3DVector] = None


@dataclass
class T3DSphereElem:
    center: T3DVector = field(default_factory=T3DVector)
    radius: float = 0.0


@dataclass
class T3DSphylElem:
    """Capsule (Sphyl) element."""
    center: T3DVector = field(default_factory=T3DVector)
    radius: float = 0.0
    length: float = 0.0
    rotation: Optional[T3DVector] = None


@dataclass
class T3DConvexElem:
    """Convex hull collision element (e.g. from copied static mesh colliders)."""
    vertices: List[T3DVector] = field(default_factory=list)
    index_data: List[int] = field(default_factory=list)
    elem_box_min: T3DVector = field(default_factory=T3DVector)
    elem_box_max: T3DVector = field(default_factory=T3DVector)


@dataclass
class T3DBodySetup:
    """Parsed SkeletalBodySetup from T3D."""
    name: str = ""
    bone_name: str = ""
    boxes: List[T3DBoxElem] = field(default_factory=list)
    spheres: List[T3DSphereElem] = field(default_factory=list)
    capsules: List[T3DSphylElem] = field(default_factory=list)
    convex_elems: List[T3DConvexElem] = field(default_factory=list)
    mass: float = 0.0
    collision_trace_flag: str = ""


@dataclass
class T3DConstraint:
    """Parsed PhysicsConstraintTemplate from T3D."""
    name: str = ""
    joint_name: str = ""
    constraint_bone1: str = ""  # child bone
    constraint_bone2: str = ""  # parent bone
    pos1: T3DVector = field(default_factory=T3DVector)
    pos2: T3DVector = field(default_factory=T3DVector)
    pri_axis1: T3DVector = field(default_factory=T3DVector)
    pri_axis2: T3DVector = field(default_factory=T3DVector)
    sec_axis1: T3DVector = field(default_factory=T3DVector)
    sec_axis2: T3DVector = field(default_factory=T3DVector)
    # Motion types: "ACM_Locked", "ACM_Limited", "ACM_Free"
    # Defaults match UE C++ defaults (ACM_Free) so T3D-omitted values are correct
    swing1_motion: str = "ACM_Free"
    swing2_motion: str = "ACM_Free"
    twist_motion: str = "ACM_Free"
    swing1_limit_deg: float = 45.0
    swing2_limit_deg: float = 45.0
    twist_limit_deg: float = 45.0
    # Linear (UE defaults are LCM_Locked for physics constraints)
    linear_x_motion: str = "LCM_Locked"
    linear_y_motion: str = "LCM_Locked"
    linear_z_motion: str = "LCM_Locked"
    linear_limit_size: float = 0.0
    # Drive
    twist_drive_stiffness: float = 0.0
    twist_drive_damping: float = 0.0
    swing_drive_stiffness: float = 0.0
    swing_drive_damping: float = 0.0
    angular_drive_mode: str = ""
    parent_dominates: bool = True


@dataclass
class T3DPhysicsAsset:
    """Full parsed PhysicsAsset from T3D."""
    name: str = ""
    preview_skeletal_mesh: str = ""
    body_setups: List[T3DBodySetup] = field(default_factory=list)
    constraints: List[T3DConstraint] = field(default_factory=list)


# ===================================================================
# Regex helpers
# ===================================================================

def _find_str(text: str, key: str) -> str:
    """Extract a quoted string value: Key="value" """
    m = re.search(rf'{key}="([^"]*)"', text)
    return m.group(1) if m else ""


def _find_float(text: str, key: str, default: float = 0.0) -> float:
    """Extract a float value: Key=123.456"""
    m = re.search(rf'{key}=(-?[\d.]+)', text)
    return float(m.group(1)) if m else default


def _find_enum(text: str, key: str, default: str = "") -> str:
    """Extract an enum value: Key=ACM_Limited"""
    m = re.search(rf'{key}=(\w+)', text)
    return m.group(1) if m else default


def _find_bool(text: str, key: str, default: bool = False) -> bool:
    """Extract a boolean: Key=True/False"""
    m = re.search(rf'{key}=(True|False)', text)
    return m.group(1) == "True" if m else default


def _find_vector(text: str, key: str) -> T3DVector:
    """Extract a vector: Key=(X=1.0,Y=2.0,Z=3.0)"""
    m = re.search(rf'{key}=\(X=(-?[\d.]+),Y=(-?[\d.]+),Z=(-?[\d.]+)\)', text)
    if m:
        return T3DVector(float(m.group(1)), float(m.group(2)), float(m.group(3)))
    return T3DVector()


def _find_balanced_parens(text: str, key: str) -> str:
    """Extract content within balanced parentheses after Key=(...).
    Returns the inner content (without the outer parens)."""
    pattern = rf'{key}=\('
    m = re.search(pattern, text)
    if not m:
        return ""
    start = m.end()
    depth = 1
    i = start
    while i < len(text) and depth > 0:
        if text[i] == '(':
            depth += 1
        elif text[i] == ')':
            depth -= 1
        i += 1
    return text[start:i - 1]


# ===================================================================
# Object block splitting
# ===================================================================

def _split_objects(t3d_text: str) -> List[Tuple[str, str, str]]:
    """Split T3D text into (class_name, object_name, content) tuples.

    Only returns the detailed object blocks (those with properties),
    not the forward-declaration stubs.
    """
    results = []
    lines = t3d_text.split("\n")
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        # Match detailed blocks: "Begin Object Name=..." (without Class= prefix)
        m = re.match(r'Begin Object Name="(\w+)"', line)
        if m:
            obj_name = m.group(1)
            # Determine class from the name
            if "ConstraintTemplate" in obj_name:
                cls = "PhysicsConstraintTemplate"
            elif "BodySetup" in obj_name:
                cls = "SkeletalBodySetup"
            else:
                cls = "Unknown"

            # Collect content until End Object
            content_lines = []
            i += 1
            while i < len(lines):
                ln = lines[i].strip()
                if ln == "End Object":
                    break
                content_lines.append(ln)
                i += 1
            results.append((cls, obj_name, "\n".join(content_lines)))
        i += 1
    return results


# ===================================================================
# Body setup parsing
# ===================================================================

def _parse_box_elems(agg_text: str) -> List[T3DBoxElem]:
    """Parse BoxElems from AggGeom text."""
    boxes = []
    box_content = _find_balanced_parens(agg_text, "BoxElems")
    if not box_content:
        return boxes
    # Each box is enclosed in inner parens: ((Center=...,X=...,Y=...,Z=...),(next))
    # Split by ),(  to find individual boxes
    # Actually they're separated by ),( at depth 0
    box_strs = _split_array_elements(box_content)
    for bs in box_strs:
        box = T3DBoxElem()
        box.center = _find_vector(bs, "Center")
        box.x = _find_float(bs, r"(?<![\w.])X", 0.0)
        box.y = _find_float(bs, r"(?<![\w.])Y", 0.0)
        box.z = _find_float(bs, r"(?<![\w.])Z", 0.0)
        # Get standalone X, Y, Z (not inside Center/Rotation)
        # After Center=(...), the remaining X=, Y=, Z= are the extents
        # Remove the Center portion first
        remaining = re.sub(r'Center=\([^)]*\)', '', bs)
        remaining = re.sub(r'Rotation=\([^)]*\)', '', remaining)
        mx = re.search(r'(?<![A-Za-z])X=(-?[\d.]+)', remaining)
        my = re.search(r'(?<![A-Za-z])Y=(-?[\d.]+)', remaining)
        mz = re.search(r'(?<![A-Za-z])Z=(-?[\d.]+)', remaining)
        if mx:
            box.x = float(mx.group(1))
        if my:
            box.y = float(my.group(1))
        if mz:
            box.z = float(mz.group(1))
        boxes.append(box)
    return boxes


def _parse_sphere_elems(agg_text: str) -> List[T3DSphereElem]:
    """Parse SphereElems from AggGeom text."""
    spheres = []
    sphere_content = _find_balanced_parens(agg_text, "SphereElems")
    if not sphere_content:
        return spheres
    for ss in _split_array_elements(sphere_content):
        s = T3DSphereElem()
        s.center = _find_vector(ss, "Center")
        s.radius = _find_float(ss, "Radius")
        spheres.append(s)
    return spheres


def _parse_sphyl_elems(agg_text: str) -> List[T3DSphylElem]:
    """Parse SphylElems (capsules) from AggGeom text."""
    capsules = []
    sphyl_content = _find_balanced_parens(agg_text, "SphylElems")
    if not sphyl_content:
        return capsules
    for cs in _split_array_elements(sphyl_content):
        c = T3DSphylElem()
        c.center = _find_vector(cs, "Center")
        c.radius = _find_float(cs, "Radius")
        c.length = _find_float(cs, "Length")
        capsules.append(c)
    return capsules


def _parse_convex_elems(agg_text: str) -> List[T3DConvexElem]:
    """Parse ConvexElems from AggGeom text.

    These appear when collision is copied from a static mesh (e.g. cylinder wheels).
    """
    convex_content = _find_balanced_parens(agg_text, "ConvexElems")
    if not convex_content:
        return []
    result = []
    for cs in _split_array_elements(convex_content):
        elem = T3DConvexElem()

        # Parse VertexData — array of (X=...,Y=...,Z=...) vectors
        vd = _find_balanced_parens(cs, "VertexData")
        if vd:
            for vs in _split_array_elements(vd):
                x = _find_float(vs, "X")
                y = _find_float(vs, "Y")
                z = _find_float(vs, "Z")
                elem.vertices.append(T3DVector(x, y, z))

        # Parse IndexData — flat list of triangle indices
        id_text = _find_balanced_parens(cs, "IndexData")
        if id_text:
            elem.index_data = [
                int(x.strip()) for x in id_text.split(",")
                if x.strip().lstrip("-").isdigit()
            ]

        # Parse ElemBox for bounding box
        eb = _find_balanced_parens(cs, "ElemBox")
        if eb:
            elem.elem_box_min = _find_vector(eb, "Min")
            elem.elem_box_max = _find_vector(eb, "Max")
        elif elem.vertices:
            # Compute bounding box from vertices
            elem.elem_box_min = T3DVector(
                min(v.x for v in elem.vertices),
                min(v.y for v in elem.vertices),
                min(v.z for v in elem.vertices))
            elem.elem_box_max = T3DVector(
                max(v.x for v in elem.vertices),
                max(v.y for v in elem.vertices),
                max(v.z for v in elem.vertices))

        result.append(elem)
    return result


def _split_array_elements(text: str) -> List[str]:
    """Split T3D array elements at depth-0 commas between parens.
    Input like: (elem1),(elem2) → ["elem1", "elem2"]
    """
    elements = []
    depth = 0
    current = []
    for ch in text:
        if ch == '(':
            if depth == 0:
                current = []
                depth += 1
                continue
            depth += 1
        elif ch == ')':
            depth -= 1
            if depth == 0:
                elements.append("".join(current))
                continue
        if depth > 0:
            current.append(ch)
    return elements


def _parse_body_setup(obj_name: str, content: str) -> T3DBodySetup:
    """Parse a SkeletalBodySetup from its T3D content."""
    body = T3DBodySetup(name=obj_name)
    body.bone_name = _find_str(content, "BoneName")
    body.collision_trace_flag = _find_enum(content, "CollisionTraceFlag")

    # Parse AggGeom
    agg_text = _find_balanced_parens(content, "AggGeom")
    if agg_text:
        body.boxes = _parse_box_elems(agg_text)
        body.spheres = _parse_sphere_elems(agg_text)
        body.capsules = _parse_sphyl_elems(agg_text)
        body.convex_elems = _parse_convex_elems(agg_text)

    # Mass override (if present)
    body.mass = _find_float(content, "MassInKg", 0.0)
    if body.mass == 0.0:
        body.mass = _find_float(content, "DefaultInstanceMassOverride", 0.0)

    return body


# ===================================================================
# Constraint parsing
# ===================================================================

def _parse_constraint(obj_name: str, content: str) -> T3DConstraint:
    """Parse a PhysicsConstraintTemplate from its T3D content."""
    c = T3DConstraint(name=obj_name)

    # The entire constraint data is in DefaultInstance=(...)
    di = _find_balanced_parens(content, "DefaultInstance")
    if not di:
        return c

    c.joint_name = _find_str(di, "JointName")
    c.constraint_bone1 = _find_str(di, "ConstraintBone1")
    c.constraint_bone2 = _find_str(di, "ConstraintBone2")
    c.pos1 = _find_vector(di, "Pos1")
    c.pos2 = _find_vector(di, "Pos2")
    c.pri_axis1 = _find_vector(di, "PriAxis1")
    c.pri_axis2 = _find_vector(di, "PriAxis2")
    c.sec_axis1 = _find_vector(di, "SecAxis1")
    c.sec_axis2 = _find_vector(di, "SecAxis2")

    # Profile instance
    pi = _find_balanced_parens(di, "ProfileInstance")
    if pi:
        # Cone limit (swing)
        cone = _find_balanced_parens(pi, "ConeLimit")
        if cone:
            c.swing1_motion = _find_enum(cone, "Swing1Motion", "ACM_Locked")
            c.swing2_motion = _find_enum(cone, "Swing2Motion", "ACM_Locked")
            c.swing1_limit_deg = _find_float(cone, "Swing1LimitDegrees", 45.0)
            c.swing2_limit_deg = _find_float(cone, "Swing2LimitDegrees", 45.0)

        # Twist limit
        twist = _find_balanced_parens(pi, "TwistLimit")
        if twist:
            c.twist_motion = _find_enum(twist, "TwistMotion", "ACM_Locked")
            c.twist_limit_deg = _find_float(twist, "TwistLimitDegrees", 45.0)

        # Linear limit
        lin = _find_balanced_parens(pi, "LinearLimit")
        if lin:
            c.linear_x_motion = _find_enum(lin, "XMotion", "LCM_Locked")
            c.linear_y_motion = _find_enum(lin, "YMotion", "LCM_Locked")
            c.linear_z_motion = _find_enum(lin, "ZMotion", "LCM_Locked")
            c.linear_limit_size = _find_float(lin, "Limit", 0.0)

        # Angular drive
        ang_drive = _find_balanced_parens(pi, "AngularDrive")
        if ang_drive:
            twist_d = _find_balanced_parens(ang_drive, "TwistDrive")
            if twist_d:
                c.twist_drive_stiffness = _find_float(twist_d, "Stiffness")
                c.twist_drive_damping = _find_float(twist_d, "Damping")
            swing_d = _find_balanced_parens(ang_drive, "SwingDrive")
            if swing_d:
                c.swing_drive_stiffness = _find_float(swing_d, "Stiffness")
                c.swing_drive_damping = _find_float(swing_d, "Damping")
            c.angular_drive_mode = _find_enum(ang_drive, "AngularDriveMode")

        c.parent_dominates = _find_bool(pi, "bParentDominates", True)

    # T3D omits default axis values: PriAxis=(1,0,0), SecAxis=(0,1,0).
    # Fix up zero vectors to UE defaults so axis computation works.
    def _is_zero(v: T3DVector) -> bool:
        return abs(v.x) < 1e-10 and abs(v.y) < 1e-10 and abs(v.z) < 1e-10

    if _is_zero(c.pri_axis1):
        c.pri_axis1 = T3DVector(1.0, 0.0, 0.0)
    if _is_zero(c.pri_axis2):
        c.pri_axis2 = T3DVector(1.0, 0.0, 0.0)
    if _is_zero(c.sec_axis1):
        c.sec_axis1 = T3DVector(0.0, 1.0, 0.0)
    if _is_zero(c.sec_axis2):
        c.sec_axis2 = T3DVector(0.0, 1.0, 0.0)

    return c


# ===================================================================
# Top-level parser
# ===================================================================

def parse_physics_asset_t3d(t3d_text: str) -> T3DPhysicsAsset:
    """Parse a PhysicsAsset T3D export string into structured data."""
    result = T3DPhysicsAsset()

    # Extract asset name from first line
    m = re.search(r'Name="(\w+)"', t3d_text.split("\n")[0])
    if m:
        result.name = m.group(1)

    # Extract PreviewSkeletalMesh
    m = re.search(r'PreviewSkeletalMesh="([^"]+)"', t3d_text)
    if m:
        result.preview_skeletal_mesh = m.group(1)

    # Parse detailed object blocks
    for cls, obj_name, content in _split_objects(t3d_text):
        if cls == "SkeletalBodySetup":
            result.body_setups.append(_parse_body_setup(obj_name, content))
        elif cls == "PhysicsConstraintTemplate":
            result.constraints.append(_parse_constraint(obj_name, content))

    return result

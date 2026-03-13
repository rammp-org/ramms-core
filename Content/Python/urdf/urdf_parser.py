"""
Pure-Python URDF parser.

Parses URDF XML into a lightweight data model (no external dependencies beyond
the Python standard library). Handles <robot>, <link>, <joint>, <inertial>,
<visual>, <collision>, and <geometry> elements.

Usage from UE Editor Python console:
    from urdf.urdf_parser import URDFRobot
    robot = URDFRobot.from_file("C:/path/to/robot.urdf")
    for joint in robot.joints:
        print(joint.name, joint.joint_type, joint.parent_link, joint.child_link)
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


# ---------------------------------------------------------------------------
# Geometry primitives
# ---------------------------------------------------------------------------

@dataclass
class URDFBox:
    size: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class URDFCylinder:
    radius: float = 0.0
    length: float = 0.0


@dataclass
class URDFSphere:
    radius: float = 0.0


@dataclass
class URDFMesh:
    filename: str = ""
    scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)


@dataclass
class URDFGeometry:
    """Exactly one of the geometry fields is set."""
    box: Optional[URDFBox] = None
    cylinder: Optional[URDFCylinder] = None
    sphere: Optional[URDFSphere] = None
    mesh: Optional[URDFMesh] = None


# ---------------------------------------------------------------------------
# Origin (xyz + rpy)
# ---------------------------------------------------------------------------

@dataclass
class URDFOrigin:
    xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# Inertial properties
# ---------------------------------------------------------------------------

@dataclass
class URDFInertia:
    ixx: float = 0.0
    ixy: float = 0.0
    ixz: float = 0.0
    iyy: float = 0.0
    iyz: float = 0.0
    izz: float = 0.0


@dataclass
class URDFInertial:
    origin: URDFOrigin = field(default_factory=URDFOrigin)
    mass: float = 0.0
    inertia: URDFInertia = field(default_factory=URDFInertia)


# ---------------------------------------------------------------------------
# Visual / Collision
# ---------------------------------------------------------------------------

@dataclass
class URDFMaterial:
    name: str = ""
    color_rgba: Optional[Tuple[float, float, float, float]] = None
    texture_filename: Optional[str] = None


@dataclass
class URDFVisual:
    name: str = ""
    origin: URDFOrigin = field(default_factory=URDFOrigin)
    geometry: Optional[URDFGeometry] = None
    material: Optional[URDFMaterial] = None


@dataclass
class URDFCollision:
    name: str = ""
    origin: URDFOrigin = field(default_factory=URDFOrigin)
    geometry: Optional[URDFGeometry] = None


# ---------------------------------------------------------------------------
# Link
# ---------------------------------------------------------------------------

@dataclass
class URDFLink:
    name: str = ""
    inertial: Optional[URDFInertial] = None
    visuals: List[URDFVisual] = field(default_factory=list)
    collisions: List[URDFCollision] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Joint limits / dynamics / safety / mimic
# ---------------------------------------------------------------------------

@dataclass
class URDFJointLimit:
    lower: float = 0.0
    upper: float = 0.0
    effort: float = 0.0
    velocity: float = 0.0


@dataclass
class URDFJointDynamics:
    damping: float = 0.0
    friction: float = 0.0


@dataclass
class URDFJointSafety:
    soft_lower_limit: float = 0.0
    soft_upper_limit: float = 0.0
    k_position: float = 0.0
    k_velocity: float = 0.0


@dataclass
class URDFJointMimic:
    joint: str = ""
    multiplier: float = 1.0
    offset: float = 0.0


# ---------------------------------------------------------------------------
# Joint
# ---------------------------------------------------------------------------

@dataclass
class URDFJoint:
    name: str = ""
    joint_type: str = "fixed"  # revolute, continuous, prismatic, fixed, floating, planar
    parent_link: str = ""
    child_link: str = ""
    origin: URDFOrigin = field(default_factory=URDFOrigin)
    axis: Tuple[float, float, float] = (1.0, 0.0, 0.0)
    limit: Optional[URDFJointLimit] = None
    dynamics: Optional[URDFJointDynamics] = None
    safety: Optional[URDFJointSafety] = None
    mimic: Optional[URDFJointMimic] = None


# ---------------------------------------------------------------------------
# Robot (top-level)
# ---------------------------------------------------------------------------

@dataclass
class URDFRobot:
    name: str = ""
    links: List[URDFLink] = field(default_factory=list)
    joints: List[URDFJoint] = field(default_factory=list)

    # Convenience lookups (populated after parsing)
    _link_map: dict = field(default_factory=dict, repr=False)
    _joint_map: dict = field(default_factory=dict, repr=False)

    def get_link(self, name: str) -> Optional[URDFLink]:
        return self._link_map.get(name)

    def get_joint(self, name: str) -> Optional[URDFJoint]:
        return self._joint_map.get(name)

    def get_root_link(self) -> Optional[URDFLink]:
        """Return the link with no parent joint (the tree root)."""
        child_links = {j.child_link for j in self.joints}
        for link in self.links:
            if link.name not in child_links:
                return link
        return self.links[0] if self.links else None

    def get_parent_joint(self, link_name: str) -> Optional[URDFJoint]:
        """Return the joint whose child_link is the given link."""
        for j in self.joints:
            if j.child_link == link_name:
                return j
        return None

    def get_child_joints(self, link_name: str) -> List[URDFJoint]:
        """Return all joints whose parent_link is the given link."""
        return [j for j in self.joints if j.parent_link == link_name]

    # ----- Factory -----

    @classmethod
    def from_file(cls, filepath: str) -> "URDFRobot":
        tree = ET.parse(filepath)
        return cls.from_element(tree.getroot())

    @classmethod
    def from_string(cls, xml_string: str) -> "URDFRobot":
        root = ET.fromstring(xml_string)
        return cls.from_element(root)

    @classmethod
    def from_element(cls, root: ET.Element) -> "URDFRobot":
        robot = cls(name=root.attrib.get("name", ""))
        for link_el in root.findall("link"):
            robot.links.append(_parse_link(link_el))
        for joint_el in root.findall("joint"):
            robot.joints.append(_parse_joint(joint_el))
        robot._link_map = {l.name: l for l in robot.links}
        robot._joint_map = {j.name: j for j in robot.joints}
        return robot

    def to_xml_string(self, indent: str = "  ") -> str:
        """Serialize back to URDF XML string."""
        return robot_to_xml_string(self, indent=indent)


# ===================================================================
# XML serialization (standalone — no UE dependency)
# ===================================================================

def robot_to_xml_string(robot: "URDFRobot", indent: str = "  ") -> str:
    """Serialize a URDFRobot to a pretty-printed URDF XML string."""
    from xml.dom import minidom
    from urdf.urdf_utils import fmt_vec3, fmt_float

    root = ET.Element("robot", name=robot.name)

    for link in robot.links:
        _write_link(root, link)
    for joint in robot.joints:
        _write_joint(root, joint)

    rough = ET.tostring(root, encoding="unicode")
    parsed = minidom.parseString(rough)
    pretty = parsed.toprettyxml(indent=indent)
    lines = pretty.split("\n")
    if lines and lines[0].startswith("<?xml"):
        lines[0] = '<?xml version="1.0" ?>'
    return "\n".join(lines)


def _write_origin(parent: ET.Element, origin: URDFOrigin):
    from urdf.urdf_utils import fmt_vec3
    if origin.xyz == (0, 0, 0) and origin.rpy == (0, 0, 0):
        return
    attrib = {}
    if origin.xyz != (0, 0, 0):
        attrib["xyz"] = fmt_vec3(origin.xyz)
    if origin.rpy != (0, 0, 0):
        attrib["rpy"] = fmt_vec3(origin.rpy)
    ET.SubElement(parent, "origin", attrib)


def _write_geometry(parent: ET.Element, geom: URDFGeometry):
    from urdf.urdf_utils import fmt_vec3, fmt_float
    geom_el = ET.SubElement(parent, "geometry")
    if geom.box:
        ET.SubElement(geom_el, "box", size=fmt_vec3(geom.box.size))
    elif geom.cylinder:
        ET.SubElement(geom_el, "cylinder",
                      radius=fmt_float(geom.cylinder.radius),
                      length=fmt_float(geom.cylinder.length))
    elif geom.sphere:
        ET.SubElement(geom_el, "sphere", radius=fmt_float(geom.sphere.radius))
    elif geom.mesh:
        attrib = {"filename": geom.mesh.filename}
        if geom.mesh.scale != (1, 1, 1):
            attrib["scale"] = fmt_vec3(geom.mesh.scale)
        ET.SubElement(geom_el, "mesh", **attrib)


def _write_link(parent: ET.Element, link: URDFLink):
    from urdf.urdf_utils import fmt_float
    link_el = ET.SubElement(parent, "link", name=link.name)

    if link.inertial:
        inertial_el = ET.SubElement(link_el, "inertial")
        _write_origin(inertial_el, link.inertial.origin)
        ET.SubElement(inertial_el, "mass", value=fmt_float(link.inertial.mass))
        i = link.inertial.inertia
        ET.SubElement(inertial_el, "inertia",
                      ixx=fmt_float(i.ixx), ixy=fmt_float(i.ixy), ixz=fmt_float(i.ixz),
                      iyy=fmt_float(i.iyy), iyz=fmt_float(i.iyz), izz=fmt_float(i.izz))

    for vis in link.visuals:
        vis_el = ET.SubElement(link_el, "visual")
        if vis.name:
            vis_el.set("name", vis.name)
        _write_origin(vis_el, vis.origin)
        if vis.geometry:
            _write_geometry(vis_el, vis.geometry)
        if vis.material:
            mat_el = ET.SubElement(vis_el, "material", name=vis.material.name)
            if vis.material.color_rgba:
                ET.SubElement(mat_el, "color",
                              rgba=f"{vis.material.color_rgba[0]} {vis.material.color_rgba[1]} "
                                   f"{vis.material.color_rgba[2]} {vis.material.color_rgba[3]}")

    for col in link.collisions:
        col_el = ET.SubElement(link_el, "collision")
        if col.name:
            col_el.set("name", col.name)
        _write_origin(col_el, col.origin)
        if col.geometry:
            _write_geometry(col_el, col.geometry)


def _write_joint(parent: ET.Element, joint: URDFJoint):
    from urdf.urdf_utils import fmt_vec3, fmt_float
    joint_el = ET.SubElement(parent, "joint", name=joint.name, type=joint.joint_type)
    _write_origin(joint_el, joint.origin)
    ET.SubElement(joint_el, "parent", link=joint.parent_link)
    ET.SubElement(joint_el, "child", link=joint.child_link)

    if joint.axis != (1, 0, 0):
        ET.SubElement(joint_el, "axis", xyz=fmt_vec3(joint.axis))

    if joint.limit:
        attrib = {
            "lower": fmt_float(joint.limit.lower),
            "upper": fmt_float(joint.limit.upper),
        }
        if joint.limit.effort > 0:
            attrib["effort"] = fmt_float(joint.limit.effort)
        if joint.limit.velocity > 0:
            attrib["velocity"] = fmt_float(joint.limit.velocity)
        ET.SubElement(joint_el, "limit", **attrib)

    if joint.dynamics:
        attrib = {}
        if joint.dynamics.damping > 0:
            attrib["damping"] = fmt_float(joint.dynamics.damping)
        if joint.dynamics.friction > 0:
            attrib["friction"] = fmt_float(joint.dynamics.friction)
        if attrib:
            ET.SubElement(joint_el, "dynamics", **attrib)

    if joint.mimic:
        attrib = {"joint": joint.mimic.joint}
        if joint.mimic.multiplier != 1.0:
            attrib["multiplier"] = fmt_float(joint.mimic.multiplier)
        if joint.mimic.offset != 0.0:
            attrib["offset"] = fmt_float(joint.mimic.offset)
        ET.SubElement(joint_el, "mimic", **attrib)


# ===================================================================
# Internal parsing helpers
# ===================================================================

def _parse_floats(s: str) -> Tuple[float, ...]:
    return tuple(float(x) for x in s.split())


def _parse_origin(el: Optional[ET.Element]) -> URDFOrigin:
    if el is None:
        return URDFOrigin()
    xyz = _parse_floats(el.attrib.get("xyz", "0 0 0"))
    rpy = _parse_floats(el.attrib.get("rpy", "0 0 0"))
    return URDFOrigin(xyz=xyz[:3], rpy=rpy[:3])


def _parse_geometry(el: Optional[ET.Element]) -> Optional[URDFGeometry]:
    if el is None:
        return None
    geom = URDFGeometry()
    box_el = el.find("box")
    if box_el is not None:
        geom.box = URDFBox(size=_parse_floats(box_el.attrib.get("size", "0 0 0"))[:3])
    cyl_el = el.find("cylinder")
    if cyl_el is not None:
        geom.cylinder = URDFCylinder(
            radius=float(cyl_el.attrib.get("radius", "0")),
            length=float(cyl_el.attrib.get("length", "0")))
    sph_el = el.find("sphere")
    if sph_el is not None:
        geom.sphere = URDFSphere(radius=float(sph_el.attrib.get("radius", "0")))
    mesh_el = el.find("mesh")
    if mesh_el is not None:
        scale = _parse_floats(mesh_el.attrib.get("scale", "1 1 1"))
        geom.mesh = URDFMesh(filename=mesh_el.attrib.get("filename", ""), scale=scale[:3])
    return geom


def _parse_inertial(el: Optional[ET.Element]) -> Optional[URDFInertial]:
    if el is None:
        return None
    inertial = URDFInertial()
    inertial.origin = _parse_origin(el.find("origin"))
    mass_el = el.find("mass")
    if mass_el is not None:
        inertial.mass = float(mass_el.attrib.get("value", "0"))
    inertia_el = el.find("inertia")
    if inertia_el is not None:
        inertial.inertia = URDFInertia(
            ixx=float(inertia_el.attrib.get("ixx", "0")),
            ixy=float(inertia_el.attrib.get("ixy", "0")),
            ixz=float(inertia_el.attrib.get("ixz", "0")),
            iyy=float(inertia_el.attrib.get("iyy", "0")),
            iyz=float(inertia_el.attrib.get("iyz", "0")),
            izz=float(inertia_el.attrib.get("izz", "0")))
    return inertial


def _parse_material(el: Optional[ET.Element]) -> Optional[URDFMaterial]:
    if el is None:
        return None
    mat = URDFMaterial(name=el.attrib.get("name", ""))
    color_el = el.find("color")
    if color_el is not None:
        rgba = _parse_floats(color_el.attrib.get("rgba", "1 1 1 1"))
        mat.color_rgba = rgba[:4]
    tex_el = el.find("texture")
    if tex_el is not None:
        mat.texture_filename = tex_el.attrib.get("filename", "")
    return mat


def _parse_visual(el: ET.Element) -> URDFVisual:
    vis = URDFVisual(name=el.attrib.get("name", ""))
    vis.origin = _parse_origin(el.find("origin"))
    vis.geometry = _parse_geometry(el.find("geometry"))
    vis.material = _parse_material(el.find("material"))
    return vis


def _parse_collision(el: ET.Element) -> URDFCollision:
    col = URDFCollision(name=el.attrib.get("name", ""))
    col.origin = _parse_origin(el.find("origin"))
    col.geometry = _parse_geometry(el.find("geometry"))
    return col


def _parse_link(el: ET.Element) -> URDFLink:
    link = URDFLink(name=el.attrib.get("name", ""))
    link.inertial = _parse_inertial(el.find("inertial"))
    for vis_el in el.findall("visual"):
        link.visuals.append(_parse_visual(vis_el))
    for col_el in el.findall("collision"):
        link.collisions.append(_parse_collision(col_el))
    return link


def _parse_joint(el: ET.Element) -> URDFJoint:
    joint = URDFJoint(
        name=el.attrib.get("name", ""),
        joint_type=el.attrib.get("type", "fixed"))
    parent_el = el.find("parent")
    if parent_el is not None:
        joint.parent_link = parent_el.attrib.get("link", "")
    child_el = el.find("child")
    if child_el is not None:
        joint.child_link = child_el.attrib.get("link", "")
    joint.origin = _parse_origin(el.find("origin"))
    axis_el = el.find("axis")
    if axis_el is not None:
        joint.axis = _parse_floats(axis_el.attrib.get("xyz", "1 0 0"))[:3]
    limit_el = el.find("limit")
    if limit_el is not None:
        joint.limit = URDFJointLimit(
            lower=float(limit_el.attrib.get("lower", "0")),
            upper=float(limit_el.attrib.get("upper", "0")),
            effort=float(limit_el.attrib.get("effort", "0")),
            velocity=float(limit_el.attrib.get("velocity", "0")))
    dynamics_el = el.find("dynamics")
    if dynamics_el is not None:
        joint.dynamics = URDFJointDynamics(
            damping=float(dynamics_el.attrib.get("damping", "0")),
            friction=float(dynamics_el.attrib.get("friction", "0")))
    safety_el = el.find("safety_controller")
    if safety_el is not None:
        joint.safety = URDFJointSafety(
            soft_lower_limit=float(safety_el.attrib.get("soft_lower_limit", "0")),
            soft_upper_limit=float(safety_el.attrib.get("soft_upper_limit", "0")),
            k_position=float(safety_el.attrib.get("k_position", "0")),
            k_velocity=float(safety_el.attrib.get("k_velocity", "0")))
    mimic_el = el.find("mimic")
    if mimic_el is not None:
        joint.mimic = URDFJointMimic(
            joint=mimic_el.attrib.get("joint", ""),
            multiplier=float(mimic_el.attrib.get("multiplier", "1")),
            offset=float(mimic_el.attrib.get("offset", "0")))
    return joint

"""
Unreal Engine Physics Asset → URDF exporter.

This module is auto-discovered by UE (lives in Plugin/Content/Python/).
Run from the UE Editor Python console:

    from urdf.urdf_exporter import export_urdf
    export_urdf("/RammsAssets/Robots/KinovaGen3/Arm/SkeletalMeshes/gen3_6dof_PhysicsAsset",
                "C:/output/kinova_gen3.urdf", "kinova_gen3")

Or use the one-click helper for the selected Content Browser asset:

    from urdf.urdf_editor_utils import export_selected_to_urdf
    export_selected_to_urdf()
"""

from __future__ import annotations

import math
import os
import tempfile
from typing import Dict, List, Optional, Tuple

try:
    import unreal
except ImportError:
    raise ImportError(
        "This script must be run inside the Unreal Editor Python environment."
    )

from urdf.urdf_parser import (
    URDFRobot, URDFLink, URDFJoint, URDFOrigin, URDFInertial, URDFInertia,
    URDFJointLimit, URDFJointDynamics, URDFGeometry, URDFBox, URDFCylinder,
    URDFSphere, URDFCollision, robot_to_xml_string,
)
from urdf.urdf_utils import (
    cm_to_m, deg_to_rad, ue_pos_to_urdf, ue_axis_to_urdf,
    ue_deg_to_urdf_rpy, ue_inertia_to_urdf,
    rotation_matrix_to_rpy,
    quat_conjugate, quat_multiply, quat_rotate_vector,
    quat_to_rotation_matrix, ue_quat_to_urdf_quat, ue_quat_to_urdf_rpy,
    rpy_to_quat,
    fmt_vec3, fmt_float,
)
from urdf.t3d_physics_parser import (
    parse_physics_asset_t3d, T3DPhysicsAsset, T3DBodySetup, T3DConstraint,
    T3DConvexElem,
)


class URDFExporter:
    """Export an Unreal Engine physics asset as a URDF file.

    Uses T3D asset export to read body/constraint data, since the UE 5.7
    Python API does not expose PhysicsAsset internals directly.
    """

    def __init__(self, verbose: bool = True):
        self.verbose = verbose

    def log(self, msg: str):
        if self.verbose:
            unreal.log(msg)

    def warn(self, msg: str):
        unreal.log_warning(msg)

    def export(
        self,
        physics_asset_path: str,
        output_path: str,
        robot_name: str = "robot",
        include_collision: bool = True,
        include_inertia: bool = True,
        mapping_file: Optional[str] = None,
        skeletal_mesh_path: Optional[str] = None,
    ) -> bool:
        """
        Export a physics asset to a URDF file.

        Args:
            physics_asset_path: UE content path to the physics asset
            output_path: File system path for the output .urdf file
            robot_name: Name for the <robot> element
            include_collision: Include collision geometry
            include_inertia: Include inertial properties
            mapping_file: Optional JSON with bone→link name overrides
            skeletal_mesh_path: Optional UE content path to the skeletal mesh.
                If None, auto-detected from the physics asset's preview mesh.

        Returns True on success.
        """
        self.log(f"=== URDF Export ===")
        self.log(f"Physics Asset: {physics_asset_path}")
        self.log(f"Output: {output_path}")

        phys_asset = unreal.load_asset(physics_asset_path)
        if phys_asset is None:
            self.warn(f"Could not load physics asset: {physics_asset_path}")
            return False

        # Export to T3D and parse
        pa_data = self._export_and_parse_t3d(phys_asset)
        if pa_data is None:
            return False

        self.log(f"Parsed T3D: {len(pa_data.body_setups)} bodies, {len(pa_data.constraints)} constraints")

        # Get bone reference pose for accurate joint origins
        ref_pose = self._get_ref_pose(phys_asset, skeletal_mesh_path)

        # Build bone world transforms dict (for world-aligned collision/axis)
        bone_world_transforms = {}
        if ref_pose is not None:
            bone_names = [body.bone_name for body in pa_data.body_setups]
            bone_world_transforms = self._get_bone_world_transforms(
                ref_pose, bone_names)

        # Load optional name mapping (bone -> URDF link name)
        bone_to_link = {}
        if mapping_file:
            try:
                import json
                with open(mapping_file, "r") as f:
                    data = json.load(f)
                bone_to_link = {v: k for k, v in data.get("links", {}).items()}
            except Exception as e:
                self.warn(f"Could not load mapping file: {e}")

        # Build URDF
        robot = URDFRobot(name=robot_name)

        # Build links from body setups
        for body in pa_data.body_setups:
            link_name = bone_to_link.get(body.bone_name, body.bone_name)
            link = URDFLink(name=link_name)

            if include_inertia and body.mass > 0:
                link.inertial = URDFInertial(mass=body.mass)

            if include_collision:
                bone_quat = None
                if body.bone_name in bone_world_transforms:
                    bone_quat = bone_world_transforms[body.bone_name][1]
                link.collisions = self._body_to_collisions(
                    body, link_name, bone_world_quat=bone_quat)

            robot.links.append(link)

        # Separate tree joints from loop-closure joints.
        # URDF requires a strict tree: each link has at most one parent.
        tree_constraints, loop_constraints = self._build_tree(
            pa_data.constraints, bone_to_link)

        for ct in tree_constraints:
            joint = self._constraint_to_joint(
                ct, bone_to_link, ref_pose, bone_world_transforms)
            if joint:
                robot.joints.append(joint)

        if loop_constraints:
            self.log(f"Excluded {len(loop_constraints)} loop-closure constraints "
                     f"(URDF requires a tree):")
            for ct in loop_constraints:
                child = bone_to_link.get(ct.constraint_bone1, ct.constraint_bone1)
                parent = bone_to_link.get(ct.constraint_bone2, ct.constraint_bone2)
                self.log(f"  {ct.joint_name}: {parent} → {child}")

        self.log(f"Built URDF: {len(robot.links)} links, {len(robot.joints)} joints")

        robot._link_map = {l.name: l for l in robot.links}
        robot._joint_map = {j.name: j for j in robot.joints}

        # Write XML
        xml_string = robot_to_xml_string(robot)
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        with open(output_path, "w") as f:
            f.write(xml_string)

        self.log(f"=== Export complete: {output_path} ===")
        return True

    # ===================================================================
    # T3D export
    # ===================================================================

    def _export_and_parse_t3d(self, phys_asset) -> Optional[T3DPhysicsAsset]:
        """Export the PhysicsAsset to T3D text and parse it."""
        try:
            tmp_path = os.path.join(tempfile.gettempdir(), "ramms_pa_export.t3d")
            task = unreal.AssetExportTask()
            task.set_editor_property("object", phys_asset)
            task.set_editor_property("filename", tmp_path)
            task.set_editor_property("automated", True)
            task.set_editor_property("prompt", False)
            task.set_editor_property("replace_identical", True)
            success = unreal.Exporter.run_asset_export_task(task)
            if not success or not os.path.exists(tmp_path):
                self.warn("T3D export failed")
                return None
            with open(tmp_path, "r") as f:
                t3d_text = f.read()
            os.remove(tmp_path)
            return parse_physics_asset_t3d(t3d_text)
        except Exception as e:
            self.warn(f"T3D export/parse failed: {e}")
            return None

    # ===================================================================
    # Skeleton reference pose
    # ===================================================================

    def _get_ref_pose(self, phys_asset, skeletal_mesh_path: Optional[str] = None):
        """Get the skeleton reference pose (AnimPose) for bone world transforms.

        Tries the explicit skeletal_mesh_path first, then auto-detects from
        the physics asset's preview_skeletal_mesh property, then falls back
        to path inference.  Returns None on failure.
        """
        skel_mesh = None
        if skeletal_mesh_path:
            skel_mesh = unreal.load_asset(skeletal_mesh_path)
            if skel_mesh is None:
                self.warn(f"Could not load skeletal mesh: {skeletal_mesh_path}")

        if skel_mesh is None:
            for prop in ("preview_skeletal_mesh", "skeletal_mesh"):
                try:
                    skel_mesh = phys_asset.get_editor_property(prop)
                    if skel_mesh is not None:
                        break
                except Exception:
                    pass

        if skel_mesh is None:
            # Fallback: infer mesh path from physics asset path
            mesh_path = phys_asset.get_path_name().split(".")[0].replace(
                "_PhysicsAsset", "")
            skel_mesh = unreal.load_asset(mesh_path)

        if skel_mesh is None:
            self.warn("Could not find skeletal mesh — using constraint data for origins")
            return None

        try:
            skeleton = skel_mesh.get_editor_property("skeleton")
            ref_pose = skeleton.get_reference_pose()
            if ref_pose.is_valid():
                bone_names = [str(b) for b in ref_pose.get_bone_names()]
                self.log(f"Using skeleton reference pose ({len(bone_names)} bones)")
                return ref_pose
        except Exception as e:
            self.warn(f"Could not get reference pose: {e}")
        return None

    def _bone_origin_from_ref_pose(self, ref_pose, child_bone: str,
                                   parent_bone: str) -> Tuple[Tuple[float, ...], Tuple[float, ...]]:
        """Compute URDF joint origin (xyz, rpy) from bone WORLD transforms.

        Uses world-aligned link frames: joint origin is a simple world-space
        position offset converted to URDF coordinates, with rpy=(0,0,0).

        Returns ((x, y, z), (roll, pitch, yaw)).
        """
        t_parent = ref_pose.get_bone_pose(parent_bone, unreal.AnimPoseSpaces.WORLD)
        t_child = ref_pose.get_bone_pose(child_bone, unreal.AnimPoseSpaces.WORLD)

        # Positions in cm (UE world/component space)
        pp = (t_parent.translation.x, t_parent.translation.y, t_parent.translation.z)
        cp = (t_child.translation.x, t_child.translation.y, t_child.translation.z)

        # World-aligned delta (child - parent) in UE coords, then convert
        delta = (cp[0] - pp[0], cp[1] - pp[1], cp[2] - pp[2])
        urdf_pos = ue_pos_to_urdf(delta)

        # Suppress near-zero values
        eps = 1e-10
        urdf_pos = tuple(0.0 if abs(v) < eps else v for v in urdf_pos)

        # World-aligned frames: rpy is always zero (rotation handled by
        # collision shapes and joint axes individually)
        return urdf_pos, (0.0, 0.0, 0.0)

    def _get_bone_world_transforms(self, ref_pose, bone_names):
        """Build a dict of bone world transforms from the reference pose.

        Returns {bone_name: ((x, y, z), (w, x, y, z))} where position is
        in UE cm and quaternion is UE convention (w, x, y, z).
        """
        result = {}
        for name in bone_names:
            try:
                t = ref_pose.get_bone_pose(name, unreal.AnimPoseSpaces.WORLD)
                pos = (t.translation.x, t.translation.y, t.translation.z)
                quat = (t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z)
                result[name] = (pos, quat)
            except Exception:
                pass
        return result

    # ===================================================================
    # Tree building (cycle detection)
    # ===================================================================

    def _build_tree(
        self,
        constraints: List[T3DConstraint],
        bone_to_link: Dict[str, str],
    ) -> Tuple[List[T3DConstraint], List[T3DConstraint]]:
        """Separate constraints into tree edges and loop-closure edges.

        URDF requires a strict tree: each link may have at most one parent
        joint.  Physics assets often include "UserConstraint" entries that
        form closed kinematic loops (e.g. 4-bar linkages).

        Returns (tree_constraints, loop_constraints).
        """
        assigned_children: Dict[str, T3DConstraint] = {}
        tree: List[T3DConstraint] = []
        loops: List[T3DConstraint] = []

        for ct in constraints:
            child = bone_to_link.get(ct.constraint_bone1, ct.constraint_bone1)
            if child in assigned_children:
                loops.append(ct)
            else:
                assigned_children[child] = ct
                tree.append(ct)

        # Verify the tree is connected by BFS from root.
        # The root is any link that never appears as a child.
        link_names = set()
        for ct in tree:
            link_names.add(bone_to_link.get(ct.constraint_bone1, ct.constraint_bone1))
            link_names.add(bone_to_link.get(ct.constraint_bone2, ct.constraint_bone2))

        child_set = set(assigned_children.keys())
        roots = link_names - child_set
        if len(roots) != 1:
            self.warn(f"Expected 1 root link, found {len(roots)}: {roots}")

        return tree, loops

    # ===================================================================
    # Conversion helpers
    # ===================================================================

    def _bone_world_rpy(self, bone_quat_ue):
        """Compute the URDF collision RPY for a bone's world rotation.

        When bone_quat_ue is None or identity, returns None (no RPY needed).
        """
        if bone_quat_ue is None:
            return None
        # Check for identity quaternion (w=+/-1, xyz~0)
        if abs(abs(bone_quat_ue[0]) - 1.0) < 1e-6:
            return None
        rpy = ue_quat_to_urdf_rpy(bone_quat_ue)
        eps = 1e-10
        rpy = tuple(0.0 if abs(v) < eps else v for v in rpy)
        if all(abs(v) < eps for v in rpy):
            return None
        return rpy

    def _apply_bone_rotation_to_collision(self, center_ue, shape_rpy, bone_quat_ue):
        """Rotate a collision shape's center and RPY into world frame.

        With world-aligned link frames, collision shapes must carry the
        bone's world rotation so they appear at the correct position and
        orientation within the world-aligned link frame.

        Args:
            center_ue: shape center in bone-local UE space (cm)
            shape_rpy: shape-local URDF RPY (e.g. cylinder axis alignment), or None
            bone_quat_ue: bone's world rotation as UE quaternion (w,x,y,z), or None

        Returns (urdf_xyz, urdf_rpy).
        """
        if bone_quat_ue is None:
            return ue_pos_to_urdf(center_ue), shape_rpy or (0, 0, 0)

        # Rotate center from bone-local to world frame (UE coords)
        center_world = quat_rotate_vector(bone_quat_ue, center_ue)
        urdf_pos = ue_pos_to_urdf(center_world)

        # Compose bone world rotation with shape-local rotation (URDF space)
        bone_urdf_q = ue_quat_to_urdf_quat(bone_quat_ue)
        if shape_rpy and any(abs(v) > 1e-10 for v in shape_rpy):
            shape_q = rpy_to_quat(*shape_rpy)
            total_q = quat_multiply(bone_urdf_q, shape_q)
        else:
            total_q = bone_urdf_q

        total_rpy = rotation_matrix_to_rpy(*quat_to_rotation_matrix(total_q))
        eps = 1e-10
        total_rpy = tuple(0.0 if abs(v) < eps else v for v in total_rpy)

        # Return None for identity RPY (cleaner XML output)
        if all(abs(v) < eps for v in total_rpy):
            total_rpy = None

        return urdf_pos, total_rpy

    def _body_to_collisions(self, body: T3DBodySetup, link_name: str,
                            bone_world_quat=None) -> List[URDFCollision]:
        """Convert T3D body collision shapes to URDF collisions.

        When bone_world_quat is provided (world-aligned link frames), shape
        centers are rotated from bone-local to world frame and the bone's
        rotation is included in each collision origin RPY.
        """
        collisions = []

        for box in body.boxes:
            center_ue = (box.center.x, box.center.y, box.center.z)
            xyz, rpy = self._apply_bone_rotation_to_collision(
                center_ue, None, bone_world_quat)
            col = URDFCollision(
                origin=URDFOrigin(xyz=xyz, rpy=rpy),
                geometry=URDFGeometry(box=URDFBox(
                    size=(cm_to_m(box.x), cm_to_m(box.y), cm_to_m(box.z)))))
            collisions.append(col)

        for sph in body.spheres:
            center_ue = (sph.center.x, sph.center.y, sph.center.z)
            xyz, _ = self._apply_bone_rotation_to_collision(
                center_ue, None, bone_world_quat)
            col = URDFCollision(
                origin=URDFOrigin(xyz=xyz),
                geometry=URDFGeometry(sphere=URDFSphere(
                    radius=cm_to_m(sph.radius))))
            collisions.append(col)

        for cap in body.capsules:
            center_ue = (cap.center.x, cap.center.y, cap.center.z)
            xyz, rpy = self._apply_bone_rotation_to_collision(
                center_ue, None, bone_world_quat)
            col = URDFCollision(
                origin=URDFOrigin(xyz=xyz, rpy=rpy),
                geometry=URDFGeometry(cylinder=URDFCylinder(
                    radius=cm_to_m(cap.radius), length=cm_to_m(cap.length))))
            collisions.append(col)

        for convex in body.convex_elems:
            col = self._convex_to_collision(convex, link_name, bone_world_quat)
            if col:
                collisions.append(col)

        return collisions

    def _convex_to_collision(self, convex: T3DConvexElem,
                             link_name: str,
                             bone_world_quat=None) -> Optional[URDFCollision]:
        """Convert a convex hull element to a URDF collision.

        Attempts to detect cylindrical shapes (common for wheels). Falls back
        to a bounding-box approximation for non-cylindrical convex hulls.
        """
        bb_min = convex.elem_box_min
        bb_max = convex.elem_box_max

        # Use vertices for tighter bounds if available
        if convex.vertices:
            bb_min_x = min(v.x for v in convex.vertices)
            bb_min_y = min(v.y for v in convex.vertices)
            bb_min_z = min(v.z for v in convex.vertices)
            bb_max_x = max(v.x for v in convex.vertices)
            bb_max_y = max(v.y for v in convex.vertices)
            bb_max_z = max(v.z for v in convex.vertices)
        else:
            bb_min_x, bb_min_y, bb_min_z = bb_min.x, bb_min.y, bb_min.z
            bb_max_x, bb_max_y, bb_max_z = bb_max.x, bb_max.y, bb_max.z

        extents = (bb_max_x - bb_min_x, bb_max_y - bb_min_y, bb_max_z - bb_min_z)
        if all(e < 0.001 for e in extents):
            return None  # degenerate

        center_ue = ((bb_min_x + bb_max_x) / 2.0,
                     (bb_min_y + bb_max_y) / 2.0,
                     (bb_min_z + bb_max_z) / 2.0)

        # Detect cylinder: two approximately equal bounding-box extents = diameter
        sorted_ext = sorted(zip(range(3), extents), key=lambda x: x[1])
        idx0, e0 = sorted_ext[0]
        idx1, e1 = sorted_ext[1]
        idx2, e2 = sorted_ext[2]

        is_cylinder = False
        if e1 > 0.01:
            if abs(e1 - e2) / max(e1, 1e-6) < 0.15:
                # Two largest extents are similar → flat disc/wheel
                radius = cm_to_m((e1 + e2) / 4.0)
                length = cm_to_m(e0)
                axis_idx = idx0
                is_cylinder = True
            elif abs(e0 - e1) / max(e1, 1e-6) < 0.15:
                # Two smallest extents are similar → tall cylinder
                radius = cm_to_m((e0 + e1) / 4.0)
                length = cm_to_m(e2)
                axis_idx = idx2
                is_cylinder = True

        origin_pos = ue_pos_to_urdf(center_ue)

        if is_cylinder:
            # URDF cylinder is along Z; rotate if UE axis differs
            shape_rpy = (0.0, 0.0, 0.0)
            if axis_idx == 0:   # UE X -> pitch 90 to align with Z
                shape_rpy = (0.0, math.pi / 2.0, 0.0)
            elif axis_idx == 1: # UE Y -> roll 90 to align with Z
                shape_rpy = (math.pi / 2.0, 0.0, 0.0)

            # Apply bone world rotation if using world-aligned frames
            xyz, rpy = self._apply_bone_rotation_to_collision(
                center_ue, shape_rpy, bone_world_quat)

            self.log(f"    Convex->cylinder ({link_name}): "
                     f"r={radius:.4f}m l={length:.4f}m axis={'XYZ'[axis_idx]}")
            return URDFCollision(
                origin=URDFOrigin(xyz=xyz, rpy=rpy),
                geometry=URDFGeometry(cylinder=URDFCylinder(
                    radius=radius, length=length)))

        # Fallback: bounding box
        size = (cm_to_m(extents[0]), cm_to_m(extents[1]), cm_to_m(extents[2]))
        self.log(f"    Convex->box ({link_name}): "
                 f"{size[0]:.4f} x {size[1]:.4f} x {size[2]:.4f}m")
        xyz, rpy = self._apply_bone_rotation_to_collision(
            center_ue, None, bone_world_quat)
        return URDFCollision(
            origin=URDFOrigin(xyz=xyz, rpy=rpy),
            geometry=URDFGeometry(box=URDFBox(size=size)))

    def _constraint_to_joint(self, ct: T3DConstraint,
                             bone_to_link: Dict[str, str],
                             ref_pose=None,
                             bone_world_transforms=None) -> Optional[URDFJoint]:
        """Convert a T3D constraint to a URDF joint.

        When ref_pose is available, joint origins use world-aligned bone
        reference transforms (position offsets in URDF world coordinates
        with zero rotation).  Joint axes are rotated to world frame using
        bone_world_transforms.  Falls back to constraint Pos2/frame data
        when ref_pose is unavailable.
        """
        child_link = bone_to_link.get(ct.constraint_bone1, ct.constraint_bone1)
        parent_link = bone_to_link.get(ct.constraint_bone2, ct.constraint_bone2)

        # Classify joint type using motion enum strings
        from urdf.urdf_utils import classify_ue_constraint
        joint_type, active_axes = classify_ue_constraint(
            ct.twist_motion, ct.swing1_motion, ct.swing2_motion,
            ct.linear_x_motion, ct.linear_y_motion, ct.linear_z_motion)

        joint = URDFJoint(
            name=ct.joint_name or f"{parent_link}_to_{child_link}",
            joint_type=joint_type,
            parent_link=parent_link,
            child_link=child_link,
        )

        # Joint origin: prefer bone reference transforms, fall back to constraint data
        if ref_pose is not None:
            try:
                pos_urdf, rpy_urdf = self._bone_origin_from_ref_pose(
                    ref_pose, ct.constraint_bone1, ct.constraint_bone2)
                joint.origin = URDFOrigin(xyz=pos_urdf, rpy=rpy_urdf)
            except Exception as e:
                self.warn(f"  Bone transform failed for {ct.joint_name}, "
                          f"using constraint data: {e}")
                pos_urdf = ue_pos_to_urdf((ct.pos2.x, ct.pos2.y, ct.pos2.z))
                rpy_urdf = self._constraint_frame_rpy(ct)
                joint.origin = URDFOrigin(xyz=pos_urdf, rpy=rpy_urdf)
        else:
            pos_urdf = ue_pos_to_urdf((ct.pos2.x, ct.pos2.y, ct.pos2.z))
            rpy_urdf = self._constraint_frame_rpy(ct)
            joint.origin = URDFOrigin(xyz=pos_urdf, rpy=rpy_urdf)

        # Joint limits (only for revolute / prismatic)
        if joint_type == "revolute":
            limit_deg = 45.0
            if "swing1" in active_axes:
                limit_deg = ct.swing1_limit_deg
            elif "swing2" in active_axes:
                limit_deg = ct.swing2_limit_deg
            elif "twist" in active_axes:
                limit_deg = ct.twist_limit_deg
            half_rad = deg_to_rad(limit_deg)
            joint.limit = URDFJointLimit(
                lower=-half_rad, upper=half_rad,
                effort=100.0, velocity=3.14)
        elif joint_type == "prismatic":
            if ct.linear_limit_size > 0:
                half_m = cm_to_m(ct.linear_limit_size)
                joint.limit = URDFJointLimit(
                    lower=-half_m, upper=half_m,
                    effort=100.0, velocity=1.0)

        # Joint axis in world frame (rotated from child bone's constraint frame)
        if joint_type in ("revolute", "continuous"):
            child_quat = None
            if bone_world_transforms and ct.constraint_bone1 in bone_world_transforms:
                child_quat = bone_world_transforms[ct.constraint_bone1][1]
            joint.axis = self._constraint_axis_child(ct, active_axes, child_quat)

        # Dynamics (damping from drive settings)
        damping = 0.0
        if ct.twist_drive_damping > 0:
            damping = ct.twist_drive_damping
        elif ct.swing_drive_damping > 0:
            damping = ct.swing_drive_damping
        if damping > 0:
            joint.dynamics = URDFJointDynamics(damping=damping / 100.0)

        self.log(f"  Joint: {joint.name} ({joint.joint_type}) "
                 f"{parent_link} → {child_link}")
        return joint

    def _constraint_frame_rpy(self, ct: T3DConstraint) -> Tuple[float, float, float]:
        """Compute the URDF RPY rotation from the parent-side constraint frame.

        The constraint frame axes (PriAxis2, SecAxis2) in parent space define
        the child bone's orientation relative to the parent.
        """
        # Convert constraint frame axes from UE to URDF coordinates
        pri = ue_axis_to_urdf((ct.pri_axis2.x, ct.pri_axis2.y, ct.pri_axis2.z))
        sec = ue_axis_to_urdf((ct.sec_axis2.x, ct.sec_axis2.y, ct.sec_axis2.z))
        # Z = X × Y (right-handed cross product)
        z = (pri[1]*sec[2] - pri[2]*sec[1],
             pri[2]*sec[0] - pri[0]*sec[2],
             pri[0]*sec[1] - pri[1]*sec[0])

        rpy = rotation_matrix_to_rpy(pri, sec, z)

        # Suppress near-zero values
        eps = 1e-10
        rpy = tuple(0.0 if abs(v) < eps else v for v in rpy)
        return rpy

    def _constraint_axis_child(self, ct: T3DConstraint,
                               active_axes: List[str],
                               child_bone_world_quat=None) -> Tuple[float, float, float]:
        """Compute the URDF joint axis from the child-side constraint frame.

        PriAxis1/SecAxis1 define the constraint axes in the child bone's local
        frame.  With world-aligned link frames, the axis is rotated from bone-
        local to UE world space before converting to URDF coordinates.
        """
        p = (ct.pri_axis1.x, ct.pri_axis1.y, ct.pri_axis1.z)
        s = (ct.sec_axis1.x, ct.sec_axis1.y, ct.sec_axis1.z)

        if "twist" in active_axes:
            ax = p  # X of constraint frame
        elif "swing2" in active_axes:
            ax = s  # Y of constraint frame
        else:
            # swing1 = Z of constraint frame = PriAxis1 x SecAxis1
            ax = (p[1]*s[2] - p[2]*s[1],
                  p[2]*s[0] - p[0]*s[2],
                  p[0]*s[1] - p[1]*s[0])

        # Normalize
        mag = math.sqrt(ax[0]**2 + ax[1]**2 + ax[2]**2)
        if mag > 1e-6:
            ax = (ax[0]/mag, ax[1]/mag, ax[2]/mag)
        else:
            ax = (0.0, 0.0, 1.0)

        # Rotate from bone-local to world frame (for world-aligned links)
        if child_bone_world_quat is not None:
            ax = quat_rotate_vector(child_bone_world_quat, ax)

        return ue_axis_to_urdf(ax)


# ===================================================================
# Convenience function
# ===================================================================

def export_urdf(
    physics_asset_path: str,
    output_path: str,
    robot_name: str = "robot",
    include_collision: bool = True,
    include_inertia: bool = True,
    mapping_file: Optional[str] = None,
    skeletal_mesh_path: Optional[str] = None,
    verbose: bool = True,
) -> bool:
    """
    One-line convenience function for URDF export.

    Usage from UE Python console:
        from urdf.urdf_exporter import export_urdf
        export_urdf("/RammsAssets/Robots/KinovaGen3/Arm/SkeletalMeshes/gen3_6dof_PhysicsAsset",
                     "C:/output/kinova.urdf", "kinova_gen3")
    """
    exporter = URDFExporter(verbose=verbose)
    return exporter.export(
        physics_asset_path, output_path, robot_name,
        include_collision, include_inertia, mapping_file,
        skeletal_mesh_path)

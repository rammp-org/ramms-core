"""
URDF → Unreal Engine Physics Asset importer.

This module is auto-discovered by UE (lives in Plugin/Content/Python/).
Run from the UE Editor Python console:

    from urdf.urdf_importer import import_urdf

    # Mode 1: Configure existing physics asset from URDF
    import_urdf("C:/path/to/robot.urdf",
                "/Game/kinova/SkeletalMeshes/gen3_6dof_PhysicsAsset")

    # Mode 2: Full import with collision geometry
    import_urdf("C:/path/to/robot.urdf",
                "/Game/kinova/SkeletalMeshes/gen3_6dof_PhysicsAsset",
                mode="full")
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

try:
    import unreal
except ImportError:
    raise ImportError(
        "This script must be run inside the Unreal Editor Python environment. "
        "Open the Output Log (Window > Developer Tools > Output Log), "
        "switch to Python, and run your script there."
    )

from urdf.urdf_parser import URDFRobot, URDFJoint, URDFLink, URDFJointLimit
from urdf.urdf_utils import (
    m_to_cm, rad_to_deg, urdf_pos_to_ue, urdf_axis_to_ue,
    urdf_rpy_to_ue_deg, urdf_inertia_to_ue,
)
from urdf.name_mapping import NameMapping


class URDFImporter:
    """Import URDF data into an Unreal Engine physics asset."""

    def __init__(self, verbose: bool = True):
        self.verbose = verbose

    def log(self, msg: str):
        if self.verbose:
            unreal.log(msg)

    def warn(self, msg: str):
        unreal.log_warning(msg)

    @staticmethod
    def _find_skeletal_mesh(phys_asset, physics_asset_path: str):
        """Try multiple approaches to find the skeletal mesh for a physics asset."""
        for prop in ("preview_skeletal_mesh", "skeletal_mesh"):
            try:
                mesh = phys_asset.get_editor_property(prop)
                if mesh is not None:
                    return mesh
            except Exception:
                pass
        mesh_path = physics_asset_path.replace("_PhysicsAsset", "")
        return unreal.load_asset(mesh_path)

    # ===================================================================
    # Mode 1: Apply URDF physics properties to an existing physics asset
    # ===================================================================

    def import_to_existing(
        self,
        urdf_path: str,
        physics_asset_path: str,
        override_mapping_path: Optional[str] = None,
        apply_masses: bool = True,
        apply_joint_limits: bool = True,
        apply_inertia: bool = True,
        apply_joint_dynamics: bool = True,
    ) -> bool:
        """
        Apply URDF joint limits, masses, and inertia to an existing physics asset.
        Does not modify collision shapes.

        Returns True on success.
        """
        self.log(f"=== URDF Import (physics-only) ===")
        self.log(f"URDF: {urdf_path}")
        self.log(f"Physics Asset: {physics_asset_path}")

        robot = URDFRobot.from_file(urdf_path)
        self.log(f"Parsed URDF: '{robot.name}' with {len(robot.links)} links, {len(robot.joints)} joints")

        # Load physics asset
        phys_asset = unreal.load_asset(physics_asset_path)
        if phys_asset is None:
            self.warn(f"Could not load physics asset: {physics_asset_path}")
            return False

        # Get the skeletal mesh to read bone names
        skel_mesh = self._find_skeletal_mesh(phys_asset, physics_asset_path)

        bone_names = self._get_bone_names(phys_asset, skel_mesh)
        if not bone_names:
            self.warn("Could not retrieve bone names from physics asset or skeletal mesh.")
            return False

        self.log(f"Found {len(bone_names)} bones: {bone_names[:10]}{'...' if len(bone_names) > 10 else ''}")

        # Build name mapping
        urdf_link_names = [link.name for link in robot.links]
        mapping = NameMapping.auto_match(
            urdf_link_names, bone_names,
            override_file=override_mapping_path,
        )
        self.log(mapping.summary())

        # Begin edit transaction
        with unreal.ScopedEditorTransaction("URDF Import Physics Properties"):
            stats = {"masses": 0, "limits": 0, "inertia": 0, "dynamics": 0}

            # Apply body properties (mass, inertia)
            if apply_masses or apply_inertia:
                self._apply_body_properties(phys_asset, robot, mapping, apply_masses, apply_inertia, stats)

            # Apply joint limits and dynamics
            if apply_joint_limits or apply_joint_dynamics:
                self._apply_constraint_properties(phys_asset, robot, mapping, apply_joint_limits, apply_joint_dynamics, stats)

        self.log(f"=== Import complete: {stats['masses']} masses, {stats['inertia']} inertias, "
                 f"{stats['limits']} joint limits, {stats['dynamics']} dynamics updated ===")

        # Mark dirty for save
        unreal.EditorAssetLibrary.save_loaded_asset(phys_asset)
        return True

    # ===================================================================
    # Mode 2: Full import with collision body creation
    # ===================================================================

    def import_full(
        self,
        urdf_path: str,
        physics_asset_path: str,
        override_mapping_path: Optional[str] = None,
        use_collision_geometry: bool = True,
    ) -> bool:
        """
        Full import: apply physics properties AND create/replace collision bodies
        from URDF geometry definitions.

        If use_collision_geometry is True, uses <collision> geometry.
        If False, uses <visual> geometry (useful when collision is missing).

        Returns True on success.
        """
        self.log(f"=== URDF Full Import (physics + geometry) ===")
        self.log(f"URDF: {urdf_path}")
        self.log(f"Physics Asset: {physics_asset_path}")

        robot = URDFRobot.from_file(urdf_path)
        self.log(f"Parsed URDF: '{robot.name}' with {len(robot.links)} links, {len(robot.joints)} joints")

        phys_asset = unreal.load_asset(physics_asset_path)
        if phys_asset is None:
            self.warn(f"Could not load physics asset: {physics_asset_path}")
            return False

        skel_mesh = self._find_skeletal_mesh(phys_asset, physics_asset_path)

        bone_names = self._get_bone_names(phys_asset, skel_mesh)
        if not bone_names:
            self.warn("Could not retrieve bone names.")
            return False

        urdf_link_names = [link.name for link in robot.links]
        mapping = NameMapping.auto_match(
            urdf_link_names, bone_names,
            override_file=override_mapping_path,
        )
        self.log(mapping.summary())

        with unreal.ScopedEditorTransaction("URDF Full Import"):
            stats = {"masses": 0, "limits": 0, "inertia": 0, "dynamics": 0, "bodies": 0}

            # Create/update collision bodies from URDF geometry
            self._apply_collision_geometry(phys_asset, robot, mapping, use_collision_geometry, stats)

            # Apply physics properties
            self._apply_body_properties(phys_asset, robot, mapping, True, True, stats)
            self._apply_constraint_properties(phys_asset, robot, mapping, True, True, stats)

        self.log(f"=== Full import complete: {stats['bodies']} bodies, {stats['masses']} masses, "
                 f"{stats['inertia']} inertias, {stats['limits']} limits, "
                 f"{stats['dynamics']} dynamics ===")

        unreal.EditorAssetLibrary.save_loaded_asset(phys_asset)
        return True

    # ===================================================================
    # Internal helpers
    # ===================================================================

    def _get_bone_names(self, phys_asset, skel_mesh) -> List[str]:
        """Get bone names from the physics asset's bodies or the skeletal mesh."""
        names = []
        # Try physics asset bodies first
        try:
            body_setups = phys_asset.get_editor_property("skeletal_body_setups")
            if body_setups:
                for body in body_setups:
                    name = str(body.get_editor_property("bone_name"))
                    if name:
                        names.append(name)
        except Exception:
            pass

        if names:
            return names

        # Fall back to skeletal mesh
        if skel_mesh is not None:
            try:
                skeleton = skel_mesh.get_editor_property("skeleton")
                if skeleton:
                    ref_skeleton = skeleton.get_editor_property("reference_skeleton")
                    # Use the mesh directly
                    num_bones = unreal.SkeletalMeshLibrary.get_num_bones(skel_mesh)
                    for i in range(num_bones):
                        name = unreal.SkeletalMeshLibrary.get_bone_name(skel_mesh, i)
                        names.append(str(name))
            except Exception:
                pass

        return names

    def _find_body_setup(self, phys_asset, bone_name: str):
        """Find the BodySetup for a given bone name in the physics asset."""
        try:
            body_setups = phys_asset.get_editor_property("skeletal_body_setups")
            for body in body_setups:
                if str(body.get_editor_property("bone_name")) == bone_name:
                    return body
        except Exception:
            pass
        return None

    def _find_constraint_setup(self, phys_asset, bone_name: str):
        """Find the ConstraintSetup for a given bone (child bone of the constraint)."""
        try:
            constraints = phys_asset.get_editor_property("constraint_setup")
            for cs in constraints:
                profile = cs.get_editor_property("default_instance")
                child_bone = str(profile.get_editor_property("constraint_bone1"))
                if child_bone == bone_name:
                    return cs, profile
        except Exception:
            pass
        return None, None

    def _apply_body_properties(self, phys_asset, robot: URDFRobot, mapping: NameMapping,
                               apply_masses: bool, apply_inertia: bool, stats: dict):
        """Apply URDF link inertial properties to physics asset bodies."""
        for link in robot.links:
            bone_name = mapping.get_bone(link.name)
            if bone_name is None:
                continue

            if link.inertial is None:
                continue

            body = self._find_body_setup(phys_asset, bone_name)
            if body is None:
                self.log(f"  No body setup for bone '{bone_name}' (URDF link '{link.name}') — skipping")
                continue

            if apply_masses and link.inertial.mass > 0:
                # UE stores mass override on the body instance
                # In UE5 physics assets, mass is typically auto-computed from volume,
                # but we can set a mass override
                try:
                    body.set_editor_property("default_instance_mass_override", True)
                    # Mass is in kg in both URDF and UE
                    body.set_editor_property("calculated_mass", link.inertial.mass)
                    stats["masses"] += 1
                    self.log(f"  Mass: {link.name} → {bone_name}: {link.inertial.mass:.4f} kg")
                except Exception as e:
                    # Different UE versions expose mass differently
                    self.log(f"  Mass override for '{bone_name}': trying alternative method")
                    try:
                        phys_type = body.get_editor_property("phys_material")
                        body.set_editor_property("mass_in_kg", link.inertial.mass)
                        stats["masses"] += 1
                    except Exception:
                        self.warn(f"  Could not set mass for '{bone_name}': {e}")

            if apply_inertia:
                inertia = link.inertial.inertia
                ue_inertia = urdf_inertia_to_ue(
                    inertia.ixx, inertia.ixy, inertia.ixz,
                    inertia.iyy, inertia.iyz, inertia.izz
                )
                try:
                    body.set_editor_property("inertia_tensor", unreal.Vector(ue_inertia[0], ue_inertia[3], ue_inertia[5]))
                    stats["inertia"] += 1
                    self.log(f"  Inertia: {link.name} → {bone_name}: "
                             f"diag=({ue_inertia[0]:.2f}, {ue_inertia[3]:.2f}, {ue_inertia[5]:.2f}) kg·cm²")
                except Exception as e:
                    self.warn(f"  Could not set inertia for '{bone_name}': {e}")

    def _apply_constraint_properties(self, phys_asset, robot: URDFRobot, mapping: NameMapping,
                                     apply_limits: bool, apply_dynamics: bool, stats: dict):
        """Apply URDF joint limits and dynamics to physics asset constraints."""
        for joint in robot.joints:
            if joint.joint_type == "fixed":
                continue

            child_bone = mapping.get_bone(joint.child_link)
            if child_bone is None:
                continue

            cs, profile = self._find_constraint_setup(phys_asset, child_bone)
            if profile is None:
                self.log(f"  No constraint for bone '{child_bone}' (URDF joint '{joint.name}') — skipping")
                continue

            if apply_limits and joint.limit:
                self._apply_joint_limits(profile, joint, stats)

            if apply_dynamics and joint.dynamics:
                self._apply_joint_dynamics(profile, joint, stats)

    def _apply_joint_limits(self, profile, joint: URDFJoint, stats: dict):
        """Apply URDF joint limits to a UE constraint profile."""
        if joint.limit is None:
            return

        try:
            if joint.joint_type in ("revolute", "continuous"):
                lower_deg = rad_to_deg(joint.limit.lower)
                upper_deg = rad_to_deg(joint.limit.upper)

                if joint.joint_type == "continuous":
                    # No limits — set motion type to Free
                    profile.set_editor_property("angular_twist_motion", unreal.AngularConstraintMotion.ACM_FREE)
                else:
                    # Calculate symmetric swing/twist limits from asymmetric URDF limits
                    range_deg = upper_deg - lower_deg
                    half_range = range_deg / 2.0

                    # Determine which angular DOF this joint uses based on axis
                    # For simplicity, apply to twist (most common for revolute)
                    profile.set_editor_property("angular_twist_motion", unreal.AngularConstraintMotion.ACM_LIMITED)
                    profile.set_editor_property("twist_limit_angle", abs(half_range))

                    self.log(f"  Joint limit: {joint.name} → twist ±{abs(half_range):.1f}° "
                             f"(URDF: [{lower_deg:.1f}°, {upper_deg:.1f}°])")

                # Velocity limit (URDF velocity is rad/s)
                if joint.limit.velocity > 0:
                    vel_deg = rad_to_deg(joint.limit.velocity)
                    self.log(f"  Velocity limit: {joint.name} → {vel_deg:.1f} deg/s")

                stats["limits"] += 1

            elif joint.joint_type == "prismatic":
                lower_cm = m_to_cm(joint.limit.lower)
                upper_cm = m_to_cm(joint.limit.upper)
                range_cm = upper_cm - lower_cm

                profile.set_editor_property("linear_x_motion", unreal.LinearConstraintMotion.LCM_LIMITED)
                profile.set_editor_property("linear_limit_size", range_cm / 2.0)

                self.log(f"  Prismatic limit: {joint.name} → ±{range_cm / 2.0:.2f} cm "
                         f"(URDF: [{lower_cm:.2f}, {upper_cm:.2f}] cm)")
                stats["limits"] += 1

        except Exception as e:
            self.warn(f"  Could not apply limits for joint '{joint.name}': {e}")

    def _apply_joint_dynamics(self, profile, joint: URDFJoint, stats: dict):
        """Apply URDF joint dynamics (damping, friction) to a UE constraint."""
        if joint.dynamics is None:
            return

        try:
            if joint.dynamics.damping > 0:
                # URDF damping is in N·m·s/rad — map to UE angular damping
                # UE twist damping is a stiffness-like value
                profile.set_editor_property("angular_drive_mode", unreal.AngularDriveMode.TWIST_AND_SWING)
                twist_drive = profile.get_editor_property("angular_drive_twist")
                if twist_drive:
                    twist_drive.set_editor_property("damping", joint.dynamics.damping * 100.0)
                self.log(f"  Dynamics: {joint.name} → damping={joint.dynamics.damping:.4f}")
                stats["dynamics"] += 1

            if joint.dynamics.friction > 0:
                self.log(f"  Note: URDF friction ({joint.dynamics.friction}) for '{joint.name}' "
                         "— not directly mapped to UE constraint (apply via physics material)")
        except Exception as e:
            self.warn(f"  Could not apply dynamics for joint '{joint.name}': {e}")

    def _apply_collision_geometry(self, phys_asset, robot: URDFRobot, mapping: NameMapping,
                                  use_collision: bool, stats: dict):
        """Create or update collision bodies from URDF geometry."""
        for link in robot.links:
            bone_name = mapping.get_bone(link.name)
            if bone_name is None:
                continue

            geom_list = link.collisions if use_collision else link.visuals
            if not geom_list:
                continue

            body = self._find_body_setup(phys_asset, bone_name)
            if body is None:
                self.log(f"  No existing body for '{bone_name}' — cannot add geometry (body must exist)")
                continue

            for geom_item in geom_list:
                geom = geom_item.geometry
                if geom is None:
                    continue

                origin_ue = urdf_pos_to_ue(geom_item.origin.xyz)
                rot_ue = urdf_rpy_to_ue_deg(geom_item.origin.rpy)

                if geom.box:
                    sx = m_to_cm(geom.box.size[0]) / 2.0
                    sy = m_to_cm(geom.box.size[1]) / 2.0
                    sz = m_to_cm(geom.box.size[2]) / 2.0
                    try:
                        box_elem = unreal.KBoxElem()
                        box_elem.set_editor_property("x", sx * 2)
                        box_elem.set_editor_property("y", sy * 2)
                        box_elem.set_editor_property("z", sz * 2)
                        box_elem.set_editor_property("center", unreal.Vector(*origin_ue))
                        boxes = list(body.get_editor_property("agg_geom").get_editor_property("box_elems"))
                        boxes.append(box_elem)
                        body.get_editor_property("agg_geom").set_editor_property("box_elems", boxes)
                        stats["bodies"] += 1
                        self.log(f"  Box: {link.name} → {bone_name}: {sx*2:.1f}×{sy*2:.1f}×{sz*2:.1f} cm")
                    except Exception as e:
                        self.warn(f"  Could not add box to '{bone_name}': {e}")

                elif geom.sphere:
                    radius_cm = m_to_cm(geom.sphere.radius)
                    try:
                        sphere_elem = unreal.KSphereElem()
                        sphere_elem.set_editor_property("radius", radius_cm)
                        sphere_elem.set_editor_property("center", unreal.Vector(*origin_ue))
                        spheres = list(body.get_editor_property("agg_geom").get_editor_property("sphere_elems"))
                        spheres.append(sphere_elem)
                        body.get_editor_property("agg_geom").set_editor_property("sphere_elems", spheres)
                        stats["bodies"] += 1
                        self.log(f"  Sphere: {link.name} → {bone_name}: r={radius_cm:.1f} cm")
                    except Exception as e:
                        self.warn(f"  Could not add sphere to '{bone_name}': {e}")

                elif geom.cylinder:
                    radius_cm = m_to_cm(geom.cylinder.radius)
                    length_cm = m_to_cm(geom.cylinder.length)
                    try:
                        # UE uses Sphyl (capsule) for cylinder approximation
                        sphyl_elem = unreal.KSphylElem()
                        sphyl_elem.set_editor_property("radius", radius_cm)
                        sphyl_elem.set_editor_property("length", length_cm)
                        sphyl_elem.set_editor_property("center", unreal.Vector(*origin_ue))
                        sphyls = list(body.get_editor_property("agg_geom").get_editor_property("sphyl_elems"))
                        sphyls.append(sphyl_elem)
                        body.get_editor_property("agg_geom").set_editor_property("sphyl_elems", sphyls)
                        stats["bodies"] += 1
                        self.log(f"  Capsule: {link.name} → {bone_name}: r={radius_cm:.1f}, l={length_cm:.1f} cm")
                    except Exception as e:
                        self.warn(f"  Could not add capsule to '{bone_name}': {e}")

                elif geom.mesh:
                    self.log(f"  Mesh geometry: {link.name} → '{geom.mesh.filename}' "
                             "— mesh import not supported (use existing collision shapes)")

        self.log(f"  Created/updated {stats['bodies']} collision shapes")


# ===================================================================
# Convenience function for quick one-line use
# ===================================================================

def import_urdf(
    urdf_path: str,
    physics_asset_path: str,
    mode: str = "physics_only",
    mapping_file: Optional[str] = None,
    verbose: bool = True,
) -> bool:
    """
    One-line convenience function for URDF import.

    Args:
        urdf_path: Path to the .urdf file
        physics_asset_path: UE content path to the physics asset (e.g., "/Game/kinova/...")
        mode: "physics_only" or "full"
        mapping_file: Optional JSON override mapping file
        verbose: Print progress

    Usage from UE Python console:
        from urdf.urdf_importer import import_urdf
        import_urdf("C:/robot.urdf", "/Game/kinova/SkeletalMeshes/gen3_6dof_PhysicsAsset")
    """
    importer = URDFImporter(verbose=verbose)
    if mode == "full":
        return importer.import_full(urdf_path, physics_asset_path, mapping_file)
    else:
        return importer.import_to_existing(urdf_path, physics_asset_path, mapping_file)

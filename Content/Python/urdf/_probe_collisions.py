"""
Diagnostic: dump T3D collision data + bone world transforms for the Kinova Gen3.
Run in UE Editor Python console:
    exec(open("C:/Users/waemf/data/Ramms/Plugins/RammsCore/Content/Python/urdf/_probe_collisions.py").read())
"""
import math
import os
import sys
import tempfile

import unreal

sys.path.insert(0, "C:/Users/waemf/data/Ramms/Plugins/RammsCore/Content/Python")
from urdf.t3d_physics_parser import parse_physics_asset_t3d
from urdf.urdf_utils import (
    ue_pos_to_urdf, ue_quat_to_urdf_quat, ue_quat_to_urdf_rpy,
    quat_rotate_vector, quat_to_rotation_matrix, rotation_matrix_to_rpy,
)

PA_PATH = "/RammsAssets/Robots/KinovaGen3/Arm/SkeletalMeshes/gen3_6dof_PhysicsAsset"

pa = unreal.load_asset(PA_PATH)
assert pa is not None, f"Could not load {PA_PATH}"

# Export T3D
tmp = os.path.join(tempfile.gettempdir(), "_probe_coll.t3d")
task = unreal.AssetExportTask()
task.set_editor_property("object", pa)
task.set_editor_property("filename", tmp)
task.set_editor_property("automated", True)
task.set_editor_property("prompt", False)
task.set_editor_property("replace_identical", True)
unreal.Exporter.run_asset_export_task(task)
with open(tmp, "r") as f:
    t3d_text = f.read()
os.remove(tmp)
pa_data = parse_physics_asset_t3d(t3d_text)

# Get skeleton ref pose
skel_mesh = None
for prop in ("preview_skeletal_mesh", "skeletal_mesh"):
    try:
        skel_mesh = pa.get_editor_property(prop)
        if skel_mesh:
            break
    except Exception:
        pass
if not skel_mesh:
    mesh_path = PA_PATH.replace("_PhysicsAsset", "")
    skel_mesh = unreal.load_asset(mesh_path)
assert skel_mesh is not None

skeleton = skel_mesh.get_editor_property("skeleton")
ref_pose = skeleton.get_reference_pose()
assert ref_pose.is_valid()

print("=" * 80)
print("=== Collision Diagnostic: Kinova Gen3 6-DOF ===")
print("=" * 80)

for body in pa_data.body_setups:
    bone = body.bone_name
    t = ref_pose.get_bone_pose(bone, unreal.AnimPoseSpaces.WORLD)
    bpos = (t.translation.x, t.translation.y, t.translation.z)
    bquat = (t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z)

    print(f"\n--- {bone} ---")
    print(f"  Bone WORLD pos (cm): ({bpos[0]:.4f}, {bpos[1]:.4f}, {bpos[2]:.4f})")
    print(f"  Bone WORLD quat (w,x,y,z): ({bquat[0]:.6f}, {bquat[1]:.6f}, {bquat[2]:.6f}, {bquat[3]:.6f})")

    for i, box in enumerate(body.boxes):
        c = (box.center.x, box.center.y, box.center.z)
        rot_str = "None"
        if box.rotation:
            rot_str = f"({box.rotation.x:.4f}, {box.rotation.y:.4f}, {box.rotation.z:.4f})"
        print(f"  Box[{i}] T3D center (cm): ({c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f})")
        print(f"  Box[{i}] T3D rotation:    {rot_str}")
        print(f"  Box[{i}] T3D size (cm):   ({box.x:.4f}, {box.y:.4f}, {box.z:.4f})")

        # With bone rotation (current approach)
        c_world = quat_rotate_vector(bquat, c)
        urdf_xyz_rotated = ue_pos_to_urdf(c_world)
        urdf_rpy = ue_quat_to_urdf_rpy(bquat)
        print(f"  Box[{i}] center_world (cm, with rot): ({c_world[0]:.4f}, {c_world[1]:.4f}, {c_world[2]:.4f})")
        print(f"  Box[{i}] URDF xyz (with rot): ({urdf_xyz_rotated[0]:.6f}, {urdf_xyz_rotated[1]:.6f}, {urdf_xyz_rotated[2]:.6f})")
        print(f"  Box[{i}] URDF rpy (bone rot): ({urdf_rpy[0]:.6f}, {urdf_rpy[1]:.6f}, {urdf_rpy[2]:.6f})")

        # Without bone rotation (alternative: T3D center is component-space)
        urdf_xyz_direct = ue_pos_to_urdf(c)
        print(f"  Box[{i}] URDF xyz (no rot):  ({urdf_xyz_direct[0]:.6f}, {urdf_xyz_direct[1]:.6f}, {urdf_xyz_direct[2]:.6f})")

        # World-space collision center (for comparison)
        world_center = (bpos[0] + c_world[0], bpos[1] + c_world[1], bpos[2] + c_world[2])
        print(f"  Box[{i}] collision world pos (cm): ({world_center[0]:.4f}, {world_center[1]:.4f}, {world_center[2]:.4f})")

        # Also show where the collision ends up if we DON'T rotate
        world_center_nr = (bpos[0] + c[0], bpos[1] + c[1], bpos[2] + c[2])
        print(f"  Box[{i}] collision world pos (cm, no rot): ({world_center_nr[0]:.4f}, {world_center_nr[1]:.4f}, {world_center_nr[2]:.4f})")

print("\n=== Bone positions for reference (all in UE cm) ===")
for body in pa_data.body_setups:
    t = ref_pose.get_bone_pose(body.bone_name, unreal.AnimPoseSpaces.WORLD)
    print(f"  {body.bone_name:20s} Z={t.translation.z:.2f}")

print("\n=== Probe complete ===")

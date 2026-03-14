"""
Diagnostic: probe AnimPose bone reference transforms.

Development-only script — not part of the public API.

Run in UE Editor Python console:
    from urdf._probe_animpose import run; run()

Probes both gen3_6dof (simple serial chain) and mebot (complex w/ cycles).
Prints local + world transforms for all bones and compares with constraint Pos2.
"""

import unreal
import os
import tempfile

ASSETS = [
    {
        "label": "Kinova Gen3 6-DOF",
        "skel_mesh": "/RammsAssets/Robots/KinovaGen3/Arm/SkeletalMeshes/gen3_6dof",
        "phys_asset": "/RammsAssets/Robots/KinovaGen3/Arm/SkeletalMeshes/gen3_6dof_PhysicsAsset",
    },
    {
        "label": "MeBot",
        "skel_mesh": "/RammsAssets/Robots/RAMMP/Base/SkeletalMeshes/mebot",
        "phys_asset": "/RammsAssets/Robots/RAMMP/Base/SkeletalMeshes/mebot_PhysicsAsset",
    },
]


def _fmt_pos(loc):
    return f"({loc.x:.4f}, {loc.y:.4f}, {loc.z:.4f})"

def _fmt_quat(rot):
    return f"({rot.w:.4f}, {rot.x:.4f}, {rot.y:.4f}, {rot.z:.4f})"


def _export_t3d(phys_asset):
    """Export physics asset to T3D and return the text."""
    tmp_path = os.path.join(tempfile.gettempdir(), "ramms_diag_pa.t3d")
    task = unreal.AssetExportTask()
    task.set_editor_property("object", phys_asset)
    task.set_editor_property("filename", tmp_path)
    task.set_editor_property("automated", True)
    task.set_editor_property("prompt", False)
    task.set_editor_property("replace_identical", True)
    if not unreal.Exporter.run_asset_export_task(task):
        return None
    with open(tmp_path, "r") as f:
        text = f.read()
    os.remove(tmp_path)
    return text


def _probe_asset(info):
    print(f"\n{'='*60}")
    print(f"=== {info['label']} ===")
    print(f"{'='*60}")

    skel_mesh = unreal.load_asset(info["skel_mesh"])
    phys_asset = unreal.load_asset(info["phys_asset"])
    if skel_mesh is None or phys_asset is None:
        print(f"ERROR: Could not load assets")
        return

    skeleton = skel_mesh.get_editor_property("skeleton")
    ref_pose = skeleton.get_reference_pose()
    bone_names = [str(b) for b in ref_pose.get_bone_names()]
    print(f"Skeleton: {skeleton.get_name()}, {len(bone_names)} bones")

    # --- Get LOCAL and WORLD for all bones ---
    local_transforms = {}
    world_transforms = {}

    for name in bone_names:
        t_local = ref_pose.get_bone_pose(name, unreal.AnimPoseSpaces.LOCAL)
        t_world = ref_pose.get_bone_pose(name, unreal.AnimPoseSpaces.WORLD)
        local_transforms[name] = t_local
        world_transforms[name] = t_world

    print(f"\n--- All bone LOCAL transforms (pos in UE units) ---")
    for name in bone_names:
        t = local_transforms[name]
        print(f"  {name:40s} pos={_fmt_pos(t.translation)} quat={_fmt_quat(t.rotation)}")

    print(f"\n--- All bone WORLD transforms (pos in UE units) ---")
    for name in bone_names[:20]:  # limit to 20 for readability
        t = world_transforms[name]
        print(f"  {name:40s} pos={_fmt_pos(t.translation)} quat={_fmt_quat(t.rotation)}")
    if len(bone_names) > 20:
        print(f"  ... and {len(bone_names) - 20} more")

    # --- Parse T3D to get constraint Pos2 values for comparison ---
    from urdf.t3d_physics_parser import parse_physics_asset_t3d
    t3d_text = _export_t3d(phys_asset)
    if t3d_text:
        pa = parse_physics_asset_t3d(t3d_text)
        print(f"\n--- Constraint Pos2 vs bone WORLD transform ---")
        print(f"  {'Joint':<35s} {'Parent':<25s} {'Child':<25s} "
              f"{'Pos2 (constraint)':<35s} {'Child WORLD pos':<35s}")
        for ct in pa.constraints[:20]:
            child = ct.constraint_bone1
            parent = ct.constraint_bone2
            pos2 = f"({ct.pos2.x:.4f}, {ct.pos2.y:.4f}, {ct.pos2.z:.4f})"

            child_world = ""
            if child in world_transforms:
                wt = world_transforms[child]
                child_world = _fmt_pos(wt.translation)

            print(f"  {ct.joint_name:<35s} {parent:<25s} {child:<25s} "
                  f"{pos2:<35s} {child_world}")

        if len(pa.constraints) > 20:
            print(f"  ... and {len(pa.constraints) - 20} more constraints")

        # --- Compare get_relative_transform with constraint data ---
        print(f"\n--- get_relative_transform(child, parent) for first constraints ---")
        for ct in pa.constraints[:15]:
            child = ct.constraint_bone1
            parent = ct.constraint_bone2
            if child in bone_names and parent in bone_names:
                try:
                    t = ref_pose.get_relative_transform(child, parent)
                    loc = t.translation
                    rot = t.rotation
                    print(f"  {child:<30s} rel to {parent:<20s}: "
                          f"pos={_fmt_pos(loc)} quat={_fmt_quat(rot)}")
                except Exception as e:
                    print(f"  {child} rel to {parent}: FAILED - {e}")

    print(f"\n=== {info['label']} probe complete ===")


def run():
    """Run probes on all configured assets."""
    for asset_info in ASSETS:
        _probe_asset(asset_info)
    print("\n=== All probes complete ===")


run()



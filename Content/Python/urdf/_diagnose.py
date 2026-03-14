"""
Diagnostic: try AnimPose, asset export, and call_method on the selected PhysicsAsset.

Development-only script — not part of the public API.

Run in UE Editor Python console:
    from urdf._diagnose import run; run()
"""
import unreal


def run():
    """Run diagnostics on the selected PhysicsAsset."""
    selected = unreal.EditorUtilityLibrary.get_selected_assets()
    pa = None
    for a in selected:
        if isinstance(a, unreal.PhysicsAsset):
            pa = a
            break

    if pa is None:
        unreal.log_error("No PhysicsAsset selected")
        return

    name = pa.get_name()
    mesh_path = pa.get_path_name().split(".")[0].replace("_PhysicsAsset", "")
    skel_mesh = unreal.load_asset(mesh_path)
    skeleton = skel_mesh.get_editor_property("skeleton") if skel_mesh else None

    # --- AnimPose ---
    if skeleton:
        try:
            anim_pose = skeleton.get_reference_pose()
            unreal.log(f"AnimPose type: {type(anim_pose).__name__}")
            ap_attrs = sorted([a for a in dir(anim_pose) if not a.startswith("_")])
            unreal.log(f"AnimPose dir(): {ap_attrs}")
            for m in ["get_bone_names", "get_bone_name", "get_num_bones",
                       "get_bone_pose", "num_bones", "bone_names"]:
                try:
                    result = getattr(anim_pose, m)
                    if callable(result):
                        try:
                            val = result()
                            unreal.log(f"  anim_pose.{m}(): {val}")
                        except TypeError:
                            pass
                    else:
                        unreal.log(f"  anim_pose.{m}: {result}")
                except Exception:
                    pass
        except Exception as e:
            unreal.log(f"AnimPose failed: {e}")

    # --- Try Exporter ---
    for cls_name in ["Exporter", "AssetExportTask", "AssetToolsHelpers", "AssetTools"]:
        if hasattr(unreal, cls_name):
            unreal.log(f"Found: unreal.{cls_name}")
            attrs = sorted([a for a in dir(getattr(unreal, cls_name)) if not a.startswith("_")])
            unreal.log(f"  dir(): {attrs[:20]}")

    # --- Try to export to T3D via AssetExportTask ---
    try:
        import tempfile, os
        tmp = os.path.join(tempfile.gettempdir(), "pa_export.t3d")
        task = unreal.AssetExportTask()
        task.set_editor_property("object", pa)
        task.set_editor_property("filename", tmp)
        task.set_editor_property("automated", True)
        task.set_editor_property("prompt", False)
        task.set_editor_property("replace_identical", True)
        result = unreal.Exporter.run_asset_export_task(task)
        unreal.log(f"Export task result: {result}")
        if result and os.path.exists(tmp):
            with open(tmp, "r") as f:
                content = f.read()
            unreal.log(f"Exported {len(content)} chars. First 2000:")
            for line in content[:2000].split("\n"):
                unreal.log(f"  {line}")
        else:
            unreal.log("Export failed or file not created")
    except Exception as e:
        unreal.log(f"Export task failed: {e}")




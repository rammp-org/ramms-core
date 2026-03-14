"""
Editor convenience functions for URDF import/export.

These work on the currently selected Content Browser asset, deriving
all paths automatically. No need to type asset paths or file paths.

Usage from the UE Editor Python console:

    from urdf.urdf_editor_utils import export_selected_to_urdf
    export_selected_to_urdf()

    from urdf.urdf_editor_utils import import_urdf_to_selected
    import_urdf_to_selected("C:/path/to/robot.urdf")
"""

from __future__ import annotations

import os
from typing import Optional

try:
    import unreal
except ImportError:
    raise ImportError(
        "This module must be run inside the Unreal Editor Python environment."
    )


def _get_selected_physics_asset():
    """Return the first selected PhysicsAsset from the Content Browser, or None."""
    selected = unreal.EditorUtilityLibrary.get_selected_assets()
    for asset in selected:
        if isinstance(asset, unreal.PhysicsAsset):
            return asset
    return None


def _get_output_dir() -> str:
    """Return (and create) the project-level urdf/ output directory."""
    project_dir = unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_dir())
    output_dir = os.path.join(project_dir, "urdf")
    os.makedirs(output_dir, exist_ok=True)
    return output_dir


def export_selected_to_urdf(
    robot_name: Optional[str] = None,
    output_dir: Optional[str] = None,
    include_collision: bool = True,
    include_inertia: bool = True,
    mapping_file: Optional[str] = None,
    skeletal_mesh_path: Optional[str] = None,
) -> Optional[str]:
    """
    Export the currently selected physics asset to a URDF file.

    Select a PhysicsAsset in the Content Browser, then run:
        from urdf.urdf_editor_utils import export_selected_to_urdf
        export_selected_to_urdf()

    Args:
        robot_name: Name for the <robot> element (defaults to asset name).
        output_dir: Directory for the output file. Defaults to <ProjectDir>/urdf/.
        include_collision: Include collision geometry in URDF.
        include_inertia: Include inertial properties in URDF.
        mapping_file: Optional JSON with bone→link name overrides.
        skeletal_mesh_path: Optional UE content path to the skeletal mesh.
            If None, auto-detected from the physics asset's preview mesh.

    Returns:
        The output file path on success, None on failure.
    """
    phys_asset = _get_selected_physics_asset()
    if phys_asset is None:
        unreal.log_error(
            "No PhysicsAsset selected. "
            "Select a PhysicsAsset in the Content Browser and try again."
        )
        return None

    asset_name = phys_asset.get_name()
    asset_path = phys_asset.get_path_name().split(".")[0]
    name = robot_name or asset_name.replace("_PhysicsAsset", "")

    out_dir = output_dir or _get_output_dir()
    output_path = os.path.join(out_dir, f"{name}.urdf")

    unreal.log(f"Exporting '{asset_name}' → {output_path}")

    from urdf.urdf_exporter import URDFExporter

    exporter = URDFExporter(verbose=True)
    success = exporter.export(
        physics_asset_path=asset_path,
        output_path=output_path,
        robot_name=name,
        include_collision=include_collision,
        include_inertia=include_inertia,
        mapping_file=mapping_file,
        skeletal_mesh_path=skeletal_mesh_path,
    )

    if success:
        unreal.log(f"URDF exported to: {output_path}")
        return output_path
    else:
        unreal.log_error("URDF export failed. Check the Output Log for details.")
        return None


def import_urdf_to_selected(
    urdf_path: str,
    mode: str = "physics_only",
    mapping_file: Optional[str] = None,
) -> bool:
    """
    Import a URDF file into the currently selected physics asset.

    Select a PhysicsAsset in the Content Browser, then run:
        from urdf.urdf_editor_utils import import_urdf_to_selected
        import_urdf_to_selected("C:/path/to/robot.urdf")

    Args:
        urdf_path: Path to the .urdf file.
        mode: "physics_only" (mass, inertia, limits) or "full" (+ collision geometry).
        mapping_file: Optional JSON override mapping file.

    Returns:
        True on success.
    """
    phys_asset = _get_selected_physics_asset()
    if phys_asset is None:
        unreal.log_error(
            "No PhysicsAsset selected. "
            "Select a PhysicsAsset in the Content Browser and try again."
        )
        return False

    if not os.path.isfile(urdf_path):
        unreal.log_error(f"URDF file not found: {urdf_path}")
        return False

    asset_path = phys_asset.get_path_name().split(".")[0]

    unreal.log(f"Importing '{urdf_path}' → {phys_asset.get_name()}")

    from urdf.urdf_importer import URDFImporter

    importer = URDFImporter(verbose=True)
    if mode == "full":
        return importer.import_full(urdf_path, asset_path, mapping_file)
    else:
        return importer.import_to_existing(urdf_path, asset_path, mapping_file)

"""
Bone ↔ URDF link/joint name mapping.

Supports:
1. Auto-matching: case-insensitive comparison, stripping underscores/prefixes
2. JSON override file: explicit { "urdf_name": "ue_bone_name" } mapping

Usage:
    mapping = NameMapping.auto_match(urdf_robot, bone_names)
    mapping = NameMapping.from_file("mapping.json")
    mapping = NameMapping.auto_match(urdf_robot, bone_names, override_file="overrides.json")
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from difflib import SequenceMatcher
from typing import Dict, List, Optional, Set, Tuple


@dataclass
class NameMapping:
    """Bidirectional mapping between URDF link names and UE bone names."""

    # URDF link name → UE bone name
    link_to_bone: Dict[str, str] = field(default_factory=dict)
    # UE bone name → URDF link name
    bone_to_link: Dict[str, str] = field(default_factory=dict)
    # URDF joint name → UE constraint profile name (optional)
    joint_to_constraint: Dict[str, str] = field(default_factory=dict)
    # Unmatched names for diagnostics
    unmatched_links: List[str] = field(default_factory=list)
    unmatched_bones: List[str] = field(default_factory=list)

    def get_bone(self, urdf_link: str) -> Optional[str]:
        return self.link_to_bone.get(urdf_link)

    def get_link(self, bone_name: str) -> Optional[str]:
        return self.bone_to_link.get(bone_name)

    def summary(self) -> str:
        lines = [f"Matched: {len(self.link_to_bone)} link↔bone pairs"]
        if self.joint_to_constraint:
            lines.append(f"Matched: {len(self.joint_to_constraint)} joint→constraint pairs")
        if self.unmatched_links:
            lines.append(f"Unmatched URDF links: {self.unmatched_links}")
        if self.unmatched_bones:
            lines.append(f"Unmatched UE bones: {self.unmatched_bones}")
        return "\n".join(lines)

    # ----- Factory methods -----

    @classmethod
    def from_file(cls, filepath: str) -> "NameMapping":
        """Load an explicit mapping from a JSON file.

        JSON format:
        {
            "links": { "urdf_link_name": "ue_bone_name", ... },
            "joints": { "urdf_joint_name": "ue_constraint_name", ... }  // optional
        }
        """
        with open(filepath, "r") as f:
            data = json.load(f)

        mapping = cls()
        for urdf_name, bone_name in data.get("links", {}).items():
            mapping.link_to_bone[urdf_name] = bone_name
            mapping.bone_to_link[bone_name] = urdf_name
        for joint_name, constraint_name in data.get("joints", {}).items():
            mapping.joint_to_constraint[joint_name] = constraint_name
        return mapping

    @classmethod
    def auto_match(
        cls,
        urdf_link_names: List[str],
        bone_names: List[str],
        override_file: Optional[str] = None,
        similarity_threshold: float = 0.6,
    ) -> "NameMapping":
        """
        Automatically match URDF link names to UE bone names.

        Match priority:
        1. Exact match (case-insensitive)
        2. Normalized match (strip common prefixes/suffixes, underscores)
        3. Fuzzy match (SequenceMatcher ratio above threshold)
        4. Overrides from JSON file (highest priority, applied last)
        """
        mapping = cls()

        # Apply overrides first so they take priority
        overrides_link = {}
        overrides_joint = {}
        if override_file:
            try:
                with open(override_file, "r") as f:
                    data = json.load(f)
                overrides_link = data.get("links", {})
                overrides_joint = data.get("joints", {})
            except (FileNotFoundError, json.JSONDecodeError) as e:
                print(f"Warning: Could not load override file '{override_file}': {e}")

        remaining_links = list(urdf_link_names)
        remaining_bones = set(bone_names)

        # Phase 0: Apply explicit overrides
        for urdf_name, bone_name in overrides_link.items():
            if urdf_name in remaining_links and bone_name in remaining_bones:
                mapping.link_to_bone[urdf_name] = bone_name
                mapping.bone_to_link[bone_name] = urdf_name
                remaining_links.remove(urdf_name)
                remaining_bones.discard(bone_name)

        for joint_name, constraint_name in overrides_joint.items():
            mapping.joint_to_constraint[joint_name] = constraint_name

        # Phase 1: Exact match (case-insensitive)
        _match_phase(remaining_links, remaining_bones, mapping, _exact_match)

        # Phase 2: Normalized match
        _match_phase(remaining_links, remaining_bones, mapping, _normalized_match)

        # Phase 3: Fuzzy match
        def fuzzy_match(a: str, b: str) -> bool:
            return _fuzzy_match(a, b, similarity_threshold)
        _match_phase(remaining_links, remaining_bones, mapping, fuzzy_match)

        mapping.unmatched_links = remaining_links
        mapping.unmatched_bones = list(remaining_bones)
        return mapping

    def save(self, filepath: str) -> None:
        """Save the mapping to a JSON file."""
        data = {"links": self.link_to_bone}
        if self.joint_to_constraint:
            data["joints"] = self.joint_to_constraint
        with open(filepath, "w") as f:
            json.dump(data, f, indent=2)
        print(f"Mapping saved to {filepath}")


# ---------------------------------------------------------------------------
# Internal matching helpers
# ---------------------------------------------------------------------------

def _normalize(name: str) -> str:
    """Normalize a name for comparison: lowercase, strip common suffixes, remove non-alnum."""
    s = name.lower()
    # Strip common URDF suffixes first (more targeted than prefix stripping)
    for suffix in ("_link", "_joint", "_body", "_bone"):
        if s.endswith(suffix) and len(s) > len(suffix):
            s = s[:-len(suffix)]
            break
    # Strip common prefixes only if the result is non-trivial
    for prefix in ("link_", "joint_", "body_", "bone_"):
        if s.startswith(prefix) and len(s) > len(prefix):
            s = s[len(prefix):]
            break
    # Remove non-alphanumeric
    s = re.sub(r"[^a-z0-9]", "", s)
    return s


def _exact_match(a: str, b: str) -> bool:
    return a.lower() == b.lower()


def _normalized_match(a: str, b: str) -> bool:
    na = _normalize(a)
    nb = _normalize(b)
    return na == nb and na != ""


def _fuzzy_match(a: str, b: str, threshold: float) -> bool:
    ratio = SequenceMatcher(None, _normalize(a), _normalize(b)).ratio()
    return ratio >= threshold


def _match_phase(
    remaining_links: List[str],
    remaining_bones: Set[str],
    mapping: NameMapping,
    match_fn,
) -> None:
    """Run a matching pass, removing matched pairs from the remaining sets."""
    matched = []
    for link_name in list(remaining_links):
        for bone_name in list(remaining_bones):
            if match_fn(link_name, bone_name):
                mapping.link_to_bone[link_name] = bone_name
                mapping.bone_to_link[bone_name] = link_name
                matched.append((link_name, bone_name))
                remaining_bones.discard(bone_name)
                break
    for link_name, _ in matched:
        if link_name in remaining_links:
            remaining_links.remove(link_name)

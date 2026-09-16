"""Control-surface client over the Remote Control API.

A robot's controls are described and driven through its
``RammsRobotControlSurfaceComponent`` ("ControlSurface" on the pawn), so a
remote client never needs to know which controller components exist::

    from unreal_remote import UnrealRemote
    from unreal_remote.control_surface import ControlSurface

    ue = UnrealRemote()
    cs = ControlSurface.find(ue)[0]          # first robot with a surface
    for axis in cs.describe()["axes"]:
        print(axis["id"], axis["kind"], axis["range"])
    cs.set("drive.forward", 1.0)            # Source = Remote: holds over local input
    cs.set("linkage.LeftCenterLinkage.height", -5.0)
    cs.trigger("gripper.toggle")
    cs.release("drive.forward")             # springs back / stops holding
    print(cs.get("lift.motor_swing_arm_l"))

Every call maps onto the component's plain UFUNCTIONs (DescribeControlSurface
via GetControlSurfaceJson, SetControl, TriggerControl, ReleaseControl,
GetControlValue), which are callable on a PIE or -game instance.
"""
from __future__ import annotations

import json
from typing import Any, Optional

from . import RemoteObjectProxy, UnrealRemote

COMPONENT_CLASS = "RammsRobotControlSurfaceComponent"


class ControlSurface:
    """One robot's control surface (its ControlSurface component)."""

    def __init__(self, client: UnrealRemote, component_path: str, actor_path: str = ""):
        self._client = client
        self._proxy = RemoteObjectProxy(client, component_path)
        self.component_path = component_path
        self.actor_path = actor_path
        self._cache: Optional[dict] = None

    # ── discovery ───────────────────────────────────────────────────

    @classmethod
    def find(cls, client: UnrealRemote) -> list["ControlSurface"]:
        """Every robot in the world that has a control surface."""
        out = []
        for entry in client.find_actors_by_component(COMPONENT_CLASS):
            if entry["component_class"].endswith(COMPONENT_CLASS):
                out.append(cls(client, entry["component_path"], entry["actor_path"]))
        return out

    @classmethod
    def find_by_robot(cls, client: UnrealRemote, robot_name: str) -> Optional["ControlSurface"]:
        """The surface whose robotName (or actor path) contains robot_name."""
        for surface in cls.find(client):
            if robot_name in surface.actor_path or robot_name in surface.describe().get("robotName", ""):
                return surface
        return None

    # ── description ─────────────────────────────────────────────────

    def describe(self, refresh: bool = False) -> dict:
        """The FRammsControlSurface as a dict: robotName, groups, axes[]."""
        if self._cache is None or refresh:
            raw = self._proxy.call("GetControlSurfaceJson")
            self._cache = json.loads(raw) if isinstance(raw, str) else (raw or {})
        return self._cache

    def axes(self, group: Optional[str] = None) -> list[dict]:
        axes = self.describe().get("axes", [])
        return [a for a in axes if group is None or a.get("group") == group]

    def ids(self) -> list[str]:
        return [a["id"] for a in self.describe().get("axes", [])]

    def axis(self, control_id: str) -> Optional[dict]:
        for a in self.describe().get("axes", []):
            if a.get("id") == control_id:
                return a
        return None

    # ── commands ────────────────────────────────────────────────────

    def set(self, control_id: str, value: float, source: str = "Remote") -> bool:
        """Set a Continuous / Position / Velocity control. Clamped to its range."""
        return bool(self._proxy.call("SetControl", Id=control_id, Value=float(value), Source=source))

    def trigger(self, control_id: str, source: str = "Remote") -> bool:
        """Fire an Action control (camera.next, gripper.toggle, sim.reset...)."""
        return bool(self._proxy.call("TriggerControl", Id=control_id, Source=source))

    def release(self, control_id: str, source: str = "Remote") -> bool:
        """Let go: a Continuous axis springs to its default, a Position axis stops holding."""
        return bool(self._proxy.call("ReleaseControl", Id=control_id, Source=source))

    def get(self, control_id: str) -> float:
        """Live value (readback) of a control."""
        value = self._proxy.call("GetControlValue", Id=control_id)
        return float(value) if value is not None else float("nan")

    def drive(self, forward: float, turn: float, source: str = "Remote") -> bool:
        """Convenience: both drive axes."""
        ok = self.set("drive.forward", forward, source)
        return self.set("drive.turn", turn, source) and ok

    def stop(self, source: str = "Remote") -> None:
        self.release("drive.forward", source)
        self.release("drive.turn", source)

    def __repr__(self) -> str:
        return f"ControlSurface({self.component_path})"

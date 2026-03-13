#!/usr/bin/env python3
"""
Control skeletal mesh poses via Unreal Engine Remote Control API.

Sets bone rotations/translations on actors with URammsSkeletalPoseComponent,
which uses UPoseableMeshComponent for kinematic (non-physics) bone posing.
Suitable for driving UI robot visualizations from external joint data.

Usage:
    python control_skeletal_pose.py --list                  # Find poseable actors
    python control_skeletal_pose.py --describe              # Show joints and values
    python control_skeletal_pose.py --set-all 0 15 -90 30   # Set all joints
    python control_skeletal_pose.py --set Shoulder 45.0     # Set joint by name
    python control_skeletal_pose.py --set-index 0 45.0      # Set joint by index
    python control_skeletal_pose.py --home                   # All joints to 0
    python control_skeletal_pose.py --interactive            # Interactive REPL

Requires Unreal Engine running with Remote Control API plugin enabled (port 30010).
"""

import argparse
import logging
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from unreal_remote import UnrealRemote, UnrealRemoteError

BRIDGE_CDO = "/Script/RammsCore.Default__RammsCoreBridge"


def find_pose_actors(ue: UnrealRemote) -> list[dict]:
    """Find actors with URammsSkeletalPoseComponent via the bridge."""
    raw = ue.call_function(BRIDGE_CDO, "FindSkeletalPoseActors")
    results = raw if isinstance(raw, list) else raw.get("ReturnValue", [])
    out = []
    for entry in results:
        parts = entry.split("|", 1)
        out.append({
            "actor_path": parts[0],
            "component_name": parts[1] if len(parts) > 1 else "",
        })
    return out


def find_pose_component(ue: UnrealRemote, actor_hint: str = ""):
    """
    Find an actor's SkeletalPoseComponent proxy.

    Returns (actor_proxy, component_proxy) or (None, None).
    """
    if actor_hint:
        comps = ue.find_components(actor_hint, "SkeletalPose")
        if comps:
            return ue.actor(actor_hint), ue.actor(comps[0]["path"])
        return ue.actor(actor_hint), None

    actors = find_pose_actors(ue)
    if not actors:
        return None, None

    actor_path = actors[0]["actor_path"]
    comps = ue.find_components(actor_path, "SkeletalPose")
    if comps:
        return ue.actor(actor_path), ue.actor(comps[0]["path"])
    return ue.actor(actor_path), None


def get_joint_values(comp) -> list:
    """Read current joint values from the component."""
    result = comp.call("GetAllJointValues")
    if isinstance(result, list):
        return result
    if isinstance(result, dict):
        return result.get("ReturnValue", [])
    return []


def get_joint_targets(comp) -> list:
    """Read current joint targets from the component."""
    result = comp.call("GetAllJointTargets")
    if isinstance(result, list):
        return result
    if isinstance(result, dict):
        return result.get("ReturnValue", [])
    return []


def set_all_joints(comp, values: list[float]):
    """Set all joint targets.

    Uses individual SetJointTarget calls to avoid UE Remote Control
    TArray<float> serialization issues.
    """
    for i, val in enumerate(values):
        comp.call("SetJointTarget", JointIndex=i, Value=float(val))


def set_joint_by_name(comp, name: str, value: float):
    """Set a single joint target by name."""
    comp.call("SetJointTargetByName", JointName=name, Value=value)


def set_joint_by_index(comp, index: int, value: float):
    """Set a single joint target by index."""
    comp.call("SetJointTarget", JointIndex=index, Value=value)


def describe_pose(comp):
    """Print current joint state."""
    values = get_joint_values(comp)
    targets = get_joint_targets(comp)
    num = max(len(values), len(targets))
    print(f"\nSkeletal Pose — {num} joints:")
    print(f"  {'Idx':<5} {'Current':>10} {'Target':>10}")
    print(f"  {'-'*5} {'-'*10} {'-'*10}")
    for i in range(num):
        cur = values[i] if i < len(values) else 0.0
        tgt = targets[i] if i < len(targets) else 0.0
        print(f"  {i:<5} {cur:>10.2f} {tgt:>10.2f}")
    print()


def interactive_mode(comp):
    """Interactive REPL for controlling poses."""
    print("\n=== Skeletal Pose Controller ===")
    print("Commands:")
    print("  state                       Show current joint values")
    print("  set <idx> <value>           Set joint by index")
    print("  setname <name> <value>      Set joint by name")
    print("  setall <v0> <v1> ...        Set all joints")
    print("  home                        All joints to 0")
    print("  snap                        Snap to targets (bypass interpolation)")
    print("  quit                        Exit")
    print()

    while True:
        try:
            line = input("pose> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nBye!")
            break

        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()

        if cmd in ("quit", "exit", "q"):
            break

        elif cmd in ("state", "describe", "values", "read"):
            describe_pose(comp)

        elif cmd == "set" and len(parts) >= 3:
            try:
                idx = int(parts[1])
                val = float(parts[2])
                set_joint_by_index(comp, idx, val)
                print(f"  Joint {idx} → {val}")
            except (ValueError, IndexError) as e:
                print(f"  Error: {e}")

        elif cmd == "setname" and len(parts) >= 3:
            try:
                name = parts[1]
                val = float(parts[2])
                set_joint_by_name(comp, name, val)
                print(f"  Joint '{name}' → {val}")
            except (ValueError, IndexError) as e:
                print(f"  Error: {e}")

        elif cmd == "setall" and len(parts) >= 2:
            try:
                vals = [float(x) for x in parts[1:]]
                set_all_joints(comp, vals)
                print(f"  Set {len(vals)} joints: {vals}")
            except ValueError as e:
                print(f"  Error: {e}")

        elif cmd == "home":
            values = get_joint_values(comp)
            home = [0.0] * len(values) if values else [0.0]
            set_all_joints(comp, home)
            print(f"  Homed {len(home)} joints to 0")

        elif cmd == "snap":
            comp.call("SnapToTargets")
            print("  Snapped to targets")

        else:
            print(f"  Unknown command: {cmd}")


def main():
    parser = argparse.ArgumentParser(
        description="Control skeletal mesh poses via Unreal Remote Control")
    parser.add_argument("--actor", default="",
                        help="Actor path (auto-discovered if omitted)")
    parser.add_argument("--component", default="",
                        help="Component path override")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=30010)
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Enable debug logging")

    group = parser.add_mutually_exclusive_group()
    group.add_argument("--list", action="store_true",
                       help="List actors with SkeletalPoseComponent")
    group.add_argument("--describe", action="store_true",
                       help="Show current joint values")
    group.add_argument("--set-all", nargs="+", type=float, metavar="VALUE",
                       help="Set all joint values")
    group.add_argument("--set", nargs=2, metavar=("NAME", "VALUE"),
                       help="Set one joint by name (e.g. --set Shoulder 45)")
    group.add_argument("--set-index", nargs=2, metavar=("IDX", "VALUE"),
                       help="Set one joint by index (e.g. --set-index 0 45)")
    group.add_argument("--home", action="store_true",
                       help="Send all joints to 0")
    group.add_argument("--interactive", "-i", action="store_true",
                       help="Interactive control mode")

    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)

    ue = UnrealRemote(host=args.host, http_port=args.port)
    print(f"Connecting to UE at http://{args.host}:{args.port}...")
    if not ue.ping():
        print("Connection failed!")
        sys.exit(1)
    print("Connected!\n")

    if args.list:
        print("Searching for actors with RammsSkeletalPoseComponent...")
        actors = find_pose_actors(ue)
        for a in actors:
            print(f"  Actor: {a['actor_path']}")
            print(f"  Component: {a['component_name']}")
            print()
        if not actors:
            print("  No actors with RammsSkeletalPoseComponent found")
        return

    # Find or connect to the component
    if args.component:
        comp = ue.actor(args.component)
    else:
        print("Searching for SkeletalPoseComponent...")
        actor, comp = find_pose_component(ue, args.actor)
        if not comp:
            print("No RammsSkeletalPoseComponent found!")
            print("Use --list to see available actors, or --actor / --component to specify.")
            sys.exit(1)
        print(f"Found: {comp.object_path}\n")

    if args.describe:
        describe_pose(comp)
    elif args.set_all:
        set_all_joints(comp, args.set_all)
        print(f"Set {len(args.set_all)} joints: {args.set_all}")
    elif args.set:
        name, val = args.set[0], float(args.set[1])
        set_joint_by_name(comp, name, val)
        print(f"Joint '{name}' → {val}")
    elif args.set_index:
        idx, val = int(args.set_index[0]), float(args.set_index[1])
        set_joint_by_index(comp, idx, val)
        print(f"Joint {idx} → {val}")
    elif args.home:
        values = get_joint_values(comp)
        home = [0.0] * len(values) if values else [0.0]
        set_all_joints(comp, home)
        print(f"Homed {len(home)} joints to 0")
    elif args.interactive:
        interactive_mode(comp)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()

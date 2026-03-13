# Robotic Assistive Mobility and Manipulation Simulation (RAMMS) Core Plugin

 This plugin provides foundational components and utilities for simulating
 robotic assistive mobility devices and manipulation tasks within Unreal
 Engine 5. It supports powered wheelchairs, robotic arms, grippers, and
 accessible environment interactions with realistic physics, sensor
 simulation, and control interfaces.

 Designed as a plugin for UE5, it can be added to any project by cloning or
 adding it as a git submodule within your project's `Plugins/` directory.

 For an example project, see the
 [RAMMS Sim Project](https://github.com/rammp-org/ramms-sim).

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Robotic Assistive Mobility and Manipulation Simulation (RAMMS) Core Plugin](#robotic-assistive-mobility-and-manipulation-simulation-ramms-core-plugin)
  - [Quick Start](#quick-start)
  - [Developing](#developing)
    - [Code style](#code-style)
  - [Components](#components)
    - [Mobility](#mobility)
    - [Manipulation](#manipulation)
    - [Environment](#environment)
  - [Kinova Gen3 Robotic Arm Controller](#kinova-gen3-robotic-arm-controller)
    - [Arm Configuration](#arm-configuration)
    - [Control Modes](#control-modes)
    - [Joint Configuration (`FRevoluteJointConfig`)](#joint-configuration-frevolutejointconfig)
  - [IK Solver System](#ik-solver-system)
    - [Shared Infrastructure](#shared-infrastructure)
      - [FK Pipeline (`ComputeChainKinematics`)](#fk-pipeline-computechainkinematics)
    - [Common Solver Parameters](#common-solver-parameters)
    - [Solver Comparison](#solver-comparison)
    - [DLS (Damped Least Squares)](#dls-damped-least-squares)
    - [FABRIK](#fabrik)
    - [CCD (Cyclic Coordinate Descent)](#ccd-cyclic-coordinate-descent)
  - [Blueprint API (`URammsIKLibrary`)](#blueprint-api-urammsiklibrary)
    - [`FIKSolveResult`](#fiksolveresult)
  - [URDF Export / Import](#urdf-export--import)
    - [Quick Start (URDF)](#quick-start-urdf)
    - [Exporting a Physics Asset to URDF](#exporting-a-physics-asset-to-urdf)
    - [Importing a URDF into a Physics Asset](#importing-a-urdf-into-a-physics-asset)
    - [Name Mapping](#name-mapping)
    - [Coordinate System Conventions](#coordinate-system-conventions)
    - [Module Reference](#module-reference)
    - [Running Tests](#running-tests)
  - [Dependencies](#dependencies)

<!-- markdown-toc end -->

## Quick Start

 1. Clone or add this repository as a git submodule in `Plugins/`.
 2. Open your UE5 project. The plugin is automatically detected and compiled.
 3. Enable the plugin in **Edit > Plugins** if not already enabled.
 4. Add the desired controller component to your actor (e.g.
    `UKinovaGen3ControllerComponent` for a robotic arm).

## Developing

If you want to contribute or customize the plugin, you can clone the repository
and set it up in your Unreal project. If you want to make changes to the code,
please follow the code style guidelines below.

### Code style

1. Ensure `clang-format` is installed
2. Ensure [pre-commit](https://pre-commit.com) is installed
3. Set up `pre-commit` for this repository:

  ``` console
  pre-commit install
  ```

This helps ensure that consistent code formatting is applied.

## Components

### Mobility

 | Component | Description |
 |---|---|
 | `URammsDifferentialDriveController` | Differential drive controller for powered wheelchair simulation |
 | `URammsDifferentialDriveLibrary` | Blueprint function library with differential drive helpers |
 | `UMebotControllerComponent` | MEBot wheelchair base with linear and angular actuator control |

### Manipulation

 | Component | Description |
 |---|---|
 | `UKinovaGen3ControllerComponent` | Physics-based 7-DOF robotic arm controller with IK solvers |
 | `UGripperControllerComponent` | Gripper controller for robotic end effectors |
 | `URammsIKLibrary` | Static Blueprint function library exposing FK and IK solvers |

### Environment

 | Component | Description |
 |---|---|
 | `UVanRampComponent` | Accessible van ramp with animation control |
 | `UVanDoorComponent` | Accessible door interaction simulation |

 ---

## Kinova Gen3 Robotic Arm Controller

 `UKinovaGen3ControllerComponent` is the primary controller for articulated
 robotic arms. It reads joint angles from physics constraints on a
 SkeletalMesh, runs IK solvers to compute target angles, and drives the
 constraints to reach end-effector target poses.

### Arm Configuration

 | Property | Default | Description |
 |---|---|---|
 | `SkeletalMeshComponentName` | `"ArmSkMesh"` | Name of the SkeletalMeshComponent on the owning actor |
 | `EndEffectorBoneName` | `"end_effector"` | Bone or socket name at the arm tip |
 | `bAutoPopulateOnSkeletalMeshChange` | `true` | Auto-detect joints from the skeleton |

### Control Modes

 | Property | Options | Description |
 |---|---|---|
 | `ArmControlMode` | `JointControl`, `EndEffectorControl` | Direct joint angles vs. IK-driven end-effector targeting |
 | `ControlMode` | `PositionControl`, `VelocityControl`, `TorqueControl` | How joint targets are applied to physics constraints |
 | `TargetActor` | (Actor reference) | Actor whose world transform is the IK target. If unset, uses `TargetEndEffectorTransform` |

### Joint Configuration (`FRevoluteJointConfig`)

 The `Joints` array contains one entry per revolute (hinge) joint:

 | Property | Default | Description |
 |---|---|---|
 | `BoneName` | — | Skeleton bone this joint corresponds to |
 | `ConstraintName` | — | Physics constraint name (defaults to `BoneName` if unset) |
 | `ControlledAxis` | `Twist` | Constraint axis actuated: `Twist` (X), `Swing1` (Y), or `Swing2` (Z) |
 | `bInvertAxisForIK` | `false` | Flip the axis direction if it points the wrong way |
 | `AngleOffset` | `0.0` | Calibration offset (deg) between constraint zero and reference pose |
 | `MinAngleLimit` / `MaxAngleLimit` | `-180` / `180` | Joint limits in degrees |
 | `MaxAngularSpeed` | `90.0` | Maximum angular velocity (deg/s) |
 | `SpeedMultiplier` | `1.0` | Scale factor (0–1) on `MaxAngularSpeed` |
 | `MaxTorque` | `39.0` | Maximum torque (N·m) for torque-controlled joints |
 | `PositionStrength` | `5000000.0` | PD position gain for position-controlled joints |
 | `PositionDamping` | `0.0` | PD damping gain |

 ---

## IK Solver System

### Shared Infrastructure

 All three solvers (DLS, FABRIK, CCD) share the same core:

 - **`ComputeChainKinematics`** — Forward kinematics from joint angles to
   world-space joint positions, axes, and end-effector transform.
 - **`ApplyAngleLimits`** — Joint limit enforcement with optional escape margin.
 - **`RotationErrorAxisAngleRad`** — Shortest-path rotation error as a
   world-space axis-angle vector.
 - **Bone-derived kinematic chain** — `JointLocalTransforms`, `JointAxesLocal`,
   and `EndEffectorOffset` are derived from the reference skeleton so FK
   matches the physics simulation exactly.

#### FK Pipeline (`ComputeChainKinematics`)

 1. Start from `BaseTransform` (world-space arm base).
 2. For each joint `i` (root to tip):
    - Apply `JointLocalTransforms[i]` (parent-to-joint at q=0).
    - Record `JointPosWorld[i]` and transform `JointAxesLocal[i]` to world.
    - Apply revolute rotation `FQuat(AxisWorld, q_i)` about the axis.
 3. Apply `EndEffectorOffset` after the last joint.

### Common Solver Parameters

 These apply regardless of which solver is selected:

 | Parameter | Default | Description |
 |---|---|---|
 | `IKSolverType` | `DLS` | Active solver: `DLS`, `FABRIK`, `CCD` |
 | `IKPositionTolerance` | `1.0` cm | Position convergence threshold |
 | `IKRotationTolerance` | `5.0` deg | Rotation convergence threshold |
 | `TaskSpaceMask` | `{true x6}` | Which DOFs to solve: `[X, Y, Z, Rx, Ry, Rz]` |
 | `IKTargetChangePosThreshold` | `0.01` cm | Min target position change to trigger re-solve |
 | `IKTargetChangeRotThreshold` | `0.1` deg | Min target rotation change to trigger re-solve |

 Set the orientation entries of `TaskSpaceMask` to `false` for position-only
 solving (faster, fewer DOFs to satisfy).

### Solver Comparison

 |  | DLS | FABRIK | CCD |
 |---|---|---|---|
 | **Algorithm** | Full Jacobian pseudoinverse | Per-joint scalar DLS (Gauss-Seidel) | Exact hinge-plane angles |
 | **FK per iteration** | 1 | 1 | N (per joint) |
 | **Joint update** | All at once (matrix solve) | Sequential, linearized with residual tracking | Sequential, exact geometry |
 | **Orientation** | Coupled in 6×N Jacobian | Separate refinement pass | Blended per-joint |
 | **Cost per iter** | 1 FK + matrix solve | 1 FK + N dot products | N FK evaluations |
 | **Convergence** | Fast (global optimal) | Fast (linearized) | Moderate (greedy) |
 | **Best for** | General-purpose, highest quality | Lightweight, no matrix library needed | Simple, robust |

 ---

### DLS (Damped Least Squares)

 The default and most capable solver. Builds the full 6×N Jacobian each
 iteration and solves: `dq = W J^T (J W J^T + λ²I)⁻¹ e`.

 | Parameter | Default | Description |
 |---|---|---|
 | `MaxIKIterations` | `300` | Maximum iterations per solve |
 | `IKDampingFactor` | `0.1` | Damping λ — higher = more stable near singularities, less precise |
 | `IKStepClip` | `0.2` rad | Per-joint step clamp per iteration (~11.5°) |
 | `JointWeights` | (uniform) | Per-joint weight diagonal `W`. Higher = more responsive |
 | `bEnableNullSpaceOptimization` | `false` | Bias angles toward preferred pose without affecting EE |
 | `NullSpaceGain` | `0.1` | Null-space bias strength (0–1) |
 | `NullSpaceBias` | (empty) | Preferred joint angles (deg) for null-space optimization |

 **Tuning:**
 - Raise `IKDampingFactor` (0.2–0.5) if the arm oscillates near singularities.
 - Lower `IKStepClip` for smoother but slower convergence.
 - Use `JointWeights` to make wrist joints lighter than shoulder joints.

 ---

### FABRIK

 A per-joint Jacobian-column solver using the same math as DLS but without
 the matrix solve. For each joint (tip to root), it computes a scalar DLS
 step from the Jacobian velocity column, then subtracts its predicted EE
 movement from the residual so the next joint sees the corrected error:

 ```
 Jv_i = axis_i × (EE - joint_i)                     // Jacobian velocity column
 dq_i = (Jv_i · dp_residual) / (Jv_i · Jv_i + λ²)  // scalar DLS
 dp_residual -= Jv_i * dq_i                          // Gauss-Seidel update
 ```

 This gives Gauss-Seidel-like convergence from a single FK evaluation
 per iteration.

 | Parameter | Default | Description |
 |---|---|---|
 | `FABRIKMaxIterations` | `200` | Maximum iterations |
 | `FABRIKPositionTolerance` | `1.0` cm | Position convergence threshold |
 | `FABRIKAngleGain` | `1.0` | Multiplier on the scalar DLS step (1.0 = optimal) |
 | `FABRIKMaxAngleStepDeg` | `12.0°` | Per-joint step clamp per iteration |
 | `FABRIKLimitEscapeDeg` | `2.0°` | Max step to move off a joint limit when stuck |
 | `FABRIKOrientationIterations` | `10` | Separate orientation refinement passes after position |
 | `FABRIKOrientationGain` | `0.5` | Orientation refinement strength |

 **Tuning:**
 - Increase `FABRIKMaxIterations` or `FABRIKMaxAngleStepDeg` if the arm
   doesn't reach the target.
 - `AngleGain > 1.0` is more aggressive but can overshoot.
 - Set `FABRIKOrientationIterations = 0` for position-only solving.

 ---

### CCD (Cyclic Coordinate Descent)

 Classic CCD constrained to 1-DOF hinge joints. Unlike FABRIK, it
 recomputes FK for each joint during the sweep, giving exact
 (non-linearized) geometry. For each joint it projects the joint-to-EE
 and joint-to-target vectors onto the hinge plane (perpendicular to the
 joint axis) and computes the exact signed angle between them.

 When orientation is enabled, each joint blends a position delta and an
 orientation delta (rotation error projected onto the joint axis).

 | Parameter | Default | Description |
 |---|---|---|
 | `CCDMaxIterations` | `100` | Maximum iterations |
 | `CCDPositionGain` | `1.0` | Scale on hinge-plane angle (1.0 = full correction) |
 | `CCDOrientationGain` | `0.3` | Orientation blend weight (0 = position only) |
 | `CCDMaxAngleStepDeg` | `12.0°` | Per-joint step clamp per iteration |

 **Tuning:**
 - CCD costs more per iteration (N FK evals) but needs fewer iterations for
   simple targets.
 - Increase `CCDOrientationGain` if EE orientation matters more than position.
 - Lower `CCDPositionGain` (0.5–0.8) to reduce oscillation.

 ---

## Blueprint API (`URammsIKLibrary`)

 All solvers and FK are exposed as static `BlueprintCallable` functions:

 ```cpp
 // Forward kinematics
 static FTransform ComputeForwardKinematics(
     const FTransform& BaseTransform,
     const TArray<float>& JointAnglesDeg,
     const TArray<FTransform>& JointLocalTransforms,
     const TArray<FVector>& JointAxesLocal,
     const FTransform& EndEffectorOffset,
     bool bReturnEEOnly);

 // IK solvers — all return FIKSolveResult
 static FIKSolveResult SolveIK_DLS(...);
 static FIKSolveResult SolveIK_FABRIK(...);
 static FIKSolveResult SolveIK_CCD(...);
 ```

### `FIKSolveResult`

 | Field | Type | Description |
 |---|---|---|
 | `bSuccess` | `bool` | Whether convergence tolerances were met |
 | `JointAngles` | `TArray<float>` | Solved joint angles (degrees) |
 | `PositionError` | `float` | Final position error (cm) |
 | `RotationError` | `float` | Final rotation error (degrees) |
 | `IterationsUsed` | `int32` | Iterations consumed |

 ---

## URDF Export / Import

 The plugin includes Python scripts for converting between Unreal Engine
 Physics Assets and [URDF](http://wiki.ros.org/urdf) (Unified Robot
 Description Format) files. This enables interoperability with ROS, MoveIt,
 PyBullet, MuJoCo, and other robotics tools.

 The scripts run inside the **UE Editor Python console** and require no
 external dependencies beyond the Python standard library.

### Quick Start (URDF)

 1. Enable the **Python Editor Script Plugin** in
    **Edit > Plugins > Scripting**.
 2. Open the **Output Log** (**Window > Developer Tools > Output Log**) and
    switch the input dropdown from *Cmd* to *Python*.
 3. Select a **PhysicsAsset** in the Content Browser.
 4. Run:

 ```python
 from urdf.urdf_editor_utils import export_selected_to_urdf
 export_selected_to_urdf()
 ```

 The URDF file is written to `<ProjectDir>/urdf/<asset_name>.urdf`.

### Exporting a Physics Asset to URDF

 `export_selected_to_urdf()` exports the currently selected Physics Asset.
 It automatically resolves the associated skeletal mesh and uses the
 skeleton's reference pose for bone transforms.

 ```python
 from urdf.urdf_editor_utils import export_selected_to_urdf

 # Defaults — auto-names from the asset, writes to <Project>/urdf/
 export_selected_to_urdf()

 # Full options
 export_selected_to_urdf(
     robot_name="my_robot",             # <robot name="...">
     output_dir="C:/output",            # output directory
     include_collision=True,            # include <collision> geometry
     include_inertia=True,              # include <inertial> properties
     mapping_file="C:/map.json",        # optional bone↔link name overrides
     skeletal_mesh_path="/Game/MyMesh", # explicit skeletal mesh (auto-detected if omitted)
 )
 ```

 You can also call the exporter directly with an asset path:

 ```python
 from urdf.urdf_exporter import URDFExporter

 exporter = URDFExporter(
     physics_asset_path="/Game/Robots/Arm/gen3_6dof_PhysicsAsset",
     robot_name="kinova_gen3",
 )
 exporter.export("C:/output/kinova_gen3.urdf")
 ```

 **What the exporter does:**

 - Exports the T3D representation of the Physics Asset to extract body
   setups (collision shapes, mass) and constraint templates (joint types,
   limits, axes).
 - Reads bone world transforms from the skeleton reference pose.
 - Builds a kinematic tree from constraint parent/child relationships,
   detecting and excluding loop-closure constraints (URDF requires a tree).
 - Converts collision geometry (boxes, spheres, capsules, convex hulls →
   cylinders) into URDF `<collision>` elements.
 - Outputs joint types: `revolute`, `prismatic`, `fixed`, or `continuous`
   based on constraint settings and limits.
 - Applies automatic corrections for misaligned bone orientations and
   snaps near-cardinal joint axes to clean values.

### Importing a URDF into a Physics Asset

 `import_urdf_to_selected()` applies URDF data to the currently selected
 Physics Asset.

 ```python
 from urdf.urdf_editor_utils import import_urdf_to_selected

 # Physics-only: apply mass, inertia, and joint limits
 import_urdf_to_selected("C:/path/to/robot.urdf")

 # Full import: also create collision bodies from URDF geometry
 import_urdf_to_selected("C:/path/to/robot.urdf", mode="full")
 ```

 | Mode | What it configures |
 |---|---|
 | `physics_only` | Mass, inertia tensors, joint limits, drive parameters |
 | `full` | Everything above + collision shapes from URDF geometry |

### Name Mapping

 The tooling automatically matches URDF link names to UE bone names using
 case-insensitive comparison with underscore/prefix stripping and fuzzy
 matching. For cases where auto-matching isn't sufficient, provide a JSON
 override file:

 ```json
 {
   "base_link": "root_bone",
   "link_1": "shoulder_bone",
   "link_2": "upper_arm_bone"
 }
 ```

 Pass it via the `mapping_file` parameter on export or import.

### Coordinate System Conventions

 | | URDF | Unreal Engine |
 |---|---|---|
 | **Units** | meters, radians, kg | centimeters, degrees, kg |
 | **Handedness** | Right-handed, Z-up | Left-handed, Z-up |
 | **Y axis** | Points left | Points right (negated) |

 All conversions are handled automatically. The Y axis is negated when
 transforming positions, quaternions, and joint axes between the two
 coordinate systems.

### Module Reference

 All scripts live in `Content/Python/urdf/` within the plugin directory.

 | Module | Description |
 |---|---|
 | `urdf_editor_utils.py` | One-click export/import entry points for the UE Editor |
 | `urdf_exporter.py` | Core export logic (Physics Asset → URDF) |
 | `urdf_importer.py` | Core import logic (URDF → Physics Asset) |
 | `urdf_parser.py` | Pure-Python URDF XML parser and writer |
 | `t3d_physics_parser.py` | Parser for UE T3D Physics Asset export format |
 | `urdf_utils.py` | Unit conversions and coordinate transforms |
 | `name_mapping.py` | Bone ↔ URDF link name matching and JSON overrides |

### Running Tests

 The test suite can be run outside of Unreal Engine (no `unreal` module
 required):

 ```console
 cd Plugins/RammsCore/Content/Python
 python -m pytest urdf/test_urdf.py -v
 ```

 ---

## Dependencies

 - **Eigen** — Matrix operations for DLS and rotation error computation.
   Included as a third-party dependency.
 - **PhysicsCore / Chaos** — Physics constraint reading and actuation.
 

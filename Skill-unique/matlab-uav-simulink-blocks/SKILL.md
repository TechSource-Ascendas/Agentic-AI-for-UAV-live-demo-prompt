---
name: matlab-uav-simulink-blocks
description: Build or modify MATLAB/Simulink UAV demonstrations by selecting, configuring, and wiring Guidance Model, Waypoint Follower, UAV Scenario, lidar, obstacle-avoidance, animation, or 6DOF blocks. Use when a task requires correct UAV block interfaces and coordinate-frame handling, not for general MATLAB calculations unrelated to UAV models.
---

# MATLAB UAV Simulink Blocks

Build the requested UAV capability from block-level contracts. Preserve an existing model unless the user asks for a replacement, and keep baseline and enhanced variants separate when comparison is useful.

## Required companion workflows

- Use `building-simulink-models` for every structural or block-parameter edit. Complete its library/policy gate before reading or editing a model.
- Use `simulating-simulink-models` when a run needs parameter overrides, selected controller modes, or structured result retrieval.
- Confirm version-specific mask parameter names from installed MATLAB help or official MathWorks documentation before configuring unfamiliar blocks. Do not guess library paths or mask parameters.

## Route the task

Read only the references needed for the requested capability:

- Step response, reduced-order multirotor plant, or waypoint following: [references/guidance-waypoint.md](references/guidance-waypoint.md)
- Lightweight visual verification of dynamics or waypoint following without a scenario: [references/uav-animation.md](references/uav-animation.md)
- UAV scene, platforms, buildings, lidar, or point clouds: [references/scenario-lidar.md](references/scenario-lidar.md)
- Reactive collision avoidance using lidar: [references/obstacle-avoidance.md](references/obstacle-avoidance.md)
- Higher-fidelity rigid-body dynamics: [references/sixdof.md](references/sixdof.md)
- End-to-end compositions and acceptance checks: [references/task-recipes.md](references/task-recipes.md)

## Shared invariants

1. State the coordinate frame at every subsystem boundary. The multirotor Guidance Model uses NED: `[north east down]`, so altitude above the origin is `-down`.
2. Keep angle order explicit. Guidance state `EulerZYX` is `[yaw pitch roll]`; blocks that expect `[roll pitch yaw]` require reordering.
3. Keep quaternion convention explicit. UAV Scenario motion ports use `[w x y z]` row vectors.
4. Convert vector orientation deliberately. Scenario Motion Write expects row vectors, while navigation blocks may use column vectors.
5. Use a single source of truth for scenario, platform, and sensor objects. Recreate handle-based scenario objects before each simulation.
6. Keep scenario and sensor sample times compatible. A lidar sample time must be an integer multiple of the Scenario Configuration sample time.
7. Treat visualization as an observer, never as the plant. The Motion Write block updates scene state but does not integrate platform kinematics.
8. Give important signals stable names such as `State`, `LookaheadPoint`, `CrossTrackError`, `WaypointStatus`, `LidarPointCloud`, and `AvoidanceStatus`.

## Staged visualization rule

Use UAV Animation first when the purpose is to verify vehicle motion, attitude, trajectory shape, or waypoint sequencing. It requires only translation and rotation signals and avoids the scenario, mesh, sensor, and point-cloud workload.

Add UAV Scenario Configuration, Motion Write, sensors, and Scenario Scope only when the task requires environmental geometry, sensor simulation, or obstacle interaction. Do not run UAV Animation and UAV Scenario Scope together by default; enable both only when their separate views are genuinely useful.

## Build and verify

Before editing, inspect the relevant scope with `model_read` and resolve block IDs. After editing each scope, read it again and run `model_check`.

For behavioral verification, run a bounded mission and report at least:

- simulation completion time;
- final waypoint status and final position error;
- maximum cross-track error for waypoint missions;
- lidar output shape and nonempty returns near scene meshes;
- avoidance status and minimum obstacle clearance when avoidance is enabled.

Do not call a model collision-free merely because it reaches the last waypoint. Measure clearance against obstacle geometry or an occupancy representation.

## Baseline comparison rule

When adding obstacle avoidance or 6DOF dynamics, retain a simpler baseline model unless the user explicitly asks to replace it. The baseline and enhanced model should use the same mission and scene so differences are attributable to the added capability.


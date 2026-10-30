# UAV Animation for Lightweight Verification

Use the UAV Animation block as the first visual check of plant dynamics or waypoint following. It does not require `uavScenario`, platform objects, scene meshes, sensors, Motion Write, or UAV Scenario Scope.

## Appropriate tasks

- Verify that a step input moves or rotates the UAV in the expected direction.
- Check NED position and Euler-angle conventions visually.
- Inspect the shape and sequencing of a waypoint trajectory.
- Display waypoint or lookahead markers without simulating lidar or obstacles.

Do not use this block to validate obstacle clearance, sensor coverage, collision, or point-cloud alignment. Those tasks require UAV Scenario and sensor blocks.

## Block interface

### Translation

Supply an M-by-3 matrix, with one row per UAV. For a single vehicle, reshape the position to 1-by-3.

When the plant is Guidance Model, connect `State.WorldPosition` and set `Inertial frame z-axis direction` to `Down` because the state uses NED coordinates.

### Rotation

Choose one representation and match the signal exactly:

- `Euler ZYX`: M-by-3 rows `[yaw pitch roll]` in radians. Guidance Model `State.EulerZYX` can connect directly after reshaping to 1-by-3.
- `Quaternion`: M-by-4 rows. Use the quaternion convention produced by the selected conversion block and verify it with a known yaw rotation before relying on the display.

Do not reorder Guidance Model `EulerZYX` to `[roll pitch yaw]` for this block when its Rotation format is `Euler ZYX`.

### Optional Marker input

Enable `Draw marker` to show waypoints, the current lookahead point, or both.

Marker rows can contain:

- N-by-3: position;
- N-by-4: position and marker size;
- N-by-7: position, size, and RGB color;
- N-by-8: add marker type (`1` round, `2` square, `3` triangle);
- N-by-9: add filled status (`1` filled, `0` unfilled).

For a simple waypoint demonstration, pass the N-by-3 waypoint positions directly. Keep marker coordinates in the same inertial frame as Translation.

## Recommended parameters

- `Number of UAVs`: `1` for the basic demo.
- `UAV Type`: `Multirotor`.
- `Inertial frame z-axis direction`: `Down` for NED plants.
- `Rotation format`: `Euler ZYX` when using Guidance Model state directly.
- `Sample time`: use a visible but lightweight update rate such as 0.05 to 0.2 seconds. Increase it if rendering slows the simulation.
- Keep dynamic mesh configuration disabled unless the task needs multiple vehicle types or run-time mesh changes.

## Wiring patterns

### Dynamics or step-response visualization

```text
Guidance Model.State
  -> Bus Selector.WorldPosition -> row reshape -> UAV Animation.Translation
  -> Bus Selector.EulerZYX      -> row reshape -> UAV Animation.Rotation
```

Keep numerical plots as the primary step-response evidence. Animation verifies direction, attitude, and qualitative motion but does not replace response metrics.

### Waypoint-following visualization

```text
Guidance Model.State.WorldPosition -> Translation
Guidance Model.State.EulerZYX      -> Rotation
Waypoint matrix(:,1:3)             -> Marker
```

Optionally combine the waypoint markers with the current lookahead point using a consistent marker-row format.

## Validation

- Positive north, east, and down motion appear in the expected plot directions.
- Yaw, pitch, and roll rotations agree with known test commands.
- The animated path agrees with logged `WorldPosition` data.
- Waypoint markers are located on the expected path.
- The simulation remains materially faster than the scenario-and-lidar version.

After these checks pass, add UAV Scenario blocks only if the next task needs buildings, lidar, or obstacle interaction.

Official reference:

- https://www.mathworks.com/help/releases/R2025b/uav/ref/uavanimation.html


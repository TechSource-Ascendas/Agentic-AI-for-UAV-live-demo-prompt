# UAV Scenario, Motion, Lidar, and Visualization

Use this reference when adding a 3-D scene, platforms, meshes, lidar measurements, or point-cloud visualization. For basic dynamics or waypoint visualization without environmental sensing, use [uav-animation.md](uav-animation.md) first.

## Scenario objects

Create these objects in a setup script that can be rerun before every simulation:

1. `uavScenario` with explicit `UpdateRate`, `StopTime`, and `ReferenceLocation`.
2. Scene meshes with `addMesh`.
3. A `uavPlatform` with an explicit reference frame and initial pose.
4. A `uavLidarPointCloudGenerator`.
5. A `uavSensor` attaching the lidar model to the platform.

Assign the scenario and any handle objects needed by block masks to the base or model workspace expected by the model.

`addMesh` polygon footprints use ENU horizontal coordinates `[east north]`. Do not pass NED `[north east]` points without swapping the first two columns. Document vertical bounds and whether they represent `up` or `down`.

## UAV Scenario Configuration block

- Point the block to the `uavScenario` workspace variable.
- Use one and only one Scenario Configuration block in the model.
- Use a discrete sample time consistent with the scenario update rate.
- Refresh/apply the block after changing platforms or sensors so mask tables and ports match the scenario.

The Scenario Configuration block must execute before the other UAV Scenario blocks.

## UAV Scenario Motion Write block

Select the target platform and set its input coordinate frame explicitly.

Inputs are row vectors:

| Port | Shape | Convention |
|---|---:|---|
| Position | 1-by-3 | selected NED or ENU frame |
| Orientation | 1-by-4 | quaternion `[w x y z]` |
| Velocity | 1-by-3 | selected frame |
| Acceleration | 1-by-3 | selected frame |
| AngularVelocity | 1-by-3 | selected frame |

Use Reshape blocks when the plant emits column vectors. Convert `EulerZYX = [yaw pitch roll]` to the ordering required by the Euler-to-quaternion block, and verify the resulting quaternion with a simple known yaw before mission simulation.

Motion Write updates scenario state; it does not integrate velocity or acceleration. Feed it the state already computed by the plant.

Execution order must be:

```text
Scenario Configuration
  -> Motion Write
  -> Lidar / Motion Read / Scenario Scope
```

## UAV Scenario Lidar block

Select the sensor using its `platform/sensor` name. Keep block settings synchronized with the attached `uavLidarPointCloudGenerator`.

- Organized output on: N-by-M-by-3.
- Organized output off: N-by-3, which directly matches the Obstacle Avoidance block input.
- Lidar sample time must be a multiple of the Scenario Configuration sample time.
- Output points are expressed in the sensor frame, so mounting location and mounting orientation must match the avoidance block parameters.

Use a practical range and angular resolution for the scene scale. Dense 360-degree scans can dominate simulation time.

## UAV Scenario Scope block

- Refresh the lidar sensor table after changing the scenario.
- Enable visualization for the intended lidar sensor; the block then exposes the corresponding point-cloud input port.
- Connect the Lidar output directly to that scope input.
- Keep the scope sample time equal to the lidar sample time or another compatible rate.

Prefer UAV Scenario Scope when the task needs meshes and lidar in one view. Running a separate UAV Animation viewer at the same time can produce scene-tree rendering warnings on some desktop sessions; disable the redundant viewer for the cleanest demo.

## Validation

- Confirm that the platform moves along the expected NED path in the scope.
- Confirm that point returns appear on known meshes at plausible ranges.
- Test one known orientation to catch yaw/order inversions.
- Verify point-cloud dimensions before connecting planning blocks.

Official references:

- https://www.mathworks.com/help/uav/ref/uavscenarioconfiguration.html
- https://www.mathworks.com/help/uav/ref/uavscenariomotionwrite.html
- https://www.mathworks.com/help/uav/ref/uavscenariolidar.html
- https://www.mathworks.com/help/uav/ref/uavscenarioscope.html


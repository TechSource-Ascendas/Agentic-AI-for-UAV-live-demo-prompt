# Lidar-Based Obstacle Avoidance

Use this reference when inserting the UAV Toolbox Obstacle Avoidance block into an existing waypoint mission.

## Preserve the nominal guidance path

Keep the Waypoint Follower as the mission-level planner. Treat its lookahead point as the target for local avoidance. Add switches so avoidance can be disabled and the original lookahead/yaw commands pass through unchanged.

Recommended flow:

```text
WorldPosition --------------------------> Obstacle Avoidance.Position
Orientation quaternion ----------------> Obstacle Avoidance.Orientation
UAV Scenario Lidar point cloud --------> Obstacle Avoidance.ObstaclePoints
Waypoint Follower.LookaheadPoint ------> Obstacle Avoidance.TargetPosition

DesiredDirection * avoidanceLookaheadDistance
  + WorldPosition
  -> avoidance lookahead point

enable && flight-phase gate
  -> select avoidance or nominal lookahead
  -> select avoidance or nominal desired yaw
```

## Block interface

Inputs:

- `Position`: `[x;y;z]`, m, in a right-handed world frame.
- `Orientation`: quaternion `[w;x;y;z]` describing UAV orientation in that frame.
- `ObstaclePoints`: N-by-3, m, in the range-sensor frame.
- `TargetPosition`: `[x;y;z]`, m, in the same world frame as Position.

Outputs:

- `DesiredDirection`: unit vector in the selected world frame.
- `DesiredYaw`: rad in `[-pi,pi]`.
- `Status`: `0` free direction, `1` no free direction, `2` free but close, `3` no free direction and close.

## Parameter consistency

The following Obstacle Avoidance parameters must match the lidar and vehicle:

- sensor range limits;
- horizontal and vertical field-of-view limits;
- sensor mounting location;
- sensor mounting orientation `[roll pitch yaw]` in degrees;
- vehicle radius;
- minimum obstacle distance.

Use the same mounting transform in both the `uavSensor` setup and the block. A mismatch can rotate the perceived obstacle field even when the point cloud looks correct in the scenario scope.

For the VFH+ cost weights, make the target-direction weight greater than the sum of current-direction and previous-direction weights when target progress should dominate. Increase smoothness weights only after verifying that a collision-free direction can be found.

## Timing and gating

- Run lidar and avoidance at compatible discrete rates; use Rate Transition blocks when crossing rates.
- Gate avoidance during takeoff until the vehicle reaches a safe altitude or flight phase.
- Define a response for statuses `1` and `3`. Do not continue blindly toward the nominal target when no free direction exists; hover, stop, or invoke a higher-level replanner.

## Validation

Compare against the no-avoidance model using the same waypoint list and scene. Report:

- minimum distance to each obstacle, including vehicle radius;
- avoidance status history;
- whether the final waypoint was reached;
- additional path length and mission time;
- any oscillation or direction switching near obstacle edges.

Offset obstacles from the path during initial demonstrations so there is a visible, achievable bypass corridor. Move them onto the centerline only after the basic data path and coordinate transforms are proven.

Official reference:

- https://www.mathworks.com/help/uav/ref/obstacleavoidance.html


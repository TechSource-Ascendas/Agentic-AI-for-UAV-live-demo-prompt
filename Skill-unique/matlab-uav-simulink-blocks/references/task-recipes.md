# Task Recipes and Acceptance Checks

Use the smallest recipe that satisfies the request.

## A. Multirotor control-input step response

Blocks:

```text
Step sources -> Control Bus -> Guidance Model -> State Bus Selector -> logging/plots
Gravity constant -> Environment Bus -----------^
```

Acceptance:

- only the selected input changes;
- hover thrust is used when the test is meant to start from hover;
- all state channels have finite values;
- response axes and signs agree with NED conventions.

## B. Waypoint-following baseline

Blocks:

```text
State -> Pose adapter -> Waypoint Follower -> position/yaw controller
  -> Control Bus -> Guidance Model -> State
```

Acceptance:

- `Status` becomes `1`;
- final position error is within the requested tolerance;
- cross-track error is bounded;
- final hover behavior is correct.

For lightweight visual verification, add UAV Animation using `WorldPosition`, `EulerZYX`, and waypoint markers. Do not add UAV Scenario blocks yet.

## C. Scenario and lidar visualization

Add:

```text
Scenario Configuration
State adapters -> Motion Write
Scenario Lidar -> Scenario Scope
```

Acceptance:

- platform follows the simulated state rather than an independent trajectory;
- meshes appear at expected ENU/NED locations;
- lidar point cloud is nonempty near meshes and correctly oriented;
- no duplicate Scenario Configuration block exists.

## D. Waypoint following with reactive avoidance

Add between Waypoint Follower and controller:

```text
State + quaternion + lidar + nominal lookahead
  -> Obstacle Avoidance
  -> enabled selection of local lookahead and yaw
```

Acceptance:

- avoidance-disabled behavior matches recipe B;
- enabled run maintains measured clearance;
- no-free-direction states have an explicit safe response;
- mission completion is reported separately from collision clearance.

## E. Higher-fidelity 6DOF experiment

Use a separate model:

```text
force/moment sources -> 6DOF (Euler Angles) -> state conversion -> plots
```

Acceptance:

- initial trim is documented;
- one-axis force test has expected sign and units;
- mass and inertia are physically valid;
- no waypoint or scenario integration is attempted until the plant-level test passes.

## Demonstration hygiene

- Keep model variants clearly named, such as `*_no_obstacle_avoidance` and `*_with_obstacle_avoidance`.
- Save figures and results with variant-specific names so runs do not overwrite one another.
- If the user asks to show the live scene, bring UAV Scenario Scope forward after the run and leave it open.
- If the user asks only to verify dynamics or waypoint following, prefer UAV Animation and omit the scenario stack.
- If a visualization warning occurs, distinguish rendering failure from simulation failure using numerical outputs and diagnostics.


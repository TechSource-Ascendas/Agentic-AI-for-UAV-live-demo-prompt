# Guidance Model and Waypoint Follower

Use this reference for reduced-order multirotor dynamics, individual control-input step responses, and waypoint tracking.

## Guidance Model block

Set `ModelType` to `MultirotorGuidance`.

### Inputs

`Control` is a bus with these exact elements:

| Element | Meaning | Unit |
|---|---|---|
| `Roll` | commanded roll angle | rad |
| `Pitch` | commanded pitch angle | rad |
| `YawRate` | commanded yaw rate | rad/s |
| `Thrust` | total vertical thrust | N |

`Environment` is a bus containing `Gravity`, the inertial-frame z-axis gravitational acceleration in m/s^2. Match the sign to the NED convention and the block configuration.

### Output

The `State` bus contains:

- `WorldPosition`: `[north east down]`, m
- `WorldVelocity`: `[vNorth vEast vDown]`, m/s
- `EulerZYX`: `[yaw pitch roll]`, rad
- `BodyAngularRateRPY`: body angular rates, rad/s
- `Thrust`: N

Use Bus Selector blocks rather than indexing the state bus by assumed element order.

### Step-response task

Create one source per control bus element. Apply a step to one element at a time while holding the others at their trim values. For vertical tests, use the hover thrust as the baseline and step around it; a zero-thrust baseline is a free-fall test, not a hover response.

Log position, velocity, Euler angles, angular rates, and thrust. Keep the waypoint controller disabled during manual step-response tests.

## Waypoint Follower block

Set `UAV type` to `multirotor`.

### Inputs

- `Pose`: `[x y z chi]`, where position is in meters and `chi` is current course in radians. Do not silently substitute yaw for course when sideslip or nonzero lateral velocity matters. For the reduced-order demo, yaw is an acceptable approximation only when this assumption is documented.
- `Waypoints`: N-by-3 `[x y z]`; N-by-4 `[x y z yaw]` when yaw input is enabled; add a fifth transition-radius column when both yaw and external transition radius are enabled.
- `LookaheadDistance`: positive scalar in meters and not below the configured minimum.

### Outputs

- `LookaheadPoint`: `[x y z]`, m
- `DesiredCourse`: rad, `[-pi, pi]`
- `DesiredYaw`: rad, `[-pi, pi]`
- `LookaheadDistFlag`: `uint8`
- optional `CrossTrackError`: m
- optional `Status`: `uint8`, with `1` meaning all waypoints completed

For a multirotor that should stop at the final waypoint, enable `Hover at last waypoint`.

### Minimal waypoint controller

Use this signal flow:

```text
LookaheadPoint - WorldPosition
  -> position gain
  -> velocity limit
  -> subtract WorldVelocity
  -> velocity gain
  -> acceleration-to-roll/pitch/thrust mapping
  -> control limits
  -> Guidance Model Control bus

wrapToPi(DesiredYaw - CurrentYaw)
  -> yaw gain
  -> yaw-rate limit
  -> Control.YawRate
```

The acceleration-to-control mapping must respect NED signs and the Guidance Model's roll, pitch, and thrust conventions. Keep its matrix or gains in named model-workspace variables.

### Common mistakes

- Passing `[roll pitch yaw]` where `EulerZYX` `[yaw pitch roll]` is expected.
- Using altitude-positive-up waypoints directly in a NED model.
- Leaving the default transition radius much larger than a small indoor-scale mission.
- Failing to wrap yaw error across `-pi/pi`.
- Assuming the first waypoint is always visited; the follower can advance because of its transition-region and hyperplane rules.

Official references:

- https://www.mathworks.com/help/releases/R2025b/uav/ref/guidancemodel.html
- https://www.mathworks.com/help/releases/R2025b/uav/ref/waypointfollower.html


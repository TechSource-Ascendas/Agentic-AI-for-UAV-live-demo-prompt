# Demo 2: Guidance Model → Waypoint Follower → 3D UAV Scenario

## Objective

Build a UAV simulation in steps through conversation with the agent. Each step extends the same Simulink model (`uav_multirotor_step_response.slx`), and the agent verifies each step with numbers before moving on:

1. **Guidance Model**: reduced-order multirotor (NED) with a step response for each control input
2. **Waypoint Follower**: follow a 5 m square while climbing to 2 m, shown with the UAV Animation block
3. **UAV Scenario**: place the same vehicle and mission in a 3D scene (Scenario Configuration → Motion Write → Scenario Scope)
4. **Obstacle**: add a square building across the flight path. This prepares the obstacle-avoidance step and shows that *waypoint completion ≠ collision-free*

## Prompts used

| # | Prompt | Key verified result |
|---|---|---|
| 1 | *Could you help me check if MATLAB MCP is available?* | MATLAB R2025b U5, MCP Server Toolbox 0.4.0 |
| 2 | `$matlab-uav-simulink-blocks` *Could you help me create a simple UAV multirotor guidance model and simulate the step response for each control input?* | Roll/pitch 5°: rise 0.67 s, settle 2.17 s. Yaw-rate/thrust: rise 0.28 s, settle 1.49 s. 0 % overshoot |
| 3 | *Please continue to develop "uav_multirotor_step_response". I would like to add a waypoint follower to this guidance model and simulate the UAV following a simple waypoint path. You can visualize it using the UAV Animation block.* | Status = 1, done at 25.81 s, final error 7e-5 m, max cross-track 0.50 m |
| 4 | `$matlab-uav-simulink-blocks` *Now I would like to put this guidance model and waypoints into a scene with the UAV Scenario block. Could you make a simple one?* | Same mission metrics inside UAV Scenario. Animation kept but disabled |
| 5 | *Could you try adding a square building to obstruct the flight path? Later we can use it for implementing obstacle avoidance.* | Status = 1, **but** path intersects building, min clearance 0 m |

`$matlab-uav-simulink-blocks` explicitly invokes the custom skill. The full transcript is in [`outputs/Guidance_Waypoint_3Dscenario_conversation.md`](outputs/Guidance_Waypoint_3Dscenario_conversation.md).

## Requirements and dependencies

| Item | Used for |
|---|---|
| MATLAB R2025b + Simulink | Model build and simulation (`Simulink.SimulationInput`, `sim`) |
| UAV Toolbox | Guidance Model, Waypoint Follower, UAV Animation, UAV Scenario Configuration / Motion Write / Scope, `uavScenario`, `uavPlatform`, `addMesh` |
| Control System Toolbox | `stepinfo` (rise and settling time) |
| MATLAB MCP Server Toolbox 0.4.0 | MATLAB execution and the Simulink model tools (`model_read`, `model_edit`, `model_check`) |

## Skills used

| Skill | Role |
|---|---|
| **`matlab-uav-simulink-blocks`** (custom, see [`../Skill-unique`](../Skill-unique/)) | Block-level contracts for UAV blocks: NED frames, `[yaw pitch roll]` order, `[w x y z]` quaternions, staged visualization (Animation first, Scenario only when needed), acceptance checks per recipe |
| `building-simulink-models` | Required companion for every structural or parameter edit |
| `simulating-simulink-models` | Runs with parameter overrides, controller-mode selection and result retrieval |

## Files (`outputs/`)

| File | Description |
|---|---|
| `uav_multirotor_step_response.slx` | Simulink model with step-response, waypoint and scenario modes |
| `run_uav_multirotor_steps.m` | Step 2: step-response runner → `uav_multirotor_step_*` (.png/.mat/.csv) |
| `run_uav_waypoint_mission.m` | Step 3: waypoint mission → `uav_waypoint_mission_*` |
| `create_simple_uav_scenario.m`, `run_uav_scenario_mission.m` | Step 4: clear scene → `uav_scenario_mission_*` |
| `create_obstructed_uav_scenario.m`, `run_uav_scenario_obstructed_mission.m` | Step 5: building scene → `uav_scenario_obstructed_*` |
| `Guidance_Waypoint_3Dscenario_conversation.md` | Prompt and response transcript |

## Next step

Add reactive **obstacle avoidance** (Scenario Lidar → Obstacle Avoidance block, recipe D in the skill). Run it against the obstructed scene, keep the current model as the baseline, and report minimum clearance separately from mission completion.

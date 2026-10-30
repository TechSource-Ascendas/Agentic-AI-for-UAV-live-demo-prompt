# Custom Skill: `matlab-uav-simulink-blocks` (recommended setup)

## Why this skill

General-purpose agents often get UAV block interfaces subtly wrong: NED vs ENU, `[yaw pitch roll]` vs `[roll pitch yaw]`, quaternion order, or row vs column vectors at Scenario ports. This skill gives the agent tested **block-level contracts** and **acceptance checks**, so a model is built correctly on the first pass and verified with numbers rather than by eye.

It is used in [Demo 2: Guidance-Waypoint-3DScenario](../Guidance-Waypoint-3DScenario/).

## What's inside

```text
matlab-uav-simulink-blocks/
├── SKILL.md                      ← routing, shared invariants, build/verify rules
└── references/
    ├── guidance-waypoint.md      ← Guidance Model, step response, Waypoint Follower
    ├── uav-animation.md          ← lightweight visual verification
    ├── scenario-lidar.md         ← UAV Scenario, meshes, lidar, point clouds
    ├── obstacle-avoidance.md     ← lidar-based reactive avoidance
    ├── sixdof.md                 ← higher-fidelity 6DOF dynamics
    └── task-recipes.md           ← recipes A–E with acceptance checks
```

Key rules the skill enforces:

- State the coordinate frame at every boundary (Guidance Model = NED, altitude = `-down`)
- Use UAV Animation first, and add UAV Scenario only when geometry or sensors are needed
- Recreate handle-based scenario objects before every simulation
- Report completion time, waypoint status, final error, cross-track error and clearance
- Never call a path collision-free just because it reached the last waypoint
- Keep a baseline model when adding avoidance or 6DOF

## Setup

1. Copy the `matlab-uav-simulink-blocks/` folder into your agent's skills directory:

   | Agent host | Location |
   |---|---|
   | OpenAI Codex (used in the recording) | `~/.agents/skills/matlab-uav-simulink-blocks/` |
   | Claude Code | `~/.claude/skills/matlab-uav-simulink-blocks/` (user) or `<project>/.claude/skills/` |

2. Make sure the companion skills are installed as well. `SKILL.md` requires them:
   - `building-simulink-models`
   - `simulating-simulink-models`
3. Make sure the MATLAB MCP Server is connected (MATLAB R2025b, Simulink, UAV Toolbox).
4. Invoke it explicitly in a prompt (for example `$matlab-uav-simulink-blocks …` in Codex), or let the agent pick it up from the description for UAV block tasks.

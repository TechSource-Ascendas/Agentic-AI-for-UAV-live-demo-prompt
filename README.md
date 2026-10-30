# UAV Development with Agentic AI + MATLAB MCP

Two short demos where an AI coding agent drives MATLAB and Simulink through the **MATLAB MCP Server**. Engineers describe what they want in plain language. The agent then builds the model, runs it, and reports back verified numbers.

> 🎬 **Reference video:** [Agentic AI with UAV - Live demo prompt.mp4](https://techsource-my.sharepoint.com/:v:/g/personal/pattarapol_ascendas-asia_com/IQBRsCB0G0JHRJRRCPEJMq9nAUpRST41OJ9GNdEiidxSVWQ?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJPbmVEcml2ZUZvckJ1c2luZXNzIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJNeUZpbGVzTGlua0NvcHkifX0&e=LTaROp). This recording walks through the live prompts for both demos. Watch it alongside the prompt tables in each demo folder.

| Demo | Folder | What it shows |
|---|---|---|
| 1. Flight log analysis | [`Flight-Log-Analysis/`](Flight-Log-Analysis/) | Read a real PX4 MAVLink `.tlog`, get a flight overview and anomaly diagnosis, then produce a playback video |
| 2. Guidance → Waypoint → 3D Scenario | [`Guidance-Waypoint-3DScenario/`](Guidance-Waypoint-3DScenario/) | Build a multirotor Guidance Model step by step, add a waypoint follower, animate it, place it in a UAV Scenario, and add an obstacle |

Supporting material:

- [`Skill-unique/`](Skill-unique/): a custom agent skill (`matlab-uav-simulink-blocks`) used in Demo 2. **Recommended to install** before running the demo.
- `UAV_Agentic_AI_Workflow_Writeup.docx`: the customer-facing write-up (challenges, objectives, workflow).

---

## Objective

Show that one repeatable, agent-assisted workflow can connect the UAV development loop from start to finish:

1. **Create** a simple guidance model for navigation experiments
2. **Add** path-following logic (waypoint follower)
3. **Simulate** the UAV in a 3D scenario with obstacle geometry
4. **Log and analyze** flight behavior (simulated or real flight logs)

The point is not that "AI writes code". The agent works inside the real MATLAB/Simulink session, checks its own results (step-response metrics, waypoint status, clearance, model checks) and keeps every artifact so engineers can reproduce it.

---

## Requirements and dependencies

### Software (as used in the recorded sessions)

| Item | Version / note |
|---|---|
| MATLAB | R2025b Update 5 |
| Simulink | Required for Demo 2 |
| UAV Toolbox | Both demos (`mavlinktlog`, `mavlinkdialect`, Guidance Model, Waypoint Follower, UAV Animation, `uavScenario`, Scenario blocks) |
| Control System Toolbox | Demo 2 (`stepinfo` for rise/settling time) |
| MATLAB MCP Server Toolbox | 0.4.0. Exposes MATLAB execution and the Simulink model tools (`model_read`, `model_edit`, `model_check`, …) to the agent |
| AI agent host | Any MCP-capable coding agent. The recordings used OpenAI Codex; Claude Code or similar also work |

### Agent skills

| Skill | Source | Used in |
|---|---|---|
| `matlab-uav-simulink-blocks` | **Custom**, in [`Skill-unique/`](Skill-unique/) | Demo 2 |
| `building-simulink-models` | MathWorks agentic skills (model-based design core) | Demo 2: required companion for every model edit |
| `simulating-simulink-models` | MathWorks agentic skills (model-based design core) | Demo 2: parameterized runs and result retrieval |
| *(none)* | MATLAB MCP + UAV Toolbox only | Demo 1 |

### Quick setup

1. Install MATLAB R2025b with Simulink, UAV Toolbox and Control System Toolbox.
2. Install and register the MATLAB MCP Server in your agent host.
3. Copy `Skill-unique/matlab-uav-simulink-blocks/` into your agent's skills folder (see [`Skill-unique/README.md`](Skill-unique/README.md)).
4. Open the agent and run the first prompt: *"Check if MATLAB MCP is available"*.

> **Note:** All scripts resolve paths relative to their own location (`fileparts(mfilename("fullpath"))`), so you can run them from any clone location without editing them.

---

## Repository layout

```text
Github-Checkout/
├── README.md                              ← this file
├── UAV_Agentic_AI_Workflow_Writeup.docx   ← customer-facing write-up
├── Flight-Log-Analysis/
│   ├── README.md
│   ├── outputs/   (Sample_Log.tlog, playback MP4 + preview, conversation transcript)
│   └── work/      (create_uav_playback.m)
├── Guidance-Waypoint-3DScenario/
│   ├── README.md
│   └── outputs/   (Simulink model, runner scripts, results .mat/.csv/.png, transcript)
└── Skill-unique/
    ├── README.md
    └── matlab-uav-simulink-blocks/  (SKILL.md + references/)
```

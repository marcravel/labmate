# AGENTS.md — AI Agent Development Guidelines

**LabMate · RobotHack 2026**

This document defines persistent rules and reference information for AI coding agents working on the LabMate repository. Read this file before making any change. It is the authoritative operational guide for automated development on this codebase.

---

## A. Project Identity

LabMate is a **simulation-based laboratory robotics application** for the Booster K1 EDU humanoid robot.

The project runs the **official Booster K1 walking policy** (`k1_walk.pt`) unmodified inside **MuJoCo 4.0.1**. LabMate adds an application layer on top: autonomous waypoint navigation, (future) inspection logic, (future) backend API, and (future) dashboard.

The codebase has two distinct parts:

1. **Official Booster Deploy framework** (`external/booster_deploy/`, `external/booster_assets/`) — third-party code. Treat as a protected dependency.
2. **Custom LabMate application layer** (`robot_control/`, `run_labmate.py`) — the only code agents should write or modify during normal development.

The working system consists of these custom files on top of Booster Deploy:

| File | Role |
|---|---|
| `run_labmate.py` | Entry point. Wiring only. |
| `robot_control/labmate_controller.py` | Controller subclass. Hook points only. |
| `robot_control/velocity_interface.py` | The only sanctioned path for velocity commands. |
| `robot_control/waypoint_navigator.py` | Patrol state machine. |
| `robot_control/route_graph.py` | Navigation graph read from the MJCF. Owns `ROUTE_EDGES`. |
| `shared/task_cfg.py` | Registers `labmate_inspection`; overrides `robot.mjcf_path`. |
| `shared/inspection.py` | The inspection checklist. Owns `STATIONS` and `STATION_ORDER`. |
| `shared/scenario.py` | Applies a scenario JSON to the model. Lab visual geoms only. |
| `shared/inspection_report.py` | Collects verdicts; renders the round report and JSON. |
| `simulation/scenarios/*.json` | Lab state per run: `nominal`, `demo_findings`, `all_faults`. |
| `simulation/scenes/labmate_scene.xml` | Scene entry point. This is the model that loads. |
| `simulation/scenes/lab_environment.xml` | The laboratory: static bodies, geoms, nav/inspect sites. |
| `tests/check_scene.py` | Static geometric validation of the scene. |
| `tests/test_route_graph.py` | Route planning stays on validated edges. |
| `tests/test_inspection.py` | Checklist matches the scene; every check can pass and fail. |

`backend/`, `dashboard/`, `perception/`, `simulation/models/` are still empty. Do not assume any planned module exists — verify first.

---

## B. Critical Constraints

These constraints must not be violated under any circumstances.

1. **Preserve the official K1 walking policy.** `k1_walk.pt` and all code that loads and runs it must remain unchanged. Never replace it with a hand-written locomotion controller.
2. **Do not modify `external/booster_deploy/`.** No file inside this directory may be changed. This includes `LocomotionPolicy`, `MujocoController`, `BaseController`, `K1WalkTaskCfg`, `K1_CFG`, and `policy_runner.py`.
3. **Do not modify `external/booster_assets/`.** Robot model files (`K1_22dof.xml`, etc.) and the `booster_assets` Python package must not be edited.
4. **Do not change simulation physics parameters** (`physics_dt`, `decimation`, `policy_dt`, PD gains, effort limits) without explicit authorization from a human developer.
5. **Do not duplicate existing functionality.** Check whether `velocity_interface.py`, `waypoint_navigator.py`, or `labmate_controller.py` already provide what is needed before writing new code.
6. **Do not introduce unnecessary dependencies.** Core runtime dependencies are MuJoCo, PyTorch, and NumPy. Add new packages only when strictly required and document them.
7. **Do not perform large architectural refactoring during the hackathon.** Extend the existing hook-and-callback pattern. Do not restructure the inheritance hierarchy or the main control loop.
8. **Do not change existing public interfaces** (`set_velocity`, `stop_robot`, `get_robot_pose`, the navigator callable signature) unless a change is necessary, explicitly requested, and all callers are updated.
9. **Do not assume a planned subsystem is implemented.** `backend/`, `dashboard/`, `perception/` and `simulation/models/` are empty. Verify before referencing.
10. **Do not commit or push changes** unless a human developer explicitly requests it.
11. **Never place a LabMate file inside `external/`.** `.gitignore` excludes `external/`, so anything put there is untracked and will be lost on a re-clone or cleanup. The laboratory scene lives in `simulation/scenes/`; load it by overriding `robot.mjcf_path` in a LabMate config subclass, never by editing a Booster file.
12. **Run `tests/check_scene.py` after any change to `simulation/scenes/`.** It is the guard on the project's central invariant — that the laboratory adds no robot state, so the official policy still sees the model it was trained on. Never weaken its equality assertions to make a change pass.
13. **Do not hand-write waypoint coordinates.** Routes come from `RouteGraph`, which reads the model's own sites. A hand-typed route once put a waypoint inside the central table. If a new position is needed, add a site to the MJCF and an edge to `ROUTE_EDGES`, then re-run both test scripts.
14. **Do not remove the closed-loop heading correction in the APPROACH phase.** It was verified that without it the robot walks off course, misses waypoints and pushes into a wall indefinitely.
15. **A scenario may only change the laboratory's visual geoms.** `shared/scenario.py` refuses the robot model (protecting the walking policy) and every collision geom (protecting the route clearances `tests/check_scene.py` swept once). Do not relax either guard. If an inspection needs a physical obstacle rather than a visible one, the route must be re-swept.
16. **An inspection check must be able to both pass and fail.** `tests/test_inspection.py` asserts this across the scenario set. A check that reads a geom no scenario ever changes reports PASS forever, which is worse than having no check at all. Add the fault state to a scenario in the same change as the check.
17. **Inspection checks read the simulation, never the scenario file.** A scenario arranges the lab; checks then observe it, exactly as a camera-based detector would. Do not shortcut a verdict by reading the scenario's declared state — that would make every check tautologically true and defeat the later swap to real perception.
18. **Do not present a scenario's `panel` readings as measurements.** They are labels for the state the scenario set up, not simulated physics. Reports must keep saying so.

---

## C. Source-of-Truth Map

This table identifies the actual file, class, and function responsible for each system responsibility. Do not invent alternatives.

| Responsibility | File | Class / Function | Key Parameters |
|---|---|---|---|
| **Application entry point** | `run_labmate.py` | `main()` | Wiring only. The route comes from `RouteGraph`; no coordinates are written here. |
| **Task registration** | `external/booster_deploy/booster_deploy/utils/registry.py` | `register_task()` / `get_task()` | `name="k1_walk"` (stock), `name="labmate_inspection"` (LabMate) |
| **LabMate task config** | `shared/task_cfg.py` | `LabMateInspectionTaskCfg` | Overrides only `robot.mjcf_path` → `LAB_SCENE_PATH`. Everything else inherited. |
| **Scene composition** | `simulation/scenes/labmate_scene.xml` | — | Includes `K1_22dof.xml` first, then `lab_environment.xml`; re-points `meshdir`. Robot must come first. |
| **Laboratory geometry** | `simulation/scenes/lab_environment.xml` | — | 15 static bodies, 197 geoms, 23 sites. `lab_vis` = visual, `lab_col` = collision. No joints, no DOF. |
| **Navigation graph** | `robot_control/route_graph.py` | `RouteGraph.__init__()` | Reads `nav_*` / `inspect_*` sites from `mj_model`. Heading from the site quaternion. |
| **Route connectivity** | `robot_control/route_graph.py` | `ROUTE_EDGES` | 16 undirected edges. **Single source of truth** — `tests/check_scene.py` imports it. |
| **Route planning** | `robot_control/route_graph.py` | `plan()` / `route()` / `patrol_route()` | BFS. `DEFAULT_STATION_ORDER` gives the perimeter round. |
| **Scene validation** | `tests/check_scene.py` | — | Robot arrays unchanged; site and route clearance. Run after any scene change. |
| **Route validation** | `tests/test_route_graph.py` | — | Generated routes use only validated edges and model positions. |
| **Task configuration** | `external/booster_deploy/tasks/locomotion/robots/k1/__init__.py` | `K1WalkTaskCfg` | `vel_command`, `policy`, `robot` sub-configs |
| **K1 joint specification** | `external/booster_deploy/booster_deploy/robots/k1.py` | `K1_CFG` (RobotCfg) | 22 joint names, stiffness, damping, effort limits, `mjcf_path` |
| **MuJoCo model loading** | `external/booster_deploy/booster_deploy/controllers/mujoco_controller.py` | `MujocoController.__init__()` | `cfg.robot.mjcf_path` → `simulation/scenes/labmate_scene.xml` |
| **Physics initialization** | `external/booster_deploy/booster_deploy/controllers/mujoco_controller.py` | `MujocoController.__init__()` | `physics_dt=0.002 s`, `decimation=10`, `init_pos=[0,0,0.6]` |
| **State read (MuJoCo → tensors)** | `external/booster_deploy/booster_deploy/controllers/mujoco_controller.py` | `MujocoController.update_state()` | reads `mj_data.qpos`, `mj_data.qvel` → `robot.data` |
| **Policy checkpoint loading** | `external/booster_deploy/tasks/locomotion/locomotion.py` | `LocomotionPolicy.__init__()` | `cfg.checkpoint_path`, `use_actor_module=True`, `device="cpu"` |
| **TorchScript loading** | `external/booster_deploy/booster_deploy/utils/policy_runner.py` | `TorchScriptRunner.__init__()` | `torch.jit.load(path).actor` |
| **Observation construction** | `external/booster_deploy/tasks/locomotion/locomotion.py` | `LocomotionPolicy.compute_observation()` | 69-dim vector; `obs_dof_vel_scale=0.1` |
| **Observation history** | `external/booster_deploy/tasks/locomotion/locomotion.py` | `LocomotionPolicy.inference()` | `actor_obs_history_length=10` → 690-dim input |
| **Policy inference** | `external/booster_deploy/tasks/locomotion/locomotion.py` | `LocomotionPolicy._forward_model()` | `obs_history.flatten()` → 20-dim action |
| **Action → joint targets** | `external/booster_deploy/tasks/locomotion/locomotion.py` | `LocomotionPolicy._action_to_targets()` | `action_scale=0.25`, `action_filter=0.8` |
| **PD actuator control** | `external/booster_deploy/booster_deploy/controllers/mujoco_controller.py` | `MujocoController.ctrl_step()` | PD from `K1_CFG`; 10 physics steps per policy step |
| **Main simulation loop** | `external/booster_deploy/booster_deploy/controllers/mujoco_controller.py` | `MujocoController.run()` | `update_state → policy_step → ctrl_step` at 50 Hz |
| **Navigator injection** | `robot_control/labmate_controller.py` | `LabMateController.set_navigator()` | stores callable `navigator(controller, elapsed)` |
| **Navigator callback call** | `robot_control/labmate_controller.py` | `LabMateController.policy_step()` | calls `self.navigator(self, elapsed)` before `super().policy_step()` |
| **Inspection checklist** | `shared/inspection.py` | `STATIONS`, `STATION_ORDER` | 5 stations, 12 checks. **Single source of truth** for what is inspected and in what order; `route_graph.DEFAULT_STATION_ORDER` imports `STATION_ORDER`. |
| **Check primitives** | `shared/inspection.py` | `IndicatorsNominal`, `PoseUnchanged`, `HeightAtMost`, `Absent`, `Present` | Each reads `mj_model` geom material / pose / rgba. |
| **Checklist validation** | `shared/inspection.py` | `validate(model)` | Called at startup by `run_labmate.py`; fails loudly if a station site or state geom is missing. |
| **Scenario load / apply** | `shared/scenario.py` | `load()`, `apply()`, `_check_writable()` | Refuses robot geoms and collision geoms. `DEFAULT_SCENARIO = "nominal"`. |
| **Inspection reporting** | `shared/inspection_report.py` | `InspectionRun.record()` / `render()` / `to_dict()` | `record` is the navigator's `on_inspect` callback. |
| **Inspection hook** | `robot_control/waypoint_navigator.py` | `WaypointNavigator.__init__(on_inspect=...)` | Fires in FACE_STATION once the robot has stopped and turned to face the station. Optional: the patrol runs without it. |
| **Inspection validation** | `tests/test_inspection.py` | — | Scenario verdicts, pass/fail coverage of every check, scenario write guards. |
| **Stdin override (disabled)** | `robot_control/labmate_controller.py` | `LabMateController.update_vel_command()` | `pass` — intentionally a no-op |
| **Velocity command write** | `robot_control/velocity_interface.py` | `set_velocity(controller, vx, vy, yaw)` | Clamps to `vx_forward_max=1.6`, `vx_backward_max=0.3`, `vy_max=0.3`, `vyaw_max=1.8` |
| **Stop robot** | `robot_control/velocity_interface.py` | `stop_robot(controller)` | Zeros all three velocity axes |
| **Read robot pose** | `robot_control/velocity_interface.py` | `get_robot_pose(controller)` | Returns `(pos_np[3], quat_np[4])` from `robot.data.root_pos_w / root_quat_w` |
| **Waypoint patrol logic** | `robot_control/waypoint_navigator.py` | `WaypointNavigator.update(controller, elapsed)` | `pos_tol=0.25 m`, `yaw_tol=0.20 rad`, `pause_duration=3.0 s` |
| **Approach heading control** | `robot_control/waypoint_navigator.py` | `WaypointNavigator.update()`, APPROACH phase | `heading_gain=1.1`, `realign_tol=0.35 rad`, `yaw_rate_max=0.45 rad/s`. Required: without it the robot walks off course and never recovers. |
| **Velocity command object** | `external/booster_deploy/booster_deploy/controllers/base_controller.py` | `VelocityCommand` | `lin_vel_x`, `lin_vel_y`, `ang_vel_yaw` (float attributes) |
| **Robot state tensors** | `external/booster_deploy/booster_deploy/controllers/base_controller.py` | `RobotData` | `joint_pos`, `joint_vel`, `root_pos_w`, `root_quat_w`, `root_ang_vel_b` |
| **Configuration dataclasses** | `external/booster_deploy/booster_deploy/controllers/controller_cfg.py` | Multiple `@configclass` | `ControllerCfg`, `PolicyCfg`, `RobotCfg`, `VelocityCommandCfg`, `MujocoControllerCfg` |
| **Perception** | `perception/` | — | **EMPTY — not implemented** |
| **Backend** | `backend/` | — | **EMPTY — not implemented** |
| **Dashboard** | `dashboard/` | — | **EMPTY — not implemented** |
| **Shared config** | `shared/` | — | **EMPTY — not implemented** |
| **MuJoCo MJCF model** | `external/booster_assets/robots/K1/K1_22dof.xml` | — | 22-DOF K1; loaded at MujocoController init |
| **Policy checkpoint** | `external/booster_deploy/tasks/locomotion/robots/k1/models/k1_walk.pt` | — | TorchScript; `.actor` submodule used |

---

## D. Robot Control Rules

### Ownership

- **`MujocoController`** (Booster Deploy) owns the simulation loop, physics stepping, and PD actuator control.
- **`LocomotionPolicy`** (Booster Deploy) owns observation construction, history management, and policy inference.
- **`LabMateController`** (LabMate) owns the application-level hook points: navigator injection and stdin suppression.
- **`WaypointNavigator`** (LabMate) owns waypoint-level decision making and translates navigation decisions into velocity commands.
- **`velocity_interface.py`** (LabMate) is the only sanctioned path for writing velocity commands.

### How high-level commands enter the control pipeline

```
Application logic (navigator, future perception, future backend)
    ↓
set_velocity(controller, vx, vy, yaw)          # velocity_interface.py
    ↓  clips to hardware limits
VelocityCommand.lin_vel_x / lin_vel_y / ang_vel_yaw
    ↓  read each policy step
LocomotionPolicy.compute_observation()          # command_obs tensor [3]
    ↓  packed into 69-dim obs → 690-dim history
k1_walk.pt actor network                        # inference
    ↓  20-dim action
MujocoController.ctrl_step()                    # 10× PD @ 500 Hz
    ↓
MuJoCo joint actuators
```

### Which interfaces must be preserved

| Interface | Type | Rule |
|---|---|---|
| `set_velocity(controller, vx, vy, yaw)` | Function | Must remain the sole entry point for velocity commands. Do not bypass. |
| `stop_robot(controller)` | Function | Must zero all three axes. Do not change semantics. |
| `get_robot_pose(controller)` | Function | Must return `(pos_np, quat_np)` as NumPy arrays. |
| `WaypointNavigator.update(controller, elapsed)` | Callable signature | Must accept exactly these two arguments. |
| `LabMateController.set_navigator(fn)` | Method | Must accept a single callable. |

### Velocity command limits (do not exceed)

| Axis | Forward | Backward | Lateral | Yaw |
|---|---|---|---|---|
| **Limit** | 1.6 m/s | −0.3 m/s | ±0.3 m/s | ±1.8 rad/s |
| **Source** | `K1WalkControllerCfg.vel_command` | | | |

`set_velocity()` enforces these limits internally. Never write to `vel_command` attributes directly.

### Safety constraint

`enable_safety_fallback=True` is active. If the projected gravity z-component exceeds −0.5 (robot falling), `LocomotionPolicy` calls `controller.stop()`, which sets `is_running=False` and terminates the viewer. There is no recovery mechanism. Agents must not disable this safeguard.

### What agents must NOT do with the control pipeline

- Do not call `policy.inference()` from application code.
- Do not call `ctrl_step()` from application code.
- Do not write to `robot.data.joint_pos`, `robot.data.root_pos_w`, or any other `RobotData` field from application code.
- Do not add non-trivial logic inside `LabMateController.update_vel_command()`.
- Do not subclass `LocomotionPolicy` or `MujocoController` for application purposes (use `LabMateController`).

---

## E. Development Protocol

Before changing any code, follow this sequence:

1. **Inspect** the relevant files. Read the full source, not just function signatures.
2. **Identify the source of truth** using the table in Section C.
3. **Understand the calling sequence** — trace from `run_labmate.py` → `LabMateController.run()` → the target code.
4. **Identify the smallest possible change.** Prefer adding new functions over modifying existing ones. Prefer new files in the correct module directory over touching existing files.
5. **Check for existing functionality.** Consult Section C before writing something new.
6. **Implement the change** in the correct location per Section F.
7. **Run relevant tests.** Until `tests/` is populated, verify manually by running `python run_labmate.py` and confirming the robot navigates correctly.
8. **Report** per Section G.

### Incremental changes only

Do not submit changes that touch more than one concern at a time. If a task requires both a new backend endpoint and a new perception function, implement and report them as separate changes.

---

## F. Modification Boundaries

### Core LabMate application code — agents write here

```
robot_control/labmate_controller.py   ← Hook points only. Add new hooks cautiously.
robot_control/velocity_interface.py   ← Add helper functions. Do not change existing ones.
robot_control/waypoint_navigator.py   ← Extend navigation logic. Keep callable signature.
robot_control/route_graph.py          ← Add nodes/edges to match the MJCF. ROUTE_EDGES is
                                        imported by tests/check_scene.py — keep them in sync.
shared/task_cfg.py                    ← Config overrides via .replace(). Add shared schemas here.
shared/inspection.py                  ← The checklist. Adding a station or check here requires a
                                        matching fault state in a scenario, or test_inspection.py
                                        fails: a check that never fails is not a check.
shared/scenario.py                    ← Scenario loading. Do not relax the robot/collision guards.
shared/inspection_report.py           ← Report rendering and JSON export. Formatting only.
simulation/scenarios/                 ← Lab state per run. Visual geoms only. Re-run
                                        tests/test_inspection.py after every edit.
simulation/scenes/                    ← The laboratory MJCF. Re-run tests/check_scene.py after
                                        every edit. Keep lab bodies static (no joints, no DOF).
                                        Inspectable state is a geom's material, pose or
                                        visibility — never a joint.
run_labmate.py                        ← Wiring only. Keep the main() structure clean.
                                        No coordinates, no route literals.
tests/                                ← Standalone scripts in the style of check_scene.py.
                                        pytest is not installed; do not add it casually.
backend/                              ← Create new FastAPI application files here.
dashboard/                            ← Create new Streamlit files here.
perception/                           ← Create new perception modules here.
simulation/models/                    ← Add custom robot model variants here.
```

### External protected dependencies — agents do NOT modify these

```
external/booster_deploy/              ← READ-ONLY. Official Booster locomotion framework.
external/booster_assets/              ← READ-ONLY. Official robot models and motion data.
```

If a Booster Deploy behavior needs to change, the correct approach is to override it in `LabMateController` using Python's inheritance mechanism, as `update_vel_command()` already demonstrates.

### Configuration — handle with care

```
external/booster_deploy/booster_deploy/controllers/controller_cfg.py   ← DO NOT MODIFY
external/booster_deploy/tasks/locomotion/robots/k1/__init__.py          ← DO NOT MODIFY
external/booster_deploy/booster_deploy/robots/k1.py                     ← DO NOT MODIFY
```

If a configuration value must change (e.g., velocity limits, spawn position), do it by creating a new `@configclass` subclass in LabMate code that uses `K1WalkTaskCfg` as a base, using the `.replace(...)` pattern already established in `K1WalkControllerCfg`.

### Generated and excluded files — do not create or track

```
.venv/              ← Virtual environment. Never commit.
__pycache__/        ← Python bytecode. Never commit.
*.pt / *.pth / *.onnx  ← Model checkpoints. Never commit (in .gitignore).
external/           ← Booster repos. Never commit (in .gitignore).
```

---

## G. Communication and Reporting

After every implementation task, provide a report with the following sections:

### Required report structure

```
## Implementation Report

### Changes made
- <file path>: <brief description of what changed and why>

### Calling sequence affected
- <describe which functions now call or depend on the new code>

### Interfaces affected
- <list any public function signatures added or changed>

### Verification
- <describe how the change was verified: command run, output observed>
- <if tests exist: which tests passed>

### Remaining work
- <list what this task did NOT complete>
```

If a task could not be completed (missing dependency, empty directory, blocked by another module), report the blocker explicitly rather than leaving partial changes.

---

## H. Documentation Maintenance

After any implementation change:

| Condition | Required action |
|---|---|
| New entry point, new CLI command, or changed setup step | Update **README.md §F (Running the Application)** |
| New subsystem becomes implemented | Update **README.md §H (Implementation Status)** table |
| Architecture or communication flow changes | Update **README.md §B (System Architecture)** and Mermaid diagram |
| Source-of-truth ownership changes | Update **AGENTS.md §C (Source-of-Truth Map)** |
| New constraint on agent behavior | Update **AGENTS.md §B (Critical Constraints)** |
| New safe/unsafe modification boundary | Update **AGENTS.md §F (Modification Boundaries)** |
| New required Python package | Update **README.md §D (Technology Stack)** and §E (Installation) |

Do not update documentation to describe planned features as implemented. The status table in `README.md §H` must reflect the actual state of the code.

---

## Quick Reference

### The only entry point

```bash
source .venv/bin/activate
python run_labmate.py                            # nominal: all stations pass
python run_labmate.py --scenario demo_findings   # the two scripted findings
python run_labmate.py --scenario all_faults --report round.json
```

### Validation before reporting any change

```bash
python tests/test_route_graph.py   # fast
python tests/test_inspection.py    # fast; required after any checklist or scenario edit
python tests/check_scene.py        # minutes; required after any scene edit
```

### The only way to issue velocity commands

```python
from robot_control.velocity_interface import set_velocity, stop_robot
set_velocity(controller, vx=0.3, vy=0.0, yaw=0.0)
stop_robot(controller)
```

### The only hook for high-level application logic

```python
# In run_labmate.py:
controller.set_navigator(my_callable)
# my_callable must have signature: fn(controller, elapsed_time: float) -> None
```

### Key file locations

| What | Where |
|---|---|
| K1 walking policy | `external/booster_deploy/tasks/locomotion/robots/k1/models/k1_walk.pt` |
| **Scene actually loaded** | `simulation/scenes/labmate_scene.xml` |
| Laboratory geometry and sites | `simulation/scenes/lab_environment.xml` |
| K1 MuJoCo model (included by the scene) | `external/booster_assets/robots/K1/K1_22dof.xml` |
| Task config override | `shared/task_cfg.py` → `LabMateInspectionTaskCfg` |
| Navigation graph and route planning | `robot_control/route_graph.py` → `RouteGraph`, `ROUTE_EDGES` |
| Inspection checklist | `shared/inspection.py` → `STATIONS`, `STATION_ORDER` |
| Scenario files | `simulation/scenarios/*.json` |
| K1 joint config | `external/booster_deploy/booster_deploy/robots/k1.py` → `K1_CFG` |
| Velocity limits | `external/booster_deploy/tasks/locomotion/robots/k1/__init__.py` → `K1WalkControllerCfg` |
| Observation build | `external/booster_deploy/tasks/locomotion/locomotion.py` → `LocomotionPolicy.compute_observation()` |
| Navigator logic | `robot_control/waypoint_navigator.py` → `WaypointNavigator.update()` |

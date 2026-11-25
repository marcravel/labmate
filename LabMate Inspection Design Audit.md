Read-only design audit · RobotHack 2026

# LabMate Inspection Design Audit

What the laboratory environment actually contains, the three problems that currently block the demonstration, and a verified path to camera-based inspection that needs no change to any Booster file.

**Date** 2026-11-25 **Branch** feature/3d-lab-environment **Files modified** none **MuJoCo** 4.0.1

verified measured in this repo during the audit recommended engineering judgement, not yet run speculative idea needing its own investigation

01

## Three findings that block the demonstration

Each one was reproduced by measurement, not inferred from reading. Fix order matters: #2 protects the work, #1 makes the demo run, #3 unlocks everything after it.

### Finding 1 The route the demo actually flies walks through the central table blocking

`run_labmate.py` runs `LAB_DEMO_ROUTE`, not the validated navigation graph. Its waypoint Checkpoint 1 (3.8, 0.0) sits inside the table's collision box `col_ctable`, which spans x 2.2–3.8, y ±0.4. I placed the robot at each waypoint and swept each straight-line edge at 2 cm intervals in the narrowest arms-down pose:

| Edge from `LAB_DEMO_ROUTE` | Result |
| --- | --- |
| Home → Checkpoint 1 | **collides** — `col_ctable`, `col_ct_items` |
| Checkpoint 1 → Checkpoint 2 | **collides** — `col_ctable`, `col_ct_items` |
| Checkpoint 2 → Checkpoint 3 | clear |
| Checkpoint 3 → Return Home | clear |

Standing at Checkpoint 1 alone already reports contact with `col_ctable`.

Meanwhile the real graph in the MJCF — 16 sites and 16 edges — passes every check with **0.636 m** minimum centreline clearance. The validated geometry is fine; the Python route simply never used it. The root cause is duplication: waypoint coordinates are hand-typed in `run_labmate.py` while the ground truth lives in the model's `nav_*` and `inspect_*` sites.

### Finding 2 The entire laboratory is untracked and one command from deletion data loss risk

`lab_environment.xml` (320 lines, 197 geoms) and `labmate_scene.xml` live in `external/booster_assets/robots/K1/`. `.gitignore` line 21 ignores `external/`, which I confirmed with `git check-ignore`. There is no commit anywhere containing the environment — `git log` for that path is empty. A re-clone of the Booster assets, or any cleanup of `external/`, destroys the best work in the project with no recovery.

Related: `external/booster_deploy/booster_deploy/robots/k1.py` line 65 has already been edited to `mjcf_path="{BOOSTER_ASSETS_DIR}/robots/K1/labmate_scene.xml"`. That contradicts AGENTS.md §B.2 and §F, which mark `booster_deploy/` read-only, and it is also invisible to review for the same gitignore reason.

Both problems have one fix, and AGENTS.md §F already prescribes the mechanism: move the scene into `simulation/scenes/` where git tracks it, restore `k1.py` to pristine, and point at the new path from a LabMate `@configclass` subclass using the `.replace(...)` pattern. See [§12](#first).

### Finding 3 No camera exists anywhere — but one attaches with zero XML edits solved

I grepped both `K1_22dof.xml` and `lab_environment.xml`: there is no `<camera>` element in either. What the robot does have is two empty mount frames, `head_realsense_rgb_link` and `head_booster_stereo_rgb_link`, carrying the correct optical poses but no geometry and no camera. So perception has no image source today, and the obvious fix — editing the robot XML — is forbidden and would break the validator's structural equality checks.

MuJoCo 4.0.1 resolves this cleanly. `mujoco.MjSpec` loads the scene as an editable model, a camera is attached to the existing head link in Python, and the result compiles to a model that is structurally identical to the one on disk. I verified that, rendered from it, and calibrated the orientation. Details in [§3](#camera).

02

## Existing architecture and environment assessment

### Code inventory

The custom application layer is 247 lines across five files. Everything else listed in the README's planned structure is an empty directory — `backend/`, `dashboard/`, `perception/`, `shared/`, `tests/`, `simulation/` all contain no files.

| File | Lines | Responsibility | Assessment |
| --- | --- | --- | --- |
| run_labmate.py | 53 | Entry point, route definition | Holds two unused routes plus the broken one |
| robot_control/waypoint_navigator.py | 128 | 8-phase patrol state machine | Solid structure, open-loop (below) |
| robot_control/velocity_interface.py | 35 | Clamped velocity write, pose read | Clean, correct, leave alone |
| robot_control/labmate_controller.py | 31 | Navigator hook, stdin suppression | Exactly the right amount of subclass |
| check_scene.py | 115 | Static geometric validator | The most valuable asset in the repo |

### What is genuinely well built

Three things in the environment are better than typical hackathon work and should be treated as design rules to preserve:

- **Strict visual/collision separation.** Every object carries detailed `class="lab_vis"` geoms with `contype=0` for appearance, plus one coarse `class="lab_col"` box for physics. 197 visual geoms cost the solver almost nothing.
- **Zero added degrees of freedom.** All 15 lab bodies are static with `density=0` and no joints. verified The validator confirms `nq`, `nv`, `nu`, `njnt`, `qpos0` and every robot array are bit-identical to the bare robot and occupy the same ID slots — so the walking policy sees exactly the model it was trained against. Measured physics cost is **+34%** (0.212 s → 0.284 s of wall-clock per 3 s of simulation), entirely from broadphase, not from new dynamics.
- **The validator itself.** `check_scene.py` includes a positive control (it asserts that deliberately placing the robot inside a bench *is* detected), sweeps all 16 sites at 24 headings in the widest arms-out pose, and sweeps every route edge at 2 cm. This is real verification infrastructure. Every environment change proposed in this document should be gated on it.

I re-ran it during the audit: **10 of 10 checks pass, 0 failures.** verified

### Room and object inventory

Interior spans x −1 … 7, y −3 … 3 (8 × 6 m). Walls collide to 2.8 m; the south and west walls are drawn only 1.0 m high so the viewer can see in. The single entrance is a 1.40 × 2.10 m opening in the west wall at y = 0, modelled with no door.

| Body | Centre (x, y) | Footprint | Notable contents |
| --- | --- | --- | --- |
| electronics_bench_a | 1.60, 2.625 | 1.80 × 0.75 | Oscilloscope w/ screen + 3 knobs, PSU w/ green display, soldering station + iron, test PCB on ESD mat, pegboard |
| electronics_bench_b | 3.80, 2.625 | 1.80 × 0.75 | Second scope, monitor, DMM, electronic load, 3 coloured banana terminals |
| server_rack | 5.60, 2.60 | 0.70 × 0.80 | 6 rack units, each with a status LED — **one is already red** |
| robotics_bench | 1.80, −2.625 | 2.00 × 0.75 | 4-link static robot arm w/ gripper, monitor, keyboard, spare wheel |
| printer_station | 3.90, −2.625 | 1.00 × 0.75 | 3D printer: frame posts, bed, gantry, part in progress |
| central_table | 3.00, 0.00 | 1.60 × 0.80 | Yellow instrument case, motor on a plate, red beacon |
| storage_shelf_1/2/3 | 6.775, −1.3 / 0 / 1.3 | 1.20 × 0.45 each | 4 shelves each, 22 bins total in three distinguishable materials |
| tool_cabinet | 6.10, −2.75 | 1.00 × 0.50 | Closed double doors with seam and two handles |
| safety_point | −0.87, 1.15 | 0.22 × 0.18 | Fire extinguisher, valve, red wall sign |

All coordinates read directly from `lab_environment.xml`. The red LED at `rack_led3` (material `mat_red` against `mat_screen_green` on the other five) is already a fault indicator — the environment author built a demonstration hook in and nothing uses it yet.

### Navigation graph

23 sites in body `lab_sites`, all in group 4 so they are hidden in the viewer by default: 9 `nav_*` corridor nodes, 7 `inspect_*` standing positions whose site quaternion encodes the required heading, and 7 `target_*` markers at equipment height naming what should be looked at. The corridor is a perimeter loop at |y| = 1.3 between x = 1.2 and x = 5.0, entered from `nav_outside` → `nav_entrance` → `nav_start`, with a spur from each loop node to its inspection site.

The `target_*` sites are the most under-used asset here. They already define, per station, the 3-D point the camera should be aimed at — which is exactly the input a perception pipeline needs for framing and for deciding what "looked at the right thing" means.

### Where the navigator is weak

The state machine (APPROACH → STOP_BEFORE_INSPECT → FACE_STATION → INSPECT_PAUSE → ROTATE_TO_NEXT → SETTLE_AFTER_TURN) is well-sequenced, and the velocity smoothing with per-step deltas of 0.03 m/s and 0.05 rad/s is a sensible way to keep a learned walking policy stable. Two real limitations:

- **No closed-loop heading correction while walking.** In the APPROACH phase the navigator calls `_send_smoothed_velocity(controller, vx_cmd, 0.0)` — the yaw argument is the literal constant `0.0`, a zero turn *rate*, never a correction derived from the bearing to the waypoint. The robot aims itself only in the discrete ROTATE_TO_NEXT phase and then walks blind. Any lateral drift from the walking gait accumulates uncorrected over the whole edge. With a 0.25 m position tolerance and a 0.636 m clearance margin, drift is the most likely cause of a demo-day collision. add cross-track correction
- **No obstacle awareness at all.** This is the correct choice for now and should stay that way — but it means every scenario object placed on a route will be walked into, not avoided. Scenario design must therefore put hazards *beside* or *under* the route as visual-only geometry, and treat "robot reports the hazard" as success rather than "robot avoids it". This single constraint shapes §5 and §7.

03

## Camera and perception feasibility

This section is the audit's main technical result. Everything in it was executed against the real model.

### Attaching a camera without touching a protected file

The scene is loaded as an `MjSpec`, a camera is added to the existing head pitch link, and the spec is compiled. No file on disk changes.

```
# verified working against the real scene, MuJoCo 4.0.1
spec = mujoco.MjSpec.from_file("simulation/scenes/labmate_scene.xml")
cam = spec.body("aahead_pitch_link").add_camera()
cam.name = "labmate_head_cam"
cam.fovy = 58.0
cam.pos  = [0.07, 0, 0.10]
cam.quat = [0.5, 0.5, -0.5, -0.5]   # view along head +X, world +Z up
model = spec.compile()
```

verified The compiled model matches the on-disk scene on `nq`, `nv`, `nu`, `njnt`, `nbody`, `ngeom`, `nsite`, `nsensor` and `qpos0`. A camera is not a body and adds no state, so every structural invariant `check_scene.py` asserts still holds. `ncam` goes 0 → 1.

I mounted on `aahead_pitch_link` rather than the realsense mount frame deliberately: the realsense frame carries a rotated optical convention, and an identity-quaternion camera there renders a view pointing away from the robot's facing. Mounting on the pitch link with the explicit quaternion above gives a forward-facing camera that still follows head motion. The quaternion is the calibration result, not a guess. verified

### What the camera sees at each station

Rendered at 640 × 480 with EGL, robot standing at the validated inspection site with the site's heading. The figures are per-geom pixel share of the frame, from segmentation rendering.

| Inspection site | Geoms in view | Largest objects by pixel share |
| --- | --- | --- |
| inspect_electronics_a | 18 | `pegboard_a` 29.3% · `ebench_a_top` 12.7% · `esd_mat` 4.4% · `scope_a` 2.6% · `psu_a` 2.0% |
| inspect_printer | 16 | `pbench_top` 13.4% · `printer_top` 7.7% · `printer_base` 5.8% · 2 × `printer_post` 1.8% |
| inspect_server_rack | 12 | `rack_body` 26.7% · `rack_unit3` 19.3% · `rack_unit2` 19.3% · `rack_unit4` 15.4% |

Each view is dominated by exactly the equipment that station is supposed to inspect. The validated sites are already well-framed for perception — no new standing positions are needed for the three stations tested.

### Segmentation rendering is free ground truth

This is the finding that most changes the plan. `Renderer.enable_segmentation_rendering()` returns a per-pixel geom ID map, so for any frame the simulator can state exactly which named objects are visible, how many pixels each occupies, and which are occluded — real visibility, real occlusion, real framing, computed from the same geometry the robot walks in.

#### Consequence for the team's plan

Mock perception does not have to mean hard-coded detections. A `MockDetector` backed by segmentation produces a genuine detection list with bounding boxes and visibility, so the inspection logic, the reporting, the dashboard and the demonstration can all be finished and debugged **before YOLO exists** — and YOLO then replaces one class behind an unchanged interface. It also gives the YOLO team automatically-labelled training images from the same scene, at no extra cost.

#### Measured limit: small indicators

At the server rack's 1.05 m standoff I counted the pixels of all six status LEDs (each 0.04 m wide):

| Geom | Height | Pixels | Verdict |
| --- | --- | --- | --- |
| rack_led0 | 0.28 m | 0 | below frame |
| rack_led1 | 0.50 m | 454 | **detectable** |
| rack_led2 | 0.72 m | 1023 | **detectable** |
| rack_led3 | 0.94 m | 992 | **detectable** — this is the red one |
| rack_led4 | 1.16 m | 1022 | **detectable** |
| rack_led5 | 1.38 m | 0 | not rendered at this pose |

Four of six LEDs are comfortably visible at roughly 1000 px each — ample for both colour classification and YOLO. The outer two fall outside the vertical field of view at a fixed head pitch. Two options, both cheap: command a head pitch during the inspection pause, or accept a documented inspection band of roughly 0.5–1.2 m at a 1 m standoff. use the head pitch joint The K1 has `aahead_pitch_joint` available and the scenario that matters — the red LED at U3 — is already dead centre in frame.

04

## What vision can and cannot establish

The brief asks for this split explicitly, and it is also the most credible thing LabMate can show a judging panel: a robot that distinguishes what it saw from what it was told.

| Condition | Source of truth | Why |
| --- | --- | --- |
| Equipment present / absent / displaced | vision | Large, rigid, distinctly coloured objects at a known standoff. The strongest vision case in this lab. |
| Fire extinguisher present and unobstructed | vision | Red cylinder against a pale wall; obstruction is an occlusion test the segmentation map answers directly. |
| Exit route blocked | vision | Free-floor-pixel test across the doorway opening. |
| Status light colour, enclosure open, cover missing, cable unplugged | vision | All are visible geometric or colour state changes. |
| Object on floor, liquid spill | vision | Detectable as a region. Note that *what* the liquid is cannot be identified visually — report "unidentified liquid", never "acid". |
| E-stop activated | vision, with care | A pressed mushroom head differs by a few millimetres of travel. Needs an exaggerated visual delta to be honest about. |
| Temperature, equipment overheating | sensor only | No thermal camera in the model and no radiometric data in MuJoCo. Must come from simulated telemetry. |
| Smoke, fire, air quality, gas leak | sensor only | MuJoCo has no participating media. A visible smoke proxy would be theatre; the alert belongs in telemetry. |
| Energised conductor, voltage present | sensor + human | An open cabinet is visible; whether it is live is not. LabMate must report "enclosure open, energisation unknown". |
| Noise, vibration anomaly | sensor only | No audio in the pipeline. Simulated vibration telemetry is the honest route. |

#### Reporting rule to adopt now

Every finding in an inspection result carries its `evidence_source`: `VISUAL`, `TELEMETRY`, or `REQUIRES_HUMAN`. This costs one enum field, prevents the project from claiming simulated detection proves real-world safety, and is the detail a judge with a robotics background will notice.

05

## Recommended environment additions

Coordinates below are placed in wall spans I confirmed to be unoccupied, and every one is a proposal that must be gated on a `check_scene.py` re-run before it is trusted. recommended Free spans I identified: north wall x 4.7–5.25 and x 5.95–7.0; south wall x 2.8–3.4, x 4.4–5.6 and x −1.0–0.8; west wall y 1.3–3.0 and y −3.0–−0.8.

### Permanent fixtures

| Object | Proposed pos | Representation | Why it earns a place |
| --- | --- | --- | --- |
| **Electrical distribution panel** | 5.00, −2.93 | Visual door + hinge seam + hazard label; one `lab_col` box. Static, with a scenario variant for the open door. | The single best safety prop. Open/closed is unmistakable on camera and drives the strongest honest finding ("open, energisation unknown"). |
| **First-aid kit** | −0.98, 1.75, z 1.15 | White box, green cross decal. Visual only, wall-mounted. | Completes the safety station beside the extinguisher; a trivial presence/absence check. |
| **Emergency exit sign** | −1.05, 0, z 2.28 | Green illuminated panel above the existing lintel. Visual only. | Makes the doorway legible as an egress route, which is what makes an obstruction scenario read. |
| **Wall E-stop** | −0.98, −1.20, z 1.10 | Yellow backplate + red mushroom. Two alternate geoms for normal/activated. | Cheap, and the activated state is a credible abnormal event. |
| **Flammables cabinet** | 0.30, −2.80 | Yellow cabinet, hazard diamond. Static. | The existing `PATROL_ROUTE` already names "Chemical Storage" that does not exist. This makes that label true. |
| **Gas cylinder + restraint strap** | 4.95, 2.85 | Cylinder r 0.12, h 0.70, plus a separate strap geom. Static cylinder, scenario strap. | Unrestrained cylinders are a real lab violation, and strap-present/absent is a clean binary vision task. |
| **Smoke detector** | 3.00, 2.98, z 2.50 | Small white disc. Visual only. | Gives the telemetry-only smoke alert a physical anchor the camera can show. |
| **E-waste + sharps bins** | 5.90, −2.20 | Two coloured bins. Static. | Low effort, raises how much the room reads as a working lab. |

### Scenario objects

These exist only when a scenario loads them, so the permanent environment stays validated. Because the navigator has no obstacle avoidance, floor hazards on a route must be **visual-only** (`contype=0`) — the robot walks through them, the camera sees them, the report names them, and the walking policy is never destabilised. This is a feature, not a compromise: it is exactly the behaviour of a robot that inspects rather than one that avoids.

| Scenario object | Proposed pos | Collision | Drives |
| --- | --- | --- | --- |
| Liquid spill (dark disc, r 0.35) | 2.40, −1.30 | none — on the route | Spill detection during routine patrol |
| Cardboard crate in the doorway | −1.05, 0.00 | solid, off-route | Blocked-egress safety finding |
| Toolbox left on the floor | 4.80, 0.60 | solid, beside route | Walkway obstruction |
| Open panel door (swapped geom) | 5.00, −2.93 | solid, off-route | Electrical hazard response |
| Loose cable run (capsules) | 3.00 → 3.00, −2.0 | none | Trip-hazard finding |
| Missing oscilloscope on bench A | 1.05, 2.80 | n/a — removed | Equipment inventory failure |
| Red fault LED at the rack | already present | n/a | Abnormal status response — **zero build cost** |

06

## Ranked by value against effort

Ranking assumes the goal is a convincing seven-minute demonstration, not the largest environment. Items 1–5 are what I would commit to; 6–9 are worth doing if time allows; 10–11 I recommend against.

| # | Feature | Demo value | Effort | Note |
| --- | --- | --- | --- | --- |
| 1 | Fix the route; derive it from the MJCF sites | **critical** | low | Nothing else matters if the robot walks into a table |
| 2 | Vendor the scene into git; restore `k1.py` | **critical** | low | Protects every hour already invested |
| 3 | Head camera via MjSpec + segmentation mock detector | **very high** | low | Already proven in this audit; unblocks the whole perception track |
| 4 | Server-rack fault response scenario | **very high** | very low | The red LED exists already |
| 5 | Inspection report + request queue | **very high** | medium | This is what the audience actually reads at the end |
| 6 | Electrical panel + open-door scenario | high | medium | Best new safety prop |
| 7 | Spill + blocked-exit scenarios | high | low | Visual-only geoms, cheap |
| 8 | Simulated telemetry service | high | low | Makes the vision/sensor distinction concrete |
| 9 | Cross-track heading correction in APPROACH | medium | low | Reduces demo-day drift risk |
| 10 | Streamlit dashboard | medium | medium | Only after the report format is settled |
| 11 | Articulated doors, movable equipment with joints | low | high | avoid Adds DOF and risks the policy-identical guarantee |
| 12 | Smoke/fire visual effects | low | high | avoid MuJoCo cannot do it honestly; keep it in telemetry |

07

## End-to-end demonstration scenarios

All three run with mock perception and scripted triggers, so none of them waits on YOLO. Each names what perception must supply, so the YOLO work has a contract to build against.

### A · Routine equipment inventory sweep

routine inspection

1. **Initial state** — Lab nominal. Expected-equipment manifest lists the instruments at all 7 stations.
2. **Trigger** — Scheduled patrol, started from the entrance.
3. **How LabMate receives it** — A patrol mission enqueues 7 `InspectionRequest`s, one per `inspect_*` site, task `INVENTORY`.
4. **Navigation** — The validated perimeter loop, spurring to each station in turn.
5. **What it inspects** — During the 3 s pause it frames the station's `target_*` point and captures RGB + segmentation.
6. **Perception must provide** — A detection list of class labels with bounding boxes and pixel areas.
7. **How the result is determined** — Detected set compared against the manifest: `PRESENT`, `MISSING`, or `DISPLACED` when found outside its expected image region.
8. **What the audience sees** — A 7-station table, 6 green and 1 red: "Station Electronics A — oscilloscope MISSING, expected at bench centre." The scenario removed `scope_a` before the run.

### B · Safety sweep — egress and fire equipment

safety inspection

1. **Initial state** — Extinguisher at the safety point, doorway clear, exit sign lit.
2. **Trigger** — Scheduled safety round; the scenario has placed a crate in the doorway at (−1.05, 0).
3. **How LabMate receives it** — Two requests, task `SAFETY`: one for the safety point, one for the egress route.
4. **Navigation** — `nav_start` → `nav_entrance`, turning to face the west wall.
5. **What it inspects** — The extinguisher and sign; then the doorway's floor region.
6. **Perception must provide** — Extinguisher detection plus its unoccluded pixel fraction, and a free-floor-pixel measurement across the opening.
7. **How the result is determined** — Extinguisher `PRESENT` and ≥ 80% unoccluded passes; free-floor fraction across the doorway below threshold raises `EGRESS_OBSTRUCTED`.
8. **What the audience sees** — "Fire extinguisher: present, accessible. **Emergency exit: OBSTRUCTED** — unidentified object occupying 38% of the opening. Evidence: visual. Severity: high. Action: clear immediately." With the camera frame attached.

### C · Abnormal event — server rack fault lowest build cost

event response

This is the scenario I would build the demonstration around. The red LED at `rack_led3` already exists and renders at \~992 px from the validated standing position, both measured in this audit. It is also the only scenario that shows LabMate fusing telemetry with vision and being explicit about which is which.

1. **Initial state** — Mid-patrol, robot somewhere on the loop. All telemetry nominal.
2. **Trigger** — The simulated telemetry service raises `rack_inlet_temp = 47 °C`, above a 40 °C threshold.
3. **How LabMate receives it** — The event service posts a priority `InspectionRequest` for `inspect_server_rack`, task `FAULT_CONFIRM`. The mission manager pre-empts the patrol and pushes the remaining stations back onto the queue.
4. **Navigation** — Re-plans from the current node to `nav_wp_northeast`, then the rack spur. The audience sees the robot abandon its round and divert.
5. **What it inspects** — The rack face, pitching the head to sweep all six LED positions.
6. **Perception must provide** — Per-rack-unit indicator colour classification with position.
7. **How the result is determined** — Telemetry says hot; vision says unit 3's indicator is red while units 1, 2 and 4 are green. Correlated, the result is a confirmed fault localised to one unit.
8. **What the audience sees** — "**FAULT CONFIRMED — Server rack, unit 3.** Telemetry: inlet 47 °C, threshold 40 °C. Visual: unit 3 indicator RED, units 1/2/4 nominal. Units 0 and 5 outside inspection band — not assessed. Energisation and internal condition cannot be determined visually. Action: human inspection required." Camera frame attached, LED circled.

**Why this lands.** It diverts mid-task, which is the only thing in the demo that proves autonomy rather than playback. And the report says what it could *not* determine — which is what distinguishes an engineering demo from a sales demo.

#### Optional fourth, if time allows

**Spill encountered during patrol.** A visual-only disc on the southwest corridor at (2.40, −1.30). The robot is not asked to look for it; the detector raises it opportunistically from a frame captured in transit, and LabMate enqueues a new inspection of its own accord. Shows initiative rather than instruction-following, and costs one geom.

08

## Modular architecture

The governing constraint from the brief — navigation must never depend on YOLO — is satisfied by making `InspectionRequest` the only thing that crosses into navigation, and by hiding the detector behind a Protocol.

```
# shared/schemas.py — the contract every team member codes against

class Task(Enum):      INVENTORY, SAFETY, FAULT_CONFIRM, SPILL_CHECK
class Evidence(Enum):  VISUAL, TELEMETRY, REQUIRES_HUMAN
class Status(Enum):    PASS, FAIL, INCONCLUSIVE, NOT_ASSESSED

@dataclass(frozen=True)
class InspectionRequest:
    site: str            # an inspect_* site name — the only spatial input
    task: Task
    priority: int = 0    # higher pre-empts the patrol
    note: str = ""

@dataclass(frozen=True)
class Detection:
    label: str; bbox: tuple[int,int,int,int]
    confidence: float; pixel_area: int
    occluded_fraction: float = 0.0

@dataclass(frozen=True)
class Finding:
    item: str; status: Status; evidence: Evidence
    detail: str; severity: str = "info"

@dataclass(frozen=True)
class InspectionResult:
    request: InspectionRequest; findings: list[Finding]
    frame_path: str | None; elapsed_s: float
```

```
# perception/detector.py — swap the implementation, never the interface

class Detector(Protocol):
    def detect(self, rgb: np.ndarray, seg: np.ndarray | None) -> list[Detection]: ...

class SegmentationDetector:   # today: ground truth from MuJoCo. VERIFIED WORKING.
class YoloDetector:           # later: same signature, ignores seg
```

#### Module ownership

- `simulation/` — scene build, MjSpec camera attach, scenario load/unload
- `telemetry/` — simulated sensors, threshold alerts
- `events/` — triggers → `InspectionRequest`
- `robot_control/` — route graph, mission manager, navigator
- `perception/` — camera capture, `Detector` implementations
- `inspection/` — `Detection` + telemetry → `Finding`
- `backend/` — FastAPI: POST events, GET results
- `dashboard/` — Streamlit reader

#### Dependency rules

- `robot_control` imports `shared` only. Never `perception`.
- `inspection` consumes `Detection` and telemetry; it does not know which detector produced them.
- `perception` never issues velocity commands.
- `set_velocity()` remains the sole path to `vel_command`, per AGENTS.md §D.
- The mission manager is a new navigator-shaped callable — it keeps the existing `fn(controller, elapsed)` signature, so `LabMateController` needs no change at all.

That last point is worth stating plainly: the entire event-driven architecture fits behind the hook that already exists. `controller.set_navigator(mission_manager.update)` is the only wiring change required in `run_labmate.py`.

09

## Inspection locations and route implications

The seven existing `inspect_*` sites are sufficient and should not be moved — they are validated at 24 headings and the three I rendered are well-framed. Recommended additions, each requiring a validator re-run:

| New site | Position | Reached from | Purpose |
| --- | --- | --- | --- |
| inspect_safety_point | −0.30, 1.15, facing −x | `nav_entrance` | Extinguisher, first-aid kit, exit sign |
| inspect_egress | −0.40, 0.00, facing −x | `nav_entrance` | Doorway obstruction check |
| inspect_elec_panel | 5.00, −1.90, facing −y | `nav_wp_southeast` | Panel door state |
| inspect_gas_cylinder | 4.95, 1.90, facing +y | `nav_wp_northeast` | Restraint strap present |

The two west-side sites sit between the entrance and the start pad, an area the validator already sweeps and which carries the widest clearance in the room. The two perimeter sites are 0.6 m spurs off existing loop nodes, comparable to the existing rack spur that holds the current 0.636 m minimum.

One route change is needed beyond the fix in §12: the mission manager must be able to plan from an *arbitrary* current node to a requested site, not just walk a fixed list, because scenario C diverts mid-patrol. The loop is small enough that breadth-first search over the 16-edge list is sufficient — no real path planner is warranted. BFS over the validated edge list

10

## Implementation roadmap

### M1 · Foundation do first, blocks everything

| Task | Deliverable | Files | Validation | Effort |
| --- | --- | --- | --- | --- |
| Vendor the scene into git | Scene tracked; `k1.py` pristine | `simulation/scenes/*`, `shared/task_cfg.py`, `run_labmate.py`, `.gitignore` | `check_scene.py` 10/10 from the new path; `git status` shows the scene tracked | S |
| Route graph from MJCF sites | `RouteGraph` with BFS; routes can no longer drift from the model | `robot_control/route_graph.py`, `run_labmate.py` | New test asserts every planned edge is in the validated set | S |
| Cross-track correction | Heading held toward the waypoint during APPROACH | `waypoint_navigator.py` | Lateral error logged over a full loop stays inside tolerance | S |

### M2 · Perception substrate

| Task | Deliverable | Files | Validation | Effort |
| --- | --- | --- | --- | --- |
| MjSpec camera + capture | `CameraSensor` returning RGB + segmentation | `simulation/scene_builder.py`, `perception/camera.py` | Structural equality assertions; a frame saved from each site | S |
| Segmentation mock detector | `SegmentationDetector` satisfying `Detector` | `perception/detector.py`, `shared/schemas.py` | Detections at all 7 sites match the expected manifest | M |
| Head pitch during inspection | Full 0.28–1.38 m band covered at the rack | `waypoint_navigator.py` | All 6 LEDs register non-zero pixels | S |

### M3 · Events and missions

| Task | Deliverable | Files | Validation | Effort |
| --- | --- | --- | --- | --- |
| Simulated telemetry | Threshold alerts on a timeline | `telemetry/` | Alert fires at the scripted time | S |
| Mission manager | Priority queue, patrol pre-emption, re-planning | `robot_control/mission_manager.py` | Scenario C diverts mid-loop and resumes | M |
| Scenario loader | YAML scenarios toggling objects at build time | `simulation/scenarios/` | Validator passes for the baseline; expected failures documented per scenario | M |

### M4 · Environment and reporting

| Task | Deliverable | Files | Validation | Effort |
| --- | --- | --- | --- | --- |
| Safety fixtures + 4 new sites | Panel, first-aid, exit sign, E-stop, cylinder | `simulation/scenes/lab_environment.xml` | Validator 10/10 with new sites and edges | M |
| Inspection analysis + report | `Finding` generation, console + JSON report | `inspection/` | All three scenarios produce correct findings | M |
| Backend + dashboard | FastAPI endpoints, Streamlit view | `backend/`, `dashboard/` | Report renders end to end | M |
| YOLO swap | `YoloDetector` behind the same Protocol | `perception/detector.py` | Scenarios pass with either detector | M |

### Parallel work without merge conflicts

After M1 lands, four tracks touch disjoint files and can run simultaneously:

| Track | Owns | Waits on |
| --- | --- | --- |
| Perception | `perception/`, `simulation/scene_builder.py` | `shared/schemas.py` frozen |
| Environment | `simulation/scenes/`, `simulation/scenarios/` | M1 vendoring |
| Mission + events | `robot_control/mission_manager.py`, `telemetry/`, `events/` | `RouteGraph` + schemas |
| Reporting | `inspection/`, `backend/`, `dashboard/` | `shared/schemas.py` frozen |

**Freeze `shared/schemas.py` before anything else starts.** Three of four tracks depend on it and nothing else is shared between them. One hour agreeing those dataclasses buys a day of parallel work.

11

## Risks and technical limitations

| Risk | Severity | Mitigation |
| --- | --- | --- |
| Environment lost with `external/` | critical | M1 vendoring. Until then, nobody should clean `external/`. |
| Walking-policy drift causes a collision on demo day | high | Cross-track correction; keep the 0.6 m clearance rule; rehearse the full route repeatedly |
| Safety fallback terminates the viewer with no recovery | high | `locomotion.py:123` stops the controller when projected gravity z > −0.5 and there is no recovery path. Do not disable it. Plan the demo so a fall means restart, and keep a recorded run as backup. |
| Scenario objects break the validated route | medium | Floor hazards on routes are visual-only; solid scenario objects stay off-route; run the validator per scenario and document expected failures |
| Segmentation-trained logic does not transfer to YOLO | medium | The mock returns confidences and occlusion fractions, not certainties; test the analysis layer with degraded mock confidence before YOLO arrives |
| Headless rendering fails on another machine | medium | EGL is present here and rendering is verified, but `MUJOCO_GL` is unset by default. Set it explicitly and test on the demo machine. |
| Added DOF destabilise the policy | medium | Keep every lab body static. The validator's structural equality checks are the guard — never weaken them. |
| Scope creep into a large environment | medium | Items 1–5 of §6 are the demonstration. Everything else is optional. |

#### Claims to avoid making

Simulated visual detection does not establish real-world safety; the segmentation detector is reading the simulator's own ground truth, which is a test harness and not evidence about the world. Nothing in this design can determine temperature, energisation, chemical identity or air quality. Any demonstration text should say "detected in simulation", and any finding that needs a sensor LabMate does not have should be reported as `REQUIRES_HUMAN` rather than asserted.

12

## The exact first implementation task

One task, two commits, no new dependencies, nothing in `external/` modified — and it removes an existing AGENTS.md violation rather than adding one.

#### Objective

Bring the laboratory scene under version control and make the patrol route derive from the model's validated sites, so the demonstration stops walking into the central table and the environment stops being one command from deletion.

### Commit 1 — vendor the scene

1. Move `lab_environment.xml` and `labmate_scene.xml` from `external/booster_assets/robots/K1/` to `simulation/scenes/`, adding a relative `meshdir` or a symlink to the K1 meshes so `K1_22dof.xml` still resolves.
2. Restore `external/booster_deploy/booster_deploy/robots/k1.py` line 65 to its original `K1_22dof.xml` path.
3. Add `shared/task_cfg.py` with a LabMate `@configclass` subclassing `K1WalkTaskCfg` and overriding the robot with `K1_CFG.replace(mjcf_path=...)` pointing at `simulation/scenes/labmate_scene.xml` — the pattern AGENTS.md §F prescribes.
4. Point `run_labmate.py` at that config instead of the bare `get_task("k1_walk")`.
5. Move `check_scene.py` to `tests/` and update its default path.

**Validation:** `check_scene.py` reports 10/10 against the new location, `git status` shows the scene files tracked, and `git diff external/` is empty.

### Commit 2 — route from the model

1. Add `robot_control/route_graph.py`: read every `nav_*` and `inspect_*` site from `mj_model`, taking position and the heading encoded in each site quaternion; hold the 16 validated edges; expose `plan(from_node, to_node)` as BFS and `patrol_route()` as the full loop.
2. Replace `PATROL_ROUTE` and `LAB_DEMO_ROUTE` in `run_labmate.py` with `RouteGraph(controller.mj_model).patrol_route()`, and delete both hand-typed lists.
3. Add `tests/test_route_graph.py` asserting that every consecutive pair in a generated route is a member of the validated edge set — which makes Finding 1 unrepeatable.

**Validation:** the new test passes; `python run_labmate.py` completes a full loop with no reported contact against lab geometry.

| File | Change |
| --- | --- |
| simulation/scenes/lab_environment.xml | moved in, now tracked |
| simulation/scenes/labmate_scene.xml | moved in, now tracked |
| shared/task_cfg.py | new — config subclass overriding `mjcf_path` |
| robot_control/route_graph.py | new — site loading and BFS planning |
| run_labmate.py | wiring; both hand-typed routes deleted |
| tests/check_scene.py | moved from the repo root |
| tests/test_route_graph.py | new — route membership assertion |
| external/booster_deploy/.../robots/k1.py | **restored to pristine** |
| .gitignore | unchanged — `external/` stays ignored, correctly |

Complexity: low. No new dependencies. Nothing in `booster_deploy/` or `booster_assets/` modified, and one existing modification reverted. Per AGENTS.md §E this is a single concern and should be reported before M2 begins.

Read-only audit. No files in the repository were created, modified or deleted. Measurements were produced by loading the existing model and by running the project's own `check_scene.py`; the camera and segmentation results come from `mujoco.MjSpec` and `mujoco.Renderer` in throwaway scripts that wrote nothing to the project. Items marked recommended and speculative have not been executed and need their own validation — in particular, every proposed coordinate must be gated on a `check_scene.py` re-run before it is relied on.
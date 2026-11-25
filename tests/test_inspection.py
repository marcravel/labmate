#!/usr/bin/env python3
"""Checks the inspection checklist and the scenario loader.

Run:  python3 tests/test_inspection.py

Standalone, in the same style as check_scene.py, so the project keeps its
mujoco + numpy only dependency set.

What this guards:

* The checklist matches the scene (every station site and state geom exists).
* Each scenario produces exactly the verdicts it claims to, so the demo takes
  are deterministic and a scenario cannot silently stop reproducing its fault.
* Every check can both pass and fail. A check that is wired the wrong way round,
  or that reads a geom nothing ever changes, would otherwise sit there
  reporting PASS forever - the most dangerous failure mode an inspection
  report has.
* Scenarios cannot touch the robot model or any collision geom, which is what
  keeps the walking policy and the swept route clearances valid.
"""
import sys
from pathlib import Path

import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from robot_control.route_graph import DEFAULT_STATION_ORDER, RouteGraph  # noqa: E402
from shared import inspection, scenario as scenario_mod  # noqa: E402
from shared.inspection import CRITICAL, FINDING, PASS  # noqa: E402
from shared.inspection_report import InspectionRun  # noqa: E402

LAB_SCENE = PROJECT_ROOT / "simulation" / "scenes" / "labmate_scene.xml"

# What each scenario must produce. Station severities in checklist order.
EXPECTED = {
    "nominal": [PASS, PASS, PASS, PASS, PASS],
    "demo_findings": [PASS, FINDING, PASS, CRITICAL, PASS],
    "all_faults": [FINDING, FINDING, FINDING, CRITICAL, FINDING],
}

fails = 0


def check(label, ok, detail=""):
    global fails
    fails += not ok
    print(("PASS " if ok else "FAIL ") + label + (f"  ({detail})" if detail else ""))


def fresh():
    """A clean model, so scenarios never inherit each other's state."""
    model = mujoco.MjModel.from_xml_path(str(LAB_SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def run_round(model, data, plan):
    run = InspectionRun(plan.name, plan.title, plan.panel_readings, verbose=False)
    for site in inspection.STATION_ORDER:
        run.results.append(inspection.inspect_site(site, model, data, 0.0))
    return run


print(f"lab scene : {LAB_SCENE}\n")
model, data = fresh()

# 1. The checklist and the scene agree
try:
    inspection.validate(model)
    check("checklist validates against the compiled model", True)
except ValueError as exc:
    check("checklist validates against the compiled model", False, str(exc))

check(f"{len(inspection.STATIONS)} stations, each with at least one check",
      all(s.checks for s in inspection.STATIONS))
check("no duplicate station sites",
      len({s.site for s in inspection.STATIONS}) == len(inspection.STATIONS))
check("no duplicate check keys within a station",
      all(len({c.key for c in s.checks}) == len(s.checks)
          for s in inspection.STATIONS))

# 2. The walked order and the reported order are the same object
check("route_graph.DEFAULT_STATION_ORDER is the checklist order",
      tuple(DEFAULT_STATION_ORDER) == tuple(inspection.STATION_ORDER),
      f"{DEFAULT_STATION_ORDER} vs {inspection.STATION_ORDER}")

graph = RouteGraph(model)
route = graph.patrol_route()
walked = [wp["name"] for wp in route if wp["inspect"]]
check("the patrol visits exactly the checklist stations, in order",
      walked == list(inspection.STATION_ORDER), str(walked))
check("every checklist station is a graph node",
      all(s in graph.nodes for s in inspection.STATION_ORDER))

# 3. Scenarios exist and are loadable
check("all three scenarios present",
      set(EXPECTED) <= set(scenario_mod.available()),
      str(scenario_mod.available()))

# 4. Each scenario produces exactly the verdicts it claims
for name, expected in EXPECTED.items():
    model, data = fresh()
    plan = scenario_mod.load(name)
    scenario_mod.apply(plan, model)
    mujoco.mj_forward(model, data)
    run = run_round(model, data, plan)
    got = [r.severity for r in run.results]
    check(f"scenario {name!r}: station severities", got == expected,
          f"got {got}, want {expected}")

    if name == "nominal":
        check("scenario 'nominal': overall PASS and no findings",
              run.severity == PASS and not run.findings)
    if name == "demo_findings":
        check("scenario 'demo_findings': exactly 2 stations flagged",
              sum(r.severity != PASS for r in run.results) == 2,
              str(got))
        check("scenario 'demo_findings': the critical station is the freezer",
              next(r.site for r in run.results if r.severity == CRITICAL)
              == "inspect_storage")
        check("scenario 'demo_findings': report renders and serialises",
              bool(run.render()) and run.to_dict()["overall"] == CRITICAL)

# 5. Every single check can both pass and fail somewhere in the scenario set.
#    A check that never changes verdict is not testing anything.
seen: dict[str, set[str]] = {}
for name in EXPECTED:
    model, data = fresh()
    plan = scenario_mod.load(name)
    scenario_mod.apply(plan, model)
    mujoco.mj_forward(model, data)
    for result in run_round(model, data, plan).results:
        for c in result.checks:
            seen.setdefault(f"{result.site}/{c.key}", set()).add(
                "pass" if c.ok else "fail")

total_checks = sum(len(s.checks) for s in inspection.STATIONS)
check(f"all {total_checks} checks evaluated across the scenarios",
      len(seen) == total_checks, f"{len(seen)} seen")
never_fails = sorted(k for k, v in seen.items() if "fail" not in v)
check("every check fails in at least one scenario", not never_fails,
      str(never_fails) if never_fails else "")
never_passes = sorted(k for k, v in seen.items() if "pass" not in v)
check("every check passes in at least one scenario", not never_passes,
      str(never_passes) if never_passes else "")

# 6. Scenarios cannot reach the robot or any collision geom.
#    A scenario addresses geoms by name, and every geom belonging to the robot
#    model is unnamed, so the robot is unaddressable in principle rather than
#    merely guarded. shared.scenario's group check is the second line of
#    defence; the collision-geom refusal below exercises it.
model, _ = fresh()
base = mujoco.MjModel.from_xml_path(
    str(PROJECT_ROOT / "external" / "booster_assets" / "robots" / "K1"
        / "K1_22dof.xml"))
named_robot_geoms = [model.geom(i).name for i in range(1, base.ngeom)
                     if model.geom(i).name]
check(f"all {base.ngeom - 1} robot link geoms are unnamed, so no scenario can "
      f"name one", not named_robot_geoms, str(named_robot_geoms[:4]))

lab_geom_names = {model.geom(i).name for i in range(base.ngeom, model.ngeom)}
check("every geom the scenarios name belongs to the laboratory",
      all(g in lab_geom_names
          for name in EXPECTED
          for g in scenario_mod.load(name).geoms))

for geom, why in (("ground", "the shared ground plane"),
                  ("col_ctable", "a lab collision geom"),
                  ("no_such_geom", "a geom absent from the model")):
    try:
        scenario_mod.apply(
            scenario_mod.Scenario(name="probe", title="probe",
                                  geoms={geom: {"visible": False}}),
            model)
        check(f"scenario writing {why} ({geom}) is refused", False,
              "no error raised")
    except ValueError:
        check(f"scenario writing {why} ({geom}) is refused", True)

try:
    scenario_mod.apply(
        scenario_mod.Scenario(name="probe", title="probe",
                              geoms={"hood_sash": {"material": "no_such_mat"}}),
        model)
    check("scenario naming an unknown material is refused", False)
except ValueError:
    check("scenario naming an unknown material is refused", True)

# 7. Applying a scenario twice changes nothing the second time
model, data = fresh()
plan = scenario_mod.load("demo_findings")
scenario_mod.apply(plan, model)
again = scenario_mod.apply(plan, model)
check("applying a scenario is idempotent", not again, str(again))

print(f"\n{fails} failure(s)")
sys.exit(bool(fails))

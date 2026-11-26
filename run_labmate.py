import argparse
import importlib
import pkgutil
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
BOOSTER_DEPLOY_PATH = PROJECT_ROOT / "external" / "booster_deploy"
sys.path.insert(0, str(BOOSTER_DEPLOY_PATH))

import tasks
for _, module_name, is_pkg in pkgutil.walk_packages(tasks.__path__, tasks.__name__ + "."):
    importlib.import_module(module_name)

from booster_deploy.utils.registry import get_task
from robot_control.labmate_controller import LabMateController
from robot_control.route_graph import RouteGraph
from robot_control.waypoint_navigator import WaypointNavigator
from shared import inspection, scenario as scenario_mod
from shared.inspection_report import InspectionRun

# Importing this registers the "labmate_inspection" task: the stock k1_walk
# config with the robot spawned inside the LabMate laboratory scene.
from shared.task_cfg import DAMPED_TASK_NAME, TASK_NAME


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Run the LabMate autonomous inspection patrol.")
    parser.add_argument(
        "--scenario", default=scenario_mod.DEFAULT_SCENARIO,
        help=f"laboratory state to set up before the run "
             f"(default: {scenario_mod.DEFAULT_SCENARIO}; "
             f"available: {', '.join(scenario_mod.available()) or 'none'})")
    parser.add_argument(
        "--damping-tweak", action="store_true",
        help="raise the ankle joint damping above the official K1 deploy gains "
             "(1.0 -> 2.0). Off by default: the normal run uses the stock "
             "gains. Provided to A/B the residual foot buzz in simulation.")
    parser.add_argument(
        "--report", type=Path, default=None,
        help="also write the inspection report to this path as JSON")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    print("=" * 72)
    print("  LabMate - Autonomous Laboratory Inspection Patrol")
    print("=" * 72)

    task_cfg = get_task(DAMPED_TASK_NAME if args.damping_tweak else TASK_NAME)
    if args.damping_tweak:
        print("Gains: ankle damping raised to 2.0 (--damping-tweak); "
              "every other PD gain is the official K1 value")
    task_cfg.policy.device = "cpu"

    controller = LabMateController(task_cfg)

    # The checklist must match the scene it is about to be run against. This
    # fails loudly at startup rather than reporting a phantom PASS mid-patrol.
    inspection.validate(controller.mj_model)

    # Arrange the laboratory's inspectable state. Only lab visual geoms can be
    # touched; the robot model and every collision geom are off limits, so the
    # walking policy and the swept route clearances are unaffected.
    plan = scenario_mod.load(args.scenario)
    changes = scenario_mod.apply(plan, controller.mj_model)
    print(f"Scenario: {plan.name} - {plan.title}")
    if plan.notes:
        print(f"          {plan.notes}")
    print(f"          {len(changes)} geom state change(s) applied")

    # The route is read from the scene's own nav_/inspect_ sites, so it can only
    # contain positions the laboratory model declares and edges that
    # tests/check_scene.py has swept for clearance.
    graph = RouteGraph(controller.mj_model)
    route = graph.patrol_route()

    report = InspectionRun(scenario_name=plan.name,
                           scenario_title=plan.title,
                           panel=plan.panel_readings)

    navigator = WaypointNavigator(route, on_inspect=report.record)
    controller.set_navigator(navigator.update)

    stations = [wp["name"] for wp in route if wp["inspect"]]
    print(f"Route: {len(route)} waypoints, {len(stations)} inspection stations")
    for name in stations:
        station = inspection.BY_SITE[name]
        print(f"  - {station.label}  [{station.standard}, {station.cadence}]")
    print("Launching autonomous patrol...")

    try:
        controller.run()
    finally:
        # Print whatever was inspected even if the run was interrupted or the
        # safety fallback stopped the controller part-way round.
        if report.results:
            print(report.render())
            if args.report:
                print(f"report written to {report.write_json(args.report)}")
        else:
            print("\nNo stations were inspected.")


if __name__ == "__main__":
    main()

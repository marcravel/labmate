import sys
import importlib
import pkgutil
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

# Importing this registers the "labmate_inspection" task: the stock k1_walk
# config with the robot spawned inside the LabMate laboratory scene.
from shared.task_cfg import TASK_NAME


def main():
    print("=" * 60)
    print("  LabMate - Autonomous Laboratory Inspection Patrol")
    print("=" * 60)

    task_cfg = get_task(TASK_NAME)
    task_cfg.policy.device = "cpu"

    controller = LabMateController(task_cfg)

    # The route is read from the scene's own nav_/inspect_ sites, so it can only
    # contain positions the laboratory model declares and edges that
    # tests/check_scene.py has swept for clearance.
    graph = RouteGraph(controller.mj_model)
    route = graph.patrol_route()

    navigator = WaypointNavigator(route)
    controller.set_navigator(navigator.update)

    stations = [wp["name"] for wp in route if wp["inspect"]]
    print(f"Route: {len(route)} waypoints, {len(stations)} inspection stations")
    for name in stations:
        print(f"  - {name}")
    print("Launching autonomous patrol...")
    controller.run()


if __name__ == "__main__":
    main()

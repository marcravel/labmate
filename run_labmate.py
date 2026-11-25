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
from robot_control.waypoint_navigator import WaypointNavigator

# Define Lab Inspection Route
PATROL_ROUTE = [
    {"name": "Station 1 (Chemical Storage)", "x": 1.20, "y": 0.00, "yaw": 0.00},
    {"name": "Station 2 (Lab Bench)",         "x": 1.20, "y": 0.80, "yaw": 1.57},
    {"name": "Home Base",                    "x": 0.00, "y": 0.00, "yaw": 3.14},
]

# Experimental Hackathon Lab Demo Route
LAB_DEMO_ROUTE = [
    {"name": "Home",         "x": 0.0, "y": 0.0, "yaw": 0.0,    "inspect": False},
    {"name": "Checkpoint 1", "x": 3.8, "y": 0.0, "yaw": 0.0,    "inspect": True},
    {"name": "Checkpoint 2", "x": 3.8, "y": 1.8, "yaw": 1.5708, "inspect": True},
    {"name": "Checkpoint 3", "x": 0.0, "y": 1.8, "yaw": 3.1416, "inspect": True},
    {"name": "Return Home",  "x": 0.0, "y": 0.0, "yaw": 0.0,    "inspect": False},
]

def main():
    print("=" * 60)
    print("  LabMate - Autonomous Waypoint Navigation (Milestone 3)")
    print("=" * 60)

    task_cfg = get_task("k1_walk")
    task_cfg.policy.device = "cpu"

    controller = LabMateController(task_cfg)
    navigator = WaypointNavigator(LAB_DEMO_ROUTE)

    # Attach autonomous navigator
    controller.set_navigator(navigator.update)

    print(f"Patrol initialized with {len(LAB_DEMO_ROUTE)} checkpoints.")
    print("Launching autonomous patrol...")
    controller.run()

if __name__ == "__main__":
    main()

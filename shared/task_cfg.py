"""LabMate task configuration.

Booster Deploy registers the stock ``k1_walk`` task, whose robot config points at
the bare ``K1_22dof.xml``. LabMate needs the same robot inside the laboratory
scene, so this module registers a second task that differs from ``k1_walk`` in
exactly one field: ``robot.mjcf_path``.

Nothing under ``external/`` is modified to achieve this. Per AGENTS.md section F,
the override is a new ``@configclass`` subclass using the ``.replace(...)``
pattern already established by ``K1WalkControllerCfg``.

Importing this module performs the registration, so it must be imported after
``external/booster_deploy`` is on ``sys.path``.
"""

from pathlib import Path

from booster_deploy.utils.isaaclab.configclass import configclass
from booster_deploy.utils.registry import register_task
from tasks.locomotion.robots.k1 import K1WalkTaskCfg

PROJECT_ROOT = Path(__file__).resolve().parents[1]

LAB_SCENE_PATH = PROJECT_ROOT / "simulation" / "scenes" / "labmate_scene.xml"
"""Top-level MJCF loaded by MujocoController: K1 + the laboratory environment."""

TASK_NAME = "labmate_inspection"


@configclass
class LabMateInspectionTaskCfg(K1WalkTaskCfg):
    """``k1_walk`` with the robot spawned inside the LabMate laboratory scene.

    Every other field - policy checkpoint, PD gains, effort limits, velocity
    limits, physics timestep - is inherited unchanged, so the official walking
    policy runs against the model it was trained on.
    """

    def __post_init__(self):
        super().__post_init__()
        if not LAB_SCENE_PATH.is_file():
            raise FileNotFoundError(
                f"LabMate scene not found at {LAB_SCENE_PATH}. It is tracked in "
                "git under simulation/scenes/; the K1 robot and meshes it "
                "includes come from external/booster_assets/."
            )
        # MujocoController expands only the {BOOSTER_ASSETS_DIR} placeholder, so
        # an absolute path is passed through untouched.
        self.robot = self.robot.replace(mjcf_path=str(LAB_SCENE_PATH))


register_task(TASK_NAME, LabMateInspectionTaskCfg())

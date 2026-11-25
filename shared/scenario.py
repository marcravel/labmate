"""Scenario loading: arrange the laboratory's inspectable state before a run.

A scenario is a JSON file under ``simulation/scenarios/``. It names lab geoms
and sets their material, pose or visibility, so a demo can be shot twice - once
all-clear, once with findings - from one build and one route, with nothing else
changed. Determinism is the point: the robot walks the same path and the report
differs only where the scenario differs.

Scenarios are applied to ``controller.mj_model`` after the controller has
loaded the scene and before ``run()``. Only the laboratory's visual geoms can
be touched:

* Lab geoms carry ``group`` 2 (``lab_vis``) or 3 (``lab_col``) from the classes
  in ``lab_environment.xml``; robot geoms do not. Anything else is refused, so
  a scenario can never reach the robot's model and invalidate the walking
  policy's view of it.
* Collision geoms are refused as well. Route clearances are swept once by
  ``tests/check_scene.py``; letting a scenario move an obstacle would silently
  invalidate that sweep. Inspection is visual, so visual geoms suffice.

Hiding a geom detaches its material and zeroes its alpha. The laboratory's
bodies are static and carry no degrees of freedom - that invariant is what
keeps the official policy seeing the model it was trained on - so a geom cannot
be removed or animated any other way.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCENARIO_DIR = PROJECT_ROOT / "simulation" / "scenarios"
DEFAULT_SCENARIO = "nominal"

LAB_GEOM_GROUPS = (2, 3)
VISUAL_GEOM_GROUP = 2

_KEYS = {"material", "pos", "quat", "visible"}


@dataclass
class Scenario:
    name: str
    title: str
    notes: str = ""
    geoms: dict[str, dict] = field(default_factory=dict)
    panel: dict[str, str] = field(default_factory=dict)
    source: Path | None = None

    @property
    def panel_readings(self) -> dict[str, str]:
        """Instrument readings this scenario *declares*.

        These are labels for the state the scenario set up, not measurements
        produced by the simulation. Reports print them as declared so no one
        mistakes them for simulated physics.
        """
        return dict(self.panel)


def available() -> list[str]:
    if not SCENARIO_DIR.is_dir():
        return []
    return sorted(p.stem for p in SCENARIO_DIR.glob("*.json"))


def load(name: str = DEFAULT_SCENARIO) -> Scenario:
    """Read a scenario by name (or by path) from simulation/scenarios/."""
    path = Path(name)
    if not path.is_file():
        path = SCENARIO_DIR / f"{Path(name).stem}.json"
    if not path.is_file():
        raise FileNotFoundError(
            f"no scenario {name!r} in {SCENARIO_DIR} (have: {', '.join(available()) or 'none'})")

    raw = json.loads(path.read_text())
    unknown = {k for spec in raw.get("geoms", {}).values() for k in spec} - _KEYS
    if unknown:
        raise ValueError(f"{path.name}: unknown geom keys {sorted(unknown)}; "
                         f"allowed: {sorted(_KEYS)}")

    return Scenario(
        name=raw.get("name", path.stem),
        title=raw.get("title", path.stem),
        notes=raw.get("notes", ""),
        geoms=raw.get("geoms", {}),
        panel=raw.get("panel", {}),
        source=path,
    )


def _check_writable(model, geom_name: str) -> int:
    try:
        gid = int(model.geom(geom_name).id)
    except KeyError as exc:
        raise ValueError(f"scenario names a geom absent from the model: "
                         f"{geom_name}") from exc

    group = int(model.geom_group[gid])
    if group not in LAB_GEOM_GROUPS:
        raise ValueError(
            f"refusing to modify {geom_name!r}: group {group} is not a "
            f"laboratory geom group {LAB_GEOM_GROUPS}. Scenarios may only "
            f"touch the laboratory, never the robot model.")
    if group != VISUAL_GEOM_GROUP or int(model.geom_contype[gid]):
        raise ValueError(
            f"refusing to modify {geom_name!r}: it is a collision geom. "
            f"Route clearances are swept once by tests/check_scene.py; moving "
            f"an obstacle at runtime would invalidate that sweep.")
    return gid


def apply(scenario: Scenario, model) -> list[str]:
    """Apply a scenario to a compiled model. Returns a log of what changed.

    Idempotent for a given scenario, but not cumulative across scenarios: each
    scenario must declare the full state of every geom it cares about, which is
    also what makes the JSON readable as a description of the lab.
    """
    changes: list[str] = []

    for geom_name, spec in scenario.geoms.items():
        gid = _check_writable(model, geom_name)

        if "material" in spec:
            material = spec["material"]
            try:
                matid = int(model.material(material).id)
            except KeyError as exc:
                raise ValueError(f"{geom_name}: unknown material "
                                 f"{material!r}") from exc
            if int(model.geom_matid[gid]) != matid:
                changes.append(f"{geom_name}.material -> {material}")
            model.geom_matid[gid] = matid
            model.geom_rgba[gid] = model.material(material).rgba

        if "pos" in spec:
            pos = np.asarray(spec["pos"], dtype=float)
            if pos.shape != (3,):
                raise ValueError(f"{geom_name}: pos must be 3 numbers")
            if not np.allclose(model.geom_pos[gid], pos):
                changes.append(f"{geom_name}.pos -> {list(np.round(pos, 3))}")
            model.geom_pos[gid] = pos

        if "quat" in spec:
            quat = np.asarray(spec["quat"], dtype=float)
            if quat.shape != (4,):
                raise ValueError(f"{geom_name}: quat must be 4 numbers (w x y z)")
            norm = float(np.linalg.norm(quat))
            if norm <= 0:
                raise ValueError(f"{geom_name}: quat has zero norm")
            model.geom_quat[gid] = quat / norm

        if "visible" in spec:
            visible = bool(spec["visible"])
            was_hidden = (int(model.geom_matid[gid]) < 0
                          and float(model.geom_rgba[gid][3]) <= 0.0)
            if visible:
                if "material" not in spec and was_hidden:
                    raise ValueError(
                        f"{geom_name}: visible=true needs a material, because "
                        f"hiding a geom detaches the one it had")
                if was_hidden:
                    changes.append(f"{geom_name} shown")
            else:
                if not was_hidden:
                    changes.append(f"{geom_name} hidden")
                model.geom_matid[gid] = -1
                model.geom_rgba[gid] = [0.0, 0.0, 0.0, 0.0]

    return changes


def apply_named(name: str, model) -> tuple[Scenario, list[str]]:
    scenario = load(name)
    return scenario, apply(scenario, model)

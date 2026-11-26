"""What LabMate inspects, and how a verdict is reached.

Every check reads the *simulation* - a geom's material, pose or visibility - and
never the scenario file that set that state up. A scenario arranges the lab; the
checks then observe it, exactly as a camera-based detector would later. Swapping
``SegmentationDetector`` or YOLO in behind these same checks therefore changes
how the state is *observed*, not what counts as a pass.

Two deliberate limits, so the reports do not overclaim:

* A check reports what an indicator *shows*, not a measured physical quantity.
  The freezer check reads a panel that is in range or in alarm; it does not
  simulate a thermal model. Instrument readings printed in a report come from
  the scenario's ``panel`` block and are labelled as declared, not measured.
* Nothing here proves real-world safety. These are the visual conditions a
  human inspector records on a round, reproduced in simulation.

``STATIONS`` is the single source of truth for the checklist. Each entry's
``site`` must be an ``inspect_*`` site in the MJCF, and every geom it names must
exist; :func:`validate` asserts both against a compiled model.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

NOMINAL_MATERIAL = "mat_screen_green"
"""Indicator material meaning "this reads normal"."""

# Severity ordering matters: a station's verdict is its worst check.
PASS = "PASS"
ADVISORY = "ADVISORY"
FINDING = "FINDING"
CRITICAL = "CRITICAL"

_SEVERITY_RANK = {PASS: 0, ADVISORY: 1, FINDING: 2, CRITICAL: 3}


def worst(severities) -> str:
    """The most serious of several severities, or PASS if there are none."""
    return max(severities, key=lambda s: _SEVERITY_RANK[s], default=PASS)


# --------------------------------------------------------------------------
# Results
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class CheckResult:
    key: str
    description: str
    severity: str
    observed: str

    @property
    def ok(self) -> bool:
        return self.severity == PASS


@dataclass(frozen=True)
class StationResult:
    site: str
    label: str
    standard: str
    cadence: str
    checks: tuple[CheckResult, ...]
    sim_time: float

    @property
    def severity(self) -> str:
        return worst(c.severity for c in self.checks)

    @property
    def ok(self) -> bool:
        return self.severity == PASS

    @property
    def failures(self) -> tuple[CheckResult, ...]:
        return tuple(c for c in self.checks if not c.ok)


# --------------------------------------------------------------------------
# Checks
# --------------------------------------------------------------------------

def _material_name(model, geom_name: str) -> str | None:
    matid = int(model.geom(geom_name).matid[0])
    return None if matid < 0 else model.material(matid).name


def _is_hidden(model, geom_name: str) -> bool:
    """True when a geom has been made invisible by a scenario.

    ``shared.scenario`` hides a geom by detaching its material and zeroing the
    alpha, which is the only way to remove something from view in a scene whose
    bodies must stay static and DOF-free.
    """
    gid = model.geom(geom_name).id
    return int(model.geom_matid[gid]) < 0 and float(model.geom_rgba[gid][3]) <= 0.0


def _world_pos(model, data, geom_name: str) -> np.ndarray:
    gid = model.geom(geom_name).id
    if data is not None:
        return np.asarray(data.geom_xpos[gid], dtype=float)
    return np.asarray(model.geom_pos[gid], dtype=float)


@dataclass(frozen=True)
class Check:
    """Base check. Subclasses implement :meth:`observe`."""

    key: str
    description: str
    severity: str = FINDING

    def geoms(self) -> tuple[str, ...]:
        raise NotImplementedError

    def observe(self, model, data) -> tuple[bool, str]:
        raise NotImplementedError

    def evaluate(self, model, data) -> CheckResult:
        ok, observed = self.observe(model, data)
        return CheckResult(
            key=self.key,
            description=self.description,
            severity=PASS if ok else self.severity,
            observed=observed,
        )


@dataclass(frozen=True)
class IndicatorsNominal(Check):
    """Every named indicator geom shows the nominal material."""

    indicators: tuple[str, ...] = ()
    nominal: str = NOMINAL_MATERIAL

    def geoms(self) -> tuple[str, ...]:
        return self.indicators

    def observe(self, model, data):
        total = len(self.indicators)
        bad = [g for g in self.indicators
               if _material_name(model, g) != self.nominal]
        if not bad:
            return True, f"{total}/{total} nominal" if total > 1 else "nominal"
        if total == 1:
            return False, "indicator in alarm"
        return False, (f"{len(bad)} of {total} in alarm "
                       f"({', '.join(bad)})")


@dataclass(frozen=True)
class PoseUnchanged(Check):
    """A geom sits at its reference pose - used for "door latched"."""

    geom: str = ""
    reference: tuple[float, float, float] = (0.0, 0.0, 0.0)
    tolerance: float = 0.02

    def geoms(self) -> tuple[str, ...]:
        return (self.geom,)

    def observe(self, model, data):
        offset = float(np.linalg.norm(
            _world_pos(model, data, self.geom) - np.asarray(self.reference)))
        if offset <= self.tolerance:
            return True, "closed"
        return False, f"displaced {offset * 100:.0f} cm"


@dataclass(frozen=True)
class HeightAtMost(Check):
    """A geom's centre is no higher than a limit - used for "sash down"."""

    geom: str = ""
    limit: float = 0.0
    closed_height: float = 0.0

    def geoms(self) -> tuple[str, ...]:
        return (self.geom,)

    def observe(self, model, data):
        z = float(_world_pos(model, data, self.geom)[2])
        if z <= self.limit:
            return True, "lowered"
        raised = z - self.closed_height
        return False, f"raised {raised * 100:.0f} cm"


@dataclass(frozen=True)
class Absent(Check):
    """A geom is not present in the scene - used for "nothing blocking"."""

    geom: str = ""
    present_as: str = "present"

    def geoms(self) -> tuple[str, ...]:
        return (self.geom,)

    def observe(self, model, data):
        hidden = _is_hidden(model, self.geom)
        return hidden, "clear" if hidden else self.present_as


@dataclass(frozen=True)
class Present(Check):
    """A geom is present - used for "the cable is still plugged in"."""

    geom: str = ""
    absent_as: str = "missing"
    present_as: str = "connected"

    def geoms(self) -> tuple[str, ...]:
        return (self.geom,)

    def observe(self, model, data):
        hidden = _is_hidden(model, self.geom)
        return not hidden, self.absent_as if hidden else self.present_as


# --------------------------------------------------------------------------
# Stations
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class Station:
    site: str
    label: str
    standard: str
    cadence: str
    checks: tuple[Check, ...] = field(default_factory=tuple)
    target: str = ""
    """Site the robot should look at, when it is not ``target_<station>``.

    A station usually takes its look-at point from its own name, which is what
    ``RouteGraph`` assumes. This names one explicitly for the case where a
    station has been built at a site that was originally something else - the
    fire extinguisher occupies what was the printer's standing position, so
    ``target_printer`` points at the printer and not at the extinguisher being
    inspected. Getting this wrong aims the robot's head, and so its cameras, at
    the wrong equipment.
    """

    def inspect(self, model, data, sim_time: float = 0.0) -> StationResult:
        return StationResult(
            site=self.site,
            label=self.label,
            standard=self.standard,
            cadence=self.cadence,
            checks=tuple(c.evaluate(model, data) for c in self.checks),
            sim_time=float(sim_time),
        )


STATIONS: tuple[Station, ...] = (
    Station(
        site="inspect_electronics_a",
        label="Emergency eyewash and drench shower",
        standard="ANSI/ISEA Z358.1",
        cadence="weekly activation",
        checks=(
            Absent(
                key="access_clear",
                description="Access to the unit is unobstructed",
                severity=FINDING,
                geom="eyewash_obstruction",
                present_as="obstruction in the keep-clear zone",
            ),
            IndicatorsNominal(
                key="tag_current",
                description="Weekly activation tag is current",
                severity=ADVISORY,
                indicators=("eyewash_tag",),
            ),
        ),
    ),
    Station(
        site="inspect_electronics_b",
        label="Chemical fume hood",
        standard="ASHRAE 110 / local sash policy",
        cadence="continuous",
        checks=(
            HeightAtMost(
                key="sash_lowered",
                description="Sash is lowered to the working position",
                severity=FINDING,
                geom="hood_sash",
                limit=1.45,
                closed_height=1.317,
            ),
            IndicatorsNominal(
                key="airflow_nominal",
                description="Face velocity monitor reads in range",
                severity=FINDING,
                indicators=("hood_flow_led",),
            ),
        ),
    ),
    Station(
        site="inspect_server_rack",
        label="Equipment status panel",
        standard="local maintenance policy",
        cadence="continuous",
        checks=(
            IndicatorsNominal(
                key="unit_leds",
                description="All six unit status LEDs are nominal",
                severity=FINDING,
                indicators=tuple(f"rack_led{i}" for i in range(6)),
            ),
        ),
    ),
    Station(
        site="inspect_storage",
        label="Ultra-low temperature sample freezer",
        standard="CAP/CLIA; CDC storage guidance",
        cadence="daily",
        checks=(
            IndicatorsNominal(
                key="temperature_in_range",
                description="Controller panel reads in range",
                severity=CRITICAL,
                indicators=("freezer_display",),
            ),
            PoseUnchanged(
                key="door_latched",
                description="Door is closed and latched",
                severity=CRITICAL,
                geom="freezer_door",
                reference=(6.585, 0.0, 0.86),
            ),
            Present(
                key="alarm_cable",
                description="Alarm probe cable is connected",
                severity=CRITICAL,
                geom="freezer_cable_ok",
                absent_as="disconnected, lying on the floor",
            ),
            IndicatorsNominal(
                key="alarm_armed",
                description="Independent alarm module is armed",
                severity=FINDING,
                indicators=("freezer_alarm_led",),
            ),
        ),
    ),
    Station(
        site="inspect_printer",
        label="Portable fire extinguisher",
        standard="29 CFR 1910.157(e)(2); NFPA 10 4-3.1",
        cadence="monthly (30-day intervals)",
        # The site is the printer's, the station is the extinguisher's.
        target="target_printer_extinguisher",
        checks=(
            Present(
                key="unit_present",
                description="Extinguisher is mounted in its bracket",
                severity=CRITICAL,
                geom="printer_ext_body",
                absent_as="missing from bracket",
                present_as="in its bracket",
            ),
            Absent(
                key="access_clear",
                description="Access to the extinguisher is unobstructed",
                severity=FINDING,
                geom="printer_ext_obstruction",
                present_as="access obstructed",
            ),
        ),
    ),
    Station(
        site="inspect_robotics_bench",
        label="Flammables storage cabinet",
        standard="29 CFR 1910.1450",
        cadence="routine",
        checks=(
            PoseUnchanged(
                key="door_l_latched",
                description="Left cabinet door is latched",
                severity=FINDING,
                geom="chem_door_l",
                reference=(1.30, -2.345, 0.60),
            ),
            PoseUnchanged(
                key="door_r_latched",
                description="Right cabinet door is latched",
                severity=FINDING,
                geom="chem_door_r",
                reference=(2.30, -2.345, 0.60),
            ),
            Absent(
                key="containment",
                description="No liquid outside secondary containment",
                severity=FINDING,
                geom="chem_spill",
                present_as="liquid pooled outside the tray",
            ),
        ),
    ),
)

BY_SITE: dict[str, Station] = {s.site: s for s in STATIONS}

STATION_ORDER: tuple[str, ...] = tuple(s.site for s in STATIONS)
"""Checklist order, chosen to read as a round: north-west, north, north-east,
east, south-east, south-west. ``robot_control.route_graph.DEFAULT_STATION_ORDER``
follows it, so the walked order and the reported order cannot drift apart."""


def all_state_geoms() -> tuple[str, ...]:
    """Every geom any check reads. Scenarios may only touch these."""
    seen: list[str] = []
    for station in STATIONS:
        for check in station.checks:
            for geom in check.geoms():
                if geom not in seen:
                    seen.append(geom)
    return tuple(seen)


def validate(model) -> None:
    """Assert the checklist matches a compiled model. Raises ValueError."""
    problems: list[str] = []

    sites = {model.site(i).name for i in range(model.nsite)}
    for station in STATIONS:
        if station.site not in sites:
            problems.append(f"station site absent from model: {station.site}")
        if station.target and station.target not in sites:
            problems.append(
                f"station names a missing look-at site: "
                f"{station.site} -> {station.target}")

    geoms = {model.geom(i).name for i in range(model.ngeom)}
    for station in STATIONS:
        if not station.checks:
            problems.append(f"station has no checks: {station.site}")
        for check in station.checks:
            for geom in check.geoms():
                if geom not in geoms:
                    problems.append(
                        f"{station.site}/{check.key} reads a missing geom: {geom}")

    if problems:
        raise ValueError("inspection checklist does not match the model:\n  "
                         + "\n  ".join(problems))


def inspect_site(site: str, model, data, sim_time: float = 0.0):
    """Run the checklist for one site, or None if the site has no station."""
    station = BY_SITE.get(site)
    return None if station is None else station.inspect(model, data, sim_time)

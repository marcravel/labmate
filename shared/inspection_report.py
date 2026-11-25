"""Collects station verdicts during a patrol and renders the round's report.

This is the artefact the inspection exists to produce. A human doing this round
signs a sheet; the robot produces the same sheet, with the simulated time each
station was observed.

Nothing here decides anything. Verdicts come from ``shared.inspection``, which
reads the simulation. This module only accumulates and formats them.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from shared.inspection import (
    ADVISORY,
    CRITICAL,
    FINDING,
    PASS,
    StationResult,
    inspect_site,
    worst,
)

_MARK = {PASS: "PASS", ADVISORY: "ADVS", FINDING: "FIND", CRITICAL: "CRIT"}


@dataclass
class InspectionRun:
    """Accumulates station results for one patrol."""

    scenario_name: str = "unknown"
    scenario_title: str = ""
    panel: dict[str, str] = field(default_factory=dict)
    results: list[StationResult] = field(default_factory=list)
    verbose: bool = True

    # -- recording --------------------------------------------------------

    def record(self, controller, waypoint, elapsed: float) -> StationResult | None:
        """Navigator hook. Runs the checklist for the station in front of us.

        Signature matches ``WaypointNavigator``'s ``on_inspect`` callback:
        ``(controller, waypoint, elapsed)``.
        """
        result = inspect_site(
            waypoint["name"], controller.mj_model, controller.mj_data, elapsed)
        if result is None:
            return None
        self.results.append(result)
        if self.verbose:
            print(self.render_station(result))
        return result

    # -- summary ----------------------------------------------------------

    @property
    def severity(self) -> str:
        return worst(r.severity for r in self.results)

    @property
    def findings(self) -> list[tuple[StationResult, object]]:
        return [(r, c) for r in self.results for c in r.failures]

    def counts(self) -> dict[str, int]:
        out = {PASS: 0, ADVISORY: 0, FINDING: 0, CRITICAL: 0}
        for r in self.results:
            out[r.severity] += 1
        return out

    # -- rendering --------------------------------------------------------

    def render_station(self, result: StationResult) -> str:
        head = (f"[Inspect] {_MARK[result.severity]}  {result.label}"
                f"   t={result.sim_time:6.1f}s")
        lines = [head, f"          standard: {result.standard}"
                       f"  ({result.cadence})"]
        for check in result.checks:
            flag = "ok " if check.ok else "!! "
            lines.append(f"          {flag}{check.description}: {check.observed}")
            reading = self.panel.get(check.key)
            if reading:
                lines.append(f"              panel (declared by scenario): {reading}")
        return "\n".join(lines)

    def render(self) -> str:
        counts = self.counts()
        width = 72
        out = ["", "=" * width,
               "LabMate inspection round",
               f"scenario : {self.scenario_name}"
               + (f" - {self.scenario_title}" if self.scenario_title else ""),
               f"stations : {len(self.results)} inspected",
               f"result   : {self.severity}"
               f"   (pass {counts[PASS]}, advisory {counts[ADVISORY]},"
               f" finding {counts[FINDING]}, critical {counts[CRITICAL]})",
               "=" * width]

        for result in self.results:
            out.append(f"{_MARK[result.severity]:>4}  {result.label}"
                       f"  [{result.sim_time:.1f}s]")
            for check in result.checks:
                mark = "      ok" if check.ok else f"      {_MARK[check.severity]}"
                out.append(f"{mark}  {check.description}: {check.observed}")

        if self.findings:
            out += ["-" * width, "Action required:"]
            for result, check in sorted(
                    self.findings,
                    key=lambda rc: -_rank(rc[1].severity)):
                out.append(f"  [{check.severity}] {result.label}"
                           f" - {check.description}: {check.observed}")
                out.append(f"            standard: {result.standard}"
                           f" ({result.cadence})")
        else:
            out += ["-" * width, "No action required. All checks passed."]

        out.append("=" * width)
        return "\n".join(out)

    def to_dict(self) -> dict:
        return {
            "scenario": self.scenario_name,
            "scenario_title": self.scenario_title,
            "overall": self.severity,
            "counts": self.counts(),
            "panel_readings_declared_by_scenario": dict(self.panel),
            "stations": [
                {
                    "site": r.site,
                    "label": r.label,
                    "standard": r.standard,
                    "cadence": r.cadence,
                    "sim_time_s": round(r.sim_time, 2),
                    "severity": r.severity,
                    "checks": [
                        {
                            "key": c.key,
                            "description": c.description,
                            "severity": c.severity,
                            "observed": c.observed,
                        }
                        for c in r.checks
                    ],
                }
                for r in self.results
            ],
        }

    def write_json(self, path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), indent=2) + "\n")
        return path


def _rank(severity: str) -> int:
    return {PASS: 0, ADVISORY: 1, FINDING: 2, CRITICAL: 3}[severity]

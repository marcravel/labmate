#!/usr/bin/env python3
"""Checks that generated routes stay on the validated navigation graph.

Run:  python3 tests/test_route_graph.py

Written in the same standalone style as check_scene.py so the project keeps its
mujoco + numpy only dependency set. Geometric checks here are cheap point tests;
check_scene.py remains the authority on full-body clearance.

This exists because a hand-typed route in run_labmate.py once placed a waypoint
at (3.8, 0.0), inside the central table's collision box. Routes are now read
from the model's own sites, and the checks below make that failure mode
unrepeatable.
"""
import sys
from pathlib import Path

import numpy as np
import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from robot_control.route_graph import (  # noqa: E402
    DEFAULT_STATION_ORDER,
    ROUTE_EDGES,
    RouteGraph,
    normalize_angle,
)

LAB_SCENE = PROJECT_ROOT / "simulation" / "scenes" / "labmate_scene.xml"

RETIRED_WAYPOINT = (3.8, 0.0)
"""The hand-typed "Checkpoint 1" that sat inside col_ctable."""

fails = 0


def check(label, ok, detail=""):
    global fails
    fails += not ok
    print(("PASS " if ok else "FAIL ") + label + (f"  ({detail})" if detail else ""))


print(f"lab scene : {LAB_SCENE}\n")
model = mujoco.MjModel.from_xml_path(str(LAB_SCENE))
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
graph = RouteGraph(model)

VALID = {frozenset(e) for e in ROUTE_EDGES}


def edges_of(waypoints):
    names = [wp["name"] for wp in waypoints]
    return [frozenset(p) for p in zip(names, names[1:])]


# 1. The graph matches the model
site_nodes = {
    model.site(i).name
    for i in range(model.nsite)
    if model.site(i).name.startswith(("nav_", "inspect_"))
}
check("graph nodes are exactly the nav_/inspect_ sites in the model",
      set(graph.nodes) == site_nodes,
      f"{len(graph.nodes)} nodes vs {len(site_nodes)} sites")
# Not every inspection site is on the round. A site with no checklist entry
# (the central table) stays in the model with validated, swept edges so it can
# be added to a checklist later without navigation work -- which is exactly how
# inspect_printer became the fire-extinguisher station: no navigation change.
check(f"{len(graph.stations)} inspection sites in the model, "
      f"{len(DEFAULT_STATION_ORDER)} on the round",
      set(DEFAULT_STATION_ORDER) <= set(graph.stations),
      f"off the round: {sorted(set(graph.stations) - set(DEFAULT_STATION_ORDER))}")

# 2. The patrol route travels only validated edges
route = graph.patrol_route()
bad = [sorted(e) for e in edges_of(route) if e not in VALID]
check(f"patrol route: all {len(edges_of(route))} transitions are validated edges",
      not bad, str(bad) if bad else "")

# 3. Every waypoint is a site position, not an invented coordinate
bad = []
for wp in route:
    site = data.site(wp["name"]).xpos[:2]
    if not np.allclose(site, [wp["x"], wp["y"]], atol=1e-9):
        bad.append(wp["name"])
check("every waypoint position comes from its site in the model", not bad,
      str(bad) if bad else "")

# 4. The route actually inspects every station on the round, once each
visited = [wp["name"] for wp in route if wp["inspect"]]
check("all stations visited exactly once",
      sorted(visited) == sorted(DEFAULT_STATION_ORDER) and
      len(visited) == len(set(visited)),
      f"{len(visited)} of {len(DEFAULT_STATION_ORDER)}")
check("route starts and ends on the start pad",
      route[0]["name"] == "nav_start" and route[-1]["name"] == "nav_start")

# 5. Headings: stations keep their site heading, corridors aim at the next node
bad = []
for i, wp in enumerate(route):
    if wp["inspect"]:
        want = graph.nodes[wp["name"]]["yaw"]
    elif i + 1 < len(route):
        nxt = route[i + 1]
        want = float(np.arctan2(nxt["y"] - wp["y"], nxt["x"] - wp["x"]))
    else:
        continue
    if abs(normalize_angle(wp["yaw"] - want)) > 1e-9:
        bad.append(wp["name"])
check("waypoint headings are station heading or bearing to next", not bad,
      str(bad) if bad else "")

# 6. No waypoint lands inside a lab collision box (the original bug)
boxes = []
for g in range(model.ngeom):
    name = model.geom(g).name
    if name.startswith("col_") and model.geom_contype[g]:
        boxes.append((name, data.geom_xpos[g][:2].copy(),
                      model.geom_size[g][:2].copy()))


def inside(point):
    p = np.asarray(point, dtype=float)
    return [n for n, c, h in boxes if np.all(np.abs(p - c) <= h)]


bad = {wp["name"]: inside((wp["x"], wp["y"])) for wp in route
       if inside((wp["x"], wp["y"]))}
check(f"no waypoint inside any of the {len(boxes)} lab collision boxes",
      not bad, str(bad) if bad else "")

# 7. Regression: the retired hand-typed waypoint was genuinely inside the table,
#    and nothing generated from the model reproduces it.
occupied = inside(RETIRED_WAYPOINT)
check("control: the retired waypoint (3.8, 0.0) is inside lab geometry",
      bool(occupied), str(occupied))
check("no generated waypoint reproduces the retired coordinate",
      not any(np.allclose([wp["x"], wp["y"]], RETIRED_WAYPOINT, atol=1e-6)
              for wp in route))

# 8. Planning between every pair of nodes stays on validated edges
names = sorted(graph.nodes)
bad, unreachable = [], []
for a in names:
    for b in names:
        try:
            path = graph.plan(a, b)
        except ValueError:
            unreachable.append((a, b))
            continue
        bad += [sorted(e) for e in
                (frozenset(p) for p in zip(path, path[1:])) if e not in VALID]
check(f"plan() over all {len(names)**2} node pairs uses only validated edges",
      not bad, str(bad[:4]) if bad else "")
check("every node pair is reachable", not unreachable,
      str(unreachable[:4]) if unreachable else "")
check("plan() is symmetric in length",
      all(len(graph.plan(a, b)) == len(graph.plan(b, a))
          for a in names for b in names))

# 9. nearest_node resolves a pose to a corridor node
check("nearest_node(0, 0) is the start pad",
      graph.nearest_node(0.0, 0.0) == "nav_start",
      graph.nearest_node(0.0, 0.0))
check("nearest_node returns a corridor node by default",
      all(graph.nearest_node(*graph.position(n)).startswith("nav_")
          for n in names))

print(f"\n{fails} failure(s)")
sys.exit(bool(fails))

"""Navigation graph read from the MuJoCo model.

The laboratory MJCF defines the validated navigation graph as sites:

* ``nav_*``      - corridor nodes. Position matters; heading does not.
* ``inspect_*``  - standing positions in front of a station. The site
                   quaternion encodes the heading the robot must face.
* ``target_*``   - the 3-D point on the equipment that should be looked at.
                   Not part of the graph; used later by perception.

Routes used to be hand-typed coordinate lists in ``run_labmate.py``. They drifted
from the model, and one waypoint ended up inside the central table's collision
box. Reading the graph from the model removes that whole class of bug: a route
can only contain positions the scene itself declares, and
``tests/test_route_graph.py`` asserts that every edge travelled is one
``tests/check_scene.py`` has swept for clearance.

``ROUTE_EDGES`` is the single source of truth for connectivity and is imported by
``tests/check_scene.py``, so the validator and the planner can never disagree.
"""

from __future__ import annotations

from collections import deque

import numpy as np

from shared.inspection import STATION_ORDER

NAV_PREFIX = "nav_"
INSPECT_PREFIX = "inspect_"

_LOOP = ["west", "northwest", "northeast", "east", "southeast", "southwest", "west"]

_SPURS = [
    ("northwest", "electronics_a"),
    ("northeast", "electronics_b"),
    ("northeast", "server_rack"),
    ("southwest", "robotics_bench"),
    ("southeast", "printer"),
    ("east", "storage"),
    ("west", "central_table"),
]

ROUTE_EDGES: tuple[tuple[str, str], ...] = (
    ("nav_outside", "nav_entrance"),
    ("nav_entrance", "nav_start"),
    ("nav_start", "nav_wp_west"),
    *((f"nav_wp_{a}", f"nav_wp_{b}") for a, b in zip(_LOOP, _LOOP[1:])),
    *((f"nav_wp_{w}", f"{INSPECT_PREFIX}{s}") for w, s in _SPURS),
)
"""Undirected edges swept for clearance by tests/check_scene.py."""

DEFAULT_STATION_ORDER: tuple[str, ...] = STATION_ORDER
"""The patrol's stations, in order, taken from the inspection checklist.

Imported rather than restated: the order the robot walks and the order the
report lists have to be the same thing, and two literals would drift. The
checklist is the source of truth for *which* stations exist and in what order;
``ROUTE_EDGES`` remains the source of truth for how to get between them.

``inspect_central_table`` is deliberately not on the round: it is furniture with
no inspectable state. It stays in the model with validated, swept edges so it can
be added to a checklist later without any navigation work.

``inspect_printer`` was held as that kind of spare and has now been spent on the
fire-extinguisher station. Adding a station there cost no edge, no site and no
change to this module - which is the clearest evidence that the checklist and the
navigation graph are genuinely separable."""


def quat_to_yaw(q) -> float:
    """Yaw about +Z from a MuJoCo ``(w, x, y, z)`` quaternion."""
    w, x, y, z = (float(v) for v in q)
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def normalize_angle(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


class RouteGraph:
    """Nodes and connectivity of the validated navigation graph.

    Waypoints are returned as plain dicts with ``name``, ``x``, ``y``, ``yaw``
    and ``inspect`` keys, which is exactly what ``WaypointNavigator`` already
    consumes - no change to the navigator's interface.
    """

    def __init__(self, mj_model):
        self._nodes: dict[str, dict] = {}
        for i in range(mj_model.nsite):
            site = mj_model.site(i)
            name = site.name
            if not name.startswith((NAV_PREFIX, INSPECT_PREFIX)):
                continue
            self._nodes[name] = {
                "name": name,
                "x": float(site.pos[0]),
                "y": float(site.pos[1]),
                "yaw": quat_to_yaw(site.quat),
                "inspect": name.startswith(INSPECT_PREFIX),
            }

        missing = sorted(
            {n for edge in ROUTE_EDGES for n in edge} - set(self._nodes)
        )
        if missing:
            raise ValueError(
                f"ROUTE_EDGES references sites absent from the model: {missing}"
            )

        self._adjacency: dict[str, set[str]] = {n: set() for n in self._nodes}
        for a, b in ROUTE_EDGES:
            self._adjacency[a].add(b)
            self._adjacency[b].add(a)

    # -- introspection ----------------------------------------------------

    @property
    def nodes(self) -> dict[str, dict]:
        return {n: dict(wp) for n, wp in self._nodes.items()}

    @property
    def stations(self) -> list[str]:
        """Inspection sites present in the model, sorted by name."""
        return sorted(n for n in self._nodes if n.startswith(INSPECT_PREFIX))

    def neighbours(self, node: str) -> set[str]:
        return set(self._adjacency[node])

    def position(self, node: str) -> np.ndarray:
        wp = self._nodes[node]
        return np.array([wp["x"], wp["y"]], dtype=float)

    def nearest_node(self, x: float, y: float, nav_only: bool = True) -> str:
        """Graph node closest to a point. Used to plan from the robot's pose."""
        candidates = [
            n for n in self._nodes
            if not nav_only or n.startswith(NAV_PREFIX)
        ]
        here = np.array([x, y], dtype=float)
        return min(candidates, key=lambda n: float(
            np.linalg.norm(self.position(n) - here)))

    # -- planning ---------------------------------------------------------

    def plan(self, start: str, goal: str) -> list[str]:
        """Shortest node path from ``start`` to ``goal``, both inclusive.

        Breadth-first search. The graph has 16 nodes and 16 edges, so a
        weighted planner would buy nothing.
        """
        for node in (start, goal):
            if node not in self._nodes:
                raise KeyError(f"unknown graph node: {node!r}")
        if start == goal:
            return [start]

        previous: dict[str, str | None] = {start: None}
        queue = deque([start])
        while queue:
            node = queue.popleft()
            for nxt in sorted(self._adjacency[node]):
                if nxt in previous:
                    continue
                previous[nxt] = node
                if nxt == goal:
                    path = [goal]
                    while previous[path[-1]] is not None:
                        path.append(previous[path[-1]])
                    return list(reversed(path))
                queue.append(nxt)
        raise ValueError(f"no route from {start!r} to {goal!r}")

    def route(
        self,
        sequence,
        start: str = "nav_start",
        end: str | None = "nav_start",
    ) -> list[dict]:
        """Waypoint list visiting ``sequence`` in order, via validated edges.

        Intermediate corridor nodes are filled in by :meth:`plan`, so callers
        name only the stations they care about.
        """
        stops = [start, *sequence] + ([end] if end is not None else [])

        nodes: list[str] = []
        for a, b in zip(stops, stops[1:]):
            leg = self.plan(a, b)
            nodes.extend(leg if not nodes else leg[1:])

        return self._as_waypoints(nodes)

    def patrol_route(self, stations=None) -> list[dict]:
        """Full inspection round: start pad, every checklist station, back to the pad."""
        return self.route(stations or DEFAULT_STATION_ORDER)

    # -- internals --------------------------------------------------------

    def _as_waypoints(self, nodes: list[str]) -> list[dict]:
        """Attach headings.

        An inspection site keeps the heading its own site quaternion encodes -
        that is the direction the station must be viewed from. A corridor node
        is a pass-through, so it takes the bearing to the next waypoint, which
        makes the navigator's turn at that node the one it actually needs.
        """
        waypoints = [dict(self._nodes[n]) for n in nodes]
        for i, wp in enumerate(waypoints):
            if wp["inspect"] or i + 1 >= len(waypoints):
                continue
            nxt = waypoints[i + 1]
            wp["yaw"] = float(np.arctan2(nxt["y"] - wp["y"], nxt["x"] - wp["x"]))
        return waypoints

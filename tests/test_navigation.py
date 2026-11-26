#!/usr/bin/env python3
"""Checks the head aiming and the reverse-out-of-a-spur decision.

Run:  python3 tests/test_navigation.py

Standalone, in the style of check_scene.py and test_route_graph.py, so the
project keeps its mujoco + numpy dependency set. No policy is loaded and no
physics is stepped: these are checks on geometry and on the navigator's
decisions, which is what can silently go wrong here.

Two invariants are worth protecting.

The head must be able to reach every station's look-at point from the standing
position in front of it. If it cannot, the body has to square up again and the
reverse-out below stops being safe, because the robot would no longer be on the
heading it arrived on.

The reverse-out must fire only where it is safe: at the end of a station spur,
where the next waypoint is close behind and the robot is still on its arrival
heading, so reversing retraces an edge tests/check_scene.py has swept. It must
not fire on the open loop, where reversing would leave the swept corridor.
"""
import sys
from pathlib import Path

import numpy as np
import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "external" / "booster_deploy"))

from robot_control.head_control import (  # noqa: E402
    CAMERA_FOV_V,
    PITCH_JOINT,
    PITCH_MARGIN,
    YAW_JOINT,
    YAW_MARGIN,
)
from robot_control.route_graph import RouteGraph, normalize_angle  # noqa: E402
from robot_control.waypoint_navigator import WaypointNavigator  # noqa: E402

LAB_SCENE = PROJECT_ROOT / "simulation" / "scenes" / "labmate_scene.xml"
HEAD_HEIGHT = 0.80
"""Head body height while standing, measured in simulation."""

fails = 0


def check(label, ok, detail=""):
    global fails
    fails += not ok
    print(("PASS " if ok else "FAIL ") + label + (f"  ({detail})" if detail else ""))


print(f"lab scene : {LAB_SCENE}\n")
model = mujoco.MjModel.from_xml_path(str(LAB_SCENE))
graph = RouteGraph(model)
route = graph.patrol_route()
nav = WaypointNavigator(route)

yaw_lo, yaw_hi = (float(v) for v in model.joint(YAW_JOINT).range)
pitch_lo, pitch_hi = (float(v) for v in model.joint(PITCH_JOINT).range)
yaw_usable = yaw_hi - YAW_MARGIN
pitch_usable = (pitch_lo + PITCH_MARGIN, pitch_hi - PITCH_MARGIN)

# 1. The head is outside what the walking policy drives, which is the whole
#    reason it is ours to aim. If the policy ever gains these joints, aiming
#    the head would be fighting it.
from tasks.locomotion.robots.k1 import K1WalkControllerCfg  # noqa: E402

k1_cfg = K1WalkControllerCfg()
policy_joints = set(k1_cfg.policy.policy_joint_names)
check("the walking policy drives neither head joint",
      not ({YAW_JOINT, PITCH_JOINT} & policy_joints),
      f"{len(policy_joints)} policy joints")

# 2. Every station on the round declares a look-at point
stations = [wp for wp in route if wp["inspect"]]
missing = [wp["name"] for wp in stations if wp.get("target") is None]
check("every station on the round has a target_* site", not missing, str(missing))

# 3. The head can reach each of them from the station's standing position
print()
for wp in stations:
    tx, ty, tz = wp["target"]
    dx, dy = tx - wp["x"], ty - wp["y"]
    yaw_need = normalize_angle(float(np.arctan2(dy, dx)) - wp["yaw"])
    ground = float(np.hypot(dx, dy))
    # Positive aahead_pitch_joint is nose-down, so looking up is negative.
    pitch_need = -float(np.arctan2(tz - HEAD_HEIGHT, ground))
    # Yaw has to be reachable outright: a yaw the head cannot cover is a yaw
    # the body must turn, which is what would make reversing out unsafe.
    # Pitch only has to keep the target inside the camera frame - the joint
    # cannot look as far up as two of these points sit.
    pitch_got = float(np.clip(pitch_need, *pitch_usable))
    in_frame = abs(pitch_need - pitch_got) <= CAMERA_FOV_V / 2.0
    short = ("" if pitch_got == pitch_need
             else f", {np.degrees(abs(pitch_need - pitch_got)):.1f} deg short "
                  f"of level aim but in frame")
    check(f"head reaches {wp['name'][len('inspect_'):]}",
          abs(yaw_need) <= yaw_usable and in_frame,
          f"yaw {np.degrees(yaw_need):+.1f} deg of +-{np.degrees(yaw_usable):.0f}, "
          f"pitch {np.degrees(pitch_need):+.1f} deg, standoff {ground:.2f} m{short}")

# 4. Reversing out fires at the end of every spur, where the next waypoint is
#    the corridor node behind the robot and it is still on its arrival heading.
print()
for i, wp in enumerate(stations):
    idx = route.index(wp)
    if idx + 1 >= len(route):
        continue
    nxt = route[idx + 1]
    prev = route[idx - 1]
    arrive = float(np.arctan2(wp["y"] - prev["y"], wp["x"] - prev["x"]))
    check(f"reverses out of {wp['name'][len('inspect_'):]}",
          nav._can_back_out(nxt, prev, wp["x"], wp["y"], arrive),
          f"back to {nxt['name']}, "
          f"{np.hypot(nxt['x']-wp['x'], nxt['y']-wp['y']):.2f} m behind")

# 5. and never on the open loop, where reversing would leave the swept corridor
print()
offenders = []
loop_cases = 0
for i, wp in enumerate(route[:-1]):
    if wp["inspect"]:
        continue
    nxt = route[i + 1]
    prev = route[i - 1] if i else None
    arrive = (float(np.arctan2(wp["y"] - prev["y"], wp["x"] - prev["x"]))
              if prev else 0.0)
    loop_cases += 1
    if nav._can_back_out(nxt, prev, wp["x"], wp["y"], arrive):
        offenders.append(f"{wp['name']} -> {nxt['name']}")
check("no corridor node reverses instead of walking on",
      not offenders, f"{loop_cases} checked, offenders: {offenders}")

# 6. Reversing anywhere the next waypoint is not the one just left would strike
#    out across floor nobody swept. That is the guard that must not regress.
print()
elsewhere = []
for i, wp in enumerate(route[:-1]):
    nxt, prev = route[i + 1], (route[i - 1] if i else None)
    if prev is not None and nxt["name"] == prev["name"]:
        continue            # a genuine doubling back: reversing is the retrace
    for heading in np.linspace(-np.pi, np.pi, 24):
        if nav._can_back_out(nxt, prev, wp["x"], wp["y"], float(heading)):
            elsewhere.append(f"{wp['name']} -> {nxt['name']}")
            break
check("never reverses towards a waypoint it did not just come from",
      not elsewhere, f"offenders: {elsewhere}")

# 7. The reversing speed stays inside what the policy accepts
check("reversing speed is within vx_backward_max",
      nav.back_out_speed <= k1_cfg.vel_command.vx_backward_max,
      f"{nav.back_out_speed} of {k1_cfg.vel_command.vx_backward_max} m/s")

print(f"\n{fails} failure(s)")
sys.exit(bool(fails))

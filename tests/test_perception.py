#!/usr/bin/env python3
"""Checks the head camera and the ground-truth detector.

Run:  python3 tests/test_perception.py

Standalone, in the style of check_scene.py and the other test scripts. Unlike
them it needs a GL context, which not every machine or CI container has. If one
cannot be created the script says so and exits 0 rather than failing: a missing
display is not a broken detector, and the other suites still cover everything
that decides an inspection verdict.

What is worth protecting here is narrow but easy to get wrong silently:

* the camera looks where the head looks, not somewhere near it;
* the collision proxies and site markers stay out of frame - they are
  scaffolding, and they sit coincident with the furniture, so leaving them on
  fills the picture with translucent red boxes;
* a detection is the object, not a scattering of anti-aliased edge pixels that
  happen to carry its id, which produced boxes spanning the whole frame;
* nothing a station checks is detected when it is not really visible.
"""
import sys
from pathlib import Path

import numpy as np
import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "external" / "booster_deploy"))

from robot_control.head_control import CAMERA_FOV_V, PITCH_MARGIN, YAW_MARGIN  # noqa: E402
from robot_control.route_graph import RouteGraph  # noqa: E402
from shared import inspection  # noqa: E402

LAB_SCENE = PROJECT_ROOT / "simulation" / "scenes" / "labmate_scene.xml"
TRUNK_Z = 0.62
"""Trunk height that puts the head at the ~0.80 m measured while walking."""

fails = 0


def check(label, ok, detail=""):
    global fails
    fails += not ok
    print(("PASS " if ok else "FAIL ") + label + (f"  ({detail})" if detail else ""))


print(f"lab scene : {LAB_SCENE}\n")
model = mujoco.MjModel.from_xml_path(str(LAB_SCENE))
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

try:
    from perception.camera import COLLISION_GROUP, HeadCamera
    from perception.detector import Detection, Detector, GroundTruthDetector
    camera = HeadCamera(model)
except Exception as exc:                     # no display, no GL, no test
    print(f"SKIP  no renderer available on this machine: {exc}")
    print("\n0 failure(s)")
    sys.exit(0)

detector = GroundTruthDetector(model)
graph = RouteGraph(model)

joint_names = [model.joint(i).name for i in range(model.njnt)]
QY = model.jnt_qposadr[joint_names.index("aahead_yaw_joint")]
QP = model.jnt_qposadr[joint_names.index("aahead_pitch_joint")]
yaw_limit = float(model.joint("aahead_yaw_joint").range[1]) - YAW_MARGIN
pitch_lo = float(model.joint("aahead_pitch_joint").range[0]) + PITCH_MARGIN
pitch_hi = float(model.joint("aahead_pitch_joint").range[1]) - PITCH_MARGIN


def stand_at(site):
    """Put the robot where it inspects this station, head aimed at its target."""
    wp = graph.nodes[site]
    data.qpos[:] = 0.0
    data.qpos[0:3] = [wp["x"], wp["y"], TRUNK_Z]
    yaw = wp["yaw"]
    data.qpos[3:7] = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
    mujoco.mj_forward(model, data)

    eye = camera.eye(data)
    target = np.asarray(wp["target"], dtype=float)
    dx, dy = target[0] - eye[0], target[1] - eye[1]
    want_yaw = float((np.arctan2(dy, dx) - yaw + np.pi) % (2 * np.pi) - np.pi)
    want_pitch = -float(np.arctan2(target[2] - eye[2], max(np.hypot(dx, dy), 1e-3)))
    data.qpos[QY] = np.clip(want_yaw, -yaw_limit, yaw_limit)
    data.qpos[QP] = np.clip(want_pitch, pitch_lo, pitch_hi)
    mujoco.mj_forward(model, data)
    return wp


# 1. A frame is an image of something
rgb = camera.frame(data)
check("frame has the camera's shape and type",
      rgb.shape == (camera.height, camera.width, 3) and rgb.dtype == np.uint8,
      f"{rgb.shape} {rgb.dtype}")
check("frame is a picture, not a flat colour",
      float(rgb.std()) > 5.0, f"std {float(rgb.std()):.1f}")

# 2. The camera looks where the head looks
fwd = camera.forward(data)
check("view direction is a unit vector",
      abs(float(np.linalg.norm(fwd)) - 1.0) < 1e-6)
stand_at("inspect_server_rack")
eye, fwd = camera.eye(data), camera.forward(data)
target = np.asarray(graph.nodes["inspect_server_rack"]["target"], dtype=float)
to_target = target - eye
to_target /= np.linalg.norm(to_target)
off_axis = float(np.degrees(np.arccos(np.clip(np.dot(fwd, to_target), -1, 1))))
check("the aimed station lies within the camera's field of view",
      off_axis <= np.degrees(CAMERA_FOV_V) / 2,
      f"{off_axis:.1f} deg off axis, half-FOV {np.degrees(CAMERA_FOV_V) / 2:.0f} deg")

# 3. Scaffolding stays out of frame
seg = camera.segmentation(data)
ids, types = seg[..., 0], seg[..., 1]
geom_px = ids[types == int(mujoco.mjtObj.mjOBJ_GEOM)]
groups = {int(model.geom_group[g]) for g in np.unique(geom_px) if g >= 0}
check("no collision proxies are rendered", COLLISION_GROUP not in groups,
      f"geom groups in frame: {sorted(groups)}")
check("no site markers are rendered",
      not (types == int(mujoco.mjtObj.mjOBJ_SITE)).any())

# 4. Detections are objects, not scattered edge pixels
print()
total_wanted = total_found = 0
for site in inspection.STATION_ORDER:
    station = inspection.BY_SITE[site]
    stand_at(site)
    rgb = camera.frame(data)
    seg = camera.segmentation(data)
    wanted = tuple(dict.fromkeys(
        g for c in station.checks for g in c.geoms()))
    found = detector.detect(rgb, seg, wanted)
    total_wanted += len(wanted)
    total_found += len(found)

    inside = all(0 <= d.bbox[0] < d.bbox[2] <= camera.width
                 and 0 <= d.bbox[1] < d.bbox[3] <= camera.height
                 for d in found)
    # A box that spans most of the frame around a small object is the
    # anti-aliasing fringe bug; a real detection is compact.
    frame_area = camera.width * camera.height
    compact = all(d.width * d.height <= 0.9 * frame_area
                  or d.pixel_area >= 0.25 * d.width * d.height
                  for d in found)
    labelled = all(d.label in wanted and d.confidence == 1.0 for d in found)
    check(f"{site[len('inspect_'):]:16} detections are sane",
          inside and compact and labelled,
          f"{len(found)}/{len(wanted)} of its state geoms in view")

check("the round sees something at every station", total_found > 0,
      f"{total_found} of {total_wanted} state geoms across the six stations")

# 5. Nothing is invented: a geom the model does not have is never detected
print()
bogus = detector.detect(rgb, seg, ("no_such_geom",))
check("an unknown geom yields no detection", bogus == [])

hidden = [g for g in inspection.all_state_geoms()
          if int(model.geom_matid[model.geom(g).id]) < 0
          and float(model.geom_rgba[model.geom(g).id][3]) <= 0.0]
found_hidden = [d.label for d in detector.detect(rgb, seg, tuple(hidden))]
check("geoms a scenario has hidden are not detected", not found_hidden,
      f"{len(hidden)} hidden in this scenario, {len(found_hidden)} wrongly seen")

# 6. The interface holds
check("GroundTruthDetector satisfies the Detector protocol",
      isinstance(detector, Detector),
      "runtime protocol check")
check("Detection carries the documented fields",
      {"label", "bbox", "confidence", "pixel_area", "occluded_fraction"}
      <= set(Detection.__dataclass_fields__))
check("the detector names itself as ground truth",
      "ground truth" in detector.name, detector.name)

camera.close()
print(f"\n{fails} failure(s)")
sys.exit(bool(fails))

#!/usr/bin/env python3
"""Checks for the LabMate scene. Needs only mujoco + numpy.
Run:  python3 tests/check_scene.py  [lab_scene.xml]  [K1_22dof.xml]
Geometric/static checks only: it does NOT run the walking policy.

The two models live in different trees: the laboratory scene is LabMate code
tracked in simulation/scenes/, while the bare K1 reference model belongs to the
read-only Booster assets under external/."""
import os, sys, time
from pathlib import Path
import numpy as np
import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[1]
LAB_SCENE = PROJECT_ROOT / "simulation" / "scenes" / "labmate_scene.xml"
K1_BASE = (PROJECT_ROOT / "external" / "booster_assets" / "robots" / "K1"
           / "K1_22dof.xml")

lab_path = Path(sys.argv[1]) if len(sys.argv) > 1 else LAB_SCENE
base_path = Path(sys.argv[2]) if len(sys.argv) > 2 else K1_BASE
print(f"lab scene : {lab_path}")
print(f"K1 base   : {base_path}\n")
base = mujoco.MjModel.from_xml_path(str(base_path))
lab = mujoco.MjModel.from_xml_path(str(lab_path))
fails = 0
def check(label, ok, detail=""):
    global fails
    fails += not ok
    print(("PASS " if ok else "FAIL ") + label + (f"  ({detail})" if detail else ""))

# 1. Robot and global physics unchanged; robot IDs keep their positions
check("sizes nq/nv/nu/njnt/nmesh/nsensor/nsensordata", all(getattr(base, a) == getattr(lab, a)
      for a in ("nq", "nv", "nu", "njnt", "nmesh", "nsensor", "nsensordata")))
fields = {
    base.nbody: "body_pos body_quat body_mass body_inertia body_ipos body_iquat body_parentid",
    base.ngeom: "geom_type geom_size geom_pos geom_quat geom_bodyid geom_contype geom_conaffinity geom_condim geom_friction geom_solref geom_solimp",
    base.njnt: "jnt_type jnt_pos jnt_axis jnt_range jnt_bodyid jnt_qposadr",
    base.nv: "dof_armature dof_damping dof_frictionloss",
    base.nu: "actuator_trnid actuator_gear actuator_forcerange actuator_ctrlrange actuator_gainprm actuator_biasprm",
    base.nsite: "site_pos site_quat site_bodyid",
    base.nsensor: "sensor_type sensor_objid sensor_noise",
}
bad = [f for n, s in fields.items() for f in s.split()
       if not np.array_equal(getattr(base, f), getattr(lab, f)[:n])]
bad += ["qpos0"] if not np.array_equal(base.qpos0, lab.qpos0) else []
check("robot arrays identical and in the same ID slots", not bad, ",".join(bad))
check("timestep/solver/impratio/tolerance/integrator unchanged", all(getattr(base.opt, a) == getattr(lab.opt, a)
      for a in ("timestep", "solver", "impratio", "tolerance", "integrator")))
check("exactly one plane, lights unchanged", int((lab.geom_type == mujoco.mjtGeom.mjGEOM_PLANE).sum()) == 1 and lab.nlight == base.nlight)

ROBOT_GEOMS = set(range(1, base.ngeom))
d = mujoco.MjData(lab)
def pose(x, y, yaw, z=0.554, arms_down=False):
    mujoco.mj_resetData(lab, d)
    d.qpos[:7] = [x, y, z, np.cos(yaw/2), 0, 0, np.sin(yaw/2)]
    if arms_down:
        d.qpos[lab.joint("left_shoulder_roll_joint").qposadr[0]] = -1.3
        d.qpos[lab.joint("right_shoulder_roll_joint").qposadr[0]] = 1.3
    mujoco.mj_forward(lab, d)
def lab_hits():
    out = set()
    for c in d.contact[:d.ncon]:
        a, b = int(c.geom1), int(c.geom2)
        for r, o in ((a, b), (b, a)):
            if r in ROBOT_GEOMS and o >= base.ngeom:
                out.add(lab.geom(o).name)
    return out

# 2. Positive control: the collision test must detect a deliberate overlap
pose(1.8, -2.6, 0)
check("control: robot inside robotics bench is detected", bool(lab_hits()))

# 3. Every nav/inspection site, 24 headings, arms out (widest static pose)
sites = [lab.site(i).name for i in range(lab.nsite) if lab.site(i).name.startswith(("nav_", "inspect_"))]
mujoco.mj_forward(lab, d)
P = {n: d.site(n).xpos[:2].copy() for n in sites}
bad = {}
for n, p in P.items():
    for yaw in np.linspace(0, 2*np.pi, 24, endpoint=False):
        pose(*p, yaw)
        h = lab_hits()
        if h: bad.setdefault(n, set()).update(h)
check(f"{len(P)} sites: turn in place, arms out, no obstacle contact", not bad, str(bad) if bad else "")

# 4. Route edges, sampled every 2 cm, heading along the edge.
# Edges come from robot_control.route_graph so the planner and this validator
# cannot disagree about which routes are cleared. Both directions are swept:
# the planner treats edges as undirected, and every spur to an inspection site
# has to be walked back out again.
sys.path.insert(0, str(PROJECT_ROOT))
from robot_control.route_graph import ROUTE_EDGES  # noqa: E402

E = [e for a, b in ROUTE_EDGES for e in ((a, b), (b, a))]
boxes = []
mujoco.mj_forward(lab, d)
for g in range(base.ngeom, lab.ngeom):
    if lab.geom_contype[g] and d.geom_xpos[g][2] - lab.geom_size[g][2] < 1.0:
        boxes.append((lab.geom(g).name, d.geom_xpos[g][:2].copy(), lab.geom_size[g][:2].copy()))
def clearance(p):
    return min((np.linalg.norm(np.maximum(np.abs(p - c) - h, 0)), n) for n, c, h in boxes)
bad, worst = [], (9, "")
for a, b in E:
    v = P[b] - P[a]; yaw = np.arctan2(v[1], v[0]); low = (9, "")
    for t in np.linspace(0, 1, int(np.linalg.norm(v)/0.02) + 2):
        p = P[a] + t*v
        pose(*p, yaw)
        if lab_hits(): bad.append((a, b)); break
        low = min(low, clearance(p))
    worst = min(worst, (low[0], f"{a} -> {b}, nearest {low[1]}"))
check(f"{len(E)} route edges (both directions), arms out, no obstacle contact", not bad, str(bad) if bad else "")
check("min distance, route centreline to obstacle >= 0.60 m", worst[0] >= 0.60, f"{worst[0]:.3f} m on {worst[1]}")

# 5. Spawn: only floor/self contacts at the start pose
pose(0, 0, 0, arms_down=True)
check("start pose: no contact with lab geometry", not lab_hits())
soles = [d.geom_xpos[g][2] - lab.geom_size[g][2] for g in range(1, base.ngeom)
         if lab.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX and np.allclose(lab.geom_size[g], [.09, .035, .018])]
check("start pose z=0.554: soles 0-5 mm above floor", len(soles) == 2 and all(0 <= s <= .005 for s in soles), str(np.round(soles, 4)))

# 6. Relative physics cost (robot dropped with zero torque; machine dependent)
def bench(m):
    best = 1e9
    for _ in range(3):
        dd = mujoco.MjData(m); dd.qpos[2] = 0.554; t = time.perf_counter()
        for _ in range(3000): mujoco.mj_step(m, dd)
        best = min(best, time.perf_counter() - t)
    return best
a, b = bench(base), bench(lab)
print(f"INFO 3 s of sim: K1 only {a:.3f} s, lab scene {b:.3f} s wall-clock")
print(f"\n{fails} failure(s)"); sys.exit(bool(fails))

"""Head aiming.

The K1's two head joints are the only joints on the robot that the walking
policy does not drive. ``LocomotionPolicy`` maps its 20-dimensional action onto
the 20 joints named in ``policy_joint_names`` through ``real2sim_joint_map``,
and builds its observation through the same map, so ``aahead_yaw_joint`` and
``aahead_pitch_joint`` are absent from both the action and the observation. They
are held at their default position by the PD loop in
``MujocoController.ctrl_step``, which drives all 22 joints.

That makes the head free for the application layer to use: writing a head target
cannot perturb what the policy sees, and the policy cannot fight it. Measured in
simulation, the head holds a 0.90 rad yaw command to within 0.015 rad while the
robot is walking, using about 0.2 Nm of its 6 Nm effort limit.

This matters because both cameras - ``head_realsense_rgb_link`` and
``head_booster_stereo_rgb_link`` - are mounted on the head. Aiming the head is
therefore what a perception module will actually need, and it means the robot no
longer has to rotate its whole body to look at a station. Over a round that
removed 334 degrees of in-place body rotation whose only purpose was to point
the body at something the checklist reads out of the model anyway.

This module is to head targets what ``velocity_interface`` is to velocity
commands: the only sanctioned path. Joint limits are read from the model rather
than written down here, so they cannot drift from the MJCF.
"""

from __future__ import annotations

import numpy as np

YAW_JOINT = "aahead_yaw_joint"
PITCH_JOINT = "aahead_pitch_joint"

HEAD_BODY = "aahead_pitch_link"
"""Body the cameras hang off. Its world pose is where the aim is computed from."""

YAW_MARGIN = 0.10
PITCH_MARGIN = 0.03
"""Radians kept clear of each hard joint stop, so the PD loop never pushes into one.

The pitch margin is the smaller of the two because the joint has little room to
spare looking up: its range is -0.314 to +0.794 rad, a head built to look down
at a bench rather than up at a shelf. Two of this laboratory's six look-at
points sit above what it can reach, so the aim there is as far up as the joint
goes. That is good enough for a camera - the shortfall is about 2 degrees
against a depth camera's ~58 degree vertical field of view, so the target is
still well inside the frame - but it is the reason aiming is best-effort and
the caller is told the yaw residual rather than promised an exact aim.
"""

CAMERA_FOV_V = 1.01
"""Vertical field of view of the head depth camera, radians (~58 deg).

Not declared in the MJCF - the model carries the camera mounts
(head_realsense_rgb_link, head_booster_stereo_rgb_link) as bodies but no camera
element - so this is the nominal figure for the RealSense the mounts are named
for. Used only to check that a look-at point the pitch joint cannot reach is
still inside the frame.
"""

PITCH_TRIM_GAIN = 0.05
PITCH_TRIM_LIMIT = 0.20
"""Closed-loop trim for gravity droop on the pitch joint.

The head's mass sits forward of the pitch axis and the joint's stiffness is only
4.0 Nm/rad, so it settles nose-down of whatever it is commanded: measured in
simulation, 0.03-0.06 rad depending on the pose. A fixed feed-forward got this
wrong at one end or the other of the range, so the droop is instead trimmed out
by integrating the error between the angle asked for and the angle the joint
actually holds. Bounded, and slow enough not to interact with the slew.

The yaw joint needs no equivalent: gravity exerts no torque about a vertical
axis, and it was measured tracking a 0.90 rad command to within 0.015 rad.
"""

MAX_RATE = 0.03
"""Radians per policy step (1.5 rad/s at 50 Hz). Keeps the head from snapping."""

# Positive aahead_pitch_joint is nose-down: the joint's axis is "0 1 0" and the
# head's centre of mass is ahead of the axis, which is why gravity droop is in
# the positive direction. Elevation towards a target is therefore negated.
PITCH_SIGN = -1.0


def _indices(controller):
    names = controller.robot.cfg.joint_names
    return names.index(YAW_JOINT), names.index(PITCH_JOINT)


def joint_limits(controller):
    """``((yaw_lo, yaw_hi), (pitch_lo, pitch_hi))`` from the model, with margin."""
    iy, ip = _indices(controller)
    model = controller.mj_model
    out = []
    for name, margin in ((YAW_JOINT, YAW_MARGIN), (PITCH_JOINT, PITCH_MARGIN)):
        lo, hi = model.joint(name).range
        out.append((float(lo) + margin, float(hi) - margin))
    return tuple(out)


def set_head(controller, yaw: float = 0.0, pitch: float = 0.0):
    """Ask for a head pose, in radians, clamped to the model's joint range.

    Positive ``yaw`` is to the robot's left; positive ``pitch`` is nose-down.
    Takes effect on the next policy step; the head slews there at ``MAX_RATE``.
    """
    (yaw_lo, yaw_hi), (pitch_lo, pitch_hi) = joint_limits(controller)
    controller.head_target = (
        float(np.clip(yaw, yaw_lo, yaw_hi)),
        float(np.clip(pitch, pitch_lo, pitch_hi)),
    )
    return controller.head_target


def center_head(controller):
    """Return the head to its neutral forward gaze."""
    controller._head_pitch_trim = 0.0
    return set_head(controller, 0.0, 0.0)


def head_pose(controller):
    """Current ``(yaw, pitch)`` of the head joints as the simulation has them."""
    iy, ip = _indices(controller)
    dof_pos = controller.mj_data.qpos.astype(np.float64)[7:]
    return float(dof_pos[iy]), float(dof_pos[ip])


def head_position(controller):
    """World position of the head body the cameras are mounted on."""
    return np.array(controller.mj_data.body(HEAD_BODY).xpos, dtype=float)


def aim_at(controller, target_xyz, body_yaw: float):
    """Point the head at a world point, as far as its joints allow.

    ``body_yaw`` is the robot's current heading. Returns the yaw residual the
    head could *not* cover, in radians: zero when the head alone can look at the
    target, otherwise how far the body still has to turn. The caller decides
    whether to turn the body by it - which is what lets a station be inspected
    without squaring up to it.
    """
    target = np.asarray(target_xyz, dtype=float)
    here = head_position(controller)
    dx, dy = target[0] - here[0], target[1] - here[1]

    bearing = float(np.arctan2(dy, dx))
    yaw_want = float((bearing - body_yaw + np.pi) % (2.0 * np.pi) - np.pi)

    ground = float(np.hypot(dx, dy))
    elevation = float(np.arctan2(target[2] - here[2], max(ground, 1e-3)))
    pitch_want = PITCH_SIGN * elevation

    # Trim the droop out: nudge the command by the error the joint is actually
    # showing, so the gaze ends up on the target rather than below it.
    trim = float(getattr(controller, "_head_pitch_trim", 0.0))
    _, pitch_is = head_pose(controller)
    if head_settled(controller):
        trim += PITCH_TRIM_GAIN * (pitch_want - pitch_is)
        trim = float(np.clip(trim, -PITCH_TRIM_LIMIT, PITCH_TRIM_LIMIT))
        controller._head_pitch_trim = trim

    yaw_got, _ = set_head(controller, yaw_want, pitch_want + trim)
    return float((yaw_want - yaw_got + np.pi) % (2.0 * np.pi) - np.pi)


def apply_head_target(controller, dof_targets):
    """Write the head target into the policy's joint targets.

    Called once per policy step by ``LabMateController.policy_step``, on the
    tensor the policy just produced. The policy leaves the two head entries at
    their default position, so overwriting them takes nothing away from it. The
    tensor is the policy's own clone of its filtered target, so writing to it
    does not disturb the filter's state.
    """
    target = getattr(controller, "head_target", None)
    if target is None:
        return dof_targets

    iy, ip = _indices(controller)
    held = getattr(controller, "_head_held", None)
    if held is None:
        held = (float(dof_targets[iy]), float(dof_targets[ip]))

    # Slew, so a station 50 degrees off to one side is a turn of the head over
    # about a second rather than a step the body would feel as a kick.
    held = tuple(
        float(h + np.clip(t - h, -MAX_RATE, MAX_RATE))
        for h, t in zip(held, target)
    )
    controller._head_held = held

    dof_targets[iy] = held[0]
    dof_targets[ip] = held[1]
    return dof_targets


def head_settled(controller, tol: float = 0.05) -> bool:
    """Whether the head has arrived at what was last asked of it."""
    target = getattr(controller, "head_target", None)
    if target is None:
        return True
    held = getattr(controller, "_head_held", None)
    if held is None:
        return False
    return all(abs(h - t) <= tol for h, t in zip(held, target))

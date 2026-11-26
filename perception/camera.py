"""The robot's head camera.

There is no ``<camera>`` element anywhere in the model. The K1 carries the two
mount frames the real robot has - ``head_realsense_rgb_link`` and
``head_booster_stereo_rgb_link`` - as empty bodies with no camera, no geom and
no site (``K1_22dof.xml:64-65``).

Rather than add one, this builds an ``MjvCamera`` from the head's live world
pose on every frame. That keeps the camera entirely in LabMate code: no MJCF is
edited, so the robot's body, geom, joint and actuator ID slots are untouched and
``tests/check_scene.py``'s invariant - that the walking policy sees exactly the
model it was trained against - is never in question.

The eye point is the RealSense mount, which is where the sensor physically sits.
The viewing direction is the forward axis of ``aahead_pitch_link``, which is the
frame ``robot_control/head_control.py`` aims. One simplification is worth being
explicit about: the real RealSense mount is tilted about 15 degrees downward
relative to that frame (its optical axis at the default pose is
``[0.965, 0, -0.262]``), and that tilt is not modelled here. Using the aimed
frame instead means a station the head is pointed at lands in the middle of the
picture rather than 15 degrees above it.
"""

from __future__ import annotations

import numpy as np
import mujoco

from robot_control.head_control import CAMERA_FOV_V, HEAD_BODY

EYE_BODY = "head_realsense_rgb_link"
"""Where the camera sits: the RealSense mount on the head."""

AIM_BODY = HEAD_BODY
"""Where it looks: the frame head_control aims (``aahead_pitch_link``)."""

WIDTH = 848
HEIGHT = 480

COLLISION_GROUP = 3
"""Geom group of the ``lab_col`` collision proxies.

These are translucent red boxes sitting coincident with the furniture they
stand in for. The passive viewer hides them by its own default, but a fresh
``MjvOption`` does not, and left on they cover most of every frame.
"""

HEADLIGHT_AMBIENT = 0.45
HEADLIGHT_DIFFUSE = 0.55
"""Lighting used while rendering from the head.

The scene's own headlight (ambient 0.25, diffuse 0.2) is set for a wide
third-person view of a lit room. Pressed up against a dark server rack it
leaves the picture almost black, so the camera brings its own light - which is
what an inspection robot working in a plant room would do anyway.
"""

_LOOKAT_DISTANCE = 1.0
"""Arbitrary positive distance along the view axis.

A free ``MjvCamera`` is specified as a point to look at plus a distance back
from it, so the eye lands at ``lookat - distance * forward``. Any distance
works as long as the lookat point is placed the same distance ahead.
"""


class HeadCamera:
    """Renders the scene from the robot's head.

    Owns one ``mujoco.Renderer``, and therefore one GL context of its own. That
    is legitimate alongside the passive viewer - the renderer re-binds its
    context around every render - but both go through GLFW by default in a
    single process. If that ever conflicts, run with ``MUJOCO_GL=egl``, which
    gives the renderer a headless context independent of the viewer's window.
    """

    def __init__(self, model, width: int = WIDTH, height: int = HEIGHT):
        if width > model.vis.global_.offwidth or height > model.vis.global_.offheight:
            raise ValueError(
                f"{width}x{height} exceeds the scene's offscreen framebuffer "
                f"({model.vis.global_.offwidth}x{model.vis.global_.offheight}); "
                "raise offwidth/offheight in simulation/scenes/labmate_scene.xml"
            )
        self.model = model
        self.width = width
        self.height = height
        self._renderer = mujoco.Renderer(model, height=height, width=width)

        # What the camera is allowed to see. Collision proxies and the
        # navigation/look-at site markers are scaffolding, not laboratory.
        self._option = mujoco.MjvOption()
        self._option.geomgroup[COLLISION_GROUP] = 0
        self._option.sitegroup[:] = 0

        self._camera = mujoco.MjvCamera()
        self._camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self._camera.distance = _LOOKAT_DISTANCE

        self._fovy = float(np.degrees(CAMERA_FOV_V))
        self._segmenting = False

    # -- pose -------------------------------------------------------------

    def eye(self, data) -> np.ndarray:
        """World position of the camera."""
        return np.array(data.body(EYE_BODY).xpos, dtype=float)

    def forward(self, data) -> np.ndarray:
        """Unit vector the camera looks along.

        Taken from the rotation matrix of the aimed frame rather than from the
        commanded head angles: the joints lag their target and droop slightly
        under gravity, and a frame should show where the camera actually points,
        not where it was asked to point.
        """
        rot = np.array(data.body(AIM_BODY).xmat, dtype=float).reshape(3, 3)
        fwd = rot[:, 0]                       # the body's local +X in world
        norm = float(np.linalg.norm(fwd))
        return fwd / norm if norm > 0 else np.array([1.0, 0.0, 0.0])

    def _aim(self, data):
        eye = self.eye(data)
        fwd = self.forward(data)
        self._camera.lookat[:] = eye + fwd * _LOOKAT_DISTANCE
        self._camera.azimuth = float(np.degrees(np.arctan2(fwd[1], fwd[0])))
        self._camera.elevation = float(np.degrees(np.arcsin(np.clip(fwd[2], -1.0, 1.0))))

    # -- rendering --------------------------------------------------------

    def _render(self, data):
        # Field of view and headlight are model-wide visual settings, shared
        # with the passive viewer. Set them for this render and put them back,
        # so looking through the robot's eyes does not change how the
        # third-person viewer lights or frames the room. Neither affects physics.
        previous = float(self.model.vis.global_.fovy)
        light = self.model.vis.headlight
        prev_ambient = np.array(light.ambient)
        prev_diffuse = np.array(light.diffuse)
        self.model.vis.global_.fovy = self._fovy
        # The scene is lit for a wide third-person view; from 0.7 m in front of
        # a dark server rack it comes out nearly black. A real inspection robot
        # would have its own lamp, and this is the equivalent.
        light.ambient[:] = HEADLIGHT_AMBIENT
        light.diffuse[:] = HEADLIGHT_DIFFUSE
        try:
            self._aim(data)
            self._renderer.update_scene(data, camera=self._camera,
                                        scene_option=self._option)
            return self._renderer.render()
        finally:
            self.model.vis.global_.fovy = previous
            light.ambient[:] = prev_ambient
            light.diffuse[:] = prev_diffuse

    def frame(self, data) -> np.ndarray:
        """An RGB ``uint8`` image of shape ``(height, width, 3)``."""
        if self._segmenting:
            self._renderer.disable_segmentation_rendering()
            self._segmenting = False
        return self._render(data)

    def segmentation(self, data) -> np.ndarray:
        """Ground-truth ``int32`` ``(height, width, 2)`` of (object type, id).

        The same view as :meth:`frame`, labelled per pixel with what is visible
        there. This is what makes a ground-truth detector possible without any
        learned model: occlusion, perspective and visibility are all real.
        """
        if not self._segmenting:
            self._renderer.enable_segmentation_rendering()
            self._segmenting = True
        return self._render(data)

    def close(self):
        self._renderer.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

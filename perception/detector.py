"""Detection, behind one interface.

The interface is the point of this module. Today the detector reads ground truth
out of MuJoCo's segmentation buffer; later it can be a learned model. Both
answer the same question - "what is in this frame, and where" - so swapping one
for the other changes how the laboratory is *observed* and nothing about what
counts as a pass.

What the ground-truth detector is, stated plainly, because the distinction
matters when this footage sits next to real camera footage: it is not a learned
model and it is not YOLO. It resolves each pixel to the geom that painted it and
reports the bounding box of the geoms a station's checks care about. Occlusion,
perspective and visibility are genuine - a geom hidden behind the bench is not
detected, and a geom a scenario has made invisible is not detected - but the
identities come from the simulator rather than from recognition. Everything it
reports is therefore true of the scene; none of it is evidence that a detector
could recognise the scene.

``shared/inspection.py`` already knows which geoms matter: every ``Check``
declares them through ``geoms()``. The detector is asked for exactly those, so
the boxes drawn on a frame are the things actually being judged, and there is no
second list of interesting objects to drift out of step with the checklist.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, Sequence, runtime_checkable

import cv2
import numpy as np
import mujoco

GROUND_TRUTH = "simulation ground truth (MuJoCo segmentation)"
"""How the ground-truth detector identifies itself, for the overlay stamp."""

MIN_PIXELS = 12
"""Below this a detection is a few stray pixels at a grazing angle, not a sighting."""


@dataclass(frozen=True)
class Detection:
    """One object seen in one frame.

    ``bbox`` is ``(x0, y0, x1, y1)`` in pixels, x1/y1 exclusive.
    """

    label: str
    bbox: tuple[int, int, int, int]
    confidence: float
    pixel_area: int
    occluded_fraction: float = 0.0

    @property
    def width(self) -> int:
        return self.bbox[2] - self.bbox[0]

    @property
    def height(self) -> int:
        return self.bbox[3] - self.bbox[1]

    @property
    def centre(self) -> tuple[int, int]:
        x0, y0, x1, y1 = self.bbox
        return (x0 + x1) // 2, (y0 + y1) // 2


@runtime_checkable
class Detector(Protocol):
    """What any detector must offer.

    ``rgb`` is the colour frame. ``seg`` is the matching segmentation buffer
    where one is available, and ``None`` otherwise - a learned detector ignores
    it, which is the whole point of passing it separately.
    """

    name: str

    def detect(self, rgb, seg, wanted: Sequence[str]) -> list[Detection]: ...


class GroundTruthDetector:
    """Reads detections out of MuJoCo's segmentation buffer.

    Honest about being ground truth: every detection carries
    ``confidence=1.0``, because there is nothing uncertain about it. A fabricated
    score would be the one dishonest number in the pipeline.
    """

    name = GROUND_TRUTH

    def __init__(self, model, min_pixels: int = MIN_PIXELS):
        self.model = model
        self.min_pixels = min_pixels
        self._geom_ids = {
            model.geom(i).name: i
            for i in range(model.ngeom)
            if model.geom(i).name
        }

    def detect(self, rgb, seg, wanted: Sequence[str]) -> list[Detection]:
        """Boxes for each named geom that is actually visible in this frame."""
        if seg is None:
            raise ValueError(
                "GroundTruthDetector needs a segmentation buffer; pass "
                "HeadCamera.segmentation(data) as seg"
            )

        # Channel order is (object id, object type), verified against this
        # scene - the reverse of what the naming suggests.
        ids = seg[..., 0]
        types = seg[..., 1]
        is_geom = types == int(mujoco.mjtObj.mjOBJ_GEOM)

        out: list[Detection] = []
        for name in wanted:
            gid = self._geom_ids.get(name)
            if gid is None:
                continue
            mask = is_geom & (ids == gid)
            if int(mask.sum()) < self.min_pixels:
                continue        # not in frame, hidden, or edge-on

            # Keep only the largest blob. The segmentation buffer is rendered
            # with anti-aliasing, so along every silhouette in the scene a thin
            # fringe of pixels blends two object ids and lands on whichever one
            # rounding picks. Those strays are scattered frame-wide, and taking
            # the extent of every pixel bearing an id gave boxes spanning most
            # of the picture around objects a few pixels across.
            count, labels, stats, _ = cv2.connectedComponentsWithStats(
                mask.astype(np.uint8), connectivity=8)
            if count < 2:
                continue
            biggest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
            x, y, w, h, area = (int(v) for v in stats[biggest])
            if area < self.min_pixels:
                continue
            bbox = (x, y, x + w, y + h)
            # How much of the box the geom actually fills. Not true occlusion -
            # that would need a second render with everything else removed -
            # but it does fall when something stands in front of the object,
            # which is the case the inspection cares about.
            box_area = max((bbox[2] - bbox[0]) * (bbox[3] - bbox[1]), 1)
            out.append(Detection(
                label=name,
                bbox=bbox,
                confidence=1.0,
                pixel_area=area,
                occluded_fraction=round(1.0 - area / box_area, 3),
            ))
        return out


class YoloDetector:
    """A learned detector, against the same interface. Not in use.

    Written so the swap is a one-line change rather than a redesign, and left
    unwired deliberately:

    * ``ultralytics`` is not installed, and ``torchvision`` - which it requires -
      is absent too. Installing it would most likely move ``torch 2.15.0+cpu``,
      which is the torch running the walking policy in this same process.
    * The weights on the repository's main branch (``yolov8n.pt``) are stock
      COCO YOLOv8-nano. COCO's 80 classes contain none of this laboratory's
      inspectable state: no fire extinguishers, no rack indicator LEDs, no
      spills, no fume-hood sash. Pointing it at these stations would detect
      nothing useful however well it ran.

    Making this real means training on lab objects, and mapping class names onto
    the geom names the checklist uses - which is what ``wanted`` is for.
    """

    name = "YOLO"

    def __init__(self, weights: str, class_to_geom: dict[str, str] | None = None,
                 confidence: float = 0.35):
        from ultralytics import YOLO        # imported here, never at module load

        self._model = YOLO(weights)
        self._class_to_geom = class_to_geom or {}
        self._confidence = confidence

    def detect(self, rgb, seg, wanted: Sequence[str]) -> list[Detection]:
        raise NotImplementedError(
            "YoloDetector is a seam, not an implementation. It needs weights "
            "trained on this laboratory's objects and a class-to-geom mapping; "
            "see the class docstring."
        )

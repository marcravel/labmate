"""Drawing detections and verdicts onto a frame.

Presentation only. Nothing here decides anything: the boxes come from the
detector, the verdicts from ``shared.inspection``, and this module puts them on
the picture.

Two rules it keeps.

Every frame names its detector. When this footage sits next to real camera
footage from a real laboratory, the viewer has to be able to tell which is
which, and the frame itself should say so rather than relying on a caption.

A check whose geom is not visible says so. The head aims at a station's
``target_*`` site, which can sit up to a metre from the geoms the checks read,
and some equipment is simply out of frame or behind something. Marking those
"not in view" is the honest reading, and it is the same limitation a real
inspection camera would have from the same standing position.
"""

from __future__ import annotations

import cv2
import numpy as np

from shared.inspection import ADVISORY, CRITICAL, FINDING, PASS

# BGR, because OpenCV.
COLOUR = {
    PASS: (120, 220, 120),
    ADVISORY: (90, 210, 240),
    FINDING: (70, 150, 250),
    CRITICAL: (70, 70, 240),
}
UNSEEN = (170, 170, 170)
INK = (245, 245, 245)
PANEL = (28, 28, 28)

FONT = cv2.FONT_HERSHEY_SIMPLEX
BAR_H = 54
LINE_H = 19


def _panel(img, y0, y1, alpha=0.72):
    """A dark translucent band, so text stays readable over any scene."""
    y0 = max(0, y0)
    y1 = min(img.shape[0], y1)
    if y1 <= y0:
        return
    strip = img[y0:y1]
    img[y0:y1] = cv2.addWeighted(
        strip, 1.0 - alpha, np.full_like(strip, PANEL), alpha, 0.0)


def _text(img, s, org, colour=INK, scale=0.45, weight=1):
    cv2.putText(img, s, org, FONT, scale, colour, weight, cv2.LINE_AA)


def annotate(rgb, station, result, detections, detector_name,
             scenario="", sim_time=None):
    """Return a BGR frame with the station's detections and verdicts drawn on.

    ``station`` is the ``shared.inspection.Station``; ``result`` its
    ``StationResult``; ``detections`` what the detector returned for this frame.
    """
    img = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2BGR)
    height, width = img.shape[:2]

    by_label = {d.label: d for d in detections}
    severity_of_geom = {}
    for check in station.checks:
        verdict = next((c for c in result.checks if c.key == check.key), None)
        for geom in check.geoms():
            severity_of_geom[geom] = verdict.severity if verdict else PASS

    # Boxes, coloured by the verdict of the check that reads the geom.
    for det in detections:
        x0, y0, x1, y1 = det.bbox
        colour = COLOUR.get(severity_of_geom.get(det.label, PASS), INK)
        cv2.rectangle(img, (x0, y0), (x1, y1), colour, 2)
        label = f"{det.label}"
        (tw, th), _ = cv2.getTextSize(label, FONT, 0.4, 1)
        ly = y0 - 6 if y0 > th + 8 else min(y1 + th + 6, height - 4)
        cv2.rectangle(img, (x0, ly - th - 4), (x0 + tw + 8, ly + 4), colour, -1)
        _text(img, label, (x0 + 4, ly), (20, 20, 20), 0.4, 1)

    # Header: which station, and the rule it is being held to.
    _panel(img, 0, BAR_H)
    _text(img, station.label, (12, 22), INK, 0.6, 2)
    _text(img, f"{station.standard}   -   {station.cadence}", (12, 42),
          (190, 190, 190), 0.42, 1)
    badge = COLOUR.get(result.severity, INK)
    (bw, _), _ = cv2.getTextSize(result.severity, FONT, 0.55, 2)
    cv2.rectangle(img, (width - bw - 26, 12), (width - 10, 40), badge, -1)
    _text(img, result.severity, (width - bw - 18, 33), (20, 20, 20), 0.55, 2)

    # Footer: one line per check, then who did the detecting.
    rows = len(result.checks)
    foot = height - (rows * LINE_H + 40)
    _panel(img, foot, height)
    y = foot + 20
    for check, verdict in zip(station.checks, result.checks):
        seen = [g for g in check.geoms() if g in by_label]
        if seen:
            mark, colour = verdict.severity, COLOUR.get(verdict.severity, INK)
            note = f"{len(seen)}/{len(check.geoms())} in view"
        else:
            mark, colour = "NOT IN VIEW", UNSEEN
            note = "out of frame or occluded"
        _text(img, f"{mark:>11}  {check.description}", (12, y), colour, 0.42, 1)
        _text(img, f"{verdict.observed}  ({note})", (width - 330, y),
              (190, 190, 190), 0.38, 1)
        y += LINE_H

    stamp = f"detector: {detector_name}"
    if scenario:
        stamp += f"   |   scenario: {scenario}"
    if sim_time is not None:
        stamp += f"   |   t = {sim_time:.1f} s"
    _text(img, stamp, (12, height - 10), (170, 170, 170), 0.4, 1)
    return img


def contact_sheet(frames, columns=3, pad=8):
    """Tile station frames into one image for a slide."""
    if not frames:
        raise ValueError("no frames to tile")
    height, width = frames[0].shape[:2]
    rows = (len(frames) + columns - 1) // columns
    sheet = np.full((rows * height + (rows + 1) * pad,
                     columns * width + (columns + 1) * pad, 3), 18, np.uint8)
    for i, frame in enumerate(frames):
        r, c = divmod(i, columns)
        y = pad + r * (height + pad)
        x = pad + c * (width + pad)
        sheet[y:y + height, x:x + width] = cv2.resize(frame, (width, height))
    return sheet

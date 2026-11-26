"""Writing out what the robot saw.

Two independent recorders, both passive observers:

* :class:`StationCapture` hangs off the navigator's existing ``on_inspect``
  hook and writes one annotated still per station, plus a contact sheet and a
  manifest.
* :class:`RoundRecorder` hangs off a per-step hook on the controller and writes
  the head-camera feed to a video for the whole round.

Neither can change a verdict, a route, or a velocity command. If either throws,
it is caught and reported rather than being allowed to end a patrol - a camera
is not allowed to stop a robot walking, which is the governing constraint from
the brief.
"""

from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

from perception.camera import HeadCamera
from perception.detector import GroundTruthDetector
from perception.overlay import annotate, contact_sheet
from shared import inspection

DEFAULT_ROOT = Path("simulation/captures")
VIDEO_FPS = 25
VIDEO_EVERY = 2
"""Policy steps per recorded frame. 50 Hz / 2 = 25 fps, matching VIDEO_FPS."""


class StationCapture:
    """One annotated still per station, written as the round goes."""

    def __init__(self, model, scenario="nominal", root=DEFAULT_ROOT,
                 camera=None, detector=None):
        self.scenario = scenario
        self.dir = Path(root) / scenario
        self.dir.mkdir(parents=True, exist_ok=True)
        self.camera = camera or HeadCamera(model)
        self.detector = detector or GroundTruthDetector(model)
        self.frames: list[np.ndarray] = []
        self.manifest: list[dict] = []
        self.errors: list[str] = []

    def on_inspect(self, controller, waypoint, elapsed):
        """Navigator hook. Same signature as ``InspectionRun.record``.

        Composed with the report's own recorder rather than replacing it, so
        the checklist keeps reading the simulation exactly as before and this
        only observes.
        """
        try:
            return self._capture(controller, waypoint, elapsed)
        except Exception as exc:                  # never break a patrol
            self.errors.append(f"{waypoint.get('name', '?')}: {exc}")
            print(f"[Capture] could not capture {waypoint.get('name')}: {exc}")
            return None

    def _capture(self, controller, waypoint, elapsed):
        site = waypoint["name"]
        station = inspection.BY_SITE.get(site)
        if station is None:
            return None

        data = controller.mj_data
        rgb = self.camera.frame(data)
        seg = self.camera.segmentation(data)

        wanted = tuple(dict.fromkeys(
            g for check in station.checks for g in check.geoms()))
        detections = self.detector.detect(rgb, seg, wanted)

        # The verdicts come from the checklist, which reads the model - the
        # detector does not vote. Recomputed here rather than passed in so the
        # capture stays independent of the report's bookkeeping.
        result = station.inspect(controller.mj_model, data, elapsed)
        frame = annotate(rgb, station, result, detections, self.detector.name,
                         scenario=self.scenario, sim_time=elapsed)

        index = len(self.frames)
        name = f"{index:02d}_{site[len('inspect_'):]}.png"
        path = self.dir / name
        cv2.imwrite(str(path), frame)
        self.frames.append(frame)
        self.manifest.append({
            "station": site,
            "label": station.label,
            "severity": result.severity,
            "sim_time_s": round(float(elapsed), 2),
            "image": str(path),
            "detector": self.detector.name,
            "detections": [
                {"label": d.label, "bbox": list(d.bbox),
                 "confidence": d.confidence, "pixel_area": d.pixel_area,
                 "occluded_fraction": d.occluded_fraction}
                for d in detections
            ],
            "checks_not_in_view": [
                check.key for check in station.checks
                if not any(g in {d.label for d in detections}
                           for g in check.geoms())
            ],
        })
        print(f"[Capture] {site}: {len(detections)} of {len(wanted)} "
              f"state geoms in view -> {path}")
        return path

    def finish(self):
        """Write the contact sheet and manifest. Safe to call twice."""
        written = []
        if self.frames:
            sheet = contact_sheet(self.frames)
            sheet_path = self.dir / "contact_sheet.png"
            cv2.imwrite(str(sheet_path), sheet)
            written.append(sheet_path)
        manifest_path = self.dir / "captures.json"
        manifest_path.write_text(json.dumps({
            "scenario": self.scenario,
            "detector": self.detector.name,
            "stations": self.manifest,
            "errors": self.errors,
        }, indent=2) + "\n")
        written.append(manifest_path)
        return written

    def close(self):
        self.camera.close()


class RoundRecorder:
    """The head-camera feed for the whole round, as a video file."""

    def __init__(self, path, model, camera=None, every=VIDEO_EVERY,
                 fps=VIDEO_FPS):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.camera = camera or HeadCamera(model)
        self.every = max(1, int(every))
        self.fps = fps
        self._writer = None
        self._step = 0
        self.frames_written = 0
        self.errors: list[str] = []

    def on_step(self, controller):
        """Per-step hook. Renders every ``every`` steps."""
        self._step += 1
        if self._step % self.every:
            return
        try:
            frame = cv2.cvtColor(self.camera.frame(controller.mj_data),
                                 cv2.COLOR_RGB2BGR)
            if self._writer is None:
                height, width = frame.shape[:2]
                self._writer = cv2.VideoWriter(
                    str(self.path), cv2.VideoWriter_fourcc(*"mp4v"),
                    self.fps, (width, height))
                if not self._writer.isOpened():
                    raise RuntimeError(f"could not open {self.path} for writing")
            self._writer.write(frame)
            self.frames_written += 1
        except Exception as exc:                  # never break a patrol
            if not self.errors:
                print(f"[Capture] video recording stopped: {exc}")
            self.errors.append(str(exc))
            self.every = 10 ** 9                  # stop trying

    def close(self):
        if self._writer is not None:
            self._writer.release()
            self._writer = None
        self.camera.close()
        return self.path if self.frames_written else None

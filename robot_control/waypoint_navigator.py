import numpy as np
from robot_control.velocity_interface import set_velocity, stop_robot, get_robot_pose

def quat_to_yaw(q):
    w, x, y, z = q[0], q[1], q[2], q[3]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(np.arctan2(siny_cosp, cosy_cosp))

def normalize_angle(angle):
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


class WaypointNavigator:
    def __init__(self, waypoints, pos_tolerance=0.25, yaw_tolerance=0.20,
                 heading_gain=1.1, realign_tol=0.35, yaw_rate_max=0.45,
                 on_inspect=None):
        self.waypoints = waypoints
        # Called once per station, at the moment the robot has stopped and
        # finished turning to face it - i.e. when it is standing where a human
        # inspector would stand. Signature: on_inspect(controller, waypoint,
        # elapsed_time). Optional: the patrol runs unchanged without it, which
        # is what keeps navigation independent of inspection and perception.
        self.on_inspect = on_inspect
        self.current_idx = 0
        self.pos_tol = pos_tolerance
        self.yaw_tol = yaw_tolerance
        self.turn_tol = min(0.08, yaw_tolerance)
        # Closed-loop heading control while walking. Without this the APPROACH
        # phase commanded a zero turn rate, so the robot walked on whatever
        # heading it happened to hold and gait drift was never corrected - it
        # could miss a waypoint entirely and walk into a wall.
        self.heading_gain = heading_gain
        self.realign_tol = realign_tol
        self.yaw_rate_max = yaw_rate_max
        self.pause_start_time = None
        self.pause_duration = 3.0
        self.is_finished = False
        self._last_vx = 0.0
        self._last_yaw = 0.0
        self.phase = "APPROACH"
        self._phase_timer = None
        self._target_depart_yaw = None

    def _send_smoothed_velocity(self, controller, target_vx, target_yaw):
        max_dvx = 0.03
        max_dyaw = 0.05
        vx = self._last_vx + float(np.clip(target_vx - self._last_vx, -max_dvx, max_dvx))
        yaw = self._last_yaw + float(np.clip(target_yaw - self._last_yaw, -max_dyaw, max_dyaw))
        self._last_vx = vx
        self._last_yaw = yaw
        set_velocity(controller, vx=vx, vy=0.0, yaw=yaw)

    def _rotate_in_place(self, controller, target_yaw, current_yaw, tol):
        yaw_err = normalize_angle(target_yaw - current_yaw)
        if abs(yaw_err) > tol:
            yaw_mag = float(np.clip(abs(yaw_err) * 1.2, 0.35, 0.60))
            yaw_cmd = float(np.sign(yaw_err) * yaw_mag)
            self._send_smoothed_velocity(controller, 0.0, yaw_cmd)
            return False
        self._send_smoothed_velocity(controller, 0.0, 0.0)
        return True

    def _advance_waypoint(self):
        self.pause_start_time = None
        self._phase_timer = None
        self._target_depart_yaw = None
        self.current_idx += 1
        self.phase = "APPROACH"

    def update(self, controller, elapsed_time):
        if self.is_finished or self.current_idx >= len(self.waypoints):
            self._send_smoothed_velocity(controller, 0.0, 0.0)
            self.is_finished = True
            self.phase = "FINISHED"
            return

        wp = self.waypoints[self.current_idx]
        pos, quat = get_robot_pose(controller)
        current_x, current_y = pos[0], pos[1]
        current_yaw = quat_to_yaw(quat)

        dx = wp["x"] - current_x
        dy = wp["y"] - current_y
        dist = float(np.hypot(dx, dy))

        if self.phase == "APPROACH":
            if dist <= self.pos_tol:
                self._send_smoothed_velocity(controller, 0.0, 0.0)
                self.phase = "STOP_BEFORE_INSPECT"
                self._phase_timer = elapsed_time
                return

            bearing = float(np.arctan2(dy, dx))
            yaw_err = normalize_angle(bearing - current_yaw)

            # Too far off course to walk out of: turn on the spot first. This
            # also covers the 180 deg reversals when leaving an inspection site.
            if abs(yaw_err) > self.realign_tol:
                self._rotate_in_place(controller, bearing, current_yaw,
                                      self.turn_tol)
                return

            # On course: walk, correcting heading proportionally, and slow down
            # the more the heading is off so corrections stay inside tolerance.
            yaw_cmd = float(np.clip(self.heading_gain * yaw_err,
                                    -self.yaw_rate_max, self.yaw_rate_max))
            vx_cmd = float(np.clip(0.6 * dist, 0.28, 0.45))
            vx_cmd *= float(max(0.45, np.cos(yaw_err)))
            self._send_smoothed_velocity(controller, vx_cmd, yaw_cmd)
            return

        if self.phase == "STOP_BEFORE_INSPECT":
            self._send_smoothed_velocity(controller, 0.0, 0.0)
            if abs(self._last_vx) < 1e-3 and elapsed_time - self._phase_timer >= 0.4:
                if wp.get("inspect", True):
                    self.phase = "FACE_STATION"
                else:
                    self.phase = "FACE_FINAL"
            return

        if self.phase in ("FACE_STATION", "FACE_FINAL"):
            if self._rotate_in_place(controller, wp["yaw"], current_yaw, self.yaw_tol):
                if self.phase == "FACE_STATION":
                    print(f"[Navigator] ARRIVED at {wp['name']}! Pausing {self.pause_duration}s for inspection...")
                    if self.on_inspect is not None:
                        self.on_inspect(controller, wp, elapsed_time)
                    self.pause_start_time = elapsed_time
                    self.phase = "INSPECT_PAUSE"
                else:
                    self._advance_waypoint()
            return

        if self.phase == "INSPECT_PAUSE":
            self._send_smoothed_velocity(controller, 0.0, 0.0)
            if elapsed_time - self.pause_start_time >= self.pause_duration:
                print(f"[Navigator] Inspection complete at {wp['name']}.")
                if self.current_idx + 1 < len(self.waypoints):
                    next_wp = self.waypoints[self.current_idx + 1]
                    self._target_depart_yaw = float(np.arctan2(next_wp["y"] - wp["y"], next_wp["x"] - wp["x"]))
                    self.phase = "ROTATE_TO_NEXT"
                else:
                    self._advance_waypoint()
            return

        if self.phase == "ROTATE_TO_NEXT":
            if self._rotate_in_place(controller, self._target_depart_yaw, current_yaw, self.turn_tol):
                self._phase_timer = elapsed_time
                self.phase = "SETTLE_AFTER_TURN"
            return

        if self.phase == "SETTLE_AFTER_TURN":
            self._send_smoothed_velocity(controller, 0.0, 0.0)
            if abs(self._last_yaw) < 1e-3 and elapsed_time - self._phase_timer >= 0.3:
                yaw_err = normalize_angle(self._target_depart_yaw - current_yaw)
                if abs(yaw_err) <= self.yaw_tol:
                    print(f"[Navigator] Orientation verified ({np.degrees(current_yaw):.1f} deg). Resuming forward walk...")
                    self._advance_waypoint()
                else:
                    self.phase = "ROTATE_TO_NEXT"
            return

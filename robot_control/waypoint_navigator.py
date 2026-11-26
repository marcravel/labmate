import numpy as np
from robot_control.head_control import aim_at, center_head, head_settled
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
                 heading_gain=1.1, realign_tol=0.35, yaw_rate_max=0.60,
                 vx_cruise=1.0, vx_min=0.35, decel_gain=0.8,
                 turn_rate_min=0.45, turn_rate_max=0.90,
                 max_dvx=0.08, max_dyaw=0.10,
                 back_out_speed=0.25, back_out_max_dist=1.3,
                 back_out_min_angle=2.4,
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
        # Exit tolerance for a rotate-in-place. Wider than it was (0.08): the
        # turn command has a magnitude floor, so it cannot taper all the way to
        # zero, and a tight tolerance made it chatter on and off at the
        # boundary. Leaving the turn a little earlier spends that discontinuity
        # once, and the APPROACH heading controller absorbs the remainder.
        self.turn_tol = min(0.12, yaw_tolerance)
        # Closed-loop heading control while walking. Without this the APPROACH
        # phase commanded a zero turn rate, so the robot walked on whatever
        # heading it happened to hold and gait drift was never corrected - it
        # could miss a waypoint entirely and walk into a wall.
        self.heading_gain = heading_gain
        self.realign_tol = realign_tol
        self.yaw_rate_max = yaw_rate_max

        # Walk at a speed the policy was actually trained for. k1_walk.pt walks
        # over a band up to vx_forward_max=1.6; the route used to be driven at
        # 0.28-0.45 m/s, which makes it take tiny shuffling steps that read as a
        # glitched gait. Driven at 1.0 m/s - verified by hand with a "1 0 0"
        # command - it takes real strides.
        #
        # The speed cannot simply be raised, because the leg lengths on this
        # route differ by 7x: the northwest-northeast corridor run is 3.6 m,
        # while the spur to inspect_electronics_a is 0.50 m, of which only
        # 0.25 m is travelled before pos_tol triggers. So speed is scheduled on
        # the distance left to run: the corridor legs get the fast gait, the
        # station spurs stay slow, and arrival is a ramp-down from ~1.25 m out
        # rather than a step from the old 0.28 m/s floor straight to zero.
        self.vx_cruise = vx_cruise
        self.vx_min = vx_min
        self.decel_gain = decel_gain

        # Turning has the same failure as walking did: a too-low yaw rate makes
        # the robot shuffle on the spot instead of stepping round.
        self.turn_rate_min = turn_rate_min
        self.turn_rate_max = turn_rate_max

        # Slew limit on the commands, applied once per policy step (50 Hz), so
        # these are 4.0 m/s^2 and 5.0 rad/s^2. This exists to take the edge off
        # the phase transitions, which all bang the command to zero. It is
        # deliberately loose: driving the robot by hand with step commands is
        # smooth, and the old 1.5 m/s^2 limit spent 0.3 s - most of a short
        # leg - just reaching the commanded speed.
        self._max_dvx = max_dvx
        self._max_dyaw = max_dyaw

        # Reversing out of a station spur instead of turning round in it.
        #
        # Every station sits at the end of a 0.50-0.95 m spur off the perimeter
        # loop, and the next waypoint is the corridor node the robot just came
        # from. Turning 180 degrees to walk back out, then turning again at the
        # corridor node, was the single largest piece of movement on the round -
        # about 290 degrees of in-place rotation per station, none of which goes
        # anywhere. Walking back out in reverse costs no rotation at all.
        #
        # This is safe because the robot is still on the heading it arrived on:
        # the head did the looking, so the body never squared up to the station.
        # Reversing therefore retraces the same ROUTE_EDGES edge it walked in
        # on, which tests/check_scene.py has already swept for clearance. It
        # would not be safe if the body had turned to face the station first.
        #
        # back_out_speed stays inside vx_backward_max = 0.3 m/s, which is what
        # the walking policy was trained to accept.
        self.back_out_speed = back_out_speed
        self.back_out_max_dist = back_out_max_dist
        self.back_out_min_angle = back_out_min_angle
        self.pause_start_time = None
        self.pause_duration = 3.0
        self.is_finished = False
        self._last_vx = 0.0
        self._last_yaw = 0.0
        self.phase = "APPROACH"
        self._phase_timer = None
        self._target_depart_yaw = None

    def _send_smoothed_velocity(self, controller, target_vx, target_yaw):
        max_dvx = self._max_dvx
        max_dyaw = self._max_dyaw
        vx = self._last_vx + float(np.clip(target_vx - self._last_vx, -max_dvx, max_dvx))
        yaw = self._last_yaw + float(np.clip(target_yaw - self._last_yaw, -max_dyaw, max_dyaw))
        self._last_vx = vx
        self._last_yaw = yaw
        set_velocity(controller, vx=vx, vy=0.0, yaw=yaw)

    def _rotate_in_place(self, controller, target_yaw, current_yaw, tol):
        yaw_err = normalize_angle(target_yaw - current_yaw)
        if abs(yaw_err) > tol:
            yaw_mag = float(np.clip(abs(yaw_err) * 1.5,
                                    self.turn_rate_min, self.turn_rate_max))
            yaw_cmd = float(np.sign(yaw_err) * yaw_mag)
            self._send_smoothed_velocity(controller, 0.0, yaw_cmd)
            return False
        self._send_smoothed_velocity(controller, 0.0, 0.0)
        return True

    def _begin_inspection(self, controller, wp, elapsed_time):
        print(f"[Navigator] ARRIVED at {wp['name']}! Pausing {self.pause_duration}s for inspection...")
        if self.on_inspect is not None:
            self.on_inspect(controller, wp, elapsed_time)
        self.pause_start_time = elapsed_time
        self.phase = "INSPECT_PAUSE"

    def _can_back_out(self, next_wp, prev_wp, x, y, yaw):
        """Whether reversing out beats turning round, and is a retrace.

        Four conditions. The decisive one is that the next waypoint is the one
        the robot arrived from: then reversing retraces the edge it just walked,
        which tests/check_scene.py has swept, rather than striking out across
        unswept floor. The rest are that the waypoint is within reversing range,
        that it really is behind the robot, and - implied by the first - that
        the robot is still on its arrival heading, which is what aiming with the
        head instead of the body preserves.
        """
        if prev_wp is None or next_wp["name"] != prev_wp["name"]:
            return False
        dist = float(np.hypot(next_wp["x"] - x, next_wp["y"] - y))
        if dist > self.back_out_max_dist:
            return False
        bearing = float(np.arctan2(next_wp["y"] - y, next_wp["x"] - x))
        return abs(normalize_angle(bearing - yaw)) >= self.back_out_min_angle

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
            vx_cmd = float(np.clip(self.decel_gain * dist,
                                   self.vx_min, self.vx_cruise))
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

        if self.phase == "FACE_STATION":
            target = wp.get("target")
            if target is None:
                # No look-at point declared for this station, so there is
                # nothing to aim at: square the body up as before.
                if self._rotate_in_place(controller, wp["yaw"], current_yaw,
                                         self.yaw_tol):
                    self._begin_inspection(controller, wp, elapsed_time)
                return

            # Aim the cameras, which are on the head, at the point on the
            # equipment the scene says to look at. The body turns only by the
            # part the head's +-58 deg of yaw cannot reach - usually nothing.
            # Squaring the whole body up to a station was 334 deg of in-place
            # rotation per round spent pointing the body at something the
            # checklist reads out of the model regardless.
            residual = aim_at(controller, target, current_yaw)
            if abs(residual) > self.yaw_tol:
                self._rotate_in_place(controller, current_yaw + residual,
                                      current_yaw, self.yaw_tol)
                return

            self._send_smoothed_velocity(controller, 0.0, 0.0)
            if head_settled(controller) and abs(self._last_yaw) < 1e-3:
                self._begin_inspection(controller, wp, elapsed_time)
            return

        if self.phase == "FACE_FINAL":
            if self._rotate_in_place(controller, wp["yaw"], current_yaw, self.yaw_tol):
                self._advance_waypoint()
            return

        if self.phase == "INSPECT_PAUSE":
            self._send_smoothed_velocity(controller, 0.0, 0.0)
            if elapsed_time - self.pause_start_time >= self.pause_duration:
                print(f"[Navigator] Inspection complete at {wp['name']}.")
                # Walk with a level forward gaze, not a cocked head.
                center_head(controller)
                if self.current_idx + 1 < len(self.waypoints):
                    next_wp = self.waypoints[self.current_idx + 1]
                    self._target_depart_yaw = float(np.arctan2(next_wp["y"] - wp["y"], next_wp["x"] - wp["x"]))
                    prev_wp = (self.waypoints[self.current_idx - 1]
                               if self.current_idx else None)
                    if self._can_back_out(next_wp, prev_wp,
                                          current_x, current_y, current_yaw):
                        print(f"[Navigator] Reversing out of {wp['name']} "
                              f"instead of turning round.")
                        self.phase = "BACK_OUT"
                    else:
                        self.phase = "ROTATE_TO_NEXT"
                else:
                    self._advance_waypoint()
            return

        if self.phase == "BACK_OUT":
            # Reversing towards the *next* waypoint, not the station underfoot,
            # so the distance and bearing are measured against that one.
            back_wp = self.waypoints[self.current_idx + 1]
            back_dx = back_wp["x"] - current_x
            back_dy = back_wp["y"] - current_y
            if float(np.hypot(back_dx, back_dy)) <= self.pos_tol:
                self._send_smoothed_velocity(controller, 0.0, 0.0)
                self._advance_waypoint()
                return
            # Hold the heading the robot walked in on, measured against the
            # reverse bearing, so it reverses straight back down the spur.
            bearing = float(np.arctan2(back_dy, back_dx))
            yaw_err = normalize_angle(normalize_angle(bearing + np.pi) - current_yaw)
            yaw_cmd = float(np.clip(self.heading_gain * yaw_err,
                                    -self.yaw_rate_max, self.yaw_rate_max))
            self._send_smoothed_velocity(controller, -self.back_out_speed, yaw_cmd)
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

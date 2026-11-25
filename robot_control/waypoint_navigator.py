import numpy as np
from robot_control.velocity_interface import set_velocity, stop_robot, get_robot_pose

def quat_to_yaw(q):
    w, x, y, z = q[0], q[1], q[2], q[3]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(np.arctan2(siny_cosp, cosy_cosp))

class WaypointNavigator:
    def __init__(self, waypoints, pos_tolerance=0.25, yaw_tolerance=0.20):
        self.waypoints = waypoints
        self.current_idx = 0
        self.pos_tol = pos_tolerance
        self.yaw_tol = yaw_tolerance
        self.pause_start_time = None
        self.pause_duration = 3.0
        self.is_finished = False
        self._last_vx = 0.0
        self._last_yaw = 0.0

    def _send_smoothed_velocity(self, controller, target_vx, target_yaw):
        max_dvx = 0.03
        max_dyaw = 0.05
        vx = self._last_vx + float(np.clip(target_vx - self._last_vx, -max_dvx, max_dvx))
        yaw = self._last_yaw + float(np.clip(target_yaw - self._last_yaw, -max_dyaw, max_dyaw))
        self._last_vx = vx
        self._last_yaw = yaw
        set_velocity(controller, vx=vx, vy=0.0, yaw=yaw)

    def update(self, controller, elapsed_time):
        if self.is_finished or self.current_idx >= len(self.waypoints):
            self._send_smoothed_velocity(controller, 0.0, 0.0)
            self.is_finished = True
            return

        wp = self.waypoints[self.current_idx]
        pos, quat = get_robot_pose(controller)
        current_x, current_y = pos[0], pos[1]
        current_yaw = quat_to_yaw(quat)

        dx = wp["x"] - current_x
        dy = wp["y"] - current_y
        dist = np.hypot(dx, dy)

        if self.pause_start_time is not None:
            self._send_smoothed_velocity(controller, 0.0, 0.0)
            if elapsed_time - self.pause_start_time >= self.pause_duration:
                print(f"[Navigator] Checkpoint completed: {wp['name']}. Moving to next...")
                self.pause_start_time = None
                self.current_idx += 1
            return

        if dist > self.pos_tol:
            target_heading = np.arctan2(dy, dx)
            heading_err = (target_heading - current_yaw + np.pi) % (2 * np.pi) - np.pi

            if abs(heading_err) > 0.45:
                yaw_cmd = float(np.clip(1.2 * heading_err, -0.6, 0.6))
                self._send_smoothed_velocity(controller, 0.0, yaw_cmd)
            else:
                heading_scale = max(0.0, 1.0 - abs(heading_err) / 0.45)
                vx_cmd = float(np.clip(0.6 * dist, 0.28, 0.45)) * heading_scale
                yaw_cmd = float(np.clip(1.0 * heading_err, -0.4, 0.4))
                self._send_smoothed_velocity(controller, vx_cmd, yaw_cmd)
        else:
            final_yaw_err = (wp["yaw"] - current_yaw + np.pi) % (2 * np.pi) - np.pi
            if abs(final_yaw_err) > self.yaw_tol:
                yaw_mag = float(np.clip(abs(final_yaw_err) * 1.2, 0.35, 0.6))
                yaw_cmd = float(np.sign(final_yaw_err) * yaw_mag)
                self._send_smoothed_velocity(controller, 0.0, yaw_cmd)
            else:
                self._send_smoothed_velocity(controller, 0.0, 0.0)
                print(f"[Navigator] ARRIVED at Checkpoint {self.current_idx + 1}: {wp['name']}! Pausing {self.pause_duration}s for inspection...")
                self.pause_start_time = elapsed_time

import time
from booster_deploy.controllers.mujoco_controller import MujocoController
from robot_control.velocity_interface import get_robot_pose

class LabMateController(MujocoController):
    def __init__(self, cfg):
        super().__init__(cfg)
        self.navigator = None
        self._sim_start_time = None
        self._last_log_time = 0.0

    def set_navigator(self, navigator):
        self.navigator = navigator

    def update_vel_command(self):
        # Override: Disable Booster Deploy terminal stdin reading
        pass

    def policy_step(self):
        elapsed = self._elapsed_s

        if self.navigator is not None:
            self.navigator(self, elapsed)

        if elapsed - self._last_log_time >= 1.0:
            pos, _ = get_robot_pose(self)
            cmd = self.vel_command
            print(f"[{elapsed:5.1f}s] Pos: X={pos[0]:.2f}, Y={pos[1]:.2f} | Cmd: vx={cmd.lin_vel_x:.2f}, vy={cmd.lin_vel_y:.2f}, yaw={cmd.ang_vel_yaw:.2f}")
            self._last_log_time = elapsed

        return super().policy_step()

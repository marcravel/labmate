import time
from booster_deploy.controllers.mujoco_controller import MujocoController
from robot_control.head_control import apply_head_target
from robot_control.velocity_interface import get_robot_pose

class LabMateController(MujocoController):
    def __init__(self, cfg):
        super().__init__(cfg)
        self.navigator = None
        # Where the head has been asked to look, as (yaw, pitch) in radians, or
        # None to leave it alone. Written only through robot_control.head_control.
        self.head_target = None
        self._head_held = None
        # Optional per-step observer, called with the controller after the
        # navigator and before the policy runs. Used by perception to record
        # the head-camera feed. It observes only: it must not command anything,
        # and it is not allowed to stop the patrol if it fails.
        self.frame_sink = None
        self._stop_requested = False
        self._sim_start_time = None
        self._last_log_time = 0.0

    def set_navigator(self, navigator):
        self.navigator = navigator

    def set_frame_sink(self, sink):
        self.frame_sink = sink

    def request_stop(self):
        """End the run cleanly once this policy step has finished.

        Calling ``stop()`` directly from a navigator or a frame sink would
        leave the rest of the step to run against a stopped controller, and
        ``BaseController.policy_step`` refuses that outright. This defers it to
        the end of the step, so the loop simply does not begin another one.
        """
        self._stop_requested = True

    def update_vel_command(self):
        # Override: Disable Booster Deploy terminal stdin reading
        pass

    def policy_step(self):
        elapsed = self._elapsed_s

        if self.navigator is not None:
            self.navigator(self, elapsed)

        if self.frame_sink is not None:
            self.frame_sink(self)

        if elapsed - self._last_log_time >= 1.0:
            pos, _ = get_robot_pose(self)
            cmd = self.vel_command
            print(f"[{elapsed:5.1f}s] Pos: X={pos[0]:.2f}, Y={pos[1]:.2f} | Cmd: vx={cmd.lin_vel_x:.2f}, vy={cmd.lin_vel_y:.2f}, yaw={cmd.ang_vel_yaw:.2f}")
            self._last_log_time = elapsed

        dof_targets = super().policy_step()

        # The walking policy drives 20 of the robot's 22 joints and neither
        # sees nor commands the two head joints, so the head is ours to aim.
        # This runs after inference, on the policy's own output tensor, and
        # touches nothing but the two head entries.
        dof_targets = apply_head_target(self, dof_targets)

        if self._stop_requested:
            self.stop()
        return dof_targets

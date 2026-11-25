
def set_velocity(controller, vx: float = 0.0, vy: float = 0.0, yaw: float = 0.0):
    cmd = controller.vel_command
    if cmd is None:
        raise RuntimeError("Controller vel_command object is not initialized.")

    if vx >= 0.0:
        cmd.lin_vel_x = min(vx, cmd.vx_forward_max)
    else:
        cmd.lin_vel_x = max(vx, -cmd.vx_backward_max)

    cmd.lin_vel_y = max(-cmd.vy_max, min(vy, cmd.vy_max))
    cmd.ang_vel_yaw = max(-cmd.vyaw_max, min(yaw, cmd.vyaw_max))


def stop_robot(controller):
    set_velocity(controller, 0.0, 0.0, 0.0)


def get_robot_pose(controller):
    pos = controller.robot.data.root_pos_w
    quat = controller.robot.data.root_quat_w

    # Convert PyTorch Tensor to NumPy array if needed
    if hasattr(pos, "clone"):
        pos = pos.clone().detach().cpu().numpy()
    else:
        pos = pos.copy()

    if hasattr(quat, "clone"):
        quat = quat.clone().detach().cpu().numpy()
    else:
        quat = quat.copy()

    return pos, quat

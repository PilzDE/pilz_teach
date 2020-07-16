def project_twist_on_plane(command, projection_plane):
    first = command.linear.x
    second = command.linear.y
    command.linear.x, command.linear.y = 0, 0
    setattr(command.linear, projection_plane[0], first)
    setattr(command.linear, projection_plane[1], second)


def scale_twist_linear_velocity(command, vel_scale):
    command.linear.x = _scale_value(command.linear.x, vel_scale)
    command.linear.y = _scale_value(command.linear.y, vel_scale)
    command.linear.z = _scale_value(command.linear.z, vel_scale)


def scale_twist_angular_velocity(command, ang_vel_scale):
    command.angular.x = _scale_value(command.angular.x, ang_vel_scale)
    command.angular.y = _scale_value(command.angular.y, ang_vel_scale)
    command.angular.z = _scale_value(command.angular.z, ang_vel_scale)


def _scale_value(value, scale):
    if value == 'max':
        return 1
    elif value == '-max':
        return -1
    else:
        return value * scale


def scale_joint_velocity(command, vel_scale):
    command.velocities = [v * vel_scale for v in command.velocities]
    command.displacements = [d * vel_scale for d in command.displacements]


def choose_joint_to_jog(command, current_joint):
    if len(command.joint_names) == 0:
        command.joint_names = [current_joint]

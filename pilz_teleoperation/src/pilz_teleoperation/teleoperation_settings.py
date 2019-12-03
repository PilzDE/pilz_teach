# Copyright (c) 2019 Pilz GmbH & Co. KG
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from pilz_teleoperation.srv import SetTeleopSettingsRequest

""" Settings for teleoperation driver to configure e.g. the jogging-plane and velocity """

FRAMES = ["world", "prbt_tcp"]
JOINTS = ["prbt_joint_1", "prbt_joint_2", "prbt_joint_3", "prbt_joint_4", "prbt_joint_5", "prbt_joint_6"]
VELOCITIES = [.2, .6, 1]

VELOCITY_SPEEDUP_LINEAR = 0.01
VELOCITY_SPEEDUP_ANGULAR = 0.01
MAX_VELOCITY_LINEAR = 1.0
MAX_VELOCITY_ANGULAR = 1.0
MIN_VELOCITY_LINEAR = 0.1
MIN_VELOCITY_ANGULAR = 0.1
DEFAULT_VELOCITY_LINEAR = 0.2
DEFAULT_VELOCITY_ANGULAR = 0.2


def _decrease_linear_velocity(settings):
    settings.linear_velocity = max(MIN_VELOCITY_LINEAR, settings.linear_velocity - VELOCITY_SPEEDUP_LINEAR)
    return True


def _increase_linear_velocity(settings):
    settings.linear_velocity = min(MAX_VELOCITY_LINEAR, settings.linear_velocity + VELOCITY_SPEEDUP_LINEAR)
    return True


def _decrease_angular_velocity(settings):
    settings.angular_velocity = max(MIN_VELOCITY_ANGULAR, settings.angular_velocity - VELOCITY_SPEEDUP_ANGULAR)
    return True


def _increase_angular_velocity(settings):
    settings.angular_velocity = min(MAX_VELOCITY_ANGULAR, settings.angular_velocity + VELOCITY_SPEEDUP_ANGULAR)
    return True


def _toggle_target_frame(settings):
    settings.frame_index = (settings.frame_index + 1) % len(FRAMES)
    settings.frame = FRAMES[settings.frame_index]
    return True


def _toggle_projection_plane(settings):
    options = [SetTeleopSettingsRequest.USE_XY_PLANE,
               SetTeleopSettingsRequest.USE_XZ_PLANE,
               SetTeleopSettingsRequest.USE_YZ_PLANE]
    settings.movement_projection_plane = options[(options.index(settings.movement_projection_plane) + 1) % len(options)]
    return True


def _toggle_joint_up(settings):
    settings.joint_index = (settings.joint_index + 1) % len(JOINTS)
    settings.joint = JOINTS[settings.joint_index]
    return True


def _toggle_joint_down(settings):
    settings.joint_index = (settings.joint_index - 1) % len(JOINTS)
    settings.joint = JOINTS[settings.joint_index]
    return True


def _toggle_velocity_up(settings):
    settings.velocity_index = (settings.velocity_index + 1) % len(VELOCITIES)
    settings.linear_velocity = VELOCITIES[settings.velocity_index]
    settings.angular_velocity = VELOCITIES[settings.velocity_index]
    return True


def _toggle_velocity_down(settings):
    settings.velocity_index = (settings.velocity_index - 1) % len(VELOCITIES)
    settings.linear_velocity = VELOCITIES[settings.velocity_index]
    settings.angular_velocity = VELOCITIES[settings.velocity_index]
    return True


def _toggle_rotation_axis_up(settings):
    settings.rotation_axis = (settings.rotation_axis + 1) % 3
    return True


def _toggle_rotation_axis_down(settings):
    settings.rotation_axis = (settings.rotation_axis - 1) % 3
    return True


class TeleoperationSettings:
    setting_change_method_bindings = {
        SetTeleopSettingsRequest.DECREASE_ANGULAR_VELOCITY: _decrease_angular_velocity,
        SetTeleopSettingsRequest.INCREASE_ANGULAR_VELOCITY: _increase_angular_velocity,
        SetTeleopSettingsRequest.DECREASE_LINEAR_VELOCITY: _decrease_linear_velocity,
        SetTeleopSettingsRequest.INCREASE_LINEAR_VELOCITY: _increase_linear_velocity,
        SetTeleopSettingsRequest.TOGGLE_TARGET_FRAME: _toggle_target_frame,
        SetTeleopSettingsRequest.TOGGLE_PLANE: _toggle_projection_plane,
        SetTeleopSettingsRequest.TOGGLE_JOINT_UP: _toggle_joint_up,
        SetTeleopSettingsRequest.TOGGLE_JOINT_DOWN: _toggle_joint_down,
        SetTeleopSettingsRequest.TOGGLE_VELOCITY_UP: _toggle_velocity_up,
        SetTeleopSettingsRequest.TOGGLE_VELOCITY_DOWN: _toggle_velocity_down,
        SetTeleopSettingsRequest.TOGGLE_ROTATION_AXIS_UP: _toggle_rotation_axis_up,
        SetTeleopSettingsRequest.TOGGLE_ROTATION_AXIS_DOWN: _toggle_rotation_axis_down
    }

    def __init__(self):
        self.angular_velocity = DEFAULT_VELOCITY_ANGULAR
        self.linear_velocity = DEFAULT_VELOCITY_LINEAR
        self.frame = FRAMES[0]
        self.frame_index = 0
        self.joint = JOINTS[0]
        self.joint_index = 0
        self.velocity_index = 0
        self.rotation_axis = 0
        self.movement_projection_plane = SetTeleopSettingsRequest.USE_XY_PLANE

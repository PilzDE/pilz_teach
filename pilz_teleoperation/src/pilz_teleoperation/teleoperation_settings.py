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


WORLD_FRAME = "world"
TCP_FRAME = "prbt_tcp"
VELOCITY_SPEEDUP_LINEAR = 0.01
VELOCITY_SPEEDUP_ANGULAR = 0.01
MAX_VELOCITY_LINEAR = 1.0
MAX_VELOCITY_ANGULAR = 1.0
MIN_VELOCITY_LINEAR = 0.1
MIN_VELOCITY_ANGULAR = 0.1
MAX_JOG_RANGE = 0.2


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


def _toggle_target_frame_world_tcp(settings):
    settings.frame = TCP_FRAME if settings.frame == WORLD_FRAME else WORLD_FRAME
    return True


def _toggle_projection_plane(settings):
    options = [SetTeleopSettingsRequest.USE_XY_PLANE,
               SetTeleopSettingsRequest.USE_XZ_PLANE,
               SetTeleopSettingsRequest.USE_YZ_PLANE]
    settings.movement_projection_plane = options[(options.index(settings.movement_projection_plane) + 1) % len(options)]
    return True


class TeleoperationSettings:
    setting_change_method_bindings = {
        SetTeleopSettingsRequest.DECREASE_ANGULAR_VELOCITY: _decrease_angular_velocity,
        SetTeleopSettingsRequest.INCREASE_ANGULAR_VELOCITY: _increase_angular_velocity,
        SetTeleopSettingsRequest.DECREASE_LINEAR_VELOCITY: _decrease_linear_velocity,
        SetTeleopSettingsRequest.INCREASE_LINEAR_VELOCITY: _increase_linear_velocity,
        SetTeleopSettingsRequest.TOGGLE_WORLD_AND_TCP_FRAME: _toggle_target_frame_world_tcp,
        SetTeleopSettingsRequest.TOGGLE_PLANE: _toggle_projection_plane
    }

    def __init__(self):
        self.angular_velocity = 0.5
        self.linear_velocity = 0.5
        self.frame = WORLD_FRAME
        self.movement_projection_plane = SetTeleopSettingsRequest.USE_XY_PLANE

    def get_current_plane_string(self):
        """ returns a string representation of the current plane, the movement takes place on """
        return "XY" if self.movement_projection_plane == SetTeleopSettingsRequest.USE_XY_PLANE \
                    else ("XZ" if self.movement_projection_plane == SetTeleopSettingsRequest.USE_XZ_PLANE else "YZ")

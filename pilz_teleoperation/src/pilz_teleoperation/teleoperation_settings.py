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
import rospy


class TeleoperationSettings(object):
    def __init__(self):
        self.__load_parameters()

        self.setting_change_method_bindings = {
            SetTeleopSettingsRequest.DECREASE_ANGULAR_VELOCITY: self._decrease_angular_velocity,
            SetTeleopSettingsRequest.INCREASE_ANGULAR_VELOCITY: self._increase_angular_velocity,
            SetTeleopSettingsRequest.DECREASE_LINEAR_VELOCITY: self._decrease_linear_velocity,
            SetTeleopSettingsRequest.INCREASE_LINEAR_VELOCITY: self._increase_linear_velocity,
            SetTeleopSettingsRequest.TOGGLE_TARGET_FRAME_UP: self._toggle_target_frame_up,
            SetTeleopSettingsRequest.TOGGLE_TARGET_FRAME_DOWN: self._toggle_target_frame_down,
            SetTeleopSettingsRequest.TOGGLE_JOINT_UP: self._toggle_joint_up,
            SetTeleopSettingsRequest.TOGGLE_JOINT_DOWN: self._toggle_joint_down,
            SetTeleopSettingsRequest.TOGGLE_PLANE_UP: self._toggle_plane_up,
            SetTeleopSettingsRequest.TOGGLE_PLANE_DOWN: self._toggle_plane_down,
            SetTeleopSettingsRequest.TOGGLE_ROTATION_AXIS_UP: self._toggle_axis_up,
            SetTeleopSettingsRequest.TOGGLE_ROTATION_AXIS_DOWN: self._toggle_axis_down
        }

        self.toggled_target_frame = self._target_frames_to_toggle[self._target_frame_index]
        self.toggled_joint = self._joints_to_toggle[self._joint_index]
        self.toggled_plane = self._planes_to_toggle[self._plane_index]
        self.toggled_axis = self._axes_to_toggle[self._axis_index]

        self.angular_velocity = rospy.get_param("~default_angular_velocity")
        self.linear_velocity = rospy.get_param("~default_linear_velocity")

    def __load_parameters(self):
        self._target_frames_to_toggle = rospy.get_param("~target_frames_to_toggle")
        self._joints_to_toggle = rospy.get_param("~joints_to_toggle")
        self._planes_to_toggle = rospy.get_param("~planes_to_toggle")
        self._axes_to_toggle = rospy.get_param("~axes_to_toggle")
        self._min_linear_velocity = rospy.get_param("~min_linear_velocity")
        self._max_linear_velocity = rospy.get_param("~max_linear_velocity")
        self._step_size_linear_velocity = rospy.get_param("~step_size_linear_velocity")
        self._min_angular_velocity = rospy.get_param("~min_angular_velocity")
        self._max_angular_velocity = rospy.get_param("~max_angular_velocity")
        self._step_size_angular_velocity = rospy.get_param("~step_size_angular_velocity")

        self._target_frame_index = rospy.get_param("~default_target_frame_index")
        self._joint_index = rospy.get_param("~default_joint_index")
        self._plane_index = rospy.get_param("~default_plane_index")
        self._axis_index = rospy.get_param("~default_axis_index")

    def get_joints(self):  # pragma: no cover
        return tuple(self._joints_to_toggle)

    def _decrease_angular_velocity(self):
        self.angular_velocity = self._change_speed(current_speed=self.angular_velocity,
                                                   step=-self._step_size_angular_velocity,
                                                   min_=self._min_angular_velocity,
                                                   max_=self._max_angular_velocity)

    def _increase_angular_velocity(self):
        self.angular_velocity = self._change_speed(current_speed=self.angular_velocity,
                                                   step=self._step_size_angular_velocity,
                                                   min_=self._min_angular_velocity,
                                                   max_=self._max_angular_velocity)

    def _decrease_linear_velocity(self):
        self.linear_velocity = self._change_speed(current_speed=self.linear_velocity,
                                                  step=-self._step_size_linear_velocity,
                                                  min_=self._min_linear_velocity,
                                                  max_=self._max_linear_velocity)

    def _increase_linear_velocity(self):
        self.linear_velocity = self._change_speed(current_speed=self.linear_velocity,
                                                  step=self._step_size_linear_velocity,
                                                  min_=self._min_linear_velocity,
                                                  max_=self._max_linear_velocity)

    def _toggle_target_frame_up(self):
        self._target_frame_index, self.toggled_target_frame = self._toggle(current_index=self._target_frame_index,
                                                                           toggables=self._target_frames_to_toggle)

    def _toggle_target_frame_down(self):
        self._target_frame_index, self.toggled_target_frame = self._toggle(current_index=self._target_frame_index,
                                                                           toggables=self._target_frames_to_toggle,
                                                                           step=-1)

    def _toggle_joint_up(self):
        self._joint_index, self.toggled_joint = self._toggle(current_index=self._joint_index,
                                                             toggables=self._joints_to_toggle)

    def _toggle_joint_down(self):
        self._joint_index, self.toggled_joint = self._toggle(current_index=self._joint_index,
                                                             toggables=self._joints_to_toggle,
                                                             step=-1)

    def _toggle_plane_up(self):
        self._plane_index, self.toggled_plane = self._toggle(current_index=self._plane_index,
                                                             toggables=self._planes_to_toggle)

    def _toggle_plane_down(self):
        self._plane_index, self.toggled_plane = self._toggle(current_index=self._plane_index,
                                                             toggables=self._planes_to_toggle,
                                                             step=-1)

    def _toggle_axis_up(self):
        self._axis_index, self.toggled_axis = self._toggle(current_index=self._axis_index,
                                                           toggables=self._axes_to_toggle)

    def _toggle_axis_down(self):
        self._axis_index, self.toggled_axis = self._toggle(current_index=self._axis_index,
                                                           toggables=self._axes_to_toggle,
                                                           step=-1)

    @staticmethod
    def _change_speed(current_speed, step, max_=99, min_=0):
        return max(min_, min(max_, current_speed + step))

    @staticmethod
    def _toggle(current_index, toggables, step=1):
        next_index = (current_index + step) % len(toggables)
        return next_index, toggables[next_index]

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

import rospy
import math
import copy
import pilz_teleoperation.teleoperation_settings as _teleop_settings

from geometry_msgs.msg import Twist, TwistStamped, Vector3
from control_msgs.msg import JointJog
from pilz_teleoperation.srv import SetTeleopSettings, SetTeleopSettingsResponse


class _TeleoperationTwist(object):
    """ Extension of Twist class to store the required math. """
    def __init__(self, twist=None, *args, **kwargs):
        super(_TeleoperationTwist, self).__init__(*args, **kwargs)
        self.linear = Vector3(twist.linear.x,
                              twist.linear.y,
                              twist.linear.z)
        self.angular = Vector3(twist.angular.x,
                               twist.angular.y,
                               twist.angular.z)

    def project_on_plane(self, projection_plane):
        first = self.linear.x
        second = self.linear.y
        self.linear.x, self.linear.y = 0, 0
        setattr(self.linear, projection_plane[0], first)
        setattr(self.linear, projection_plane[1], second)

    def change_rotation_axis(self, axis):
        value = self.angular.x
        self.angular.x = 0
        setattr(self.angular, axis, value)

    def normalize(self):
        norm = math.sqrt(self.linear.x ** 2 + self.linear.y ** 2 + self.linear.z ** 2)
        if norm > 0:
            self.scale_linear_velocity(1/norm)

    def scale_linear_velocity(self, vel_scale):
        self.linear.x = self.scale_value(self.linear.x, vel_scale)
        self.linear.y = self.scale_value(self.linear.y, vel_scale)
        self.linear.z = self.scale_value(self.linear.z, vel_scale)

    def scale_angular_velocity(self, ang_vel_scale):
        self.angular.x = self.scale_value(self.angular.x, ang_vel_scale)
        self.angular.y = self.scale_value(self.angular.y, ang_vel_scale)
        self.angular.z = self.scale_value(self.angular.z, ang_vel_scale)

    def scale_value(self, value, scale):
        if value == 'max':
            return 1
        elif value == '-max':
            return -1
        else:
            return value * scale


class _TeleoperationJointJog(object):
    """ Extension of JointJog class to store required math. """
    def __init__(self, joint_jog, *args, **kwargs):
        super(_TeleoperationJointJog, self).__init__(*args, **kwargs)
        self.joint_names = joint_jog.joint_names
        self.displacements = joint_jog.displacements
        self.velocities = joint_jog.velocities

    def scale_velocity(self, vel_scale):
        self.velocities = [v * vel_scale for v in self.velocities]
        self.displacements = [d * vel_scale for d in self.displacements]

    def choose_joint_to_jog(self, current_joint):
        if len(self.joint_names) == 0:
            self.joint_names = [current_joint]

    def copy_jog_data(self, js):
        js.joint_names = self.joint_names
        js.velocities = self.velocities
        js.displacements = self.displacements


class TeleoperationDriver(object):
    """ Main class of the teleoperation driver.

        This driver can be used to mount any input device to the jog server for jogging purpose.
        The input node has to provide at least a 2D twist for a movement on a 2D plane.

        TOPICS:
            - /teleop_twist: incoming move commands
            - /jog_server/delta_jog_cmds: outgoing jog commands

        SERVICES:
            - /set_teleop_settings: teleoperation settings like velocity scaling, target frame, ...
    """
    KEY_INPUT_TIMEOUT = 0.05

    def __init__(self, window):
        """
        :param window: To display current settings
        :type window: TeleoperationWindow
        """
        super(TeleoperationDriver, self).__init__()
        self._output_window = window
        self._settings = _teleop_settings.TeleoperationSettings()
        self.__ros_init()
        self._update_settings_display()

    def __ros_init(self):
        self._sv_settings = \
            rospy.Service("%s/set_settings" % rospy.get_name(), SetTeleopSettings, self.set_teleop_settings)
        self._sub_twist = rospy.Subscriber("%s/twist" % rospy.get_name(), Twist, self.set_twist_command, queue_size=1)
        self._sub_joint_jog = \
            rospy.Subscriber("%s/joint_jog" % rospy.get_name(), JointJog, self.set_joint_jog_command, queue_size=1)
        self._twist_publisher = rospy.Publisher("/jog_server/delta_jog_cmds", TwistStamped, queue_size=1)
        self._jog_publisher = rospy.Publisher("/jog_server/joint_delta_jog_cmds", JointJog, queue_size=1)

    def set_teleop_settings(self, req):
        for command in req.pressed_commands:
            try:
                self._settings.setting_change_method_bindings[command]()
                self._update_settings_display()
                return SetTeleopSettingsResponse(success=True)
            except KeyError:
                return SetTeleopSettingsResponse(success=False, error_msg="Unsupported Command")

    def _update_settings_display(self):
        self._output_window.driver_settings_changed(copy.copy(self._settings))

    def set_twist_command(self, twist_):
        self._send_updated_twist(self.__get_stamped_twist(twist_))

    def set_joint_jog_command(self, joint_jog_):
        self._send_updated_jog(joint_jog_)

    def __get_stamped_twist(self, twist_=None):
        st = TwistStamped(twist=twist_)
        st.header.frame_id = self._settings.toggled_target_frame
        st.header.stamp = rospy.Time.now()
        return st

    def _send_updated_twist(self, ts):
        new_twist = _TeleoperationTwist(twist=ts.twist)
        new_twist.project_on_plane(self._settings.toggled_plane)
        new_twist.scale_linear_velocity(self._settings.linear_velocity)
        new_twist.scale_angular_velocity(self._settings.angular_velocity)
        ts.twist = new_twist
        self._twist_publisher.publish(ts)

    def _send_updated_jog(self, js):
        js.header.stamp = rospy.Time.now()
        js.header.frame_id = self._settings.toggled_target_frame

        new_jog = _TeleoperationJointJog(joint_jog=js)
        new_jog.choose_joint_to_jog(self._settings.toggled_joint)
        new_jog.copy_jog_data(js)
        new_jog.scale_velocity(self._settings.angular_velocity)

        self._jog_publisher.publish(js)

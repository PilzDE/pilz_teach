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
import copy
import pilz_teleoperation.teleoperation_settings as _teleop_settings


from geometry_msgs.msg import Twist, TwistStamped
from control_msgs.msg import JointJog
from pilz_teleoperation.srv import SetTeleopSettings, SetTeleopSettingsResponse
from pilz_teleoperation import *


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
        self._send_updated_twist(twist_)

    def _send_updated_twist(self, ts):
        self._twist_publisher.publish(self._update_twist(ts))

    def _update_twist(self, ts):
        project_twist_on_plane(ts, self._settings.toggled_plane)
        scale_twist_linear_velocity(ts, self._settings.linear_velocity)
        scale_twist_angular_velocity(ts, self._settings.angular_velocity)
        return self._create_stamped_twist(ts)

    def _create_stamped_twist(self, ts):
        st = TwistStamped(twist=ts)
        st.header.frame_id = self._settings.toggled_target_frame
        st.header.stamp = rospy.Time.now()
        return st

    def set_joint_jog_command(self, joint_jog_):
        self._send_updated_jog(joint_jog_)

    def _send_updated_jog(self, js):
        self._jog_publisher.publish(self._update_jog(js))

    def _update_jog(self, js):
        js.header.stamp = rospy.Time.now()
        js.header.frame_id = self._settings.toggled_target_frame
        choose_joint_to_jog(js, self._settings.toggled_joint)
        scale_joint_velocity(js, self._settings.angular_velocity)
        return js

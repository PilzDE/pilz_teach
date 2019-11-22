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
import pilz_teleoperation.teleoperation_settings as _teleop_settings

from geometry_msgs.msg import Twist, TwistStamped, Vector3
from pilz_teleoperation.srv import SetTeleopSettings, SetTeleopSettingsResponse, SetTeleopSettingsRequest


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
        if projection_plane == SetTeleopSettingsRequest.USE_XZ_PLANE:
            self.linear.y, self.linear.z = 0, self.linear.y
        elif projection_plane == SetTeleopSettingsRequest.USE_YZ_PLANE:
            self.linear.x, self.linear.y, self.linear.z = 0, self.linear.x, self.linear.y

    def normalize(self):
        norm = math.sqrt((self.linear.x)**2 + (self.linear.y)**2 + (self.linear.z)**2)
        if norm > 0:
            self.scale_linear_velocity(1/norm)

    def scale_linear_velocity(self, lin_vel):
        self.linear.x *= lin_vel
        self.linear.y *= lin_vel
        self.linear.z *= lin_vel


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
        self.__last_twist_msg = TwistStamped()
        self.__ros_init()
        self._update_settings_display()

    def __ros_init(self):
        self._hz = rospy.Rate(20)
        self._sv_settings = rospy.Service("set_teleop_settings", SetTeleopSettings, self._set_teleop_settings)
        self._sub_twist = rospy.Subscriber("teleop_twist", Twist, self._twist_command_cb, queue_size=1)
        self._twist_publisher = rospy.Publisher("/jog_server/delta_jog_cmds", TwistStamped, queue_size=1)

    def _set_teleop_settings(self, req):
        for command in req.pressed_commands:
            try:
                success = self._settings.setting_change_method_bindings[command](self._settings)
                if success is True:
                    self._update_settings_display()
                    return SetTeleopSettingsResponse(success=success)
                else:
                    return SetTeleopSettingsResponse(success=False, error_msg=success)
            except KeyError:
                return SetTeleopSettingsResponse(success=False, error_msg="Unsupported Command")

    def _update_settings_display(self):
        self._output_window.driver_settings_changed(self._settings.linear_velocity,
                                                    self._settings.angular_velocity,
                                                    self._settings.frame,
                                                    self._settings.get_current_plane_string())

    def _twist_command_cb(self, twist_):
        self.__last_twist_msg = self.__get_stamped_twist(twist_)

    def __get_stamped_twist(self, twist_=None):
        st = TwistStamped(twist=twist_)
        st.header.frame_id = self._settings.frame
        st.header.stamp = rospy.Time.now()
        return st

    def update_loop(self):
        while not rospy.is_shutdown():
            self.send_updated_twist()
            self._hz.sleep()

    def send_updated_twist(self):
        """ Publishes the current twist to the jog arm driver.
            Has to be called at least with 10 HZ!
        """
        self._send_updated_twist()

    def _send_updated_twist(self):
        ts = self.__get_stamped_twist()
        if self.__key_input_is_new_enough():
            new_twist = _TeleoperationTwist(twist=self.__last_twist_msg.twist)
            new_twist.project_on_plane(self._settings.movement_projection_plane)
            new_twist.normalize()
            new_twist.scale_linear_velocity(self._settings.linear_velocity)
            ts.twist = new_twist
        self._twist_publisher.publish(ts)

    def __key_input_is_new_enough(self):
        return rospy.Time.now() - self.__last_twist_msg.header.stamp \
               < rospy.Duration.from_sec(TeleoperationDriver.KEY_INPUT_TIMEOUT)

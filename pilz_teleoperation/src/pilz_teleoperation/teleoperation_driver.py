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
from control_msgs.msg import JointJog
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
        norm = math.sqrt(self.linear.x ** 2 + self.linear.y ** 2 + self.linear.z ** 2)
        if norm > 0:
            self.scale_linear_velocity(1/norm)

    def scale_linear_velocity(self, lin_vel):
        self.linear.x *= lin_vel
        self.linear.y *= lin_vel
        self.linear.z *= lin_vel


class _TeleoperationJointJog(object):
    """ Extension of JointJog class to store required math. """
    def __init__(self, joint_jog, *args, **kwargs):
        super(_TeleoperationJointJog, self).__init__(*args, **kwargs)
        self.joint_names = joint_jog.joint_names
        self.displacements = joint_jog.displacements
        self.velocities = joint_jog.velocities

    def scale_linear_velocity(self, lin_vel):
        self.velocities = [v * lin_vel for v in self.velocities]
        self.displacements = [d * lin_vel for d in self.displacements]

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
        self.__last_twist_msg = TwistStamped()
        self.__last_jog_msg = JointJog()
        self.__ros_init()
        self._update_settings_display()

    def __ros_init(self):
        self._hz = rospy.Rate(20)
        self._sv_settings = \
            rospy.Service("/%s/set_settings" % rospy.get_name(), SetTeleopSettings, self.set_teleop_settings)
        self._sub_twist = rospy.Subscriber("/%s/twist" % rospy.get_name(), Twist, self.set_twist_command, queue_size=1)
        self._twist_publisher = rospy.Publisher("/jog_server/delta_jog_cmds", TwistStamped, queue_size=1)
        self._jog_publisher = rospy.Publisher("/jog_server/joint_delta_jog_cmds", JointJog, queue_size=1)

    def set_teleop_settings(self, req):
        for command in req.pressed_commands:
            print(command, SetTeleopSettingsRequest.DECREASE_LINEAR_VELOCITY)
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
                                                    self._settings.movement_projection_plane,
                                                    self._settings.joint)

    def set_twist_command(self, twist_):
        self.__last_twist_msg = self.__get_stamped_twist(twist_)

    def set_joint_jog_command(self, joint_jog_):
        joint_jog_.header.stamp = rospy.Time.now()
        self.__last_jog_msg = joint_jog_

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
        if self.__last_twist_msg.header.stamp > self.__last_jog_msg.header.stamp:
            self._send_updated_twist()
        else:
            self._send_updated_jog()

    def _send_updated_twist(self):
        ts = self.__get_stamped_twist()
        if self.__key_input_is_new_enough(self.__last_twist_msg):
            new_twist = _TeleoperationTwist(twist=self.__last_twist_msg.twist)
            new_twist.project_on_plane(self._settings.movement_projection_plane)
            new_twist.normalize()
            new_twist.scale_linear_velocity(self._settings.linear_velocity)
            ts.twist = new_twist
        self._twist_publisher.publish(ts)

    def _send_updated_jog(self):
        js = JointJog()
        js.header.stamp = rospy.Time.now()
        js.header.frame_id = self._settings.frame
        if self.__key_input_is_new_enough(self.__last_jog_msg):
            new_jog = _TeleoperationJointJog(joint_jog=self.__last_jog_msg)
            new_jog.choose_joint_to_jog(self._settings.joint)
            new_jog.scale_linear_velocity(self._settings.linear_velocity)
            new_jog.copy_jog_data(js)
        else:
            js.joint_names = _teleop_settings.JOINTS
            js.velocities = [0] * len(_teleop_settings.JOINTS)
            js.displacements = [0] * len(_teleop_settings.JOINTS)
        self._jog_publisher.publish(js)

    @staticmethod
    def __key_input_is_new_enough(msg):
        return rospy.Time.now() - msg.header.stamp \
               < rospy.Duration.from_sec(TeleoperationDriver.KEY_INPUT_TIMEOUT)

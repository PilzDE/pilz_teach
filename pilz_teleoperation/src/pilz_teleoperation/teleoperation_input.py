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

import abc
import rospkg
import yaml

import rospy
from geometry_msgs.msg import Twist
from pilz_teleoperation.srv import SetTeleopSettings, SetTeleopSettingsRequest
from rospy_message_converter import message_converter
from std_msgs.msg import String


class TeleoperationInput(object):
    """ Base Class for input devices """

    MOVE_BINDINGS = {}
    SETTING_BINDINGS = {}
    INPUT_DESCRIPTION = ""

    def __init__(self, driver=None, *args, **kwargs):
        super(TeleoperationInput, self).__init__(*args, **kwargs)
        self.__driver = driver
        if self.__driver is None:
            self.__init_driver_connection()
        key_config_path = rospy.get_param("~bindings", self.__get_default_config_path())
        self._try_to_load_config(key_config_path)
        self._publish_bindings_to_driver_window()

    def __init_driver_connection(self):
        self.__twist_publisher = rospy.Publisher("/teleoperation/twist", Twist, queue_size=1)
        self.__setting_service = rospy.ServiceProxy("/teleoperation/set_settings", SetTeleopSettings)

    @staticmethod
    def __get_default_config_path():
        return rospkg.RosPack().get_path("pilz_teleoperation") + "/config/keyboard_binding.yaml"

    def _try_to_load_config(self, config_path):
        try:
            with open(config_path, "r") as file_:
                bindings = yaml.load(file_.read())
                for k, v in bindings["MovementBindings"].items():
                    self.MOVE_BINDINGS[self._get_real_key(k)] = \
                        message_converter.convert_dictionary_to_ros_message('geometry_msgs/Twist', v)
                for k, v in bindings["SettingBindings"].items():
                    self.SETTING_BINDINGS[self._get_real_key(k)] = \
                        getattr(SetTeleopSettingsRequest, v)
                self.INPUT_DESCRIPTION = bindings["Description"]
        except (KeyError, AttributeError):
            rospy.logerr("Unable to parse key binding config")
            raise KeyError("Error in binding-config syntax")

    @staticmethod
    def _get_real_key(k):
        return k

    def _publish_bindings_to_driver_window(self):
        win_conf_pub = rospy.Publisher("/teleoperation/display_input_config", String, queue_size=1, latch=True)
        win_conf_pub.publish(self.INPUT_DESCRIPTION)

    def resolve_key_input(self):
        """ Read currently pressed key and publish it to the driver.
            Should be called with at least 10 HZ!
        """
        key_code = self._read_keyboard_input()
        if key_code in self.MOVE_BINDINGS:
            self.__publish_twist_command(self.MOVE_BINDINGS[key_code])
        elif key_code in self.SETTING_BINDINGS:
            self.__change_settings(SetTeleopSettingsRequest(pressed_commands=[self.SETTING_BINDINGS[key_code]]))

    def __publish_twist_command(self, command):
        if self.__driver is not None:
            self.__driver.set_twist_command(command)
        else:
            self.__twist_publisher.publish(command)

    def __change_settings(self, command):
        if self.__driver is not None:
            self.__driver.set_teleop_settings(command)
        else:
            self.__setting_service(command)

    @abc.abstractmethod
    def _read_keyboard_input(self):
        return

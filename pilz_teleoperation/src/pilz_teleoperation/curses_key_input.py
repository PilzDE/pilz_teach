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

import curses
import rospy
import yaml
import rospkg

from std_msgs.msg import String
from rospy_message_converter import message_converter
from pilz_teleoperation.srv import SetTeleopSettingsRequest


class CursesKeyInput(object):
    """ Class that handles reading keyboard input with curses module.
        The move commands are published on a 2D plane that can be toggled between XY, XZ and YZ
        Additional Settings like target frame or velocity scaling are published separately.
    """
    MOVE_BINDINGS = {}
    SETTING_BINDINGS = {}
    INPUT_DESCRIPTION = ""

    def __init__(self, stdscr, driver):
        super(CursesKeyInput, self).__init__()
        key_config_path = rospy.get_param("~bindings", self.__get_default_config_path())
        self._try_to_load_config(key_config_path)
        self.__driver = driver
        self.__init_curses(stdscr)
        self._publish_bindings_to_driver_window()

    @staticmethod
    def __get_default_config_path():
        return rospkg.RosPack().get_path("pilz_teleoperation") + "/config/keyboard_binding.yaml"

    def _try_to_load_config(self, config_path):
        try:
            with open(config_path, "r") as file_:
                bindings = yaml.load(file_.read())
                for k, v in bindings["MovementBindings"].items():
                    self.MOVE_BINDINGS[self._get_curses_key(k)] = \
                        message_converter.convert_dictionary_to_ros_message('geometry_msgs/Twist', v)
                for k, v in bindings["SettingBindings"].items():
                    self.SETTING_BINDINGS[self._get_curses_key(k)] = \
                        getattr(SetTeleopSettingsRequest, v)
                self.INPUT_DESCRIPTION = bindings["Description"]
        except (KeyError, AttributeError):
            rospy.logerr("Unable to parse key binding config")
            raise KeyError("Error in binding-config syntax")

    @staticmethod
    def _get_curses_key(k):
        try:
            return ord(k)
        except TypeError:
            return getattr(curses, k)

    def __init_curses(self, stdscr):
        curses.cbreak()
        self._screen = stdscr
        self._screen.nodelay(True)

    def _publish_bindings_to_driver_window(self):
        win_conf_pub = rospy.Publisher("teleop_input_config_msg", String, queue_size=1, latch=True)
        win_conf_pub.publish(self.INPUT_DESCRIPTION)

    def resolve_key_input(self):
        """ Read currently pressed key and publish it to the driver.
            Should be called with at least 10 HZ!
        """
        key_code = self._read_keyboard_input()
        if key_code in self.MOVE_BINDINGS:
            self.__driver.set_twist_command(self.MOVE_BINDINGS[key_code])
        elif key_code in self.SETTING_BINDINGS:
            new_settings = SetTeleopSettingsRequest(pressed_commands=[self.SETTING_BINDINGS[key_code]])
            self.__driver.set_teleop_settings(new_settings)

    def _read_keyboard_input(self):
        key_code = self._screen.getch()
        curses.flushinp()
        return key_code


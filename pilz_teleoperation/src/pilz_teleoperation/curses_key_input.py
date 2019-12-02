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
from geometry_msgs.msg import Twist, Vector3
from rospy_message_converter import message_converter
from pilz_teleoperation.srv import SetTeleopSettingsRequest, SetTeleopSettings


class CursesKeyInput(object):
    """ Class that handles reading keyboard input with curses module.
        The move commands are published on a 2D plane that can be toggled between XY, XZ and YZ
        Additional Settings like target frame or velocity scaling are published separately.
    """

    def __init__(self, stdscr, driver):
        super(CursesKeyInput, self).__init__()
        with open(rospkg.RosPack().get_path("pilz_teleoperation") + "/config/keyboard_binding.yaml", "r") as file_:
            self.bindings = yaml.safe_load(file_.read())
            for k, v in self.bindings["MovementBindings"].items():
                self.bindings["MovementBindings"][k] = \
                    message_converter.convert_dictionary_to_ros_message('geometry_msgs/Twist', v)
        self.__driver = driver
        self.__init_curses(stdscr)
        self.__publish_bindings_to_driver_window()

    def __init_curses(self, stdscr):
        curses.cbreak()
        self._screen = stdscr
        self._screen.nodelay(True)

    def __publish_bindings_to_driver_window(self):
        win_conf_pub = rospy.Publisher("teleop_input_config_msg", String, queue_size=1, latch=True)
        win_conf_pub.publish(self.bindings["Description"])

    def resolve_key_input(self):
        """ Read currently pressed key and publish it to the driver.
            Should be called with at least 10 HZ!
        """
        key_code = self._read_keyboard_input()
        if key_code in self.bindings["MovementBindings"]:
            self.__driver.set_twist_command(self.bindings["MovementBindings"][key_code])
        elif key_code in self.bindings["SettingBindings"]:
            new_settings = SetTeleopSettingsRequest(pressed_commands=[self.bindings["SettingBindings"][key_code]])
            self.__driver.set_teleop_settings(new_settings)

    def _read_keyboard_input(self):
        key_code = self._screen.getch()
        curses.flushinp()
        try:
            return chr(key_code)
        except ValueError:
            return -1


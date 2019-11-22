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

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from pilz_teleoperation.srv import SetTeleopSettingsRequest, SetTeleopSettings


class CursesKeyInput(object):
    """ Class that handles reading keyboard input with curses module.
        The move commands are published on a 2D plane that can be toggled between XY, XZ and YZ
        Additional Settings like target frame or velocity scaling are published separately.
    """
    MOVE_BINDINGS = {
        ord('7'): Vector3(-1, 1, 0),
        ord('8'): Vector3(0, 1, 0),
        ord('9'): Vector3(1, 1, 0),
        ord('6'): Vector3(1, 0, 0),
        ord('3'): Vector3(1, -1, 0),
        ord('2'): Vector3(0, -1, 0),
        ord('1'): Vector3(-1, -1, 0),
        ord('4'): Vector3(-1, 0, 0)
    }
    SETTING_BINDINGS = {
        ord('+'): SetTeleopSettingsRequest.INCREASE_LINEAR_VELOCITY,
        ord('-'): SetTeleopSettingsRequest.DECREASE_LINEAR_VELOCITY,
        ord('*'): SetTeleopSettingsRequest.TOGGLE_PLANE,
        ord('/'): SetTeleopSettingsRequest.TOGGLE_WORLD_AND_TCP_FRAME,
        ord(','): SetTeleopSettingsRequest.TOGGLE_CONTROLLER
    }

    BINDING_TEXT = "   - Keybard Configuration:\n" \
                   "     Use numpad numbers to move!\n" \
                   "     + = increase velocity\n" \
                   "     - = decrease velocity\n" \
                   "     * = Toggle Plane to move on\n" \
                   "     / = Toggle between 'world' and 'tcp' frame\n" \
                   "     , = Toggle between controllers"

    def __init__(self, stdscr):
        super(CursesKeyInput, self).__init__()
        self.__init_curses(stdscr)
        self.__init_ros()
        self.__publish_bindings_to_driver_window()

    def __init_curses(self, stdscr):
        curses.cbreak()
        self._screen = stdscr
        self._screen.nodelay(True)

    def __init_ros(self):
        self._hz = rospy.Rate(20)
        self._twist_pub = rospy.Publisher("/teleop_twist", Twist, queue_size=1)
        self._setting_srv = rospy.ServiceProxy("/set_teleop_settings", SetTeleopSettings)

    def __publish_bindings_to_driver_window(self):
        win_conf_pub = rospy.Publisher("teleop_input_config_msg", String, queue_size=1, latch=True)
        win_conf_pub.publish(self.BINDING_TEXT)

    def resolve_key_input(self):
        """ Read currently pressed key and publish it to the driver.
            Should be called with at least 10 HZ!
        """
        key_code = self._screen.getch()
        if key_code in CursesKeyInput.MOVE_BINDINGS:
            self._twist_pub.publish(Twist(linear=CursesKeyInput.MOVE_BINDINGS[key_code]))
        elif key_code in CursesKeyInput.SETTING_BINDINGS:
            self._setting_srv(SetTeleopSettingsRequest(pressed_commands=[CursesKeyInput.SETTING_BINDINGS[key_code]]))

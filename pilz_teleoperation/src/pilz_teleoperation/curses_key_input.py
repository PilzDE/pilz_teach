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

from pilz_teleoperation import TeleoperationInput


class CursesKeyInput(TeleoperationInput):
    """ Class that handles reading keyboard input with curses module.
        The move commands are published on a 2D plane that can be toggled between XY, XZ and YZ
        Additional Settings like target frame or velocity scaling are published separately.
    """
    MOVE_BINDINGS = {
        ord('7'): Twist(linear=Vector3(-1, 1, 0)),
        ord('8'): Twist(linear=Vector3(0, 1, 0)),
        ord('9'): Twist(linear=Vector3(1, 1, 0)),
        ord('6'): Twist(linear=Vector3(1, 0, 0)),
        ord('3'): Twist(linear=Vector3(1, -1, 0)),
        ord('2'): Twist(linear=Vector3(0, -1, 0)),
        ord('1'): Twist(linear=Vector3(-1, -1, 0)),
        ord('4'): Twist(linear=Vector3(-1, 0, 0)),
        
        ord('u'): Twist(angular=Vector3(-1, 0, 0)),
        ord('i'): Twist(angular=Vector3(0, -1, 0)),
        ord('o'): Twist(angular=Vector3(0, 0, -1)),
        ord('j'): Twist(angular=Vector3(1, 0, 0)),
        ord('k'): Twist(angular=Vector3(0, 1, 0)),
        ord('l'): Twist(angular=Vector3(0, 0, 1)),
    }
    SETTING_BINDINGS = {
        ord('+'): SetTeleopSettingsRequest.INCREASE_LINEAR_VELOCITY,
        ord('-'): SetTeleopSettingsRequest.DECREASE_LINEAR_VELOCITY,
        ord('*'): SetTeleopSettingsRequest.TOGGLE_PLANE,
        ord('/'): SetTeleopSettingsRequest.TOGGLE_WORLD_AND_TCP_FRAME,
        ord(','): SetTeleopSettingsRequest.TOGGLE_CONTROLLER  # not yet implemented
    }

    BINDING_TEXT = "   - Keybard Configuration:\n" \
                   "     Use numpad numbers to move!\n" \
                   "     + = increase velocity\n" \
                   "     - = decrease velocity\n" \
                   "     * = Toggle Plane to move on\n" \
                   "     / = Toggle between 'world' and 'tcp' frame"

    def __init__(self, stdscr, driver):
        super(CursesKeyInput, self).__init__()
        self.__driver = driver
        self.__init_curses(stdscr)
        self.__publish_bindings_to_driver_window()

    def __init_curses(self, stdscr):
        curses.cbreak()
        self._screen = stdscr
        self._screen.nodelay(True)

    def __publish_bindings_to_driver_window(self):
        win_conf_pub = rospy.Publisher("teleop_input_config_msg", String, queue_size=1, latch=True)
        win_conf_pub.publish(self.BINDING_TEXT)

    def resolve_key_input(self):
        """ Read currently pressed key and publish it to the driver.
            Should be called with at least 10 HZ!
        """
        key_code = self._read_keyboard_input()

        if key_code in CursesKeyInput.MOVE_BINDINGS:
            self.__driver.set_twist_command(CursesKeyInput.MOVE_BINDINGS[key_code])
        elif key_code in CursesKeyInput.SETTING_BINDINGS:
            new_settings = SetTeleopSettingsRequest(pressed_commands=[CursesKeyInput.SETTING_BINDINGS[key_code]])
            self.__driver.set_teleop_settings(new_settings)

    def _read_keyboard_input(self):
        key_code = self._screen.getch()
        self._clear_input_queue()
        return key_code

    def _clear_input_queue(self):
        while self._screen.getch() != -1:
            pass


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

    def __init__(self, stdscr, *args, **kwargs):
        super(CursesKeyInput, self).__init__(*args, **kwargs)
        self.__init_curses(stdscr)

    @staticmethod
    def _get_key_symbol(k):
        try:
            return ord(k)
        except TypeError:
            return getattr(curses, k)

    def __init_curses(self, stdscr):
        curses.cbreak()
        self._screen = stdscr
        self._screen.nodelay(True)

    def _read_keyboard_input(self):
        key_code = self._screen.getch()
        if key_code in CursesKeyInput.MOVE_BINDINGS:
            self._twist_pub.publish(CursesKeyInput.MOVE_BINDINGS[key_code])
        elif key_code in CursesKeyInput.SETTING_BINDINGS:
            self._setting_srv(SetTeleopSettingsRequest(pressed_commands=[CursesKeyInput.SETTING_BINDINGS[key_code]]))


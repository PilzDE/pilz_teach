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
from pilz_teleoperation import TeleoperationWindow


class TerminalTextWindow(TeleoperationWindow):
    """ A curses window implementation of the teleoperation window. """
    def __init__(self, stdscr, *args, **kwargs):
        """
        :param stdscr: curses window (optained by using curses.wrapper)
        """
        super(TerminalTextWindow, self).__init__(*args, **kwargs)
        self._screen = stdscr
        curses.curs_set(0)

    def _redraw(self):
        self._start_page()
        self._write_line(1, "PILZ teleoperation driver")
        self._write_line(4, "Settings:")
        self._write_line(6, "linear velocity: %.2f m/s \t angular velocity: %.2f rad/s"
                         % (self._infos.linear_velocity, self._infos.angular_velocity))
        self._write_line(7, "moves_on_plane:  %s" % self._infos.movement_projection_plane)
        self._write_line(8, "current_joint:   %s" % self._infos.joint)
        self._write_line(9, "target frame:    %s" % self._infos.frame)
        self._write_line(10, "_______________________________________________________")
        self._write_line(12, "Controller (keyboard):")
        self._write_line(13, self._input_configuration_text)
        self._end_page()

    def _start_page(self):
        self._screen.clear()

    def _write_line(self, line_number, message):
        height, width = self._screen.getmaxyx()
        y = (height / 25) * line_number
        x = 10
        for i, text in enumerate(message.split('\n')):
            text = text.ljust(width)
            self._screen.addstr(y + i, x, text)

    def _end_page(self):
        self._screen.refresh()

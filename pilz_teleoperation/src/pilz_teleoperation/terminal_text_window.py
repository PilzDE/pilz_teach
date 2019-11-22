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
import rospy
import curses

from std_msgs.msg import String


class TeleoperationWindow(object):
    """ Abstract class for the teleoperation driver window
        Classes of this type should be able to display the current driver settings.
        Additionally this Class provides an api to display the input device configuration / bindings

        TOPICS:
            - teleop_input_config_msg: A message stating how the input device can be used to jog or change settings
    """

    def __init__(self, *args, **kwargs):
        super(TeleoperationWindow, self).__init__(*args, **kwargs)
        self._infos = {
            "input_configuration": "",
            "lin_vel": 0,
            "ang_vel": 0,
            "target_frame": "",
            "plane": ""
        }
        self._input_conf_subscriber = rospy.Subscriber("teleop_input_config_msg",
                                                       String,
                                                       self._telop_input_config_msg,
                                                       queue_size=1)

    def _telop_input_config_msg(self, msg):
        self._infos["input_configuration"] = msg.data
        self._redraw()

    def driver_settings_changed(self, lin_vel, ang_vel, target_frame, plane):
        """ Method to add new setting state.
            Automatically updates the window.
        :param lin_vel:
        :param ang_vel:
        :param target_frame:
        :param plane:
        :return:
        """
        self._infos["lin_vel"] = lin_vel
        self._infos["ang_vel"] = ang_vel
        self._infos["target_frame"] = target_frame
        self._infos["plane"] = plane
        self._redraw()

    @abc.abstractmethod
    def _redraw(self):
        pass


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
        self._write_line(3, "Settings:")
        self._write_line(4, "  - linear velocity: %.2f%% \t\t angular velocity: %.2f%%"
                         % (self._infos["lin_vel"], self._infos["ang_vel"]))
        self._write_line(5, "  - target frame:    %s moves on plane:   %s"
                         % (self._infos["target_frame"].ljust(17), self._infos["plane"]))
        self._write_line(7, "input configuration:")
        self._write_line(8, self._infos["input_configuration"])
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

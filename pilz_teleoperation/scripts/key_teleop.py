#! /usr/bin/env python

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
import curses
import pilz_teleoperation


def main(stdscr):
    """ Callback for curses
    :param stdscr: terminal screen
    """
    key = pilz_teleoperation.CursesKeyInput(stdscr)
    win = pilz_teleoperation.TerminalTextWindow(stdscr)
    driver = pilz_teleoperation.TeleoperationDriver(win)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        key.resolve_key_input()
        driver.send_updated_twist()
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('key_teleop')
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass


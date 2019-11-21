#! /usr/bin/env python
import rospy
import curses
import pilz_teleoperation


def main(stdscr):
    rospy.init_node('pilz_teleop_driver')
    win = pilz_teleoperation.TerminalTextWindow(stdscr)
    pilz_teleoperation.TeleoperationDriver(win).update_loop()


if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass

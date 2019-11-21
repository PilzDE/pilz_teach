#! /usr/bin/env python
import rospy
import curses
import pilz_teleoperation


def main(stdscr):
    rospy.init_node('key_teleop')
    key = pilz_teleoperation.CursesKeyInput(stdscr)
    win = pilz_teleoperation.TerminalTextWindow(stdscr)
    driver = pilz_teleoperation.TeleoperationDriver(win)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        key.resolve_key_input()
        driver.send_updated_twist()
        rate.sleep()


if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass


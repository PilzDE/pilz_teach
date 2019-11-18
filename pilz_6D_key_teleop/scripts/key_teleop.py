#! /usr/bin/env python
import rospy
import curses
import pilz_6D_key_teleop
from std_msgs.msg import String


#def main(stdscr):
#    rospy.init_node('key_teleop')
#    win = pilz_6D_key_teleop.TerminalTextWindow(stdscr)


if __name__ == '__main__':
    try:
        # curses.wrapper(main)
        rospy.init_node('key_teleop')
        p = rospy.Publisher('jo', String, queue_size=1)
        rospy.sleep(1)
        p.publish("What")
    except rospy.ROSInterruptException:
        pass


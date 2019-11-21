import curses
import rospy
from std_msgs.msg import String


class TerminalTextWindow(object):
    def __init__(self, stdscr):
        self._screen = stdscr
        self._infos = {
            "input_configuration": "",
            "lin_vel": 0,
            "ang_vel": 0,
            "target_frame": "",
            "plane": ""
        }

        curses.curs_set(0)
        self._input_conf_subscriber = rospy.Subscriber("teleop_input_config_msg",
                                                       String,
                                                       self._telop_input_config_msg,
                                                       queue_size=1)

    def _telop_input_config_msg(self, msg):
        self._infos["input_configuration"] = msg.data
        self._redraw()

    def driver_settings_changed(self, lin_vel, ang_vel, target_frame, plane):
        self._infos["lin_vel"] = lin_vel
        self._infos["ang_vel"] = ang_vel
        self._infos["target_frame"] = target_frame
        self._infos["plane"] = plane
        self._redraw()

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

    def _beep(self):
        curses.flash()

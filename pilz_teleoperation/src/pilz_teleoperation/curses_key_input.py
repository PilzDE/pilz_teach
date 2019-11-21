import curses
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String
from pilz_teleoperation.srv import SetTeleopSettingsRequest, SetTeleopSettings


class CursesKeyInput(object):
    move_bindings = {
        curses.KEY_HOME: Vector3(-1, 1, 0),
        curses.KEY_UP: Vector3(0, 1, 0),
        curses.KEY_PPAGE: Vector3(1, 1, 0),
        curses.KEY_RIGHT: Vector3(1, 0, 0),
        curses.KEY_NPAGE: Vector3(1, -1, 0),
        curses.KEY_DOWN: Vector3(0, -1, 0),
        curses.KEY_END: Vector3(-1, -1, 0),
        curses.KEY_LEFT: Vector3(-1, 0, 0),
        ord('7'): Vector3(-1, 1, 0),
        ord('8'): Vector3(0, 1, 0),
        ord('9'): Vector3(1, 1, 0),
        ord('6'): Vector3(1, 0, 0),
        ord('3'): Vector3(1, -1, 0),
        ord('2'): Vector3(0, -1, 0),
        ord('1'): Vector3(-1, -1, 0),
        ord('4'): Vector3(-1, 0, 0)
    }
    setting_bindings = {
        ord('+'): SetTeleopSettingsRequest.INCREASE_LINEAR_VELOCITY,
        ord('-'): SetTeleopSettingsRequest.DECREASE_LINEAR_VELOCITY,
        ord('*'): SetTeleopSettingsRequest.TOGGLE_PLANE,
        ord('/'): SetTeleopSettingsRequest.TOGGLE_WORLD_AND_TCP_FRAME,
        ord(','): SetTeleopSettingsRequest.TOGGLE_CONTROLLER
    }

    def __init__(self, stdscr):
        super(CursesKeyInput, self).__init__()
        curses.cbreak()
        self._screen = stdscr
        self._screen.nodelay(True)
        self._twist_pub = rospy.Publisher("/teleop_twist", Twist, queue_size=5)
        self._setting_srv = rospy.ServiceProxy("/set_teleop_settings", SetTeleopSettings)
        win_conf_pub = rospy.Publisher("teleop_input_config_msg", String, queue_size=1, latch=True)
        rospy.sleep(.5)
        win_conf_pub.publish("   - Keybard Configuration:\n"
                             "     Use numpad numbers to move!\n"
                             "     + = increase velocity\n"
                             "     - = decrease velocity\n"
                             "     * = Toggle Plane to move on\n"
                             "     / = Toggle between 'world' and 'tcp' frame\n"
                             "     , = Toggle between controllers")

    def read_key_loop(self):
        while not rospy.is_shutdown():
            try:
                self.resolve_key_input()
            except KeyboardInterrupt:
                return

    def resolve_key_input(self):
        key_code = self._screen.getch()
        if key_code in CursesKeyInput.move_bindings:
            self._twist_pub.publish(Twist(linear=CursesKeyInput.move_bindings[key_code]))
        elif key_code in CursesKeyInput.setting_bindings:
            self._setting_srv(SetTeleopSettingsRequest(pressed_commands=[CursesKeyInput.setting_bindings[key_code]]))

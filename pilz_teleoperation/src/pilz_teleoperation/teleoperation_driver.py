import rospy
import pilz_teleoperation.teleoperation_settings as _teleop_settings

from copy import deepcopy
from geometry_msgs.msg import Twist, TwistStamped
from pilz_teleoperation.srv import SetTeleopSettings, SetTeleopSettingsResponse, SetTeleopSettingsRequest


class TeleoperationDriver(object):
    """ Main class of the teleoperation driver.

        This driver can be used to mount any input device to the jog server for jogging purpose.
        The input node has to provide at least a 2D twist for a movement on a 2D plane.

        TOPICS:
            - /teleop_twist: incoming move commands
            - /jog_server/delta_jog_cmds: outgoing jog commands

        SERVICES:
            - /set_teleop_settings: teleoperation settings like velocity scaling, target frame, ...
    """
    KEY_INPUT_TIMEOUT = 0.1

    def __init__(self, window):
        """
        :param window: To display current settings
        :type window: TeleoperationWindow
        """
        super(TeleoperationDriver, self).__init__()
        self._output_window = window
        self._settings = _teleop_settings.TeleoperationSettings()
        self.__last_twist_msg = TwistStamped()
        self.__ros_init()
        self._update_settings_display()

    def __ros_init(self):
        self._hz = rospy.Rate(20)
        self._sv_settings = rospy.Service("set_teleop_settings", SetTeleopSettings, self._set_teleop_settings)
        self._sub_twist = rospy.Subscriber("teleop_twist", Twist, self._twist_command_cb, queue_size=1)
        self._twist_publisher = rospy.Publisher("/jog_server/delta_jog_cmds", TwistStamped, queue_size=1)

    def _set_teleop_settings(self, req):
        for command in req.pressed_commands:
            try:
                success = self._settings.setting_change_method_bindings[command](self._settings)
                if success is True:
                    self._update_settings_display()
                    return SetTeleopSettingsResponse(success=success)
                else:
                    return SetTeleopSettingsResponse(success=False, error_msg=success)
            except KeyError:
                return SetTeleopSettingsResponse(success=False, error_msg="Unsupported Command")

    def _update_settings_display(self):
        self._output_window.driver_settings_changed(self._settings.linear_velocity,
                                                    self._settings.angular_velocity,
                                                    self._settings.frame,
                                                    self._settings.get_current_plane_string())

    def _twist_command_cb(self, twist_):
        self.__last_twist_msg = self.__get_stamped_twist(twist_)

    def __get_stamped_twist(self, twist_=None):
        st = TwistStamped(twist=twist_)
        st.header.frame_id = self._settings.frame
        st.header.stamp = rospy.Time.now()
        return st

    def update_loop(self):
        while not rospy.is_shutdown():
            self.send_updated_twist()
            self._hz.sleep()

    def send_updated_twist(self):
        """ Publishes the current twist to the jog arm driver.
            Has to be called at least with 10 HZ!
        """
        self._send_updated_twist()

    def _send_updated_twist(self):
        ts = self.__get_stamped_twist()
        if self.__key_input_is_new_enough():
            new_twist = deepcopy(self.__last_twist_msg.twist)
            self.__project_twist_on_plane(new_twist.linear)
            self.__norm_twist(new_twist.linear)
            self.__scale_linear_speed_of_twist(new_twist.linear)
            ts.twist = new_twist
        self._twist_publisher.publish(ts)

    def __key_input_is_new_enough(self):
        return rospy.Time.now() - self.__last_twist_msg.header.stamp \
               < rospy.Duration.from_sec(TeleoperationDriver.KEY_INPUT_TIMEOUT)

    def __project_twist_on_plane(self, twist_lin):
        if self._settings.movement_projection_plane == SetTeleopSettingsRequest.USE_XZ_PLANE:
            twist_lin.y, twist_lin.z = 0, twist_lin.y
        elif self._settings.movement_projection_plane == SetTeleopSettingsRequest.USE_YZ_PLANE:
            twist_lin.x, twist_lin.y, twist_lin.z = 0, twist_lin.x, twist_lin.y

    @staticmethod
    def __norm_twist(twist_lin):
        t_sum = abs(twist_lin.x) + abs(twist_lin.y) + abs(twist_lin.z)
        if t_sum > 0:
            twist_lin.x = twist_lin.x / t_sum
            twist_lin.y = twist_lin.y / t_sum
            twist_lin.z = twist_lin.z / t_sum

    def __scale_linear_speed_of_twist(self, twist_lin):
        twist_lin.x = twist_lin.x * _teleop_settings.MAX_JOG_RANGE * self._settings.linear_velocity
        twist_lin.y = twist_lin.y * _teleop_settings.MAX_JOG_RANGE * self._settings.linear_velocity
        twist_lin.z = twist_lin.z * _teleop_settings.MAX_JOG_RANGE * self._settings.linear_velocity

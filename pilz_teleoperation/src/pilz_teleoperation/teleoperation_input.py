import abc
import rospkg
import yaml

import rospy
from geometry_msgs.msg import Twist
from control_msgs.msg import JointJog
from pilz_teleoperation.srv import SetTeleopSettings, SetTeleopSettingsRequest
from rospy_message_converter import message_converter
from std_msgs.msg import String


class TeleoperationInput(object):
    """ Base Class for input devices """

    MOVE_BINDINGS = {}
    JOINT_BINDINGS = {}
    SETTING_BINDINGS = {}
    INPUT_DESCRIPTION = ""

    def __init__(self, driver=None, *args, **kwargs):
        super(TeleoperationInput, self).__init__(*args, **kwargs)
        self.__driver = driver
        self.__init_driver_connection()
        key_config_path = rospy.get_param("~bindings", self.__get_default_config_path())
        self._try_to_load_config(key_config_path)
        self._publish_bindings_to_driver_window()

    def __init_driver_connection(self):
        self.__twist_publisher = rospy.Publisher("%s/twist" % rospy.get_name(), Twist, queue_size=1)
        self.__joint_publisher = rospy.Publisher("%s/joint_jog" % rospy.get_name(), JointJog, queue_size=1)
        self.__setting_service = rospy.ServiceProxy("%s/set_settings" % rospy.get_name(), SetTeleopSettings)

    @staticmethod
    def __get_default_config_path():
        return rospkg.RosPack().get_path("pilz_teleoperation") + "/config/keyboard_binding.yaml"

    def _try_to_load_config(self, config_path):
        try:
            with open(config_path, "r") as file_:
                bindings = yaml.load(file_.read())
                for k, v in bindings["MovementBindings"].items():
                    self.MOVE_BINDINGS[self._get_real_key(k)] = \
                        message_converter.convert_dictionary_to_ros_message('geometry_msgs/Twist', v)
                for k, v in bindings["JointJogBindings"].items():
                    self.JOINT_BINDINGS[self._get_key_symbol(k)] = \
                        message_converter.convert_dictionary_to_ros_message('control_msgs/JointJog', v)
                for k, v in bindings["SettingBindings"].items():
                    self.SETTING_BINDINGS[self._get_real_key(k)] = \
                        getattr(SetTeleopSettingsRequest, v)
                self.INPUT_DESCRIPTION = bindings["Description"]
        except (KeyError, AttributeError):
            rospy.logerr("Unable to parse key binding config")
            raise KeyError("Error in binding-config syntax")

    @staticmethod
    def _get_real_key(k):
        return k

    def _publish_bindings_to_driver_window(self):
        win_conf_pub = rospy.Publisher("/teleoperation/display_input_config", String, queue_size=1, latch=True)
        win_conf_pub.publish(self.INPUT_DESCRIPTION)

    def resolve_key_input(self):
        """ Read currently pressed key and publish it to the driver.
            Should be called with at least 10 HZ!
        """
        key_code = self._read_keyboard_input()
        if key_code in self.MOVE_BINDINGS:
            self.__publish_twist_command(self.MOVE_BINDINGS[key_code])
        elif key_code in self.JOINT_BINDINGS:
            self.__publish_joint_jog_command(self.JOINT_BINDINGS[key_code])
        elif key_code in self.SETTING_BINDINGS:
            self.__change_settings(SetTeleopSettingsRequest(pressed_commands=[self.SETTING_BINDINGS[key_code]]))

    def __publish_twist_command(self, command):
        self.send_command("set_twist_command", self.__twist_publisher.publish, command)

    def __publish_joint_jog_command(self, command):
        self.send_command("set_joint_jog_command", self.__joint_publisher.publish, command)

    def __change_settings(self, command):
        self.send_command("set_teleop_settings", self.__setting_service, command)

    def send_command(self, direct, fallback, command):
        if self.__driver is not None:
            getattr(self.__driver, direct)(command)
        else:
            fallback(command)

    @abc.abstractmethod
    def _read_keyboard_input(self):
        return

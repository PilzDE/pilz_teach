import abc

import rospy
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
            "plane": "",
            "joint": ""
        }
        self._input_conf_subscriber = rospy.Subscriber("/teleoperation/display_input_config",
                                                       String,
                                                       self._telop_input_config_msg,
                                                       queue_size=1)

    def _telop_input_config_msg(self, msg):
        self._infos["input_configuration"] = msg.data
        self._redraw()

    def driver_settings_changed(self, lin_vel, ang_vel, target_frame, plane, joint):
        """ Method to add new setting state.
            Automatically updates the window.
        :param lin_vel:
        :param ang_vel:
        :param target_frame:
        :param plane:
        :param joint:
        :return:
        """
        self._infos["lin_vel"] = lin_vel
        self._infos["ang_vel"] = ang_vel
        self._infos["target_frame"] = target_frame
        self._infos["plane"] = plane
        self._infos["joint"] = joint
        self._redraw()

    @abc.abstractmethod
    def _redraw(self):
        pass

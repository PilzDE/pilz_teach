import abc

import rospy
from std_msgs.msg import String
from pilz_teleoperation.teleoperation_settings import TeleoperationSettings


class TeleoperationWindow(object):
    """ Abstract class for the teleoperation driver window
        Classes of this type should be able to display the current driver settings.
        Additionally this Class provides an api to display the input device configuration / bindings

        TOPICS:
            - teleop_input_config_msg: A message stating how the input device can be used to jog or change settings
    """

    def __init__(self, *args, **kwargs):
        super(TeleoperationWindow, self).__init__(*args, **kwargs)
        self._infos = TeleoperationSettings()
        self._input_configuration_text = ""
        self._input_conf_subscriber = rospy.Subscriber("/teleoperation/display_input_config",
                                                       String,
                                                       self._telop_input_config_msg,
                                                       queue_size=1)

    def _telop_input_config_msg(self, msg):
        self._input_configuration_text = msg.data
        self._redraw()

    def driver_settings_changed(self, current_settings):
        """ Method to add new setting state.
            Automatically updates the window.
        :param current_settings:
        :return:
        """
        self._infos = current_settings
        self._redraw()

    @abc.abstractmethod
    def _redraw(self):
        pass

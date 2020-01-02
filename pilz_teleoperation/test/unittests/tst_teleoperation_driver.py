#!/usr/bin/env python

import unittest
import mock
from pilz_teleoperation import TeleoperationDriver
from pilz_teleoperation.srv import SetTeleopSettingsRequest

import pytest


class TestTeleoperationDriver(unittest.TestCase):
    @mock.patch('rospy.rostime.get_rostime')
    def test_window_update_after_config_change(self, mocked_rostime):
        mocked_rostime.return_value = 0
        win_mock = mock.MagicMock()
        driver = TeleoperationDriver(window=win_mock)
        self.assertEquals(win_mock.driver_settings_changed.call_count, 1)
        driver.set_teleop_settings(SetTeleopSettingsRequest(
            pressed_commands=[SetTeleopSettingsRequest.TOGGLE_JOINT_UP]))
        self.assertEquals(win_mock.driver_settings_changed.call_count, 2)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('pilz_teleoperation', 'test_teleoperation_driver', TestTeleoperationDriver)

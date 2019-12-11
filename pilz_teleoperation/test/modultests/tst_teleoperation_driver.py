#!/usr/bin/env python

import unittest
import mock
from pilz_teleoperation import TeleoperationDriver, TeleoperationWindow
from pilz_teleoperation.srv import SetTeleopeSettingsRequest


class TestTeleoperationDriver(unittest.TestCase):
    def test_window_update_after_config_change(self):
        win_mock = mock.MagicMock()
        driver = TeleoperationDriver(window=win_mock)
        self.assertEquals(win_mock.driver_settings_changed.call_count, 1)
        driver.set_teleop_settings([SetTeleopeSettingsRequest.TOGGLE_VELOCITY_UP])
        self.assertEquals(win_mock.driver_settings_changed.call_count, 2)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('pilz_teleoperation', 'test_teleoperation_driver', TestTeleoperationDriver)

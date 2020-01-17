#!/usr/bin/env python

# Copyright (c) 2020 Pilz GmbH & Co. KG
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import rospy
import mock
from pilz_teleoperation import TeleoperationDriver
from pilz_teleoperation.srv import SetTeleopSettingsRequest
from param_server_mock import mocked_get_param


def test_window_update_after_config_change(monkeypatch):
    monkeypatch.setattr(rospy.rostime, "get_rostime", lambda: rospy.Time())
    monkeypatch.setattr(rospy, "get_param", mocked_get_param)
    win_mock = mock.MagicMock()
    driver = TeleoperationDriver(window=win_mock)
    assert win_mock.driver_settings_changed.call_count == 1
    driver.set_teleop_settings(SetTeleopSettingsRequest(
        pressed_commands=[SetTeleopSettingsRequest.TOGGLE_JOINT_UP]))
    assert win_mock.driver_settings_changed.call_count == 2

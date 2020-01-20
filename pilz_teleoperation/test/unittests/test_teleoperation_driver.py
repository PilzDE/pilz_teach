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
import pytest
from pilz_teleoperation import TeleoperationDriver
from pilz_teleoperation.teleoperation_driver import _TeleoperationTwist, _TeleoperationJointJog
from pilz_teleoperation.srv import SetTeleopSettingsRequest, SetTeleopSettingsResponse
from param_server_mock import mocked_get_param
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from std_msgs.msg import Header
from control_msgs.msg import JointJog


@pytest.fixture
def patch_ros(monkeypatch):
    monkeypatch.setattr(rospy.rostime, "get_rostime", lambda: rospy.Time(secs=5))
    monkeypatch.setattr(rospy, "get_param", mocked_get_param)
    monkeypatch.setattr(rospy, "Service", mock.MagicMock())


def test_window_update_after_config_change(patch_ros):
    win_mock = mock.MagicMock()
    driver = TeleoperationDriver(window=win_mock)
    assert win_mock.driver_settings_changed.call_count == 1
    driver.set_teleop_settings(SetTeleopSettingsRequest(
        pressed_commands=[SetTeleopSettingsRequest.TOGGLE_JOINT_UP]))
    assert win_mock.driver_settings_changed.call_count == 2


def test_invalid_setting(patch_ros):
    driver = TeleoperationDriver(window=mock.MagicMock())
    response = driver.set_teleop_settings(SetTeleopSettingsRequest(pressed_commands=["invalid"]))
    assert response == SetTeleopSettingsResponse(success=False, error_msg="Unsupported Command")


@pytest.mark.parametrize("method, argument, expected",
                         [("set_twist_command",
                           Twist(linear=Vector3(y=1), angular=Vector3(z=1)),
                           TwistStamped(header=Header(stamp=rospy.Time(secs=5), frame_id="world"),
                                        twist=_TeleoperationTwist(linear=Vector3(y=0.2), angular=Vector3(z=0.2)))),
                          ("set_twist_command",
                           Twist(linear=Vector3(y="max"), angular=Vector3(z="-max")),
                           TwistStamped(header=Header(stamp=rospy.Time(secs=5), frame_id="world"),
                                        twist=_TeleoperationTwist(linear=Vector3(y=1.0), angular=Vector3(z=-1.0)))),
                          ("set_joint_jog_command",
                           JointJog(joint_names=("joint1", "joint2"), velocities=(1, 1)),
                           _TeleoperationJointJog(header=Header(stamp=rospy.Time(secs=5), frame_id="world"),
                                                  joint_names=["joint1", "joint2"], velocities=[.2, .2])),
                          ("set_joint_jog_command",
                           JointJog(velocities=[-1]),
                           _TeleoperationJointJog(header=Header(stamp=rospy.Time(secs=5), frame_id="world"),
                                                  joint_names=["prbt_joint_1"], velocities=[-.2]))
                          ])
def test_twist_modification(method, argument, expected, patch_ros, monkeypatch):
    driver = TeleoperationDriver(window=mock.MagicMock())
    jog_server_mock = mock.Mock()
    monkeypatch.setattr(rospy.Publisher, "publish", jog_server_mock)

    getattr(driver, method)(argument)
    jog_server_mock.assert_called_with(expected)

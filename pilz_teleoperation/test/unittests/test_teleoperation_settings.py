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

import yaml
import os
import rospy
import pytest
from pilz_teleoperation.teleoperation_settings import TeleoperationSettings
from pilz_teleoperation.srv import SetTeleopSettingsRequest

PKG = 'pilz_teleoperation'

_package_dir = "/".join(os.path.dirname(os.path.realpath(__file__)).split("/")[:-2])
with open(_package_dir+ "/config/teleoperation_settings.yaml") as f:
    setting_defaults = yaml.load(f.read())


def mocked_get_param(key, *args, **kwargs):
    return setting_defaults[key[1:]]


velocity_changes = [("linear_velocity", .2, SetTeleopSettingsRequest.INCREASE_LINEAR_VELOCITY, .21),
                    ("linear_velocity", 1, SetTeleopSettingsRequest.INCREASE_LINEAR_VELOCITY, 1),
                    ("linear_velocity", .2, SetTeleopSettingsRequest.DECREASE_LINEAR_VELOCITY, .19),
                    ("linear_velocity", .1, SetTeleopSettingsRequest.DECREASE_LINEAR_VELOCITY, .1),
                    ("angular_velocity", .2, SetTeleopSettingsRequest.INCREASE_ANGULAR_VELOCITY, .21),
                    ("angular_velocity", 1, SetTeleopSettingsRequest.INCREASE_ANGULAR_VELOCITY, 1),
                    ("angular_velocity", .2, SetTeleopSettingsRequest.DECREASE_ANGULAR_VELOCITY, .19),
                    ("angular_velocity", .1, SetTeleopSettingsRequest.DECREASE_ANGULAR_VELOCITY, .1),
                    ("angular_velocity", 0, SetTeleopSettingsRequest.DECREASE_ANGULAR_VELOCITY, .1),
                    ("angular_velocity", 0, SetTeleopSettingsRequest.INCREASE_ANGULAR_VELOCITY, .1),
                    ("angular_velocity", 2, SetTeleopSettingsRequest.INCREASE_ANGULAR_VELOCITY, 1),
                    ("angular_velocity", 2, SetTeleopSettingsRequest.DECREASE_ANGULAR_VELOCITY, 1)]

toggles_up = [("_target_frame_index", 0, SetTeleopSettingsRequest.TOGGLE_TARGET_FRAME_UP, 1),
              ("_target_frame_index", 1, SetTeleopSettingsRequest.TOGGLE_TARGET_FRAME_UP, 0),
              ("_plane_index", 0, SetTeleopSettingsRequest.TOGGLE_PLANE_UP, 1),
              ("_plane_index", 2, SetTeleopSettingsRequest.TOGGLE_PLANE_UP, 0),
              ("_axis_index", 0, SetTeleopSettingsRequest.TOGGLE_ROTATION_AXIS_UP, 1),
              ("_axis_index", 2, SetTeleopSettingsRequest.TOGGLE_ROTATION_AXIS_UP, 0),
              ("_joint_index", 0, SetTeleopSettingsRequest.TOGGLE_JOINT_UP, 1),
              ("_joint_index", 5, SetTeleopSettingsRequest.TOGGLE_JOINT_UP, 0)]

toggles_down = [("_target_frame_index", 0, SetTeleopSettingsRequest.TOGGLE_TARGET_FRAME_DOWN, 1),
                ("_target_frame_index", 1, SetTeleopSettingsRequest.TOGGLE_TARGET_FRAME_DOWN, 0),
                ("_plane_index", 0, SetTeleopSettingsRequest.TOGGLE_PLANE_DOWN, 2),
                ("_plane_index", 2, SetTeleopSettingsRequest.TOGGLE_PLANE_DOWN, 1),
                ("_axis_index", 0, SetTeleopSettingsRequest.TOGGLE_ROTATION_AXIS_DOWN, 2),
                ("_axis_index", 2, SetTeleopSettingsRequest.TOGGLE_ROTATION_AXIS_DOWN, 1),
                ("_joint_index", 0, SetTeleopSettingsRequest.TOGGLE_JOINT_DOWN, 5),
                ("_joint_index", 5, SetTeleopSettingsRequest.TOGGLE_JOINT_DOWN, 4)]


@pytest.mark.parametrize("setting_key, start, operation, expected",
                         velocity_changes + toggles_up + toggles_down)
def test_change_setting(setting_key, start, operation, expected, monkeypatch):
    monkeypatch.setattr(rospy, "get_param", mocked_get_param)
    teleop_settings = TeleoperationSettings()

    # set test default value
    setattr(teleop_settings, setting_key, start)
    # invoke operation
    teleop_settings.setting_change_method_bindings[operation]()
    # assert change
    assert getattr(teleop_settings, setting_key) == pytest.approx(expected)

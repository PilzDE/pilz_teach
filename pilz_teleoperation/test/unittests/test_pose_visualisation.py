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

import os
import rospy
import pytest
from pilz_teleoperation import PoseBroadcaster


PKG = 'pilz_teleoperation'


def test_visualisation(monkeypatch):
    """
    serializes a ros msg, read back and compare with original message
    """
    _test_data_dir = os.path.dirname(os.path.realpath(__file__)) + "/test_data"
    monkeypatch.syspath_prepend(_test_data_dir)
    pose_list_file = __import__("pose_file")
    PoseBroadcaster().publish_poses_from_file(pose_list_file)

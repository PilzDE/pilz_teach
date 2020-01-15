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
import tf2_ros
import rospkg
from pilz_teleoperation import PoseBroadcaster
from mock import Mock, call
from geometry_msgs.msg import TransformStamped, Transform, Pose, Point, Quaternion
from std_msgs.msg import Header


PKG = 'pilz_teleoperation'


def test_visualisation(monkeypatch):
    """
    serializes a ros msg, read back and compare with original message
    """
    monkeypatch.setattr(rospy.Time, "now", Mock(return_value=rospy.Time(100, 101)))

    publish_mock = Mock()
    monkeypatch.setattr(tf2_ros.TransformBroadcaster, "sendTransform", publish_mock)

    _test_data_dir = rospkg.RosPack().get_path("pilz_teleoperation") + "/test/unittests/test_data"
    monkeypatch.syspath_prepend(_test_data_dir)
    pose_list_file = __import__("pose_file")
    PoseBroadcaster().publish_poses_from_file(pose_list_file)

    publish_mock.assert_has_calls((call(TransformStamped(header=Header(stamp=rospy.Time(100, 101), frame_id="tcp"),
                                                         child_frame_id="goal_pose",
                                                         transform=Transform(translation=Point(y=.8),
                                                                             rotation=Quaternion(y=1.0)))),
                                   call(TransformStamped(header=Header(stamp=rospy.Time(100, 101), frame_id="world"),
                                                         child_frame_id="start_pose_test",
                                                         transform=Transform(translation=Point(x=7.0),
                                                                             rotation=Quaternion(w=-1.0)))),
                                   ))

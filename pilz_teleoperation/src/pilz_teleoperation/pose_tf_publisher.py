# Copyright (c) 2019 Pilz GmbH & Co. KG
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
from geometry_msgs.msg import TransformStamped


class PoseBroadcaster(object):
    def __init__(self):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.sleep(.5)

    def publish_poses_from_file(self, python_file):
        for k, v in python_file.__dict__.items():
            try:
                t = TransformStamped(child_frame_id=k)
                t.transform.translation = v.pose.position
                t.transform.rotation = v.pose.orientation
                t.header.frame_id = v.header.frame_id
                t.header.stamp = rospy.Time.now()
                self.tf_broadcaster.sendTransform(t)
            except AttributeError:
                pass

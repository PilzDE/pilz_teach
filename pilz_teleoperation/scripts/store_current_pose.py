#!/usr/bin/env python
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

import unittest
from pilz_robot_programming import *
from geometry_msgs.msg import Pose
from pilz_teleoperation import RosMessageSerializer
import rospy

__REQUIRED_API_VERSION__ = '1'


if __name__ == '__main__':
    rospy.init_node('tst_jog_arm_motion')

    # get current robot position
    r = Robot(__REQUIRED_API_VERSION__)
    current_pose = r.get_current_pose()

    serializer = RosMessageSerializer()
    serializer.write_messages_to_file({'pick_pose': current_pose}, 'points.py')

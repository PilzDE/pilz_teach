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
from rospkg import RosPack
import rospy


class TestJogArmMotion(unittest.TestCase):
    """
    Test a jog arm motion.
    """

    def setUp(self):
        rospy.loginfo("SetUp called...")

    def tearDown(self):
        rospy.loginfo("TearDown called...")

    def test_ptp_execution(self):
        """ Test execution of valid ptp command works successfully.

            Test sequence:
                1. Execute a valid ptp command

            Test results:
                1. Move function returns without throwing an exception.
        """
        # Execute robot motion via service
        pass


if __name__ == '__main__':
    import rostest
    rospy.init_node('tst_jog_arm_motion')
    rostest.rosrun('pilz_jog_arm_suppport', 'tst_jog_arm_motion', TestJogArmMotion)

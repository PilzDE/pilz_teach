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
from pilz_robot_programming import *
from geometry_msgs.msg import TwistStamped
import rospy

__REQUIRED_API_VERSION__ = '1'

class TestJogArmMotion(unittest.TestCase):
    """
    Test that the jog arm launch file brings up a simulated robot
    we can move.
    """

    def setUp(self):
        rospy.loginfo("SetUp called...")
        self.r = Robot(__REQUIRED_API_VERSION__)

    def tearDown(self):
        rospy.loginfo("TearDown called...")
        self.r.__del__

    def test_move(self):
        """ Test,  if the robot moves on sending a twist command message"""
        # Wait until jog-arm-server is ready
        # topic: /jog_server/delta_jog_cmds
        publisher = rospy.Publisher('/jog_server/delta_jog_cmds', TwistStamped)
        while (publisher.get_num_connections() < 1) and (not rospy.is_shutdown()):
            rospy.sleep(0.1)
        
        # get current robot position
        start_pose = self.r.get_current_pose()
        
        # send single command
        msg = TwistStamped()
        msg.header.stamp = rospy.Time().now()
        msg.twist.linear.x = 0.1
        msg.twist.linear.y = 0.1
        publisher.publish(msg)
              
        # wait until current robot position changes
        timeout = rospy.Time().now() + rospy.Duration(10.)
        while (start_pose == self.r.get_current_pose()) and (not rospy.is_shutdown()):
            rospy.sleep(0.1)
            self.assertTrue(rospy.Time().now() < timeout, "robot did not move until timeout")
        publisher.unregister()


if __name__ == '__main__':
    import rostest
    rospy.init_node('tst_jog_arm_motion')
    rostest.rosrun('prbt_jog_arm_suppport', 'tst_jog_arm_motion', TestJogArmMotion)

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

import rospy
import unittest

from geometry_msgs.msg import TwistStamped


class TestKeyTeleoperation(unittest.TestCase):
    """
    Test that the key_teleoperation file brings up the teleoperation driver and calls its update method
    """
    def __init__(self, *args, **kwargs):
        super(TestKeyTeleoperation, self).__init__(*args, **kwargs)
        self.twist_published = False

    def setUp(self):
        rospy.loginfo("SetUp called...")

    def tearDown(self):
        rospy.loginfo("TearDown called...")

    def callback(self):
        self.twist_published = True

    def test_startup(self):
        """ Test if driver gets started """
        subscriber = rospy.Subscriber('/jog_server/delta_jog_cmds', TwistStamped, self.callback)

        timeout = rospy.Time().now() + rospy.Duration(secs=10)
        while not rospy.is_shutdown() and not self.twist_published:
            rospy.sleep(0.1)
            self.assertTrue(rospy.Time().now() < timeout, "driver did not publish")
        subscriber.unregister()


if __name__ == '__main__':
    import rostest

    rospy.init_node('tst_key_teleoperation')
    rostest.rosrun('pilz_teleoperation', 'tst_key_teleop', TestKeyTeleoperation)

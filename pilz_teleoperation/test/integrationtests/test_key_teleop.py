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

from geometry_msgs.msg import TwistStamped, Twist, Vector3
from control_msgs.msg import JointJog


class TestKeyTeleoperation(unittest.TestCase):
    """
    Test that the key_teleoperation file brings up the teleoperation driver and calls its update method
    """
    def __init__(self, *args, **kwargs):
        super(TestKeyTeleoperation, self).__init__(*args, **kwargs)
        self.twist_published = False

    def setUp(self):
        rospy.loginfo("SetUp called...")
        self.twist_published = False

    def tearDown(self):
        rospy.loginfo("TearDown called...")

    def callback(self, msg):
        self.twist_published = True

    def test_startup_twist(self):
        """ Test if driver gets started """
        rospy.Subscriber('/jog_server/delta_jog_cmds', TwistStamped, self.callback)
        self._publish_object_to_topic_and_assert(what=Twist(linear=Vector3(y=1)),
                                                 topic='/key_input/twist')

    def _publish_object_to_topic_and_assert(self, what, topic):
        publisher_twist = rospy.Publisher(topic, type(what), queue_size=1)
        self._wait_for_event_with_timeout(lambda: publisher_twist.get_num_connections() < 1,
                                          secs=5,
                                          msg="subscriber did not come up")
        publisher_twist.publish(what)
        self._wait_for_event_with_timeout(lambda: self.twist_published, secs=5, msg="driver did not publish")

    def _wait_for_event_with_timeout(self, event, secs, msg):
        timeout = rospy.Time().now() + rospy.Duration(secs=secs)
        while not rospy.is_shutdown() and event():
            rospy.sleep(0.1)
            self.assertTrue(rospy.Time().now() < timeout, msg)

    def test_startup_jog(self):
        """ Test if driver gets started """
        rospy.Subscriber('/jog_server/joint_delta_jog_cmds', JointJog, self.callback)
        self._publish_object_to_topic_and_assert(what=JointJog(joint_names=["joint1"], velocities=[1]),
                                                 topic='/key_input/joint_jog')


if __name__ == '__main__':
    import rostest

    rospy.init_node('tst_key_teleoperation')
    rostest.rosrun('pilz_teleoperation', 'tst_key_teleop', TestKeyTeleoperation)

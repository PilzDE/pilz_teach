#! /usr/bin/env python

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
from visualization_msgs.msg import Marker

class UsabilityTestMarkerPublisher(object):

    def __init__(self):
        self.pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        rospy.init_node('usability_test_markers')
        self.rate = rospy.Rate(1)

        self.setup_generic_marker()

    def setup_generic_marker(self):
        self.m = Marker()
        self.m.action = Marker.ADD
        self.m.header.frame_id = '/world'
        self.m.ns = 'usability_test_marker'
        self.m.type = Marker.ARROW
        self.m.color.r = 1.0
        self.m.color.g = 1.0
        self.m.color.b = 1.0
        self.m.color.a = 1.0
        self.m.scale.x = 0.15
        self.m.scale.y = 0.01
        self.m.scale.z = 0.01

        self.m.header.stamp = rospy.Time.now()
        self.m.id = 0
        self.m.pose.position.x = 1.0
        self.m.pose.position.y = 1.0
        self.m.pose.position.z = 1.0
        self.m.pose.orientation.w = 1

    def publish_markers(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.m)
            self.rate.sleep()

if __name__ == '__main__':
    publisher = UsabilityTestMarkerPublisher()
    try:
        publisher.publish_markers()
    except rospy.ROSInterruptException:
        pass

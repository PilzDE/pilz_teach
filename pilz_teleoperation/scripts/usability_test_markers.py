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
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion

class UsabilityTestMarkerPublisher(object):

    def __init__(self):
        self.pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        rospy.init_node('usability_test_markers')
        self.rate = rospy.Rate(1)

        self.set_generic_arrow_marker()
        self.set_custom_text_marker()
        self.set_custom_marker_poses()

    def set_generic_arrow_marker(self):
        self.arrow = Marker()
        self.arrow.action = Marker.ADD
        self.arrow.header.frame_id = '/world'
        self.arrow.ns = 'usability_test_arrow'
        self.arrow.type = Marker.ARROW
        self.arrow.color.r = 1.0
        self.arrow.color.g = 1.0
        self.arrow.color.b = 1.0
        self.arrow.color.a = 1.0
        self.arrow.scale.x = .5
        self.arrow.scale.y = 0.01
        self.arrow.scale.z = 0.01

    def set_custom_text_marker(self):
        self.order_marker = Marker()
        self.order_marker.action = Marker.ADD
        self.order_marker.header.frame_id = '/world'
        self.order_marker.ns = 'usability_test_order'
        self.order_marker.type = Marker.TEXT_VIEW_FACING
        self.order_marker.color.r = 1.0
        self.order_marker.color.g = 1.0
        self.order_marker.color.b = 1.0
        self.order_marker.color.a = 1.0
        self.order_marker.scale.z = 0.2

    def set_custom_marker_poses(self):
        self.set_custom_marker_positions()
        self.set_custom_marker_orientations()
        self.poses = list()
        for ii in range(3):
            self.poses.append(Pose(self.positions[ii], self.orientations[ii]))

    def set_custom_marker_positions(self):
        self.positions = list()
        self.positions.append(Point(-.4, -.2, .3))
        self.positions.append(Point(-.4, .4, .2))
        self.positions.append(Point(0, .4, .5))

    def set_custom_marker_orientations(self):
        self.orientations = list()
        self.orientations.append(Quaternion(0, 1, 0, 0))
        self.orientations.append(Quaternion(0, math.sqrt(.5), math.sqrt(.5), 0))
        self.orientations.append(Quaternion(0, 1, 0, 0))

    def get_arrow(self, ii):
        self.arrow.header.stamp = rospy.Time.now()
        self.arrow.id = ii
        self.arrow.pose = self.poses[ii]
        return self.arrow

    def get_order_marker(self, ii):
        self.order_marker.header.stamp = rospy.Time.now()
        self.order_marker.id = ii
        self.order_marker.pose.position = self.positions[ii]
        self.order_marker.text = str(ii + 1)
        return self.order_marker

    def publish_markers(self):
        while not rospy.is_shutdown():
            for ii in range(3):
                self.pub.publish(self.get_arrow(ii))
                self.pub.publish(self.get_order_marker(ii))
            self.rate.sleep()

if __name__ == '__main__':
    publisher = UsabilityTestMarkerPublisher()

    user_input = raw_input("Start test? (y)")

    if user_input == 'y':
        try:
            publisher.publish_markers()
        except rospy.ROSInterruptException:
            pass

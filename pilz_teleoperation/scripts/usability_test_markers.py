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
from geometry_msgs.msg import Pose, Point, Quaternion

class UsabilityTestMarkerPublisher(object):

    def __init__(self):
        self.pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        rospy.init_node('usability_test_markers')
        self.rate = rospy.Rate(1)

        self.setup_generic_marker()
        self.set_custom_marker_poses()

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
        self.m.scale.x = .5
        self.m.scale.y = 0.01
        self.m.scale.z = 0.01

    def set_custom_marker_poses(self):
        self.set_custom_marker_positions()
        self.set_custom_marker_orientations()
        self.poses = list()
        for ii in range(3):
            self.poses.append(Pose(self.points[ii], self.orientations[ii]))

    def set_custom_marker_positions(self):
        self.points = list()
        self.points.append(Point(1, 1, 0))
        self.points.append(Point(0, 1, 1))
        self.points.append(Point(1, 0, 1))

    def set_custom_marker_orientations(self):
        self.orientations = list()
        self.orientations.append(Quaternion(0, 0, 0, 1))
        self.orientations.append(Quaternion(0, 1, 0, 1))
        self.orientations.append(Quaternion(1, 0, 0, 1))

    def get_marker(self, ii):
        self.m.header.stamp = rospy.Time.now()
        self.m.id = ii
        self.m.pose = self.poses[ii]
        return self.m

    def publish_markers(self):
        while not rospy.is_shutdown():
            for ii in range(3):
                self.pub.publish(self.get_marker(ii))
            self.rate.sleep()

if __name__ == '__main__':
    publisher = UsabilityTestMarkerPublisher()
    try:
        publisher.publish_markers()
    except rospy.ROSInterruptException:
        pass

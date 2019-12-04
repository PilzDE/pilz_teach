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

def setup_generic_marker():
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = '/world'
    m.header.stamp = rospy.Time.now()
    m.ns = 'usability_test_marker'
    m.id = 0
    m.type = Marker.SPHERE
    m.pose.position.x = 1.0
    m.pose.position.y = 1.0
    m.pose.position.z = 1.0
    m.pose.orientation.x = 1.0
    m.pose.orientation.w = 1.0
    m.scale.x = 1.0
    m.scale.y = 1.0
    m.scale.z = 1.0
    m.color.r = 0.0
    m.color.g = 1.0
    m.color.b = 0.0
    m.color.a = 1.0
    return m

def publish_markers():
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.init_node('usability_test_markers')
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(setup_generic_marker())
        rate.sleep()


if __name__ == '__main__':
    try:
        publish_markers()
    except rospy.ROSInterruptException:
        pass

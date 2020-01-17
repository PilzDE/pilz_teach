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

import sys
import rospy
from pilz_teleoperation import PoseBroadcaster


def _import_pose_list():
    file_path = rospy.get_param("~file_path")
    file_name_without_extension, path_to_file = _get_path_and_name(file_path)
    sys.path.append(path_to_file)
    return __import__(file_name_without_extension)


def _get_path_and_name(file_path):
    last_slash_index = file_path.rfind("/")
    path_to_file = file_path[:last_slash_index]
    file_name = file_path[last_slash_index + 1:]
    file_name_without_extension = file_name.split(".")[0]
    return file_name_without_extension, path_to_file


if __name__ == '__main__':
    rospy.init_node("pose_visualisation")
    try:
        pose_list = _import_pose_list()
        pb = PoseBroadcaster()
        while not rospy.is_shutdown():
            pb.publish_poses_from_file(pose_list)
            rospy.Rate(1).sleep()
    except KeyError:
        rospy.logerr("path invalid!")
    except rospy.ROSInterruptException:
        pass

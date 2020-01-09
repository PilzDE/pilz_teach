#!/usr/bin/env python
import os
import sys
import rospy
import pytest
from pilz_teleoperation.ros_message_serializer import RosMessageSerializer


PKG = 'pilz_teleoperation'


def single_pose():
    from geometry_msgs.msg import PoseStamped

    start_pose = PoseStamped()
    start_pose.header.stamp = rospy.Time(12345, 6789)
    start_pose.header.frame_id = "world"
    start_pose.pose.position.x = 7
    start_pose.pose.orientation.w = -1.0
    return {"start_pose_test": start_pose}


def multiple_poses():
    result = single_pose()
    from geometry_msgs.msg import PoseStamped

    goal_pose = PoseStamped()
    goal_pose.header.stamp = rospy.Time(123, 89)
    goal_pose.header.frame_id = "tcp"
    goal_pose.pose.position.y = 0.8
    goal_pose.pose.orientation.y = 1.0

    result["goal_pose"] = goal_pose
    return result


@pytest.mark.parametrize("test_input", [{"hallo": "welt"},
                                        {},
                                        {"e": None},
                                        single_pose(),
                                        multiple_poses()])
def test_writeback(test_input, tmpdir):
    """
    serializes a ros msg, read back and compare with original message
    """

    # save Pose with unique name
    module_name = str(tmpdir).split('/')[-1]
    RosMessageSerializer().write_messages_to_file(test_input, str(tmpdir.join(module_name + ".py")))

    # load module
    sys.path.insert(0, str(tmpdir))
    readback = __import__(module_name)
    sys.path.pop(0)

    # compare all variables
    for k in test_input.keys():
        assert test_input[k] == getattr(readback, k), "Could not read back pose"

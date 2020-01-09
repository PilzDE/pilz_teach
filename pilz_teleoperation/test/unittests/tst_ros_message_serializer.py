#!/usr/bin/env python
import unittest
import os
import sys
import rospy
from pilz_teleoperation.ros_message_serializer import RosMessageSerializer


PKG = 'pilz_teleoperation'


# A sample python unit test
class TestMessageSerializer(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestMessageSerializer, self).__init__(*args, **kwargs)

        from geometry_msgs.msg import PoseStamped

        self._start_pose = PoseStamped()
        self._start_pose.header.stamp = rospy.Time(12345, 6789)
        self._start_pose.header.frame_id = "world"
        self._start_pose.pose.position.x = 7
        self._start_pose.pose.orientation.w = -1.0

    def test_writeback(self):
        """
        serializes a ros msg, read back and compare with original message
        """
        # save Pose
        RosMessageSerializer().write_messages_to_file([("start_pose_test", self._start_pose)], "/tmp/test_writeback.py")

        # load Pose
        sys.path.append("/tmp")
        from test_writeback import start_pose_test
        os.remove("/tmp/test_writeback.py")

        # compare Pose
        self.assertEquals(self._start_pose, start_pose_test,
                          "Could not read back pose %s %s" % (self._start_pose, start_pose_test))


if __name__ == '__main__':
    import rosunit

    rosunit.unitrun(PKG, 'test_message_serialization', TestMessageSerializer)

#!/usr/bin/env python
PKG='pilz_teleoperation'
import unittest
import os
import sys
import rospy
from pilz_teleoperation.ros_message_serializer import RosMessageSerializer

## A sample python unit test
class TestMessageSerializer(unittest.TestCase):

    def test_writeback(self):
        """
        serializes a ros msg, read back and compare with original message
        :return:
        """
        from geometry_msgs.msg import PoseStamped

        start_pose = PoseStamped()
        start_pose.header.stamp = rospy.Time.now()
        start_pose.header.frame_id = "world"
        start_pose.pose.position.x = 7
        start_pose.pose.orientation.w=-1.0

        # save Pose
        RosMessageSerializer().write_messages_to_file({"start_pose_test":start_pose}, "/tmp/test_writeback.py")

        # load Pose
        sys.path.append("/tmp")
        from test_writeback import start_pose_test
        #os.remove("/tmp/test_writeback.py")


        # compare Pose
        self.assertEquals(start_pose, start_pose_test, "Could not read back pose %s %s" % (start_pose, start_pose_test))

        def fail_test(self):
            self.assertEquals(1,2, "1 != 2")


if __name__ == '__main__':
    rospy.init_node("test_message_serialization")
    import rosunit
    rosunit.unitrun(PKG, 'test_message_serialization', TestMessageSerializer)

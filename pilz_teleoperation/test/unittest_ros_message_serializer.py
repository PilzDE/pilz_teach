#!/usr/bin/env python
PKG='pilz_teleoperation'
import unittest
import RosMessageSerializer

## A sample python unit test
class TestBareBones(unittest.TestCase):

    def test_writeback(self):
        """
        serializes a ros msg, read back and compare with original message
        :return:
        """
        from Quaternion import geometry_msgs.msg._Quaternion
        from Pose import geometry_msgs.msg._Pose
        from Point import geometry_msgs.msg._Point

        start_pose = Pose(
            position=Point(
                x=7,
                y=0.0,
                z=0.0
            ),
            orientation=Quaternion(
                x=0.0,
                y=0.0,
                z=0.0,
                w=-1.0
            )
        )

        # save Pose
        write_messages_to_file({"start_pose_test":start_pose}, "test_writeback.py")

        # load Pose
        import test_writeback

        # compare Pose
        self.assertEquals(start_pose, start_pose_test, "Could not read back pose")


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_bare_bones', TestBareBones)

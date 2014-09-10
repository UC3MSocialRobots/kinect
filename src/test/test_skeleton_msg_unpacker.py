PKG = 'kinect'

import roslib; roslib.load_manifest(PKG)
# import rospy
import unittest

import kinect.msg as kin
import kinect.nite_skeleton_msg_utils as nsku


class TestPoseDatasetBuilderSkeletonUnpacker(unittest.TestCase):
    """Tests"""
    def __init__(self, *args):
        super(TestPoseDatasetBuilderSkeletonUnpacker, self).__init__(*args)
            
    def setUp(self):
        pass
        
    def tearDown(self):
        pass
        
    def test_unpack_skeleton_joint(self):
        joint = nsku.generate_fake_joint_data('head')
        unpacked = nsku.unpack_skeleton_joint(joint)
        self.assertEqual(joint.joint_id, unpacked.next())
        self.assertEqual(joint.child_frame_id, unpacked.next())
        self.assertEqual(joint.pose3D.position.x, unpacked.next())
        self.assertEqual(joint.pose3D.position.y, unpacked.next())
        self.assertEqual(joint.pose3D.position.z, unpacked.next())
        self.assertEqual(joint.pose3D.orientation.x, unpacked.next())
        self.assertEqual(joint.pose3D.orientation.y, unpacked.next())
        self.assertEqual(joint.pose3D.orientation.z, unpacked.next())
        self.assertEqual(joint.pose3D.orientation.w, unpacked.next())
        self.assertEqual(joint.position_confidence, unpacked.next())
        self.assertEqual(joint.orientation_confidence, unpacked.next())

    # @unittest.skip("Skpping this Test")
    def test_unpack_skeleton_msg_raises_TypeError_if_no_skeleton_passed(self):
        skel = kin.NiteSkeleton()
        self.assertRaises(TypeError, nsku.unpack_skeleton_msg, None)
        self.assertRaises(TypeError, nsku.unpack_skeleton_msg, skel)

    def test_unpack_skeleton_msg_returns_proper_joint_names(self):
        joint_names = ['head', 'neck', 'torso_01', 'left_hand', 'left_foot_1']
        clean_joint_names = ['head', 'neck', 'torso', 'left_hand', 'left_foot']
        skel = nsku.generate_fake_skelmsg(joint_names)
        unpacked_data, unpacked_joint_names = nsku.unpack_skeleton_msg(skel)
        
        self.assertEqual(clean_joint_names, list(unpacked_joint_names))


    def test_unpack_skeleton_msg(self):
        skel = nsku.generate_fake_skelmsg(['head', 'neck'])
        unpacked_data, unpacked_joint_names = nsku.unpack_skeleton_msg(skel)
        # self.assertEqual(['head', 'neck'], list(unpacked_joint_names))
        
        self.assertEqual(skel.user_id, unpacked_data.next())
        self.assertEqual(skel.header.stamp.to_sec(), unpacked_data.next())
        for j in skel.joints:
            self.assertEqual(j.pose3D.position.x, unpacked_data.next())
            self.assertEqual(j.pose3D.position.y, unpacked_data.next())
            self.assertEqual(j.pose3D.position.z, unpacked_data.next())
            self.assertEqual(j.pose3D.orientation.x, unpacked_data.next())
            self.assertEqual(j.pose3D.orientation.y, unpacked_data.next())
            self.assertEqual(j.pose3D.orientation.z, unpacked_data.next())
            self.assertEqual(j.pose3D.orientation.w, unpacked_data.next())
            self.assertEqual(j.position_confidence, unpacked_data.next())
            self.assertEqual(j.orientation_confidence, unpacked_data.next())


if __name__ == '__main__':
    # import rostest
    # rostest.rosrun(PKG, 'test_skeleton_msg_unpacker', TestPoseDatasetBuilderSkeletonUnpacker)
    
    import rosunit
    rosunit.unitrun(PKG, 'test_skeleton_msg_unpacker', TestPoseDatasetBuilderSkeletonUnpacker)

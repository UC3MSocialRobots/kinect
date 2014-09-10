PKG = 'kinect'
import roslib; roslib.load_manifest(PKG)
import copy
import random
from itertools import (imap, izip, chain)

import kinect.msg as kin


def generate_random_data(msg, seed=1):
    ''' Helper function that 
        returns a copy of passed message filled with random data '''
    random.seed(seed)
    data = copy.copy(msg)       
    foo = [data.__setattr__(s, random.random()) for s in data.__slots__]
    return data


def generate_fake_joint_data(joint_name):
    ''' Helper function that 
        returns a NiteSkeletonJoint filled with random data '''
    joint = kin.NiteSkeletonJoint()
    joint.child_frame_id = joint_name
    joint.pose3D.position = generate_random_data(joint.pose3D.position)
    joint.pose3D.orientation = generate_random_data(joint.pose3D.orientation)
    joint.pose2D = generate_random_data(joint.pose2D)
    joint.position_confidence = random.random()
    joint.orientation_confidence = random.random()
    return joint


def generate_fake_skelmsg(joint_names):
    ''' Helper method to generate a NiteSkeleton filled with random data
        Parameters:
        @name joint_names 
        @type joint_names: iterable Is an iterable containing the names
                           of the joints to be generated
    '''
    return kin.NiteSkeleton(joints=map(generate_fake_joint_data, joint_names))


def generate_fake_NiteSkeletonList_msg(n_skels, joint_names):
    ''' returns a kinect.msg.NiteSkeletonList message with num_skels skeletons.
        The skeletons will have joint_names as the joint names
        Parameters:
        @name n_skels num of skeletons in the NiteSkeletonList message
        @type int
        @name joint_names a list of the joint names of each skeleton message
        @type list of strings

        '''
    return kin.NiteSkeletonList(
        skeletons=[generate_fake_skelmsg(joint_names) for _ in xrange(n_skels)])
    

def unpack_skeleton_joint(joint):
    ''' Receives a NiteSkeletonJoint and yields an iterator of each of its fields'''
    yield joint.joint_id
    yield joint.child_frame_id
    yield joint.pose3D.position.x
    yield joint.pose3D.position.y
    yield joint.pose3D.position.z
    yield joint.pose3D.orientation.x
    yield joint.pose3D.orientation.y
    yield joint.pose3D.orientation.z
    yield joint.pose3D.orientation.w
    yield joint.position_confidence
    yield joint.orientation_confidence


def __clean_trailing_num(s):
        return s.rsplit('_',1)[0] if s[-1].isdigit() else s


def __get_unpacked_joint(joint):
    j = unpack_skeleton_joint(joint)
    j.next() # Drop jointID
    j_name = __clean_trailing_num(j.next()) 
    return j, j_name


def unpack_skeleton_msg(skeleton):
    ''' Converts a NiteSkeleton msg to list

        @param skeleton: The skeleton to be converted
        @type: NiteSkeleton
        Returns:
        @return: (skeleton_data, skeleton_joint_names)
        @rtype: tuple(iterator, list)
        @raise: TypeError if passed an empty skeleton
        
        Returns a tuple with a list with joint data and a list with joint names
        First Element contains:
        
        E.g: 

            >>> data, joint_names = list(unpack_skeleton_msg)
            >>> user_id, time_stamp, joint_data = data[0], data[1], data[2:]
    '''
    if not skeleton or not skeleton.joints:
        raise TypeError
    msg_header = [skeleton.user_id, skeleton.header.stamp.to_sec()]
    data, joint_names = izip(*imap(__get_unpacked_joint, skeleton.joints))
    joint_data = chain.from_iterable([msg_header, chain.from_iterable(data)])
    return joint_data, joint_names

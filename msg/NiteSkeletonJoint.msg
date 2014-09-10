Header header

# the ID of the joint (see constants below)
int8 joint_id

# the name of the frame defined by this joint, obtained by concatenating
# the name of the joint and the user ID, for instance "head2" or "left_hand0"
string child_frame_id

# the 3D pose of the joints
geometry_msgs/Pose pose3D

# the x and y coordinates in the image of the joint (in [0..1] )
# the orientation theta is not used
geometry_msgs/Pose2D pose2D

# the NITE confidence in the 3D position of the joint, in [0..1].
float32 position_confidence
# the NITE confidence in the 3D orientation of the joint, in [0..1].
float32 orientation_confidence

### predefined ID for the joints
### comes from XnTypes.h
int8 SKEL_HEAD = 1
int8 SKEL_NECK = 2
int8 SKEL_TORSO = 3
int8 SKEL_WAIST = 4

int8 SKEL_LEFT_COLLAR = 5
int8 SKEL_LEFT_SHOULDER = 6
int8 SKEL_LEFT_ELBOW = 7
int8 SKEL_LEFT_WRIST = 8
int8 SKEL_LEFT_HAND = 9
int8 SKEL_LEFT_FINGERTIP =10

int8 SKEL_RIGHT_COLLAR =11
int8 SKEL_RIGHT_SHOULDER =12
int8 SKEL_RIGHT_ELBOW =13
int8 SKEL_RIGHT_WRIST =14
int8 SKEL_RIGHT_HAND =15
int8 SKEL_RIGHT_FINGERTIP =16

int8 SKEL_LEFT_HIP =17
int8 SKEL_LEFT_KNEE =18
int8 SKEL_LEFT_ANKLE =19
int8 SKEL_LEFT_FOOT =20

int8 SKEL_RIGHT_HIP =21
int8 SKEL_RIGHT_KNEE =22
int8 SKEL_RIGHT_ANKLE =23
int8 SKEL_RIGHT_FOOT =24

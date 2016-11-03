/*!
  \file        NiteSkeletonLite.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/10/12
  
________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

\todo Description of the file

\section Parameters
  - \b "foo"
        [string] (default: "bar")
        Description of the parameter.
        
\section Subscriptions
  - \b "/foo"
        [xxx]
        Descrption of the subscription
        
\section Publications
  - \b "~foo"
        [xxx]
        Descrption of the publication
        
 */

#ifndef NITESKELETONLITE_H
#define NITESKELETONLITE_H

#include <vector>
#include "vision_utils/foo_point.h"

////////////////////////////////////////////////////////////////////////////////

namespace ros {
// typedef double Time;
struct Header {
  std::string frame_id;
  double stamp;
};

//! good old ros::ok()
inline bool ok() { return true; }
} // end namespace ros

////////////////////////////////////////////////////////////////////////////////

namespace geometry_msgs {
struct Pose2D {
  double x, y, theta;
};
struct Pt3d : vision_utils::FooPoint3d {
  inline double getX() const { return x; }
  inline double getY() const { return y; }
  inline double getZ() const { return z; }
};
struct Pt4d : Pt3d {
  double w;
  inline double getW() const { return w; }
};
struct Pose {
  Pt3d position;
  Pt4d orientation;
};
} // end namespace geometry_msgs

////////////////////////////////////////////////////////////////////////////////

namespace tf {
struct Transform {
  geometry_msgs::Pt3d translation;
  geometry_msgs::Pt4d rotation;
  inline geometry_msgs::Pt3d getOrigin()   const { return translation; }
  inline geometry_msgs::Pt4d getRotation() const { return rotation; }
};
} // end namespace geometry_msgs

////////////////////////////////////////////////////////////////////////////////

namespace kinect {

typedef short SkeletonJointName;

class NiteSkeletonJoint {
public:
  static const SkeletonJointName SKEL_HEAD=1;
  static const SkeletonJointName SKEL_NECK=2;
  static const SkeletonJointName SKEL_TORSO=3;
  static const SkeletonJointName SKEL_WAIST=4;
  static const SkeletonJointName SKEL_LEFT_COLLAR=5;
  static const SkeletonJointName SKEL_LEFT_SHOULDER=6;
  static const SkeletonJointName SKEL_LEFT_ELBOW=7;
  static const SkeletonJointName SKEL_LEFT_WRIST=8;
  static const SkeletonJointName SKEL_LEFT_HAND=9;
  static const SkeletonJointName SKEL_LEFT_FINGERTIP=10;
  static const SkeletonJointName SKEL_RIGHT_COLLAR=11;
  static const SkeletonJointName SKEL_RIGHT_SHOULDER=12;
  static const SkeletonJointName SKEL_RIGHT_ELBOW=13;
  static const SkeletonJointName SKEL_RIGHT_WRIST=14;
  static const SkeletonJointName SKEL_RIGHT_HAND=15;
  static const SkeletonJointName SKEL_RIGHT_FINGERTIP=16;
  static const SkeletonJointName SKEL_LEFT_HIP=17;
  static const SkeletonJointName SKEL_LEFT_KNEE=18;
  static const SkeletonJointName SKEL_LEFT_ANKLE=19;
  static const SkeletonJointName SKEL_LEFT_FOOT=20;
  static const SkeletonJointName SKEL_RIGHT_HIP=21;
  static const SkeletonJointName SKEL_RIGHT_KNEE=22;
  static const SkeletonJointName SKEL_RIGHT_ANKLE=23;
  static const SkeletonJointName SKEL_RIGHT_FOOT=24;

  //////////////////////////////////////////////////////////////////////////////

  ros::Header header;
  SkeletonJointName joint_id;
  std::string child_frame_id;
  geometry_msgs::Pose pose3D;
  geometry_msgs::Pose2D pose2D;
  float confidence;
}; // end class NiteSkeletonJoint

////////////////////////////////////////////////////////////////////////////////

class NiteSkeleton {
public:
  static const SkeletonJointName SKEL_MAX_JOINTS=25;
  ros::Header header;
  int user_id;
  std::vector<NiteSkeletonJoint> joints;
}; // end class NiteSkeleton

////////////////////////////////////////////////////////////////////////////////

class NiteSkeletonList {
public:
  ros::Header header;
  std::vector<NiteSkeleton> skeletons;
}; // end class NiteSkeletonList

} // end namespace kinect

#endif // NITESKELETONLITE_H

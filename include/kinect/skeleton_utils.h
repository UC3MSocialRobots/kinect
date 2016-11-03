/*!
  \file        skeleton_utils.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/11/15

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

\namespace skeleton_utils
\brief Some useful functions for handling with skeletons.
 */

#ifndef SKELETON_UTILS_H
#define SKELETON_UTILS_H

#ifdef NITE_FX
#include "kinect/NiteSkeletonLite.h"
#else  // not NITE_FX
#include "kinect/NiteSkeletonList.h"
#endif // not NITE_FX

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "kinect/std_utils.h"

namespace skeleton_utils {

#ifdef NITE_FX // some ROS-independant declarations
typedef double Time;
struct Rate {
  Rate(double r) { _sleep_time_ms = 1000. / r;}
  void sleep()   {
    Timer::Time now = _timer.time();
    if (now < _sleep_time_ms)
      usleep( 1000 * (_sleep_time_ms - now));
    _timer.reset();
  }
  double _sleep_time_ms;
  Timer _timer;
}; // end struct Rate
#else  // not NITE_FX
typedef ros::Time Time;
typedef ros::Rate Rate;
#endif // not NITE_FX

typedef kinect::NiteSkeletonJoint NSJ;
typedef int8_t JointId;

////////////////////////////////////////////////////////////////////////////////

inline int get_index_in_skeleton(const kinect::NiteSkeleton & skeleton,
                                 const JointId & joint_id) {
  unsigned int njoints = skeleton.joints.size();
  for (unsigned int joint_idx = 0; joint_idx < njoints; ++joint_idx) {
    if (skeleton.joints[joint_idx].joint_id == joint_id)
      return joint_idx;
  } // end loop joint_idx
  // ROS_INFO("skeleton does not contain joint %i", joint_id);
  return -1;
}

////////////////////////////////////////////////////////////////////////////////

class JointId2StringConverter {
public:
  JointId2StringConverter() {
    _names.insert(Entry(NSJ::SKEL_HEAD, "head"));
    _names.insert(Entry(NSJ::SKEL_NECK, "neck"));
    _names.insert(Entry(NSJ::SKEL_TORSO, "torso"));
    _names.insert(Entry(NSJ::SKEL_WAIST, "waist"));
    _names.insert(Entry(NSJ::SKEL_LEFT_COLLAR, "left_collar"));
    _names.insert(Entry(NSJ::SKEL_LEFT_SHOULDER, "left_shoulder"));
    _names.insert(Entry(NSJ::SKEL_LEFT_ELBOW, "left_elbow"));
    _names.insert(Entry(NSJ::SKEL_LEFT_WRIST, "left_wrist"));
    _names.insert(Entry(NSJ::SKEL_LEFT_HAND, "left_hand"));
    _names.insert(Entry(NSJ::SKEL_LEFT_FINGERTIP, "left_fingertip"));
    _names.insert(Entry(NSJ::SKEL_RIGHT_COLLAR, "right_collar"));
    _names.insert(Entry(NSJ::SKEL_RIGHT_SHOULDER, "right_shoulder"));
    _names.insert(Entry(NSJ::SKEL_RIGHT_ELBOW, "right_elbow"));
    _names.insert(Entry(NSJ::SKEL_RIGHT_WRIST, "right_wrist"));
    _names.insert(Entry(NSJ::SKEL_RIGHT_HAND, "right_hand"));
    _names.insert(Entry(NSJ::SKEL_RIGHT_FINGERTIP, "right_fingertip"));
    _names.insert(Entry(NSJ::SKEL_LEFT_HIP, "left_hip"));
    _names.insert(Entry(NSJ::SKEL_LEFT_KNEE, "left_knee"));
    _names.insert(Entry(NSJ::SKEL_LEFT_ANKLE, "left_ankle"));
    _names.insert(Entry(NSJ::SKEL_LEFT_FOOT, "left_foot"));
    _names.insert(Entry(NSJ::SKEL_RIGHT_HIP, "right_hip"));
    _names.insert(Entry(NSJ::SKEL_RIGHT_KNEE, "right_knee"));
    _names.insert(Entry(NSJ::SKEL_RIGHT_ANKLE, "right_ankle"));
    _names.insert(Entry(NSJ::SKEL_RIGHT_FOOT, "right_foot"));
  }

  /*! convert a joint name to a string
   * \example SKEL_HEAD -> "head"
   */
  std::string direct_search(const JointId id) {
    std::string ans;
    if (vision_utils::direct_search(_names, id, ans))
      return ans;
    return "ERROR";
  }

  /*! convert a string to a joint name
   * \example "head" -> SKEL_HEAD
   */
  JointId reverse_search(const std::string name) {
    JointId ans;
    if (vision_utils::reverse_search(_names, name, ans))
      return ans;
    return -1;
  }

protected:
  typedef std::pair<JointId, std::string> Entry;
  std::map<JointId, std::string> _names;
}; // enc class  JointNameStringConverter

////////////////////////////////////////////////////////////////////////////////

inline bool get_point_in_skeleton(const kinect::NiteSkeleton & skeleton,
                                  const JointId & joint_id,
                                  const int w, const int h,
                                  cv::Point & pt_out) {
  int joint_index = get_index_in_skeleton(skeleton, joint_id);
  if (joint_index == -1)
    return false;
  pt_out.x = skeleton.joints[joint_index].pose2D.x * w;
  pt_out.y = skeleton.joints[joint_index].pose2D.y * h;
  if (pt_out.x < -w || pt_out.x > 2 * w || pt_out.y < -h || pt_out.y > 2 * h)
    return false; // out of bounds of image
  return true;
}

////////////////////////////////////////////////////////////////////////////////

inline void draw_line_if_exists(cv::Mat3b & img,
                                const kinect::NiteSkeleton & skeleton,
                                const JointId & j1,
                                const JointId & j2,
                                const cv::Scalar& color,
                                int thickness=1, int lineType=8, int shift=0) {
  static JointId2StringConverter converter;
  cv::Point p1, p2;
  if (get_point_in_skeleton(skeleton, j1, img.cols, img.rows, p1)
      &&
      get_point_in_skeleton(skeleton, j2, img.cols, img.rows, p2)) {
    cv::line(img, p1, p2, color, thickness, lineType, shift);
    cv::circle(img, p1, thickness, cv::Scalar::all(0), -1, lineType, shift);
    cv::circle(img, p2, thickness, cv::Scalar::all(0), -1, lineType, shift);
    cv::putText(img, converter.direct_search(j1), p1, CV_FONT_HERSHEY_PLAIN, 1, color);
    cv::putText(img, converter.direct_search(j2), p2, CV_FONT_HERSHEY_PLAIN, 1, color);
  }
} // end draw_line_if_exists();

////////////////////////////////////////////////////////////////////////////////

inline void draw_skeleton(cv::Mat3b & img,
                          const kinect::NiteSkeleton & skeleton,
                          const cv::Scalar& color,
                          int thickness=1, int lineType=8, int shift=0) {
  draw_line_if_exists(img, skeleton, NSJ::SKEL_HEAD, NSJ::SKEL_NECK, color, thickness, lineType, shift);
  draw_line_if_exists(img, skeleton, NSJ::SKEL_NECK, NSJ::SKEL_TORSO, color, thickness, lineType, shift);
  // draw_line_if_exists(img, skeleton, NS::SKEL_TORSO, NS::SKEL_WAIST, color, thickness, lineType, shift);

  // LEFT side
  // draw_line_if_exists(img, skeleton, NS::SKEL_TORSO, NS::SKEL_LEFT_COLLAR, color, thickness, lineType, shift);
  // draw_line_if_exists(img, skeleton, NS::SKEL_NECK, NS::SKEL_LEFT_COLLAR, color, thickness, lineType, shift);
  // draw_line_if_exists(img, skeleton, NS::SKEL_LEFT_COLLAR, NS::SKEL_LEFT_SHOULDER, color, thickness, lineType, shift);
  draw_line_if_exists(img, skeleton, NSJ::SKEL_NECK, NSJ::SKEL_LEFT_SHOULDER, color, thickness, lineType, shift);
  draw_line_if_exists(img, skeleton, NSJ::SKEL_TORSO, NSJ::SKEL_LEFT_SHOULDER, color, thickness, lineType, shift);
  draw_line_if_exists(img, skeleton, NSJ::SKEL_LEFT_SHOULDER, NSJ::SKEL_LEFT_ELBOW, color, thickness, lineType, shift);
  // draw_line_if_exists(img, skeleton, NS::SKEL_LEFT_ELBOW, NS::SKEL_LEFT_WRIST, color, thickness, lineType, shift);
  // draw_line_if_exists(img, skeleton, NS::SKEL_LEFT_WRIST, NS::SKEL_LEFT_HAND, color, thickness, lineType, shift);
  // draw_line_if_exists(img, skeleton, NS::SKEL_LEFT_HAND, NS::SKEL_LEFT_FINGERTIP, color, thickness, lineType, shift);
  draw_line_if_exists(img, skeleton, NSJ::SKEL_LEFT_ELBOW, NSJ::SKEL_LEFT_HAND, color, thickness, lineType, shift);

  // draw_line_if_exists(img, skeleton, NS::SKEL_WAIST, NS::SKEL_LEFT_HIP, color, thickness, lineType, shift);
  draw_line_if_exists(img, skeleton, NSJ::SKEL_TORSO, NSJ::SKEL_LEFT_HIP, color, thickness, lineType, shift);
  draw_line_if_exists(img, skeleton, NSJ::SKEL_LEFT_HIP, NSJ::SKEL_LEFT_KNEE, color, thickness, lineType, shift);
  // draw_line_if_exists(img, skeleton, NS::SKEL_LEFT_KNEE, NS::SKEL_LEFT_ANKLE, color, thickness, lineType, shift);
  // draw_line_if_exists(img, skeleton, NS::SKEL_LEFT_ANKLE, NS::SKEL_LEFT_FOOT, color, thickness, lineType, shift);
  draw_line_if_exists(img, skeleton, NSJ::SKEL_LEFT_KNEE, NSJ::SKEL_LEFT_FOOT, color, thickness, lineType, shift);

  // RIGHT side
  // draw_line_if_exists(img, skeleton, NS::SKEL_TORSO, NS::SKEL_RIGHT_COLLAR, color, thickness, lineType, shift);
  // draw_line_if_exists(img, skeleton, NS::SKEL_NECK, NS::SKEL_RIGHT_COLLAR, color, thickness, lineType, shift);
  // draw_line_if_exists(img, skeleton, NS::SKEL_RIGHT_COLLAR, NS::SKEL_RIGHT_SHOULDER, color, thickness, lineType, shift);
  draw_line_if_exists(img, skeleton, NSJ::SKEL_NECK, NSJ::SKEL_RIGHT_SHOULDER, color, thickness, lineType, shift);
  draw_line_if_exists(img, skeleton, NSJ::SKEL_TORSO, NSJ::SKEL_RIGHT_SHOULDER, color, thickness, lineType, shift);
  draw_line_if_exists(img, skeleton, NSJ::SKEL_RIGHT_SHOULDER, NSJ::SKEL_RIGHT_ELBOW, color, thickness, lineType, shift);
  // draw_line_if_exists(img, skeleton, NS::SKEL_RIGHT_ELBOW, NS::SKEL_RIGHT_WRIST, color, thickness, lineType, shift);
  // draw_line_if_exists(img, skeleton, NS::SKEL_RIGHT_WRIST, NS::SKEL_RIGHT_HAND, color, thickness, lineType, shift);
  // draw_line_if_exists(img, skeleton, NS::SKEL_RIGHT_HAND, NS::SKEL_RIGHT_FINGERTIP, color, thickness, lineType, shift);
  draw_line_if_exists(img, skeleton, NSJ::SKEL_RIGHT_ELBOW, NSJ::SKEL_RIGHT_HAND, color, thickness, lineType, shift);

  // draw_line_if_exists(img, skeleton, NS::SKEL_WAIST, NS::SKEL_RIGHT_HIP, color, thickness, lineType, shift);
  draw_line_if_exists(img, skeleton, NSJ::SKEL_TORSO, NSJ::SKEL_RIGHT_HIP, color, thickness, lineType, shift);
  draw_line_if_exists(img, skeleton, NSJ::SKEL_RIGHT_HIP, NSJ::SKEL_RIGHT_KNEE, color, thickness, lineType, shift);
  // draw_line_if_exists(img, skeleton, NS::SKEL_RIGHT_KNEE, NS::SKEL_RIGHT_ANKLE, color, thickness, lineType, shift);
  // draw_line_if_exists(img, skeleton, NS::SKEL_RIGHT_ANKLE, NS::SKEL_RIGHT_FOOT, color, thickness, lineType, shift);
  draw_line_if_exists(img, skeleton, NSJ::SKEL_RIGHT_KNEE, NSJ::SKEL_RIGHT_FOOT, color, thickness, lineType, shift);
} // end draw_skeleton();

////////////////////////////////////////////////////////////////////////////////

inline void draw_skeleton_list(cv::Mat3b & img,
                               const kinect::NiteSkeletonList & skeleton_list,
                               int thickness=1, int lineType=8, int shift=0) {
  for (unsigned int skeleton_idx = 0;
       skeleton_idx < skeleton_list.skeletons.size();
       ++skeleton_idx) {
    cv::Scalar curr_color(255, 255, 255);
    // int curr_user_id = skeleton_list.skeletons[skeleton_idx].user_id;
    //    vision_utils::indexed_color255
    //        (curr_color[0], curr_color[1], curr_color[2], curr_user_id);
    draw_skeleton(img, skeleton_list.skeletons[skeleton_idx], curr_color,
                  thickness, lineType, shift);
  } // end loop skeleton_idx
} // end draw_skeleton_list();

} // end namespace skeleton_utils

#endif // SKELETON_UTILS_H

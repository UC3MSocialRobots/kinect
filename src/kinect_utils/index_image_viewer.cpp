/*!
  \file        index_image_viewer.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/10/8

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

A very simple node to show index images
(images containing indices of objects, such as 1, 2, ...)
in meaningful colors.

\section Parameters
  - \b "input_topic"
    [string] (default: "index_image")
    Where to get the index images.

  - \b "data_size"
    [int] (default: "16")
    The size of the indices.

  - \b "resize_scale"
    [double] (default: 1)
    The scale of the interface.
    The original image is 320x240,
    the GUI size scales it by this parameter.

\section Subscriptions
  - \b {input_topic}
    [sensor_msgs::Image]
    The index images
    The index images are of type unsigned short
    (16-bit unsigned integer - CV_16UC1)

  - \b "skeletons"
    [height_detector::NiteSkeletonList]
    The list of skeletons

\section Publications
  None.
 */
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
// kinect
#include "kinect/user_image_to_rgb.h"
#include "kinect/skeleton_utils.h"

cv_bridge::CvImageConstPtr bridge_img;
cv::Mat3b _out_img, _out_img_scaled;
int _data_size = 16;
double _resize_scale = 1;
std::string _window_name;
kinect::NiteSkeletonList _skeleton_list;
ros::Time _skeleton_time;

////////////////////////////////////////////////////////////////////////////////

void skeleton_cb(const kinect::NiteSkeletonList::ConstPtr msg) {
  _skeleton_list = *msg;
  _skeleton_time = msg->header.stamp;
}

////////////////////////////////////////////////////////////////////////////////

void image_callback(const sensor_msgs::ImageConstPtr& msg) {
  // ROS_INFO_THROTTLE(5, "index_image_viewer::image_callback()");
  // convert to OpenCV mat
  try {
    bridge_img = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  user_image_to_rgb(bridge_img->image, _out_img, _data_size);
  if ((ros::Time::now() - _skeleton_time).toSec() < 1)
    vision_utils::draw_skeleton_list(_out_img, _skeleton_list);

  // display image
  if (_resize_scale == 1) {
    cv::imshow(_window_name, _out_img);
  }
  else {
    cv::resize(_out_img, _out_img_scaled, cv::Size(),
               _resize_scale, _resize_scale, cv::INTER_NEAREST);
    cv::imshow(_window_name, _out_img_scaled);
  }
  int key = (char) cv::waitKey(5);
  if (key == 27)
    ros::shutdown();
} // end image_callback();

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "index_image_viewer");
  ros::NodeHandle nh_public, nh_private("~");
  // get params
  std::string _input_topic = "index_image_viewer";
  nh_private.param("data_size", _data_size, _data_size);
  nh_private.param("input_topic", _input_topic, _input_topic);
  nh_private.param("resize_scale", _resize_scale, _resize_scale);

  // subscribe to image topic
  image_transport::ImageTransport transport(nh_public);
  image_transport::Subscriber sub = transport.subscribe
      (_input_topic, 1, image_callback);
  ros::Subscriber skel_sub = nh_public.subscribe("skeletons", 1, skeleton_cb);

  ROS_INFO("index_image_viewer: display index image '%s'",
           sub.getTopic().c_str());

  // create window
  _window_name = sub.getTopic();
  cv::namedWindow(_window_name);

  // spin!
  ros::spin();
  return 0;
}



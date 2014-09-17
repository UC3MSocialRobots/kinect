/*!
  \file        empty_skeleton_publisher.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/11/20

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

\section Publications
  - \b "/skeletons"
        [vision/NiteSkeletonList]
        Empty skeletons
 */

#include <ros/ros.h>
#include "kinect/NiteSkeletonList.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <rosgraph_msgs/Clock.h>

typedef sensor_msgs::Image InputMessage;
//typedef sensor_msgs::CompressedImage InputMessage;
//typedef rosgraph_msgs::Clock InputMessage;
kinect::NiteSkeletonList skeleton_list_msg;
std::string input_image_topic = "input_image_topic", output_skeleton_list_topic = "skeletons";
ros::Subscriber header_sub;
ros::Publisher skeleton_list_pub;

////////////////////////////////////////////////////////////////////////////////

void header_cb(const InputMessage::ConstPtr & msg) {
  ROS_INFO_THROTTLE(10, "empty_skeleton_publisher::header_cb()");
  skeleton_list_msg.header = msg->header;
  //skeleton_list_msg.header.stamp = msg->clock;
  skeleton_list_pub.publish(skeleton_list_msg);
} // end header_cb();

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "empty_skeleton_publisher");
  ros::NodeHandle nh_public, nh_private("~");
  // get params
  nh_private.param<std::string>("input_image_topic", input_image_topic, input_image_topic);
  nh_private.param<std::string>
      ("output_skeleton_list_topic", output_skeleton_list_topic, output_skeleton_list_topic);
  header_sub = nh_public.subscribe<InputMessage>(input_image_topic, 1, header_cb);
  skeleton_list_pub = nh_public.advertise
      <kinect::NiteSkeletonList>(output_skeleton_list_topic, 1);
  ROS_INFO("empty_skeleton_publisher: getting headers on '%s', "
           "publishing empty skeleton lists on '%s'",
           header_sub.getTopic().c_str(), skeleton_list_pub.getTopic().c_str());
  ros::spin();
  return 0;
}



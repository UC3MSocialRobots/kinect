/*!
  \file        kinect_aux_tf_broadcaster.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/4/26
  
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

Publish a TF transfrom for the Kinect that takes into account the angle
of the Kinect obtained with the "kinect_aux driver"

\section Parameters
  - \b "~frame_id"
        [string] (default: "/base_link")
        The parent frame ID.
  - \b "~child_frame_id"
        [string] (default: "/camera_link")
        The parent frame ID.
  - \b "x, y, z, roll, yaw"
        [double] (default: 0)
        The other parameters of the TF, that are static.

\section Subscriptions
  - \b "tilt_angle"
        [std_msgs/Float64]
        The tilt angle of the Kinect,
        most probably obtained with the "kinect_aux driver".
        As it is in degrees, a degree->radian conversion is made.
        
\section Publications
  - \b "/tf"
        [TF]
        The transform {frame_id} -> {child_frame_id}
        
 */
// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>

std::string frame_id = "/base_link", child_frame_id = "/camera_link";
double roll = 0, pitch = 0, yaw = 0;
double x = 0, y = 0, z = 0;
tf::StampedTransform transform;

inline void refresh_transform_with_cached_values() {
  transform = tf::StampedTransform(
                tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw),
                              tf::Vector3(x, y, z)),
                ros::Time::now(), frame_id, child_frame_id
                );
}

////////////////////////////////////////////////////////////////////////////////

void tilt_callback(const std_msgs::Float64::ConstPtr & msg) {
  pitch = msg->data  * -1 // for a weird reason the angle is inversed...
          * 0.01745329251994329577; // deg2rad
  refresh_transform_with_cached_values();
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv){
  ros::init(argc, argv, "kinect_aux_tf_broadcaster");
  ros::NodeHandle nh_public, nh_private("~");

  // get params
  nh_private.param("frame_id", frame_id, frame_id);
  nh_private.param("child_frame_id", child_frame_id, child_frame_id);
  nh_private.param("roll", roll, roll);
  // pitch obtained by kinect_aux
  nh_private.param("yaw", yaw, yaw);
  nh_private.param("x", x, x);
  nh_private.param("y", y, y);
  nh_private.param("z", z, z);

  // subscribe to kinect angle
  ros::Subscriber tilt_sub = nh_public.subscribe("cur_tilt_angle", 1, tilt_callback);
  tf::TransformBroadcaster broadcaster;
  ros::Rate r(10);
  ROS_INFO("kinect_aux_tf_broadcaster: starting to emit '%s'->'%s' every %g ms...",
           frame_id.c_str(), child_frame_id.c_str(),
           r.expectedCycleTime().toSec() * 1000.f);
  refresh_transform_with_cached_values();

  while(ros::ok()){
    //ROS_INFO("Emitting");
    transform.stamp_ = ros::Time::now();
    broadcaster.sendTransform(transform);
    ros::spinOnce();
    r.sleep();
  } // end while(ok)
  return 0;
}

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
- \b "tilt"
[std_msgs/Float64]
The tilt angle of the Kinect,
most probably obtained with the "kinect_aux driver".
As it is in degrees, a degree->radian conversion is made.

\section Publications
- \b "/tf"
[TF]
The transform {frame_id} -> {child_frame_id}

*/
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>
#define MAX_TILT_ANGLE 31.
#define MIN_TILT_ANGLE (-31.)

int up_button = 1, down_button = 2;
double current_angle = 0, incr = 5;
ros::Publisher pub_tilt;
std_msgs::Float64 string_msg;

////////////////////////////////////////////////////////////////////////////////

void joy_cb(const sensor_msgs::Joy::ConstPtr& joy) {
  int nbuttons = joy->buttons.size();
  if (nbuttons < up_button+1 || nbuttons < down_button+1) {
    ROS_WARN_THROTTLE(1, "joy2kinect_tilt: not enough buttons on the joystick: "
      "we have %i, we expected at least %i\n" , nbuttons, std::max(up_button+1, down_button+1));
    return;
  }
  if (joy->buttons[up_button]) {
    string_msg.data = current_angle + incr;
    pub_tilt.publish(string_msg);
  }
  else if (joy->buttons[down_button]) {
    string_msg.data = current_angle - incr;
    pub_tilt.publish(string_msg);
  }
} // end joy_cb();

////////////////////////////////////////////////////////////////////////////////

void tilt_cb(const std_msgs::Float64::ConstPtr& angle) {
  // discard erroneous readings
  if (angle->data > MIN_TILT_ANGLE && angle->data < MAX_TILT_ANGLE)
    current_angle = angle->data;
} // end tilt_cb();

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "joy2kinect_tilt");
  ros::NodeHandle nh_public, nh_private("~");
  // params
  nh_private.param<int>("up_button", up_button, up_button);
  nh_private.param<int>("down_button", down_button, down_button);
  // subscribers
  ros::Subscriber joy_sub = nh_public.subscribe<sensor_msgs::Joy>("joy", 1,  joy_cb);
  ros::Subscriber sub_tilt = nh_public.subscribe("cur_tilt_angle", 1, tilt_cb);
  // publishers
  pub_tilt = nh_public.advertise<std_msgs::Float64>("tilt_angle", 1);
  ros::spin();
  return 0;
}

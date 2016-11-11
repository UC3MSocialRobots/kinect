/*!
  \file        publish_kinect_serial.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/3/6

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

lsusb | grep "Xbox NUI Camera" | grep -Po '[0-9a-z]*:[0-9a-z][0-9a-z]*'
-> get product ID
lsusb -v -d 045e:02ae | grep -e "iSerial" | awk '{print $3}'

 */

#include <ros/ros.h>
#include "kinect/std_utils.h"
#include <vision_utils/exec_system.h>
#include <vision_utils/exec_system_get_output.h>
#include <vision_utils/find_and_replace.h>
#include <vision_utils/retrieve_file.h>

static const char* KINECT_lsusb_FILE = "/tmp/kinect_lsusb.tmp";

//////////////////////////////////////////////////////////////////////////////

inline bool get_serial_from_lsusb(std::string & ans) {
  std::ostringstream order;
  // get device ID
  std::string device_id;
  order << "lsusb | grep \"Xbox NUI Camera\" | grep -Po '[0-9a-z]*:[0-9a-z][0-9a-z]*'"
        << " > " << KINECT_lsusb_FILE;
  vision_utils::exec_system(order.str());
  bool success = vision_utils::retrieve_file(KINECT_lsusb_FILE, device_id);
  vision_utils::find_and_replace(device_id, "\n", ""); // remove breaklines
  if (!success || device_id.size() == 0) {
    ROS_WARN("Could not retrieve file '%s'. Cannot get serial nb of kinect", KINECT_lsusb_FILE);
    ROS_WARN( "vendor:product is %s\n", device_id.c_str() );
    return false;
  }

  // get serial
  order.str("");
  order << "lsusb -v -d " << device_id << " | grep -e \"iSerial\" | awk '{print $3}'"
        << " > " << KINECT_lsusb_FILE;
  vision_utils::exec_system(order.str());
  success = vision_utils::retrieve_file(KINECT_lsusb_FILE, ans);
  ROS_WARN( "Retrieved device ID" );
  vision_utils::find_and_replace(ans, "\n", ""); // remove breaklines
  if (!success || ans.size() == 0) {
    ROS_WARN("Could not convert device ID to serial (file '%s'). Cannot get serial nb of kinect",
             KINECT_lsusb_FILE);
    return false;
  }
  // ROS_WARN("Serial number of the Kinect:'%s'", ans.c_str());
  return true;
} // end get_serial_from_lsusb()

inline bool is_asus() {
  std::ostringstream order;
  //order << "lsusb -v 2> /dev/null | grep PrimeSense";
  order << "lsusb -v -d 1d27:0600";
  std::string output = vision_utils::exec_system_get_output(order.str().c_str());
  // printf("output:'%s'\n", output.c_str());
  if (output.find("PrimeSense Device") != std::string::npos)
    return true;
  return false;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "publish_kinect_serial");
  ros::NodeHandle nh_public;
  std::string kinect_serial_number = "??";
  bool success = false;
  while(!success && ros::ok()) {
    std::string name_resolved = nh_public.resolveName("kinect_serial_number");
    if (nh_public.hasParam(name_resolved)) {
      //nh_public.param(name_resolved, kinect_serial_number, kinect_serial_number);
      //ros::param::get(name_resolved, kinect_serial_number);
      nh_public.getParam(name_resolved, kinect_serial_number);
      ROS_WARN("Seems the serial of the Kinect in parameter '%s' "
               "was set to value '%s' from somewhere else. Quitting.",
               name_resolved.c_str(), kinect_serial_number.c_str());
      break;
    }
    //    else
    //      ROS_WARN("Parameter '%s' not defined.", name_resolved.c_str());

    success = get_serial_from_lsusb(kinect_serial_number);
    if (success) {
      ROS_WARN("Succesfully obtained the serial of the Kinect:'%s', publishing it.", kinect_serial_number.c_str());
      nh_public.setParam(name_resolved, kinect_serial_number);
      break;
    }
    // check if ASUS
    if (is_asus()) {
      ROS_WARN("The device is a ASUS Xtion device with no serial");
      break;
    }

    ROS_WARN("Could not obtain the serial of the Kinect, waiting two seconds.");
    sleep(2);
    ros::spinOnce();
  } // end while(!success)
} // end main()

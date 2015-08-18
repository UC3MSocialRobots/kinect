/*!
  \file        nite_utils.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/11/13

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

\namespace nite_utils
\brief useful functions for working with a Kinect, specifically using NITE middleware
 */

#ifndef KINECT_UTILS_H
#define KINECT_UTILS_H

// C++
#include <algorithm>    // std::find
// NITE
#include <XnCppWrapper.h>
// AD
#include "string/StringUtils.h"
#include "system/system_utils.h"

namespace nite_utils {

static const char* KINECT_USB_INFO_FILE = "/tmp/kinect_usb_info.tmp";

////////////////////////////////////////////////////////////////////////////////

/*! \brief   split a string
 * \param   str the long string
 * \param   delim the string which is the separator between words, for instance '/'
 * \param   results a pointer towards a vector of string, will contain the results
 */
inline void StringSplit(const std::string & str, const std::string & delim,
                        std::vector<std::string>* results) {
  // maggieDebug3("StringSplit(str:'%s', delim:'%s')", str.c_str(), delim.c_str());
  results->clear();
  if (str == "")
    return;
  size_t delim_pos, search_pos = 0;
  while (search_pos <= str.size() - 1) {
    delim_pos = str.find(delim, search_pos);
    //maggieDebug1("delim_pos:%i, search_pos:%i", delim_pos, search_pos);
    if (delim_pos == std::string::npos) { // no more delim
      results->push_back(str.substr(search_pos));
      return;
    }
    if (delim_pos > 0) // == 0 only happens if str starts with delim
      results->push_back(str.substr(search_pos, delim_pos - search_pos));
    search_pos = delim_pos + delim.size();
    // quit if we reached the end of the std::string or std::string empty
  }
} // end StringSplit();

////////////////////////////////////////////////////////////////////////////////

/*!
  \return the serial number of a kinect
  from http://stackoverflow.com/questions/8988090/how-can-i-get-the-kinect-serial-number-with-openni
  */
std::string get_kinect_serial_number(const xn::Context & context) {
  // get device node from Nite context
  xn::Device device_node;
  XnStatus nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEVICE, device_node);
  if (nRetVal != XN_STATUS_OK) {
    ROS_WARN("get_kinect_serial_number() : Could not get device node of given context");
    return "";
  }

  // get creation info from device node
  const XnChar* creation_info = device_node.GetInfo().GetCreationInfo();
  std::string creation_info_str(creation_info);

  // remove junk from this creation info (the port, the '/')
  std::string creation_info_str_cleaned;
  std::string::size_type arobase_pos = creation_info_str.find("@");
  if (arobase_pos != std::string::npos)
    creation_info_str_cleaned = creation_info_str.substr(0, arobase_pos);
  else
    creation_info_str_cleaned = creation_info_str;
  std_utils::find_and_replace(creation_info_str_cleaned, "/", ":");
  //  ROS_WARN("creation_info_str:'%s', creation_info_str_cleaned:'%s'",
  //           creation_info_str.c_str(), creation_info_str_cleaned.c_str());
  if (creation_info_str_cleaned.size() == 0) {
    ROS_WARN("get_kinect_serial_number() : Could not creation info of device node");
    return "";
  }

  // exec something like 'lsusb -v -d 045e:02ae > foo.txt'
  std::ostringstream usb_info_cmd;
  usb_info_cmd << "lsusb -v -d " << creation_info_str_cleaned
               << " > " << KINECT_USB_INFO_FILE;
  int return_value = system_utils::exec_system(usb_info_cmd.str());
  if (return_value < 0) {
    ROS_WARN("get_kinect_serial_number() : Could not exec '%s'",
                usb_info_cmd.str().c_str());
    return "";
  }

  // read the output file
  std::string usb_info_file;
  bool file_reading_success = std_utils::retrieve_file
      (KINECT_USB_INFO_FILE, usb_info_file);
  if (!file_reading_success) {
    ROS_WARN("get_kinect_serial_number() : Could not read file '%s'",
                KINECT_USB_INFO_FILE);
    return "";
  }

  // remove line breaks
  std_utils::find_and_replace(usb_info_file, "\n", " ");
  // remove all double spaces
  while (std_utils::find_and_replace(usb_info_file, "  ", " ") > 0)
  {}

  // look for 'iSerial' in the parsed file
  std::string searched_token = "iSerial";
  std::vector<std::string> tokens;
  StringSplit(usb_info_file, " ", & tokens);
  std::vector<std::string>::iterator iSerial_it =
      std::find(tokens.begin(), tokens.end(), searched_token);
  if (iSerial_it == tokens.end()) {
    ROS_WARN("get_kinect_serial_number() : Could not find token '%s' in file '%s'",
                searched_token.c_str(), KINECT_USB_INFO_FILE);
    return "";
  }
  // ROS_WARN("tokens:'%s'", std_utils::accessible_to_string(tokens).c_str());

  // advance the pointer by 2
  for (unsigned int i = 0; i < 2; ++i) {
    ++iSerial_it;
    if (iSerial_it == tokens.end()) {
      ROS_WARN("get_kinect_serial_number() : There is no serial number after '%s' in file '%s'",
                  searched_token.c_str(), KINECT_USB_INFO_FILE);
      return "";
    }
  } // end loop i

  // work done, iterator pointing at serial number. celebrate.
  std::string serial_nb_str = *iSerial_it;
  return serial_nb_str;
} // end get_kinect_serial_number()

////////////////////////////////////////////////////////////////////////////////

} // end namespace nite_utils

#endif // KINECT_UTILS_H

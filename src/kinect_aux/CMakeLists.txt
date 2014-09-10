include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules (LIBUSB REQUIRED libusb-1.0)
include_directories (${LIBUSB_INCLUDE_DIRS})
link_directories (${LIBUSB_LIBRARY_DIRS})

rosbuild_add_executable(kinect_aux_node kinect_aux.cpp)
target_link_libraries (kinect_aux_node usb-1.0)

rosbuild_add_executable(kinect_aux_tf_broadcaster kinect_aux_tf_broadcaster.cpp)

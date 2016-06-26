### 1 - Required CMake Version
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra") # add extra warnings
### 2- Package Name
project(kinect)
### 3 - Find other CMake/Catkin packages needed for build
SET(ROSDEPS cv_bridge message_generation geometry_msgs image_geometry image_transport rosbag tf)
find_package(catkin REQUIRED COMPONENTS ${ROSDEPS})
find_package(PCL 1.2 REQUIRED)
find_package(orocos_kdl REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

find_library(OROCOS_KDL orocos-kdl)
set(orocos_kdl_LIBRARIES ${OROCOS_KDL})
### 4 - Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())
add_message_files(DIRECTORY msg
  FILES
  NiteSkeleton.msg
  NiteSkeletonJoint.msg
  NiteSkeletonList.msg
)
### 5 - Invoke message/service/action generation (generate_messages())
generate_messages(
  DEPENDENCIES
  geometry_msgs
)
### 6 - Specify package build info export (catkin_package())
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${ROSDEPS}
  DEPENDS orocos_kdl
)
### 7 - Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/openni /usr/include/nite /usr/include/ni
  ${orocos_kdl_INCLUDE_DIRS}
)
ADD_SUBDIRECTORY(src)
### 8 - Tests to build (catkin_add_gtest())
catkin_add_nosetests(src/test/test_skeleton_msg_unpacker.py)
### 9 - Install rules (install())
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

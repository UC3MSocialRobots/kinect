### nite_primitive_standalone
add_executable(nite_primitive_standalone.exe
               nite_primitive_standalone.cpp)
target_link_libraries(nite_primitive_standalone.exe
                      OpenNI ${orocos_kdl_LIBRARIES} #XnVNite
                      ${catkin_LIBRARIES})
add_dependencies(nite_primitive_standalone.exe kinect_generate_messages_cpp)

### nite_primitive
add_library(nite_primitive
            nite_primitive.cpp)
target_link_libraries(nite_primitive
                      OpenNI ${orocos_kdl_LIBRARIES} #XnVNite
                      ${catkin_LIBRARIES})
add_dependencies(nite_primitive  kinect_generate_messages_cpp)
install(TARGETS nite_primitive
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

### nite_primitive_standalone
add_executable(empty_skeleton_publisher.exe empty_skeleton_publisher.cpp)
target_link_libraries(empty_skeleton_publisher.exe nite_primitive ${catkin_LIBRARIES})

### publish_kinect_serial
add_executable(publish_kinect_serial.exe publish_kinect_serial.cpp)
target_link_libraries(publish_kinect_serial.exe ${catkin_LIBRARIES})

### nite_fx: as a library, for compiling it (and making it visible in QtCreator),
###, but not complaining about linking problems with ROS
#add_library(nite_fx             nite_fx.cpp )
#target_link_libraries(nite_fx   OpenNI XnVNite )

ADD_SUBDIRECTORY(kinect_utils)
ADD_SUBDIRECTORY(kinect_aux)
ADD_SUBDIRECTORY(nite)
ADD_SUBDIRECTORY(test)

### nite_primitive_standalone
add_executable(nite_primitive_standalone.exe 
               nite_primitive_standalone.cpp
               nite_primitive.h
               kinect_utils/nite_utils.h
               kinect_utils/kinect_openni_utils.h
               )

target_link_libraries(nite_primitive_standalone.exe 
                      OpenNI orocos-kdl #XnVNite
                      ${catkin_LIBRARIES}
                      )

### link it in addition with effect_collection, as it is needed here
#~ target_link_libraries(nite_primitive_standalone effect_collection)

### nite_primitive
add_library(nite_primitive 
            nite_primitive.cpp nite_primitive.h
            # fake dependencies of nite_fx (making it visible in QtCreator)
            NiteSkeletonLite.h)
            
target_link_libraries(nite_primitive 
                      OpenNI #XnVNite
                      ${catkin_LIBRARIES}
                      )

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
#add_library(nite_fx             nite_fx.cpp NiteSkeletonLite.h)
#target_link_libraries(nite_fx   OpenNI XnVNite )

ADD_SUBDIRECTORY(kinect_utils)
ADD_SUBDIRECTORY(kinect_aux)
ADD_SUBDIRECTORY(test)

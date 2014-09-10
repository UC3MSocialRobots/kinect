### nite_primitive_standalone
rosbuild_add_executable(nite_primitive_standalone
                        nite_primitive_standalone.cpp
                        nite_primitive.h
                        kinect_utils/nite_utils.h
                        kinect_utils/kinect_openni_utils.h
)

target_link_libraries(nite_primitive_standalone
                      OpenNI XnVNite orocos-kdl
                      )

### link it in addition with effect_collection, as it is needed here
#~ target_link_libraries(nite_primitive_standalone effect_collection)

### nite_primitive
rosbuild_add_library(nite_primitive
                     nite_primitive.cpp nite_primitive.h
                     # fake dependencies of nite_fx (making it visible in QtCreator)
                     NiteSkeletonLite.h
)

target_link_libraries(nite_primitive
                      OpenNI XnVNite
                      )


### nite_primitive_standalone
rosbuild_add_executable(empty_skeleton_publisher
                        empty_skeleton_publisher.cpp
)

### publish_kinect_serial
add_maggie_test(publish_kinect_serial.exe
  SOURCES       publish_kinect_serial.cpp
  LINK
)

### nite_fx: as a library, for compiling it (and making it visible in QtCreator),
###, but not complaining about linking problems with ROS
#add_library(nite_fx             nite_fx.cpp NiteSkeletonLite.h)
#target_link_libraries(nite_fx   OpenNI XnVNite )

ADD_SUBDIRECTORY(kinect_utils)
ADD_SUBDIRECTORY(kinect_aux)
ADD_SUBDIRECTORY(test)

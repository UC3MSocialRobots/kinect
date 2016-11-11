add_executable       (foo_nite_subscriber.exe foo_nite_subscriber.cpp)
target_link_libraries(foo_nite_subscriber.exe OpenNI ${catkin_LIBRARIES}) #XnVNite)
add_dependencies     (foo_nite_subscriber.exe
  ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

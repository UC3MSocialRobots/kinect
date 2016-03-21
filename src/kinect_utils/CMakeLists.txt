add_executable(index_image_viewer.exe index_image_viewer.cpp)
target_link_libraries(index_image_viewer.exe ${catkin_LIBRARIES})
add_dependencies( index_image_viewer.exe
  ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

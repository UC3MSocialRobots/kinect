add_executable(test_kinect_openni_utils.exe 
               test_kinect_openni_utils.cpp
               
               
               
)
				
target_link_libraries(test_kinect_openni_utils.exe ${PCL_LIBRARIES}
                                                   ${catkin_LIBRARIES}
)

add_executable(index_image_viewer.exe 
               index_image_viewer.cpp
               
)

target_link_libraries(index_image_viewer.exe ${catkin_LIBRARIES})

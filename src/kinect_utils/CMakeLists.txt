add_executable(test_kinect_openni_utils.exe 
			   test_kinect_openni_utils.cpp
               kinect_openni_utils.h
               std_utils.h
               color_utils_ros.h
			   )
				
target_link_libraries(test_kinect_openni_utils.exe 
					  ${PCL_LIBRARIES}
					  )

add_executable(index_image_viewer.exe 
			   index_image_viewer.cpp 
			   user_image_to_rgb.h
			   )

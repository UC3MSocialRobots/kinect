add_maggie_test(test_kinect_openni_utils.exe
  SOURCES       test_kinect_openni_utils.cpp
                kinect_openni_utils.h
                std_utils.h
                color_utils_ros.h
)
target_link_libraries(test_kinect_openni_utils.exe ${PCL_LIBRARIES} )

add_maggie_test(index_image_viewer.exe
    SOURCES     index_image_viewer.cpp user_image_to_rgb.h
    LINK
)

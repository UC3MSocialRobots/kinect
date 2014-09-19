catkin_add_gtest(gtest_kinect_openni_utils
  gtest_kinect_openni_utils.cpp
)
target_link_libraries(gtest_kinect_openni_utils ${catkin_LIBRARIES})

catkin_add_gtest(gtest_skeleton_utils
  gtest_skeleton_utils.cpp
  ../kinect_utils/skeleton_utils.h
)
target_link_libraries(gtest_skeleton_utils ${catkin_LIBRARIES})

catkin_add_gtest(gtest_user_image_to_rgb
  gtest_user_image_to_rgb.cpp
  ../kinect_utils/user_image_to_rgb.h
)
target_link_libraries(gtest_user_image_to_rgb ${catkin_LIBRARIES})

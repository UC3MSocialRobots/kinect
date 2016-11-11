/*!
  \file        gtest_user_image_to_rgb.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/4/22

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

Some tests for the file user_image_to_rgb

 */

#include "kinect/user_image_to_rgb.h"
#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>
#include <gtest/gtest.h>

TEST(TestSuite, simple_mask) {
  std::string IMG_DIR  = ros::package::getPath("kinect") + std::string("/data/tests/");
  cv::Mat multimask = cv::imread(IMG_DIR +  + "masks.png", CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat3b multimask_illus;
  unsigned int ntimes = 10;
  for (unsigned int time = 0; time < ntimes; ++time) {
    user_image_to_rgb(multimask, multimask_illus);
  } // end for (time)
  ASSERT_TRUE(multimask_illus.size() == multimask.size());
  //cv::imshow("multimask_illus", multimask_illus);
  //cv::waitKey(0);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

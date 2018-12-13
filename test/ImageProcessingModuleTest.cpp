/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2018, KapilRawal, Hrishikesh Tawade.
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *  @copyright (c) BSD
 *
 *  @file   ImageProcessingModuleTest.cpp
 *
 *  @author   Kapil Rawal (krawal@terpmail.umd.edu)
 *  @author   Hrishikesh Tawde (htawade@terpmail.umd.edu)
 *
 *  @copyright BSD License
 *
 *  @brief  Image Processing module test file
 *
 *  @section DESCRIPTION
 *
 *  This program will check the methods, boundary conditions and exceptions of ImageProcessingModule
 *  class
 */
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include "../include/ImageProcessingModule.h"


/**
 * @brief  checks if the convertImage is executed
 *
 * @param  ImageProcessingModuleTest name of the test suite
 *
 * @param  convertImageTest name of the test
 */
TEST(ImageProcessingModuleTest, convertImageTest) {
  ImageProcessingModule processor;
  ros::WallDuration(5).sleep();
  ros::spinOnce();
  EXPECT_EQ(false, processor.getImage().empty());
}

/**
 * @brief  checks if the detectContour is executed
 *
 * @param  ImageProcessingModuleTest name of the test suite
 *
 * @param  detectContourTest name of the test
 */

TEST(ImageProcessingModuleTest, detectContourTest) {
  ImageProcessingModule processor_;
  std::vector<double> location = processor_.detectContour("1", 1);
  std::cout << location.size() << std::endl;
  EXPECT_NEAR(2, (location.size()) / 2, 1);
}

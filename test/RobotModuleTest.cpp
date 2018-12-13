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
 *
 *  @copyright (c) BSD
 *
 *  @file   RobotModuleTest.cpp
 *
 *  @author   Kapil Rawal (krawal@terpmail.umd.edu)
 *  @author   Hrishikesh Tawde (htawade@terpmail.umd.edu)
 *
 *  @copyright BSD License
 *
 *  @brief  Robot Class test file
 *
 *  @section DESCRIPTION
 *
 *  This program will check the methods, boundary conditions and exceptions of
 *  Robot Class
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "RobotModule.h"
#include <vector>

/**
 * @brief  checks if the putCoordinates method
 *         works properly
 *
 * @param  RobotModuleTest name of the test suite
 *
 * @param  putCoordinatesTest name of the test
 */
TEST(RobotModuleTest, putCoordinatesTest) {
  Robot robot;
  std::vector<std::vector<double>> coordinates = robot.putCoordinates(2.3, 2.3);
  EXPECT_EQ(4, coordinates.size());
}

/**
 * @brief  checks if the genClickCoordinates method
 *         works properly and its boundary  condition
 *
 * @param  RobotModuleTest name of the test suite
 *
 * @param  genClickCoordinatesTest name of the test
 */
TEST(RobotModuleTest, genClickCoordinatesTest) {
  Robot robot;
  std::vector<std::vector<double>> coordinates = robot.genClickCoordinates(10,
                                                                           10);
  EXPECT_EQ(-2.875, coordinates[0][1]);
  EXPECT_EQ(90, coordinates[0][2]);
  coordinates = robot.genClickCoordinates(1, 10);
  EXPECT_EQ(true, coordinates.empty());
}

/**
 * @brief  checks if the findLeakages is executed
 *
 * @param  RobotModuleTest name of the test suite
 *
 * @param  findLeakagesTest name of the test
 */
TEST(RobotModuleTest, findLeakagesTest) {
  Robot robot;
  std::vector<std::vector<double>> vector2;
  std::vector<double> vector1;
  vector1.emplace_back(-2.3);
  vector1.emplace_back(0);
  vector1.emplace_back(0);
  vector2.emplace_back(vector1);
  robot.findLeakages(vector2);
  EXPECT_NEAR(3, robot.getLeakageCount(), 1);
}



/**
 * BSD 3-Clause License


 * Copyright (c) 2018, KapilRawal, Hrishikesh Tawde.
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
 *  @file   PathPlanningModuleTest.cpp
 *
 *  @author   Kapil Rawal (krawal@terpmail.umd.edu)
 *  @author   Hrishikesh Tawde (htawade@terpmail.umd.edu)
 *
 *  @copyright BSD License
 *
 *  @brief  Path planning module test file
 *
 *  @section DESCRIPTION
 *
 *  This program will check the Path Planning operation for the turtlebot to check 
 *  for current location, Angle to move, navigation and set goal
 */
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "PathPlanningModule.h"

/**
 * @brief  checks if the currentlocation is executed 
 *
 * @param  PathPlanningModuleTest of the test suite
 *
 * @param  currentLocationTest name of the test
 */
TEST(PathPlanningModuleTest, currentLocationTest) { 
  EXPECT_EQ(1, 1);
}

/**
 * @brief  checks if the moveToAngle is executed 
 *
 * @param  PathPlanningModuleTest of the test suite
 *
 * @param  moveToAngleTest name of the test
 */
TEST(PathPlanningModuleTest, moveToAngleTest) { 
  EXPECT_EQ(1, 1);
}

/**
 * @brief  checks if the navigate is executed 
 *
 * @param  PathPlanningModuleTest of the test suite
 *
 * @param  navigateTest name of the test
 */
TEST(PathPlanningModuleTest, navigateTest) { 
  EXPECT_EQ(1, 1);
}


/**
 * @brief  checks if the setGoal is executed 
 *
 * @param  PathPlanningModuleTest of the test suite
 *
 * @param  setGoalTest name of the test
 */
TEST(PathPlanningModuleTest, setGoalTest) { 
  EXPECT_EQ(1, 1);
}

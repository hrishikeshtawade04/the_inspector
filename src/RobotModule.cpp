/**
 * BSD 3-Clause License


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
 *  @file   RobotModule.cpp
 *
 *  @author   Kapil Rawal (krawal@terpmail.umd.edu)
 *  @author   Hrishikesh Tawde (htawade@terpmail.umd.edu)
 *
 *  @copyright   BSD License
 *
 *  @brief    Robot Class Implementation file
 *
 *  @section   DESCRIPTION
 *
 *  This file contains the implementations of the methods of Robot Class
 */

#include "../include/RobotModule.h"
#include <iostream>
#include <vector>
#include <string>
#include "ros/ros.h"

Robot::Robot()
    : leakageCount_(0) {
}

Robot::~Robot() {
}

std::vector<std::vector<double>> Robot::putCoordinates(double wall13,
                                                       double wall24) {
  std::vector<std::vector<double>> vector2;
  {
    std::vector<double> vector1;
    vector1.emplace_back(0);
    vector1.emplace_back(-wall24);
    vector1.emplace_back(90);
    vector2.emplace_back(vector1);
  }
  {
    std::vector<double> vector1;
    vector1.emplace_back(wall13);
    vector1.emplace_back(0);
    vector1.emplace_back(180);
    vector2.emplace_back(vector1);
  }
  {
    std::vector<double> vector1;
    vector1.emplace_back(0);
    vector1.emplace_back(wall24);
    vector1.emplace_back(-90);
    vector2.emplace_back(vector1);
  }
  {
    std::vector<double> vector1;
    vector1.emplace_back(-wall13);
    vector1.emplace_back(0);
    vector1.emplace_back(0);
    vector2.emplace_back(vector1);
  }
  return vector2;
}
std::vector<std::vector<double>> Robot::genClickCoordinates(double length,
                                                            double breadth) {
  /// finding the  coordinates based on turtlebot camera field of view
  if (length - breadth * 0.7875 > 0 && (breadth - length * 0.7875 > 0)) {
    double wall13 = 0;
    double wall24 = 0;
    wall13 = breadth * 0.7875 - length / 2;
    wall24 = length * 0.7875 - breadth / 2;
    return putCoordinates(wall13, wall24);
  } else {
    /// cannot create coordinates since the dimensions of infrastructure
    /// improper
    ROS_INFO("Cannot create coordinates");
    std::vector<std::vector<double>> noCoordinates;
    return noCoordinates;
  }
}

void Robot::findLeakages(std::vector<std::vector<double>> mapCoordinates) {
  std::vector<double> locations;
  int counter = 0;
  double prevX = 0;
  int wallCount = 1;
  ros::Rate rate(50.0);
  /// go to the  coordinates, click images and find leakage locations
  /// with respect to wall and then print
  for (std::vector<double> pos : mapCoordinates) {
    pathPlanner_.setGoal(pos[0], pos[1]);
  ROS_INFO("Goal Set");
  pathPlanner_.navigate();
    pathPlanner_.moveToAngle(pos[2]);
  ros::Duration(3.0).sleep();
  ROS_INFO("Navigation Completed");
  leakageLocations_ = imageProcessor_.detectContours(wallcount,1)
  }
  ROS_INFO_STREAM("Total Leakages = " << leakageCount_);
  for (auto leakage : leakageLocations_) {
    if (counter % 16 == 0) {
      ROS_INFO_STREAM("Wall " << wallCount << "Leakages");
      ++wallCount;
    }
    if (counter % 2 == 0) {
      prevX = leakage;
    } else {
      ROS_INFO_STREAM("X = " << prevX << ", Y = " << leakage);
    }
    ++counter;
  }
  ROS_INFO("Leakge finding complete");
}

int Robot::getLeakageCount() {
  return leakageCount_;
}


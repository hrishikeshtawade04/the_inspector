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
 *  @file   RobotModule.h
 *
 *  @author   Kapil Rawal (krawal@terpmail.umd.edu)
 *  @author   Hrishikesh Tawde (htawade@terpmail.umd.edu)
 *
 *  @copyright   BSD License
 *
 *  @brief    Robot Class Header file
 *
 *  @section   DESCRIPTION
 *
 *  Defines the methods and variables of Robot Class
 */

#ifndef INCLUDE_ROBOTMODULE_H_
#define INCLUDE_ROBOTMODULE_H_

#include "ImageProcessingModule.h"
#include "PathPlanningModule.h"
#include <iostream>
#include <vector>
#include <string>
#include "ros/ros.h"

/**
 * @brief Class for robot
 */
class Robot {
 public:
  /**
   *  @brief   Constructor
   */
  Robot();

  /**
   *  @brief   Destroys the object
   */
  ~Robot();

  /**
   * @brief finds leakages in the infrastructure
   *
   * @param mapCoordinates coordinates where the robot should stop and take picture
   *                       and find leakage
   *
   * @param path path where the images need to be stored
   *
   * @return Nothing
   */
  void findLeakages(std::vector<std::vector<double>> mapCoordinates,
                    std::string path);

  /**
   * @brief puts the calculated coordinates in a vector for easy accessibility
   *
   * @param wall13 distance from wall 1 and 3
   *
   * @param wall24 distance from wall 2 and 4
   *
   * @return vector containing the coordinates in a vector
   */
  std::vector<std::vector<double>> putCoordinates(double wall13, double wall24);

  /**
   * @brief finds the coordinates where the robot should stop
   *        based on map's dimensions
   *
   * @param length length of the map
   *
   * @param breadth breadth of the map
   *
   * @return coordinates where the robot should stop and take picture
   *         and find leakage
   */
  std::vector<std::vector<double>> genClickCoordinates(double length,
                                                       double breadth);
  /**
   * @brief gets the leakage count
   *
   * @return private variable leakageCount_
   */
  int getLeakageCount();

 private:
  ImageProcessingModule imageProcessor_;
  PathPlanningModule pathPlanner_;
  double leakageCount_;
};
#endif  // INCLUDE_ROBOTMODULE_H_


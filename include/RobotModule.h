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
 *  @file   RobotModule.h
 *
 *  @author   Kapil Rawal (krawal@terpmail.umd.edu)
 *  @author   Hrishikesh Tawde (htawade@terpmail.umd.edu)
 *
 *  @copyright   BSD License
 *
 *  @brief    Robot file
 *
 *  @section   DESCRIPTION
 *
 *  Header file for the Robot class conatining decleration of the methods
 */

#ifndef INCLUDE_THE_INSPECTOR_ROBOT_H_
#define INCLUDE_THE_INSPECTOR_ROBOT_H_

#include <iostream>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "ImageProcessingModule.h"
#include "PathPlanningModule.h"

class Robot{

private:
  ImageProcessingModule imageProcessor_;
  PathPlanningModule pathPlanner_;
  std::vector<double> leakageLocations_;
  double leakageCount_;
  
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
   *  @brief  Finds leakage by calling the pathplanning module and 
   *          image processing module
   *
   *  @return nothing
   */
  void findLeakages();

};

#endif /// INCLUDE_THEINSPECTOR_ROBOT_H_

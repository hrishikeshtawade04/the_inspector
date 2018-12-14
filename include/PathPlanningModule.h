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
 *  @file   PathPlanningModule.h
 *
 *  @author   Kapil Rawal (krawal@terpmail.umd.edu)
 *  @author   Hrishikesh Tawde (htawade@terpmail.umd.edu)
 *
 *  @copyright   BSD License
 *
 *  @brief    PathPlanningModule Class header file
 *
 *  @section   DESCRIPTION
 *
 *  This is the header file for the path planning module contains
 *  declaration of methods
 */

#ifndef INCLUDE_PATHPLANNINGMODULE_H_
#define INCLUDE_PATHPLANNINGMODULE_H_

#include <tf/transform_listener.h>
#include <iostream>
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

/**
 * @brief Class PathPlanningModule
 */
class PathPlanningModule {
 private:
/// Created a node handle
  ros::NodeHandle n;
/// Publisher for cv bridge
  ros::Publisher velocityPublish;
/// Created a listener to listen to transform
  tf::TransformListener listener;
/// Created a ontainer to store transform
  tf::StampedTransform transform;
/// Created msg to store robot speed
  geometry_msgs::Twist vel_msg;
/// Variable to store current and goal pose
  double nextX_, nextY_, currentX_, currentY_, currentAngle_;
/// Variables to check if exception was produced
  bool exceptionHandled_;
  bool angleExceptionHandled_;

 public:
  /**
   *  @brief   Constructor of class
   */
  PathPlanningModule();

  /**
   *  @brief   Destructor of class
   */
  ~PathPlanningModule();

  /**
   *  @brief  finds the current location of the turtlebot
   *
   *  @param loopOnce 1 function exits on catch immediately
   *                  0 function listens to transform till
   *                    no exception is  thrown
   *  @returns nothing
   */
  void currentLocation(int loopOnce);

  /**
   *  @brief  Moves the turtlebot at a desired angle
   *
   *  @param  desired angle as input parameter
   *
   *  @returns nothing
   */
  void moveToAngle(double);


/**
   *  @brief  Returns current angle of robot
   *
   *  @param loopOnce 1 function exits on catch immediately
   *                  0 function listens to transform till
   *                    no exception is thrown
   *
   *  @returns current angle of robot
   */
  double getCurrentAngle(int loopOnce);

  /**
   *  @brief  navigates to the set next goals
   *
   *  @returns nothing
   */
  void navigate();

  /**
   *  @brief  sets the coordinates of the goal to be reached
   *
   *  @param  X coordinate position of goal
   *
   *  @param  Y coordinate position of goal
   *
   *  @returns nothing
   */
  void setGoal(double, double);
  /**
   *  @brief  gets current X coordinate of robot
   *
   *  @returns returns x coordinate
   */
  double getX();

  /**
   *  @brief  gets current Y coordinate of robot
   *
   *  @returns returns Y coordinate
   */
  double getY();

  /**
   *  @brief  gets X coordinate of goal
   *
   *  @returns returns nextX_ coordinate
   */
  double getNextX();

  /**
   *  @brief  gets Y coordinate of goal
   *
   *  @returns returns nextY_ coordinate
   */
  double getNextY();

  /**
   * @brief gets the value of exceptionHandled_
   *
   * @return exceptionHandled_ value
   */
  bool getExceptionHandled();

  /**
   * @brief gets the value ofangleExceptionHandled_
   *
   * @return angleExceptionHandled_ value
   */
  bool getAngleExceptionHandled();
};
#endif  // INCLUDE_PATHPLANNINGMODULE_H_


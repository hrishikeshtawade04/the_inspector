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
 *  @file   PathPlanningModule.h
 *
 *  @author   Kapil Rawal (krawal@terpmail.umd.edu)
 *  @author   Hrishikesh Tawde (htawade@terpmail.umd.edu)
 *
 *  @copyright   BSD License
 *
 *  @brief    Path planning module file
 *
 *  @section   DESCRIPTION
 *
 *  This is the header file for the path planning module containing   
 *  declaration of methods 
 */

#ifndef INCLUDE_THE_INSPECTOR_PATHPLANNINGMODULE_H_
#define INCLUDE_THE_INSPECTOR_PATHPLANNINGMODULE_H_

#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_listener.h>

class PathPlanningModule{

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

double  nextX_, nextY_, currentX_, currentY_, currentAngle_;

public:

  /**
   *  @brief   Constructor
   */
  PathPlanningModule();

  /**
   *  @brief   Destroys the object
   */ 
  ~PathPlanningModule();
 
 /**
   *  @brief  finds the current location of the turtlebot
   *
   *
   *  @returns nothing
   */ 
 void currentLocation();

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
   *  @returns current angle of robot
   */ 
 double getCurrentAngle();
 
 /**
   *  @brief  navigates along the given path
   *
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
   *  @brief  gets current X coordinate of goal
   *
   *  @returns returns x coordinate
   */
  double getNextX();
  
  /**
   *  @brief  gets current Y coordinate of goal
   *
   *  @returns returns y coordinate
   */
  double getNextY();

};
#endif /// INCLUDE_THEINSPECTOR_PATHPLANNINGMODULE_H_

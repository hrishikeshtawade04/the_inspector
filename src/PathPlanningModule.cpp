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
 *  @file   PathPlanningModule.cpp
 *
 *  @author   Kapil Rawal (krawal@terpmail.umd.edu)
 *  @author   Hrishikesh Tawde (htawade@terpmail.umd.edu)
 *
 *  @copyright   BSD License
 *
 *  @brief    PathPlanningModule Class implementation
 *
 *  @section   DESCRIPTION
 *
 *  This file contains implementation of pathPlanningModule class
 *  methods
 *
 */

#include "../include/PathPlanningModule.h"
#include <tf/transform_listener.h>
#include<iostream>
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

PathPlanningModule::PathPlanningModule()
    : nextX_(0),
      nextY_(0),
      currentX_(0),
      currentY_(0),
      currentAngle_(0),
      exceptionHandled_(false),
      angleExceptionHandled_(false),
      velocityPublish(
          n.advertise < geometry_msgs::Twist > ("/cmd_vel_mux/input/navi", 1)) {
  /// publisher for velocity commands to turtlebot
  /// setting variables to initial values
}

PathPlanningModule::~PathPlanningModule() {
}

void PathPlanningModule::currentLocation(int loopOnce) {
  int in = 0;
  /// read the  odometry data and find current robot location
  while (ros::ok() && in == 0) {
    /// get the base_link to odom transform and print any exceptions
    try {
      listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
      currentX_ = transform.getOrigin().x();
      currentY_ = transform.getOrigin().y();
      in = 1;
    } catch (tf::TransformException &ex) {
      ROS_INFO("Trying again for transform and printing the exception below");
      ROS_WARN("%s", ex.what());
      ros::Duration(0.001).sleep();
      exceptionHandled_ = true;
      if (loopOnce == 1) {
        break;
      }
    }
    ros::spinOnce();
  }
}

bool PathPlanningModule::getExceptionHandled() {
  return exceptionHandled_;
}

bool PathPlanningModule::getAngleExceptionHandled() {
  return angleExceptionHandled_;
}

void PathPlanningModule::moveToAngle(double angle) {
  while (ros::ok()) {
    /// get the base_link to odom transform and print any exceptions
    try {
      listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
      tf::Matrix3x3 rotationMatrix(transform.getRotation());
      double roll;
      double pitch;
      double prevYaw;
      rotationMatrix.getRPY(roll, pitch, prevYaw);
      double yaw = ((prevYaw * 180.0 / 3.142));
      if ((fabs(angle - yaw) <= 0.5)) {
        break;
      }
      if (fabs(angle - yaw) >= 20) {
        vel_msg.angular.z = 0.5;
        velocityPublish.publish(vel_msg);
      } else {
        vel_msg.angular.z = 0.05;
        velocityPublish.publish(vel_msg);
      }
    } catch (tf::TransformException &ex) {
      ROS_INFO("Trying again for transform and printing the exception below");
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    ros::spinOnce();
  }
  vel_msg.angular.z = 0.0;
  velocityPublish.publish(vel_msg);
  ros::Duration(0.1).sleep();
}

void PathPlanningModule::navigate() {
  /// Navigate to x-coordinate
  moveToAngle(0);
  currentLocation(0);
  while (ros::ok() && (fabs((currentX_ - nextX_)) >= 0.1)) {
    currentLocation(0);
    if ((currentX_ - nextX_) < 0) {
      if (fabs((currentX_ - nextX_)) > 1.0) {
        vel_msg.linear.x = 1.0;
      } else if (fabs((currentX_ - nextX_)) > 0.75) {
        vel_msg.linear.x = 0.5;
      } else {
        vel_msg.linear.x = 0.1;
      }
      velocityPublish.publish(vel_msg);
    } else {
      if (fabs((currentX_ - nextX_)) > 1.0) {
        vel_msg.linear.x = -1.0;
      } else if (fabs((currentX_ - nextX_)) > 0.75) {
        vel_msg.linear.x = -0.5;
      } else {
        vel_msg.linear.x = -0.1;
      }
      velocityPublish.publish(vel_msg);
  }
    ros::spinOnce();
  }
  vel_msg.linear.x = 0.0;
  velocityPublish.publish(vel_msg);
  ros::Duration(0.1).sleep();
  /// navigate to y-coordinate
  moveToAngle(90);
  currentLocation(0);
  while (ros::ok() && (fabs((currentY_ - nextY_)) >= 0.1)) {
    currentLocation(0);
    if ((currentY_ - nextY_) < 0) {
      if (fabs((currentY_ - nextY_)) > 1.0) {
        vel_msg.linear.x = 1.0;
      } else if (fabs((currentY_ - nextY_)) > 0.75) {
        vel_msg.linear.x = 0.5;
      } else {
        vel_msg.linear.x = 0.1;
      }
      velocityPublish.publish(vel_msg);
    } else {
      if (fabs((currentY_ - nextY_)) > 1.0) {
        vel_msg.linear.x = -1.0;
      } else if (fabs((currentY_ - nextY_)) > 0.75) {
        vel_msg.linear.x = -0.5;
      } else {
        vel_msg.linear.x = -0.1;
      }
      velocityPublish.publish(vel_msg);
  }
    ros::spinOnce();
  }
  vel_msg.linear.x = 0.0;
  velocityPublish.publish(vel_msg);
  ros::Duration(0.1).sleep();
}

double PathPlanningModule::getX() {
  return currentX_;
}

double PathPlanningModule::getY() {
  return currentY_;
}

double PathPlanningModule::getNextX() {
  return nextX_;
}

double PathPlanningModule::getNextY() {
  return nextY_;
}

double PathPlanningModule::getCurrentAngle(int loopOnce) {
  double yaw;
 
    /// get the base_link to odom transform and print any exceptions
  
      listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
      tf::Matrix3x3 rotationMatrix(transform.getRotation());
      double roll;
      double pitch;
      double prevYaw;
      rotationMatrix.getRPY(roll, pitch, prevYaw);
      yaw = ((prevYaw * 180.0 / 3.142));
      return yaw;
    
    
  
}
void PathPlanningModule::setGoal(double x, double y) {
  nextX_ = x;
  nextY_ = y;
}


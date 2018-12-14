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
 *  @file   ImageProcessingModule.cpp
 *
 *  @author   Kapil Rawal (krawal@terpmail.umd.edu)
 *  @author   Hrishikesh Tawde (htawade@terpmail.umd.edu)
 *
 *  @copyright   BSD License
 *
 *  @brief    ImageProcessingModule Class Implementation
 *
 *  @section   DESCRIPTION
 *
 *  This file contains the implementations for class ImageProcessingModule
 *
 */

#include "../include/ImageProcessingModule.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "ros/ros.h"


ImageProcessingModule::ImageProcessingModule() {
  image_transport::ImageTransport it(n);
  pubImage_ = it.advertise("camera/image", 10);
  subImage =
      n.subscribe < sensor_msgs::Image
          > ("/camera/rgb/image_raw", 10,
          &ImageProcessingModule::convertImage, this);
  ROS_INFO("Configuration setting done");
}

ImageProcessingModule::~ImageProcessingModule() {
}

void ImageProcessingModule::convertImage(
    const sensor_msgs::Image::ConstPtr& msg) {
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  /// storing the leakage image
  imageLeakage = cv_ptr->image;
}

cv::Mat ImageProcessingModule::getImage() {
  return imageLeakage;
}

std::vector<double> ImageProcessingModule::detectContour(
    std::string wallNumber,
                                                         int storeImage,
                                                         std::string path) {
  std::vector<double> countourLocations;
  int complete = 0;
  /// runs till all contours are found.
  while (ros::ok() && complete == 0) {
    /// run only if image is available
    if (!imageLeakage.empty()) {
      typedef cv::Point2i Point;
      cv::Scalar lowGreen = cv::Scalar(40, 100, 50);
      cv::Scalar highGreen = cv::Scalar(67, 255, 255);
      std::vector < std::vector < Point >> contours;
      std::vector < cv::Vec4i > hierarchy;
      int bboxbrX, bboxbrY, bboxtlX, bboxtlY;
      /// Detect Wall
      cv::Mat grayImage;
      cv::Mat threshY;
      cv::cvtColor(imageLeakage, grayImage, CV_BGR2GRAY);
      cv::threshold(grayImage, threshY, 125, 255, cv::THRESH_BINARY_INV);
      cv::findContours(threshY, contours, hierarchy, CV_RETR_TREE,
                       CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
      int i = 0;
      for (std::vector<Point> point : contours) {
        if (cv::contourArea(point) > 155000) {
          cv::Scalar color = cv::Scalar(255, 255, 0);
          cv::drawContours(imageLeakage, contours, i, color, 2, 8, hierarchy,
                           0,
                           Point());
          bboxtlY = cv::boundingRect(point).tl().y;
          bboxbrY = cv::boundingRect(point).br().y;
          bboxtlX = cv::boundingRect(point).tl().x;
          bboxbrX = cv::boundingRect(point).br().x;
        }
        ++i;
      }
      cv::Mat gryImage;
      cv::Mat hsvImage;
      cv::Mat greenMask;
      cv::Mat bitwiseImage;
      cv::Mat threshImage;
      cv::cvtColor(imageLeakage, hsvImage, cv::COLOR_BGR2HSV);
      cv::inRange(hsvImage, lowGreen, highGreen, greenMask);
      cv::bitwise_and(hsvImage, hsvImage, bitwiseImage, greenMask);
      cv::cvtColor(bitwiseImage, gryImage, CV_BGR2GRAY);
      cv::threshold(gryImage, threshImage, 40, 255, cv::THRESH_BINARY);
      cv::findContours(threshImage, contours, hierarchy, CV_RETR_TREE,
                       CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
      i = 0;
      for (std::vector<Point> point : contours) {
        if (cv::contourArea(point) < 10000
            &&
        cv::contourArea(point) > 500) {
          cv::Scalar color = cv::Scalar(255, 0, 255);
          cv::drawContours(imageLeakage, contours, i, color, 2, 8, hierarchy,
                           0,
                           Point());
          /// Height of leakage on wall
          auto sboxhY = cv::boundingRect(point).y;
          auto positionY = ((3 * 1.0) * (sboxhY - bboxtlY))
              / (bboxbrY - bboxtlY);
          /// Width of leakage on wall
          auto sboxhX = cv::boundingRect(point).x;
          auto positionX = ((8 * 1.0) * (sboxhX - bboxtlX))
              / (bboxbrX - bboxtlX);
          countourLocations.emplace_back(positionX);
          countourLocations.emplace_back(3 - positionY);
        }
        ++i;
      }
      /// Storing Image in data folder
      if (storeImage == 1) {
        std::stringstream sstream;
        sstream << path << "FinalOutput" << wallNumber << ".png";
        ROS_ASSERT(cv::imwrite(sstream.str(), imageLeakage));
      }
      complete = 1;
    }
    ros::spinOnce();
  }
  return countourLocations;
}



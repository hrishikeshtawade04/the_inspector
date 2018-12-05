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
 *  @file   ImageProcessingModule.cpp
 *
 *  @author   Kapil Rawal (krawal@terpmail.umd.edu)
 *  @author   Hrishikesh Tawde (htawade@terpmail.umd.edu)
 *
 *  @copyright   BSD License
 *
 *  @brief    Image processing module file
 *
 *  @section   DESCRIPTION
 *
 *  Image processing module .cpp file which takes image from turtlebot   
 *  camera and gives location of leakges on the wall. 
 */

#include <iostream>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "../include/ImageProcessingModule.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
 
ImageProcessingModule::ImageProcessingModule() {
   image_transport::ImageTransport it (n);
   pub_image = it.advertise("camera/image", 10);
   subImage = n.subscribe <sensor_msgs::Image> ("/camera/rgb/image_raw", 10, &ImageProcessingModule::convertImage, this);
   ROS_INFO("Configuration setting done");
}

 ImageProcessingModule::~ImageProcessingModule() {
}

void ImageProcessingModule::convertImage(const sensor_msgs::Image::ConstPtr& msg) {
      imageLeakage = cv_ptr->image;
}

cv::Mat ImageProcessingModule::getImage() {
      return imageLeakage;
}

std::vector<double> ImageProcessingModule::detectContour(std::string wallNumber) {
std::vector<double> countourLocations;
return countourLocations;
}

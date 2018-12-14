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
 *  @file   ImageProcessingModule.h
 *
 *  @author   Kapil Rawal (krawal@terpmail.umd.edu)
 *  @author   Hrishikesh Tawde (htawade@terpmail.umd.edu)
 *
 *  @copyright   BSD License
 *
 *  @brief    ImageProcessingModule Class Header file
 *
 *  @section   DESCRIPTION
 *
 * Image processing module headerfile containing definitions of its methods
 */

#ifndef INCLUDE_IMAGEPROCESSINGMODULE_H_
#define INCLUDE_IMAGEPROCESSINGMODULE_H_

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief ImageProcessingModule Class
 */
class ImageProcessingModule {
 private:
  /// Created a node handle
  ros::NodeHandle n;
  /// Publisher for cv bridge
  image_transport::Publisher pubImage_;
  /// Obejct of Image pointer
  sensor_msgs::ImagePtr msgpub;
  /// Subscriber named subImage
  ros::Subscriber subImage;
  /// cv_ptr is the mage message pointer
  cv_bridge::CvImagePtr cv_ptr;
  /// Image of leakage on wall
  cv::Mat imageLeakage;
  /// passing node handle to image transport

 public:
  /**
   *  @brief   Constructor
   */
  ImageProcessingModule();

  /**
   *  @brief   Destructor of class
   */
  ~ImageProcessingModule();

  /**
   *  @brief   Converts image from ROS sensor::Image format to openCV cv::Mat format
   *
   *  @param   dataImage is the input image from ros camera topic
   *
   *  @return  Nothing
   */
  void convertImage(const sensor_msgs::Image::ConstPtr& dataImage);

  /**
   *  @brief   Gets the clicked image by the robot
   *
   *  @return  Image
   */
  cv::Mat getImage();

  /**
   *  @brief   Detects the contour from the input image taken from turtlebot camera.
   *
   *  @param   wallNumber combines with path to store image
   *
   *  @param   storeImage 1 stores image in data folder
   *                      0 doesn't store image
   *
   *  @param   path path to where the image needs to be stored
   *
   *  @return  contour locations for the particular input image
   */
  std::vector<double> detectContour(std::string wallNumber, int storeImage,
                                    std::string path);
};
#endif  // INCLUDE_IMAGEPROCESSINGMODULE_H_


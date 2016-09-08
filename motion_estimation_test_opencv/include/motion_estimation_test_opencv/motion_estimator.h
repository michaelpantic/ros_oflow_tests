/*
 * Copyright (c) 2016, Michael Pantic [mpantic@student.ethz.ch],
 *                     Autonomous Systems Lab, ETH Zurich, Switzerland
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MBZIRC_TASK1_MOTION_ESTIMATION_TEST_NODE_H_
#define MBZIRC_TASK1_MOTION_ESTIMATION_TEST_NODE_H_

#include <stdio.h>
#include <stdlib.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/video/tracking.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>


namespace mbzirc_task1 {
class MotionEstimator {
 public:
  explicit MotionEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  MotionEstimator() = delete;
  void receiveImageCallback(const sensor_msgs::ImageConstPtr &msg);
  virtual ~MotionEstimator();



 private:
  // helpers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  image_transport::ImageTransport it_;

  // publisher / subscribers
  ros::Publisher pub_oflow_image_;
  image_transport::Subscriber image_sub_;
  
  bool first_image_ = true;

  //GPU DATA
  cv::UMat gpu_first_image_;
  cv::UMat gpu_second_image_;
  cv::UMat gpu_flow_;
  };
}
#endif /* MBZIRC_TASK1_MOTION_ESTIMATION_TEST_NODE_H_ */

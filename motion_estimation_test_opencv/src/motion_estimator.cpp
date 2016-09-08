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

#include <motion_estimation_test_opencv/motion_estimator.h>
#define OFLOW_OUTPUT
namespace mbzirc_task1 {

MotionEstimator::MotionEstimator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_private_(nh_private),
      nh_(nh),
      it_(nh) {

  // Set up callback for image message
  image_sub_ = it_.subscribe("image_raw", 1,
                             &MotionEstimator::receiveImageCallback, this);

   pub_oflow_image_ =
        nh_.advertise<sensor_msgs::Image>("motion_estimator_oflow_image", 1);

  if(cv::ocl::haveOpenCL())
        {
            cv::ocl::setUseOpenCL(true);

               cv::ocl::Device device = cv::ocl::Device::getDefault();
                ROS_WARN("Device Name: %s\n", device.name().c_str());
                ROS_WARN("Compute Units: %i\n", device.maxComputeUnits());
                ROS_WARN("imageSupport: %i\n", device.imageSupport());
 

   ROS_WARN("OPENCL");
 }
}



/*
 * Runs quad detection and reprojects coordinates upon image receival
 */
void MotionEstimator::receiveImageCallback(
    const sensor_msgs::ImageConstPtr &msg) {
//  ROS_WARN("Image received");

  ros::Time timestamp = msg->header.stamp;
  int height = msg->height;
  int width = msg->width;

  
  if(first_image_)
  {

    // allocate device memory
    gpu_first_image_.create(height,width,CV_8U,cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    gpu_second_image_.create(height,width,CV_8U,cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    gpu_flow_.create(height,width,CV_32FC2,cv::USAGE_ALLOCATE_DEVICE_MEMORY);

    // preload first image
    cv::Mat host_temp(height,width,CV_8U,(void*)&(msg->data[0]));
    host_temp.copyTo(gpu_first_image_);


		//create sampling
		int sampleDistance = 15;
		pts_sampling_.clear();
		pts_sampling_.reserve((width/sampleDistance)*(height/sampleDistance));
		
		for(int x=0;x<width;x+=sampleDistance)
		{
			for(int y=0;y<height;y+=sampleDistance)
			{
				pts_sampling_.push_back(cv::Point2f(x,y));
			}
		}	


    first_image_ = false;
  }

   //load to gpu
   cv::Mat host_temp(height,width,CV_8U,(void*)&(msg->data[0]));
   host_temp.copyTo(gpu_second_image_);

	 cv::calcOpticalFlowPyrLK(gpu_first_image_,
													 gpu_second_image_,
													 pts_sampling_,
													 pts_output_,
													 flow_status_,
													 flow_err_,
													 cv::Size(20,20), //winsize
													 1); //pyramid levels
														
  //copy second to first
   gpu_second_image_.copyTo(gpu_first_image_);
#ifdef OFLOW_OUTPUT
   //copy result to host
	 cv::Mat oflow;
	 cv::cvtColor(host_temp, oflow, CV_GRAY2RGB);

	//draw a line for each point
	for(int i=0;i<pts_sampling_.size();i++)
  {

		if(flow_status_[i])
{		cv::Point2f start = pts_sampling_[i];
		cv::Point2f end = pts_output_[i];

		//draw line from start to end
		cv::line(oflow, start, end, cv::Scalar(255,0,0),2,8,0);}
  }

   sensor_msgs::Image img_msg;
  img_msg.width = width;
  img_msg.height =height;
  img_msg.encoding = "rgb8";
  img_msg.step = sizeof(unsigned char)*3*width;
  img_msg.data = std::vector<unsigned char>(oflow.datastart,oflow.dataend);
 pub_oflow_image_.publish(img_msg);

#else
 //publish empty to measure hz
  sensor_msgs::Image empty_msg;
   pub_oflow_image_.publish(empty_msg);
#endif
  // ROS_WARN("Subsequent pass");
}

MotionEstimator::~MotionEstimator() {
}

}

// *****************************************************************************
//
// Copyright (c) 2020, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

// Convention is for the include guard to be PACKAGE_HEADER
#ifndef SIMPLE_AIRSIM_INTERFACE_AIRSIM_CPP_EXAMPLE_H_
#define SIMPLE_AIRSIM_INTERFACE_AIRSIM_CPP_EXAMPLE_H_

// Main ROS header that includes many definitions for a ROS system
#include <ros/ros.h>

// In C++, camera images are sent using image_transport, not the normal
// message transport system
#include <image_transport/image_transport.h>

// These are the header files for the messages we will subscribe to
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

// Namespace is typically the package name
namespace simple_airsim_interface
{
class AirsimCppExample
{
  public:
    AirsimCppExample();
    ~AirsimCppExample() = default;

  private:
    // These are subscribers to the different topics we will subscribe to
    ros::Subscriber lidar_one_sub_;
    ros::Subscriber lidar_two_sub_;
    image_transport::Subscriber image_sub_;

    // Publisher for putting the car into gear
    ros::Publisher gear_pub_;

    // Timer for periodically sending out debugging information
    ros::Timer timer_;

    // These are callbacks that process the messages we receive
    void lidarOneCallback(const sensor_msgs::PointCloud2ConstPtr & msg);
    void lidarTwoCallback(const sensor_msgs::PointCloud2ConstPtr & msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    // Timer event for outputting debugging information
    void timerCallback(const ros::TimerEvent&);

    int32_t image_count_;
    int32_t lidar_one_count_;
    int32_t lidar_two_count_;
};
}

#endif
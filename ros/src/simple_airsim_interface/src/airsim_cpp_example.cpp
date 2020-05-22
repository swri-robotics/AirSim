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

// This includes the class definition for this node
#include <simple_airsim_interface/airsim_cpp_example.h>

// This is a utility for converting between ROS image types and OpenCV image
// types
#include <cv_bridge/cv_bridge.h>

// Message for changing the gear
#include <std_msgs/String.h>

namespace simple_airsim_interface
{
  // Constructor
  AirsimCppExample::AirsimCppExample() :
    image_count_(0),
    lidar_one_count_(0),
    lidar_two_count_(0)
  {
    // This node handle accesses the global ROS parameter server
    ros::NodeHandle nh("");
    // This handle is for accessing the node's private parameters
    ros::NodeHandle private_nh("~");

    // Publisher for changing the car gear
    gear_pub_ = nh.advertise<std_msgs::String>("/gear_command", 2);

    image_transport::ImageTransport it(nh);

    image_sub_ = it.subscribe(
      "/airsim_node/PhysXCar/front_left_bumblebee/Scene",
      1,
      &AirsimCppExample::imageCallback,
      this);

    lidar_one_sub_ = nh.subscribe(
      "/airsim_node/PhysXCar/lidar/VLP16_1",
      1,
      &AirsimCppExample::lidarOneCallback,
      this);

    lidar_two_sub_ = nh.subscribe(
      "/airsim_node/PhysXCar/lidar/VLP16_2",
      1,
      &AirsimCppExample::lidarTwoCallback,
      this);

    // Create a periodic timer that prints debugging information every 100ms
    timer_ = nh.createTimer(
      ros::Duration(0.1),
      &AirsimCppExample::timerCallback,
      this);
  }

  // Callback for processing first lidar data
  void AirsimCppExample::lidarOneCallback(const sensor_msgs::PointCloud2ConstPtr & msg)
  {
    ROS_ERROR("Received data from first lidar");
    lidar_one_count_++;
  }

  // Callback for processing second lidar data
  void AirsimCppExample::lidarTwoCallback(const sensor_msgs::PointCloud2ConstPtr & msg)
  {
    ROS_ERROR("Received data from second lidar");
    lidar_two_count_++;
  }

  // Callback for processing images
  void AirsimCppExample::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    ROS_ERROR("Received image");
    image_count_++;
    // Example of how to convert the ROS encoded image to an OpenCV image
    cv::Mat mat = cv_bridge::cvtColor(
        cv_bridge::toCvCopy(*msg),
        sensor_msgs::image_encodings::BGR8)->image;
  }

  // Callback that gets called periodically from the timer
  void AirsimCppExample::timerCallback(const ros::TimerEvent&)
  {
    // Output some debugging messages
    ROS_ERROR("Received %d camera images", image_count_);
    ROS_ERROR("Received %d lidar messages from the first lidar", lidar_one_count_);
    ROS_ERROR("Received %d lidar messages from the second lidar", lidar_two_count_);

    // Put the car into gear
    std_msgs::String msg;
    msg.data = "forward";
    gear_pub_.publish(msg);
  }
}

int main(int argc, char** argv)
{
  // Create the example node and name it "airsim_cpp_example". The anonymous
  // name option will also append a random number to it, so if we create
  // multiple nodes of this type, they will all have distinct names
  ros::init(argc, argv, "airsim_cpp_example", ros::init_options::AnonymousName);
  simple_airsim_interface::AirsimCppExample example;
  // Spin, which will cause the message queues to be processed
  ros::spin();
  return 0;
}
// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MINIMAL_COMPOSITION__SUBSCRIBER_PCL_NODE_HPP_
#define MINIMAL_COMPOSITION__SUBSCRIBER_PCL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "minimal_composition_pcl/visibility.h"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

class SubscriberPCLNode : public rclcpp::Node
{
public:
  MINIMAL_COMPOSITION_PUBLIC SubscriberPCLNode(rclcpp::NodeOptions options);

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pointcloud_;
  int index_;
  float downsample_leafsize_x_;
  float downsample_leafsize_y_;
  float downsample_leafsize_z_;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_filterd_ros_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::string frame_id_;
};

#endif  // MINIMAL_COMPOSITION__SUBSCRIBER_PCL_NODE_HPP_

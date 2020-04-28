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

#include "minimal_composition_pcl/subscriber_pcl_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <pcl_conversions/pcl_conversions.h>

SubscriberPCLNode::SubscriberPCLNode(rclcpp::NodeOptions options)
: Node("subscriber_pcl_node", options)
{
  index_ = 0;
  subscription_ = create_subscription<std_msgs::msg::String>(
    "topic_pcl",
    10,
    [this](std_msgs::msg::String::UniquePtr msg) {
      RCLCPP_INFO(this->get_logger(), "SubscriberPCL: '%s'", msg->data.c_str());
    });
#define DEFDEF true
#ifdef DEFDEF
  subscription_pointcloud_= create_subscription<sensor_msgs::msg::PointCloud2>(
    "sc/rgbd/points",
    10,
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {

      RCLCPP_INFO(this->get_logger(),
                  "\ncount: %d\nwidth: '%d'\nheight: '%d'\n",
                  index_++, msg->width, msg->height);

      pcl::PointCloud<pcl::PointXYZRGB> cloud2;
      pcl::fromROSMsg(*msg, cloud2);
      RCLCPP_INFO(this->get_logger(),
                  "cloud2.points.size(): %d\n====================",
                  cloud2.points.size());

    });
#endif
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(SubscriberPCLNode)

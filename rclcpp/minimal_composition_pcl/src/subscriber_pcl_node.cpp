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
#include <pcl/filters/voxel_grid.h>

SubscriberPCLNode::SubscriberPCLNode(rclcpp::NodeOptions options)
: Node("subscriber_pcl_node", options)
, downsample_leafsize_x_(0.01)
, downsample_leafsize_y_(0.01)
, downsample_leafsize_z_(0.01)
, frame_id_("sc_map")
{
  index_ = 0;
  cloud_filterd_ros_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

  pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("downsampled_cloud", 10);
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
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_input_ros_msg) {

      RCLCPP_INFO(this->get_logger(),
                  "\ncount: %d\ncloud_input_ros_msg->width: '%d'\ncloud_input_ros_msg->height: '%d'\n",
                  index_++, cloud_input_ros_msg->width, cloud_input_ros_msg->height);

      pcl::PointCloud<pcl::PointXYZ> cloud_input_pcl;
      pcl::fromROSMsg(*cloud_input_ros_msg, cloud_input_pcl);
      RCLCPP_INFO(this->get_logger(),
                  "\ncloud_input_pcl.points.size(): %d\n",
                  cloud_input_pcl.points.size());

      pcl::PointCloud<pcl::PointXYZ> cloud_downsampled_pcl;
      pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
      voxelSampler.setInputCloud(cloud_input_pcl.makeShared());
      voxelSampler.setLeafSize(downsample_leafsize_x_, downsample_leafsize_y_, downsample_leafsize_z_);
      voxelSampler.filter(cloud_downsampled_pcl);
      RCLCPP_INFO(this->get_logger(),
                  "\ncloud_downsampled_pcl.points.size(): %d\n",
                  cloud_downsampled_pcl.points.size());

      pcl::toROSMsg(cloud_downsampled_pcl, *cloud_filterd_ros_);

      RCLCPP_INFO(this->get_logger(),
                  "\ncloud_filterd_ros_->width: '%d'\ncloud_filterd_ros_->height: '%d'\n====================",
                  cloud_filterd_ros_->width, cloud_filterd_ros_->height);

      cloud_filterd_ros_->header.stamp = now();

      pub_->publish(*cloud_filterd_ros_);
    });
#endif
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(SubscriberPCLNode)

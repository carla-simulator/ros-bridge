/*
 * Copyright (c) 2019-2020 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */
#include <sstream>
#include <string>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "PclRecorderROS2.h"

PclRecorderROS2::PclRecorderROS2() : Node("pcl_recorder")
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfListener = new tf2_ros::TransformListener(*tf_buffer_);

  if (mkdir("/tmp/pcl_capture", 0777) == -1) {
    RCLCPP_WARN(this->get_logger(), "Could not create directory!");
  }

  // Create a ROS subscriber for the input point cloud
  std::string roleName;
  if (!this->get_parameter("role_name", roleName)) {
    roleName = "ego_vehicle";
  }
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/carla/" + roleName + "/lidar", 10, std::bind(&PclRecorderROS2::callback, this, std::placeholders::_1), sub_opt);
}

void PclRecorderROS2::callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  if ((cloud->width * cloud->height) == 0) {
    return;
  }

  std::stringstream ss;
  ss << "/tmp/pcl_capture/capture" << cloud->header.stamp.sec << cloud->header.stamp.nanosec << ".pcd";


  RCLCPP_INFO (this->get_logger(), "Received %d data points. Storing in %s",
           (int)cloud->width * cloud->height,
           ss.str().c_str());

  Eigen::Affine3d transform;
  try {
    transform = tf2::transformToEigen (tf_buffer_->lookupTransform(fixed_frame_, cloud->header.frame_id,  cloud->header.stamp, rclcpp::Duration::from_seconds(1)));

    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pcl::fromROSMsg(*cloud, pclCloud);

    pcl::PointCloud<pcl::PointXYZ> transformedCloud;
    pcl::transformPointCloud (pclCloud, transformedCloud, transform);

    pcl::PCDWriter writer;
    writer.writeBinary(ss.str(), transformedCloud);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(), "Could NOT transform: %s", ex.what());
  }
}

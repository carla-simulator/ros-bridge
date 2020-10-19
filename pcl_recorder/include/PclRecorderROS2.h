/*
 * Copyright (c) 2019-2020 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */
#pragma once

#include "rclcpp/rclcpp.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <rclcpp/time_source.hpp>

class PclRecorderROS2 : public rclcpp::Node
{
public:

  PclRecorderROS2();

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
  tf2_ros::TransformListener *tfListener;
  static constexpr const char* fixed_frame_ = "map";

};

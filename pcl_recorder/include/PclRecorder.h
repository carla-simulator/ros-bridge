/*
 * Copyright (c) 2019 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */
#pragma once

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

class PclRecorder
{
public:

  PclRecorder();

  void callback(const pcl::PCLPointCloud2::ConstPtr& cloud);

private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener *tfListener;
  static constexpr const char* fixed_frame_ = "map";

};

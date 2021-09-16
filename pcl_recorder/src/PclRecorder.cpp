/*
 * Copyright (c) 2019 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */
#include "PclRecorder.h"
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <sstream>

PclRecorder::PclRecorder()
{
  tfListener = new tf2_ros::TransformListener(tf_buffer_);

  if (mkdir("/tmp/pcl_capture", 0777) == -1) {
    ROS_WARN("Could not create directory!");
  }

  // Create a ROS subscriber for the input point cloud
  std::string roleName;
  if (!ros::param::get("~role_name", roleName)) {
    roleName = "ego_vehicle";
  }
  sub = nh.subscribe("/carla/" + roleName + "/lidar", 1, &PclRecorder::callback, this);
}

void PclRecorder::callback(const boost::shared_ptr<const pcl::PCLPointCloud2>& cloud)
{
  if ((cloud->width * cloud->height) == 0) {
    return;
  }

  std::stringstream ss;
  ss << "/tmp/pcl_capture/capture" << cloud->header.stamp << ".pcd";

  ROS_INFO ("Received %d data points. Storing in %s",
           (int)cloud->width * cloud->height,
           ss.str().c_str());

  Eigen::Affine3d transform;
  try {
    transform = tf2::transformToEigen (tf_buffer_.lookupTransform(fixed_frame_, cloud->header.frame_id,  pcl_conversions::fromPCL (cloud->header.stamp), ros::Duration(1)));

    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pcl::fromPCLPointCloud2(*cloud, pclCloud);

    pcl::PointCloud<pcl::PointXYZ> transformedCloud;
    pcl::transformPointCloud (pclCloud, transformedCloud, transform);

    pcl::PCDWriter writer;
    writer.writeBinary(ss.str(), transformedCloud);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("Could NOT transform: %s", ex.what());
  }
}

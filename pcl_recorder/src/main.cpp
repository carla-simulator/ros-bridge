/*
 * Copyright (c) 2019 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */
#include <ros/ros.h>
#include "PclRecorder.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_recorder");
  PclRecorder pclRecorder;
  ros::spin();
};

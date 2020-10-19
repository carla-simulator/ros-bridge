/*
 * Copyright (c) 2019 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */
#include "rclcpp/rclcpp.hpp"
#include "PclRecorderROS2.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PclRecorderROS2>());
  rclcpp::shutdown();
  return 0;
}

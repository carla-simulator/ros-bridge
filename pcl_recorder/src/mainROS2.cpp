/*
 * Copyright (c) 2019 Intel Corporation
 *
 * This work is licensed under the terms of the MIT license.
 * For a copy, see <https://opensource.org/licenses/MIT>.
 */
#include "PclRecorderROS2.h"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto recorder = std::make_shared<PclRecorderROS2>();
  executor.add_node(recorder);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

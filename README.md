# ROS2 bridge for CARLA simulator v0.9.15

[![Actions Status](https://github.com/carla-simulator/ros-bridge/workflows/CI/badge.svg)](https://github.com/carla-simulator/ros-bridge)
[![Documentation](https://readthedocs.org/projects/carla/badge/?version=latest)](http://carla.readthedocs.io)
[![GitHub](https://img.shields.io/github/license/carla-simulator/ros-bridge)](https://github.com/carla-simulator/ros-bridge/blob/master/LICENSE)
[![GitHub release (latest by date)](https://img.shields.io/github/v/release/carla-simulator/ros-bridge)](https://github.com/carla-simulator/ros-bridge/releases/latest)

This ROS package is a modified fork of the carla-simulator/ros-bridge package that is adopted to work with ROS2 Humble running on Ubuntu 22.04 LTS with Scenario Runner v0.9.15. The Ros bridge enables two-way communication between ROS and CARLA. The information from the CARLA server is translated to ROS topics. In the same way, the messages sent between nodes in ROS get translated to commands to be applied in CARLA.

## Main Requirements

- OS: Ubuntu 22.04 LTS
- CARLA Version: 0.9.15
- Scenario Runner Version: 0.9.15
- ROS Version: Humble

![rviz setup](./docs/images/ad_demo.png "AD Demo")

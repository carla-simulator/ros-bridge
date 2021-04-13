# ROS Bridge Documentation

This is the documentation for the ROS bridge which enables two-way communication between ROS and CARLA. The information from the CARLA server is translated to ROS topics. In the same way, the messages sent between nodes in ROS get translated to commands to be applied in CARLA.

The ROS bridge is compatible with both ROS 1 and ROS 2.

The ROS bridge boasts the following features:

- Provides sensor data for LIDAR, Semantic LIDAR, Cameras (depth, segmentation, rgb, dvs), GNSS, Radar and IMU.
- Provides object data such as transforms, traffic light status, visualisation markers, collision and lane invasion.
- Control of AD agents through steering, throttle and brake.
- Control of aspects of the CARLA simulation like synchronous mode, playing and pausing the simulation and setting simulation parameters.

## Get started

- [__Installing ROS bridge for ROS 1__](ros_installation_ros1.md)
- [__Installing ROS bridge for ROS 2__](ros_installation_ros2.md)

## Learn about the main ROS bridge package

- [__Carla ROS bridge__](run_ros.md) - The main package required to run the ROS bridge

## Learn about the additional ROS bridge packages

- [__Carla Spawn Objects__](carla_spawn_objects.md) - Provides a generic way to spawn actors
- [__Carla Manual Control__](carla_manual_control.md)- A ROS-based visualization and control tool for an ego vehicle (similar to carla_manual_control.py provided by CARLA)
- [__Carla Ackerman Control__](carla_ackermann_control.md) - A controller to convert ackermann commands to steer/throttle/brake
- [__RQT Plugin__](rqt_plugin.md) - A RQT plugin to control CARLA
- [__RVIZ plugin__](rviz_plugin.md) - An RVIZ plugin to visualize/control CARLA.

## Explore the reference material

- [__ROS Sensors__](ros_sensors.md)
- [__ROS messages__](ros_msgs.md)

# ROS Bridge Documentation

This is the documentation for the ROS bridge which enables two-way communication between ROS and CARLA. The information from the CARLA server is translated to ROS topics. In the same way, the messages sent between nodes in ROS get translated to commands to be applied in CARLA.

The ROS bridge is compatible with both ROS 1 and ROS 2.

The ROS bridge boasts the following features:

- Provides sensor data for LIDAR, Semantic LIDAR, Cameras (depth, segmentation, rgb, dvs), GNSS, Radar and IMU.
- Provides object data such as transforms, traffic light status, visualisation markers, collision and lane invasion.
- Control of AD agents through steering, throttle and brake.
- Control of aspects of the CARLA simulation like synchronous mode, playing and pausing the simulation and setting simulation parameters.

---

## Get started

- [__Installing ROS bridge for ROS 1__](ros_installation_ros1.md)
- [__Installing ROS bridge for ROS 2__](ros_installation_ros2.md)

---

## Learn about the main ROS bridge package

- [__CARLA ROS bridge__](run_ros.md) - The main package required to run the ROS bridge
- [__ROS Compatiblity Node__](ros_compatibility.md) - The interface that allows the same API to call either ROS 1 or ROS 2 functions

---

## Learn about the additional ROS bridge packages

- [__CARLA Spawn Objects__](carla_spawn_objects.md) - Provides a generic way to spawn actors
- [__CARLA Manual Control__](carla_manual_control.md)- A ROS-based visualization and control tool for an ego vehicle (similar to `carla_manual_control.py` provided by CARLA)
- [__CARLA Ackerman Control__](carla_ackermann_control.md) - A controller to convert ackermann commands to steer/throttle/brake
- [__CARLA Waypoint Publisher__](carla_waypoint.md) - Publish and query CARLA waypoints
- [__CARLA AD Agent__](carla_ad_agent.md) - An example agent that follows a route, avoids collisions and respects traffic lights
- [__CARLA AD Demo__](carla_ad_demo.md) - An example package that provides everything needed to launch a CARLA ROS environment with an AD vehicle
- [__CARLA ROS Scenario Runner__](carla_ros_scenario_runner.md) - A wrapper to execute OpenScenarios with the CARLA Scenario Runner via ROS
- [__CARLA Twist to Control__](carla_twist_to_control.md) - Convert twist controls to CARLA vehicle controls
- [__RVIZ plugin__](rviz_plugin.md) - An RVIZ plugin to visualize/control CARLA
- [__RQT Plugin__](rqt_plugin.md) - A RQT plugin to control CARLA
- [__PCL Recorder__](pcl_recorder.md) - Create point cloud maps from data captured from simulations

---

## Explore the reference material

- [__ROS Sensors__](ros_sensors.md) - Reference topics available in the different sensors
- [__ROS messages__](ros_msgs.md) - Reference parameters available in CARLA ROS messages

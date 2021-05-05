# ROS Compatiblity Node

The [ROS compatibility node](https://github.com/carla-simulator/ros-bridge/tree/master/ros_compatibility) is an interface that allows packages to be used seamlessly with both ROS 1 and ROS 2. Depending on the environment variable `ROS_VERSION`, the same API will call either ROS 1 or ROS 2 functions. It is used by creating classes that inherit from the `CompatibleNode`.

---

## ROS parameters

Parameters need to be declared before being set or accessed in ROS 2 by default. This is not the case in ROS 1. In order to keep both ROS 1 and ROS 2 modes working in a similar way, the parameter `allow_undeclared_parameters` is set to `True` in the ROS 2 version of the `CompatibleNode`, allowing the use of parameters without declaring them beforehand.

---

## Services

In ROS 2, services can be called asynchronously. This is not the case in ROS 1. Consequently, the `call_service()` method of the ROS 2 version waits for the server's response after calling it asynchronously, in order to mimic the ROS 1 synchronous behavior.

!!! Warning
    While waiting for the response, the ROS 2 `call_service()` methods spins the node. This can cause problems (errors or deadlocks) if another thread spins the same node in parallel.

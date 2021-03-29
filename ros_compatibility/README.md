# ROS compatibility node

This acts as an interface over ROS1 and ROS2 to allow nodes to be used seamlessly with both versions.
Depending on the environment variable `ROS_VERSION`, the same api will call either ROS1 or ROS2 functions.
It is used by creating classes that inherit from the `CompatibleNode`.

## ROS parameters

By default in ROS2, parameters need to be declared before being set or accessed, which is not the case in ROS1. In order to keep both ROS1 and ROS2 modes working in a similar way, the parameter `allow_undeclared_parameters` is set to True in the ROS2 version of the `CompatibleNode`, allowing to use parameters without declaring them beforehand.

## Services

In ROS2 services can be called asynchronously, this is not the case in ROS1. Consequently, the `call_service()` method of the ROS2 version waits for the server's response after calling it asynchronously, in order to mimic the ROS1 synchronous behavior.

WARNING: While waiting for the response, the ROS2 `call_service()` methods spins the node. This can cause problems (errors or deadlocks) if another thread spins the same node in parallel.

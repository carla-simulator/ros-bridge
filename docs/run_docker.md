# Run Docker

Running docker is as easy as running two scripts and sending one command. One script to build the container, one to run the container and one ros launch command.

This page is speficially designated for ROS 2 Humble. 

# Requirements

Docker has to be installed in the system. This is the only requirements.

# Building the Container

It is recommended for ROS 2 Humble to use the `build_user.sh` script:

```
cd <ros-bridge folder>
./build_user.sh
```

# Running the Container

```
./run.sh
```

Now the window is inside the ros-bridge container.

# Launching ROS 2

Make sure the carla server is running before launching ros:

```
source install/setup.bash
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch
```
# Carla Manual Control

The node `carla_manual_control` is a ROS-only version of the Carla `manual_control.py`. All data is received
via ROS topics.

## Prerequistes

To be able to use `carla_manual_control`, some sensors need to be attached to the ego vehicle:

-   to display an image: a camera with role-name 'view' and resolution 800x600
-   to display the current gnss position: a gnss sensor with role-name 'gnss1'
-   to get a notification on lane invasions: a lane invasion sensor
-   to get a notification on lane invasions: a collision sensor

## Manual steering

In order to steer manually, press 'B'. This will toggle manual-driving mode within carla_ros_bridge.

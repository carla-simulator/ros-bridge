# Carla Manual Control

The node `carla_manual_control` is a ROS-only version of the Carla `manual_control.py`. All data is received
via ROS topics.

## Prerequistes
To be able to use `carla_manual_control`, some sensors need to be attached to the ego vehicle:
- to display an image: a camera with role-name 'view' and resolution 800x600
- to display the current gnss position: a gnss sensor with role-name 'gnss1'
- to get a notification on lane invasions: a lane invasion sensor
- to get a notification on lane invasions: a collision sensor


## Manual steering

In order to steer manually, you might need to disable sending vehicle control commands within another ROS node.

Therefore the manual control is able to publish to `/carla/<ego vehicle role name>/vehicle_control_manual_override` ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html)).

Press `B` to toggle the value.

Note: As sending the vehicle control commands is highly dependent on your setup, you need to implement the subscriber to that topic yourself.


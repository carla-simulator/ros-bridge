# Carla Spectator Camera

This node allows to spawn a camera, attached to an ego vehicle and move its pose via a ros topic.

## Startup

To run it:

    roslaunch carla_spectator_camera carla_spectator_camera.launch

## Set the camera pose

The camera pose can be set by publishing to:

| Topic                                      | Type                                                                                         |
| ------------------------------------------ | -------------------------------------------------------------------------------------------- |
| `/carla/<ego vehicle name>/spectator_pose` | [geometry_msgs.PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html) |

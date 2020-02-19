# Carla Waypoint Publisher

Carla supports waypoint calculations.
The node `carla_waypoint_publisher` makes this feature available in the ROS context.

It uses the current pose of the ego vehicle with role-name "ego_vehicle" as starting point. If the
vehicle is respawned or moved, the route is newly calculated.

Additionally, services are provided to query CARLA waypoints.

## Startup

As the waypoint publisher requires some Carla PythonAPI functionality that is not part of the python egg-file, you
have to extend your PYTHONPATH.

    export PYTHONPATH=$PYTHONPATH:<path-to-carla>/PythonAPI/carla/dist/carla-<carla_version_and_arch>.egg:<path-to-carla>/PythonAPI/carla/

To run it:

    roslaunch carla_waypoint_publisher carla_waypoint_publisher.launch

## Set a goal

The goal is either read from the ROS topic `/carla/<ROLE NAME>/goal`, if available, or a fixed spawnpoint is used.

The prefered way of setting a goal is to click '2D Nav Goal' in RVIZ.

![set goal](../docs/images/rviz_set_start_goal.png)

## Published waypoints

The calculated route is published:

| Topic                                 | Type                                                                 |
| ------------------------------------- | -------------------------------------------------------------------- |
| `/carla/<ego vehicle name>/waypoints` | [nav_msgs.Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html) |


## Available services

| Service                                                     | Description | Type                                                         |
| ----------------------------------------------------------- | ----------- | -------------------------------------------------------------------- |
| `/carla_waypoint_publisher/<ego vehicle name>/get_waypoint` | Get the waypoint for a specific location | [carla_waypoint_types.GetWaypoint](../carla_waypoint_types/srv/GetWaypoint.srv) |
| `/carla_waypoint_publisher/<ego vehicle name>/get_actor_waypoint` | Get the waypoint for an actor id | [carla_waypoint_types.GetActorWaypoint](../carla_waypoint_types/srv/GetActorWaypoint.srv) |

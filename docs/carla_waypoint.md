# CARLA Waypoint Publisher

The [CARLA Waypoint Publisher](https://github.com/carla-simulator/ros-bridge/tree/master/carla_waypoint_publisher) makes waypoint calculations available to the ROS context and provides services to query CARLA waypoints. To find out more about waypoints, see the CARLA [documentation](https://carla.readthedocs.io/en/latest/core_map/#navigation-in-carla).

- [__Run the Waypoint Publisher__](#run-the-waypoint-publisher)
    - [Set a goal](#set-a-goal)
- [__Using the Waypoint Publisher__](#using-the-waypoint-publisher)
- [__ROS API__](#ros-api)
    - [Publications](#publications)
    - [Services](#services)

---

## Run the Waypoint Publisher

With a CARLA server running, execute the following command:

```sh
# ROS 1
roslaunch carla_waypoint_publisher carla_waypoint_publisher.launch

# ROS 2
ros2 launch carla_waypoint_publisher carla_waypoint_publisher.launch.py
```

### Set a goal

If available a goal is read from the topic `/carla/<ROLE NAME>/goal`, otherwise a fixed spawn point is used. 

The preferred way of setting a goal is to click '2D Nav Goal` in RVIZ.

![rviz_set_goal](images/rviz_set_start_goal.png)

---

### Using the Waypoint Publisher

The [CARLA AD demo](carla_ad_demo.md) uses the Waypoint Publisher to plan a route for the [CARLA AD agent](carla_ad_agent.md). See the CARLA AD demo [launchfile](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_ad_demo/launch/carla_ad_demo_with_scenario.launch) for an example on how it is used.  

---

## ROS API

#### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/<ego vehicle name>/waypoints` | [nav_msgs/Path](https://docs.ros.org/en/api/nav_msgs/html/msg/Path.html) | Publishes the calculated route |

<br>

#### Services

| Service | Type | Description |
|-------|------|-------------|
| `/carla_waypoint_publisher/<ego vehicle name>/get_waypoint` | [carla_waypoint_types/GetWaypoint](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_waypoint_types/srv/GetWaypoint.srv) | Get the waypoint for a specific location |
| `/carla_waypoint_publisher/<ego vehicle name>/get_actor_waypoint` | [carla_waypoint_types/GetActorWaypoint](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_waypoint_types/srv/GetActorWaypoint.srv) | Get the waypoint for an actor id |

<br>

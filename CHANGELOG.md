## Latest

## CARLA-ROS-Bridge 0.9.12

*   Fixed scenario runner node shutdown for foxy
*   Fixed actor synchronization creation
*   Fix of scenario runner status

## CARLA-ROS-Bridge 0.9.11

*   Added static markers for carla environment objects.
*   Added hybrid ROS1/ROS2 bridge.
*   Added passive mode. Wordl configuration and ticking are performed by other clients.
*   Support spawning of pseudo-actors through service
*   Use new spawning service to combine carla_infrastructure and carla_ego_vehicle into carla_spawn_objects
*   Reworked ROS topics
*   rework tf in sync mode to represent attachment of sensors to a vehicle
*   Updated debian packaging pipeline

## CARLA-ROS-Bridge 0.9.10.1

*   Support noetic and python 3
*   Have all sensor types in sensors.json
*   Update ad-demo rviz config to visualize more sensor types
*   Remove CarlaRadarMeasurement message publishing (radar data is published as PointCloud2 only)
*   Remove sensors.json from carla_ad_demo. Use example from carla_ego_vehicle instead
*   Add radar PointCloud2 publisher
*   Add DVS camera sensor
*   Fix rgb camera attributes
*   Add intensity value to point cloud message
*   Fixed wrong TF for ego_vehicle
*   Improve version check
*   Fix cleanup
*   Rework tf frame names
*   ObjectSensor: Fix object twist
*   Support loading OpenDRIVE map
*   Traffic Lights: Only publish to /carla/traffic_lights on change
*   Traffic Lights: Publish /carla/traffic_lights_info, containing the location and the trigger volume
*   Added ROS Parameter to set the CARLA client timeout value for all nodes consistently

## CARLA-ROS-Bridge 0.9.8

*   change Lidar range in meters
*   add new attributes for Gnss and Camera sensor
*   add IMU and Radar sensor
*   Fix tf publishing in synchronous mode
*   Add node to convert a twist to a vehicle control command
*   Add node carla_spectator_camera
*   Update carla_waypoint_publisher
*   Add carla_ros_scenario_runner
*   Add rviz_carla_plugin
*   Add carla_ad_agent
*   Add carla_ad_demo

## CARLA-ROS-Bridge 0.9.6

*   remove launchfile check from rqt_carla_control
*   Add roslaunch check to all nodes
*   support kinetic and melodic
*   added possibility to connect to an existing ego vehicle
*   support different ego vehicle rolenames in pclrecorder
*   publish odometry for all traffic participants
*   support drawing markers in CARLA
*   replace /carla/map (msg: CarlaMapInfo) by /carla/world_info (msg: CarlaWorldInfo)
*   added option to reload the CARLA world
*   added node to spawn infrastructure sensors
*   rename /carla/vehicle_marker to /carla/marker (and include walkers)
*   support walkers
*   create rqt plugin to control synchronous mode
*   support synchronous mode
*   publish CarlaStatus
*   remove global_id mapping
*   publish /carla/actor_list
*   carla_ego_vehicle: support sensor_tick within camera/lidar definition
*   support twist_cmd to set velocity of ego vehicle (without respecting the vehicle constraints)

## CARLA-ROS-Bridge 0.9.5.1

*   add id to CarlaEgoVehicleInfo datatype
*   rename carla_ros_bridge_msgs to carla_msgs
*   remove 'challenge' mode

## CARLA-ROS-Bridge 0.9.5

*   rename gnss topic from '../gnss' to '../fix'
*   Add lane invasion sensor
*   Add collision sensor
*   Rename CarlaVehicleControl to CarlaEgoVehicleControl (and add some more message types)
*   move PID controller into separate ROS node
*   Add challenge mode
*   Split actor-monitoring + data publishing
*   Use sensor data timestamp
*   support simple-pid 0.1.5

## CARLA-ROS-Bridge 0.9.3

*   Send vehicle commands to CARLA, only when a ROS command provider is available
*   Added interface for GNSS sensor

## CARLA-ROS-Bridge 0.9.2

*   Updated ROS-bridge to PythonAPI of CARLA 0.9.x
*   Supported sensors: LiDAR, RGB camera, depth camera, segmentation camera
*   Vehicle control options: Ackermann-command or Throttle/Brake/Steering
*   Added PID controller to convert Ackermann-command into Throttle/Brake/Steering
*   RViz support
*   Publish TF tree, map, CARLA clock
*   Separate ROS topic for each vehicle

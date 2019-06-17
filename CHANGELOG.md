## Latest changed

## CARLA-ROS-Bridge 0.9.5.1

* add id to CarlaEgoVehicleInfo datatype
* rename carla_ros_bridge_msgs to carla_msgs
* remove 'challenge' mode

## CARLA-ROS-Bridge 0.9.5

* rename gnss topic from '../gnss' to '../fix'
* Add lane invasion sensor
* Add collision sensor
* Rename CarlaVehicleControl to CarlaEgoVehicleControl (and add some more message types)
* move PID controller into separate ROS node
* Add challenge mode
* Split actor-monitoring + data publishing
* Use sensor data timestamp
* support simple-pid 0.1.5


## CARLA-ROS-Bridge 0.9.3

* Send vehicle commands to CARLA, only when a ROS command provider is available
* Added interface for GNSS sensor


## CARLA-ROS-Bridge 0.9.2

* Updated ROS-bridge to PythonAPI of CARLA 0.9.x
  * Supported sensors: LiDAR, RGB camera, depth camera, segmentation camera
  * Vehicle control options: Ackermann-command or Throttle/Brake/Steering
  * Added PID controller to convert Ackermann-command into Throttle/Brake/Steering
  * RViz support
  * Publish TF tree, map, CARLA clock
  * Separate ROS topic for each vehicle

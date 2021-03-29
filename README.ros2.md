# ROS2

Currently supported: [ROS2 Foxy](https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/)

## Setup

Colcon and ROS2 Foxy need to be installed on your system.

```shell
# setup folder structure
mkdir -p ~/carla-ros-bridge && cd ~/carla-ros-bridge
git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git src/ros-bridge
source /opt/ros/foxy/setup.bash

# install required ros-dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r

# build
colcon build
```

For more information about configuring a ROS2 environment see
<https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/>

## Start the ROS bridge

First run the simulator (see CARLA documentation: <http://carla.readthedocs.io/en/latest/>)

```shell
# run carla in background
SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl

# Add the carla modules to your python environment
export CARLA_ROOT=<path-to-carla>
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-<carla_version_and_arch>.egg:$CARLA_ROOT/PythonAPI/carla

source ./install/setup.bash
```

Start the ros bridge (choose one option):

```shell
# Option 1: start the ros bridge
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py

# Option 2: start the ros bridge together with an example ego vehicle
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```

## Testing

To execute the tests using colcon, use the following commands:

```shell
# build
colcon build --packages-up-to carla_ros_bridge
# run
launch_test carla_ros_bridge/test/ros_bridge_client_ros2_test.py
```

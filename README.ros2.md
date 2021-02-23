# ROS2

Currently supported: [Foxy](https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/)

# Setup

Colcon and ROS2 Foxy needs to be installed on your system.

    git clone https://github.com/carla-simulator/ros-bridge.git
    cd ros-bridge
    git submodule update --init
    source /opt/ros/foxy/setup.bash
    colcon build

For more information about configuring a ROS2 environment see
<https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/>

## Start the ROS bridge

First run the simulator (see carla documentation: <http://carla.readthedocs.io/en/latest/>)

    # run carla in background
    SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl

    # Add the 
    export CARLA_ROOT=<path-to-carla>
    export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-<carla_version_and_arch>.egg:$CARLA_ROOT/PythonAPI/carla

##### For Users

    source /opt/carla-ros-bridge/<kinetic or melodic or noetic>/setup.bash

##### For Developers

    source ~/carla-ros-bridge/catkin_ws/devel/setup.bash

Start the ros bridge (choose one option):

    # Option 1: start the ros bridge
    roslaunch carla_ros_bridge carla_ros_bridge.launch

    # Option 2: start the ros bridge together with an example ego vehicle
    roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch

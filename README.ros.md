# ROS

## Setup

### For Users

First add the apt repository:

    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1AF1527DE64CB8D9
    sudo add-apt-repository "deb [arch=amd64] http://dist.carla.org/carla $(lsb_release -sc) main"

Then simply install the ROS bridge:

    sudo apt-get update
    sudo apt-get install carla-ros-bridge

This will install carla-ros-bridge in /opt/carla-ros-bridge

### For Developers

    Create a catkin workspace and install carla_ros_bridge package

    #setup folder structure
    mkdir -p ~/carla-ros-bridge/catkin_ws/src
    cd ~/carla-ros-bridge
    git clone https://github.com/carla-simulator/ros-bridge.git
    cd ros-bridge
    git submodule update --init
    cd ../catkin_ws/src
    ln -s ../../ros-bridge
    source /opt/ros/<kinetic or melodic or noetic>/setup.bash
    cd ..

    #install required ros-dependencies
    rosdep update
    rosdep install --from-paths src --ignore-src -r

    #build
    catkin_make

For more information about configuring a ROS environment see
<http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment>

## Start the ROS bridge

First run the simulator (see carla documentation: <http://carla.readthedocs.io/en/latest/>)

    # run carla in background
    SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl

    # Add the carla modules to your python environment
    export CARLA_ROOT=<path-to-carla>
    export PYTHONPATH=$PYTHONPATH:<path-to-carla>/PythonAPI/carla/dist/carla-<carla_version_and_arch>.egg

### For Users

    source /opt/carla-ros-bridge/<kinetic or melodic or noetic>/setup.bash

### For Developers

    source ~/carla-ros-bridge/catkin_ws/devel/setup.bash

Start the ros bridge (choose one option):

    # Option 1: start the ros bridge
    roslaunch carla_ros_bridge carla_ros_bridge.launch

    # Option 2: start the ros bridge together with an example ego vehicle
    roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch

## Testing

To execute the tests, using catkin, use the following commands:
    
    # build
    catkin_make -DCATKIN_ENABLE_TESTING=0
    # run
    rostest carla_ros_bridge ros_bridge_client.test

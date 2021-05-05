# ROS bridge installation for ROS 2

This section is a guide on how to install the ROS bridge on Linux for use with ROS 2. You will find the prerequisites, installation steps, how to run a basic package to make sure everything is working well and commands to run tests.

- [__Before you begin__](#before-you-begin)
- [__ROS bridge installation __](#ros-bridge-installation)
- [__Run the ROS bridge__](#run-the-ros-bridge)
- [__Testing__](#testing)

!!! Important
    ROS is still [experimental](http://wiki.ros.org/noetic/Installation) for Windows. It has only been tested for Linux systems.

---

## Before you begin

You will need to fulfill the following software requirements before using the ROS bridge:

- Install ROS:
    - [__ROS 2 Foxy__](https://docs.ros.org/en/foxy/Installation.html) — For Ubuntu 20.04 (Focal)
- Additional ROS packages may be required depending on your needs. [rviz](https://wiki.ros.org/rviz) is highly recommended to visualize ROS data.
- CARLA 0.9.11 or later — Previous versions are not compatible with the ROS bridge. Follow the [quick start installation](https://carla.readthedocs.io/en/latest/start_quickstart/) or make the build for [Linux](https://carla.readthedocs.io/en/latest/build_linux/). It is recommended to match the ROS bridge version to the CARLA version when possible.

---

## ROS bridge installation

!!! Note
    The Debian package installation is not yet available for ROS 2.

__1.__ Set up the project directory and clone the ROS bridge repository and submodules:

```sh
    mkdir -p ~/carla-ros-bridge && cd ~/carla-ros-bridge
    git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git src/ros-bridge
```

__2.__ Set up the ROS environment:

```sh
    source /opt/ros/foxy/setup.bash
```

__3.__ Install the ROS dependencies:

```sh
    rosdep update
    rosdep install --from-paths src --ignore-src -r
```

__4.__ Build the ROS bridge workspace using colcon:

```sh
    colcon build
```

---

## Run the ROS bridge

__1.__ Start a CARLA server according to the installation method used to install CARLA:

```sh
    # Package version in carla root folder
    ./CarlaUE4.sh

    # Debian installation in `opt/carla-simulator/`
    ./CarlaUE4.sh

    # Build from source version in carla root folder
    make launch
```

__2.__ Add the correct CARLA modules to your Python path:

```sh
    export CARLA_ROOT=<path-to-carla>
    export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-<carla_version_and_arch>.egg:$CARLA_ROOT/PythonAPI/carla
```
__3.__ Add the source path for the ROS bridge workspace:

```sh
    source ./install/setup.bash
```

__4.__ In another terminal, start the ROS 2 bridge. You can run one of the two options below:

```sh
    # Option 1, start the basic ROS bridge package
    ros2 launch carla_ros_bridge carla_ros_bridge.launch.py

    # Option 2, start the ROS bridge with an example ego vehicle
    ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```

!!! Note

    If you receive the error: `ImportError: no module named CARLA` then the path to the CARLA Python API is missing. The apt installation sets the path automatically, but it may be missing for other installations.

    You will need to add the appropriate `.egg` file to your Python path. You will find the file in either `/PythonAPI/` or `/PythonAPI/dist/` depending on the CARLA installation. Execute the following command with the complete path to the `.egg` file, using the file that corresponds to your installed version of Python:

    `export PYTHONPATH=$PYTHONPATH:path/to/carla/PythonAPI/<your_egg_file>`

    It is recommended to set this variable permanently by adding the previous line to your `.bashrc` file.

    To check the CARLA library can be imported correctly, run the following command and wait for a success message:

            python3 -c 'import carla;print("Success")' # python3

            or

            python -c 'import carla;print("Success")' # python2

---

## Testing

To execute tests using colcon:

__1.__ Build the package:

```sh
    colcon build --packages-up-to carla_ros_bridge
```

__2.__ Run the tests:

```sh
    launch_test carla_ros_bridge/test/ros_bridge_client_ros2_test.py
```

---
